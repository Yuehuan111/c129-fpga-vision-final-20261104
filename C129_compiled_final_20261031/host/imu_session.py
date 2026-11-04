"""Persistent, read-only by default IMU/C124/C125 compensation session.

One FPGA handle belongs to this object. Neither the UI nor a telemetry utility
should open it again. A source frame is observed after output; this protocol
does not bind a requested pose to an exposure or a chosen future frame.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, is_dataclass
import json
import math
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import threading
import time
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT / "vendor") not in sys.path:
    sys.path.insert(0, str(ROOT / "vendor"))
from complete_geometry import configuration, check_geometry, load_calibration, validate_words
from complete_control import configuration_packets, enhancement_status, verify_firmware
from read_latency import MixedFramer, LatencyAnalysis
from pose_protocol import REALTIME_BUILD_ID, REALTIME_BUILD_IDS, pose_packets, verify_pose
from imu_pose import ImuDecoder, PoseHistory, compensation_matrix, normalize_quaternion

IMU_100HZ_COMMAND = bytes.fromhex("7E230760645FCB")
MAX_COMMITS = 0xFFFFFFFF


def _jsonable(value):
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, deque)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _forward(new, old, bits=32):
    return 0 < ((int(new) - int(old)) & ((1 << bits) - 1)) < (1 << (bits - 1))


def list_serial_ports(ports_provider=None):
    """Identify known USB adapters without opening or probing any port."""
    if ports_provider is None:
        from serial.tools import list_ports
        ports_provider = list_ports.comports
    answer = []
    for item in ports_provider():
        role = None
        if (item.vid, item.pid) == (0x10C4, 0xEA60):
            role = "fpga"
        elif item.vid == 0x1A86 and item.pid in (0x7523, 0x5523):
            role = "imu"
        answer.append(dict(device=item.device, description=item.description,
                           vid=item.vid, pid=item.pid, role=role))
    return answer


def _geometry_process(words, allow_black, output):
    """Spawn target: exact integer validation never runs on a receiver thread."""
    try:
        output.put((True, check_geometry(tuple(words), allow_black=allow_black)))
    except Exception as exc:
        output.put((False, f"{type(exc).__name__}: {exc}"))


from geometry_recovery import GeometryRecoveryMixin


class HostSession(GeometryRecoveryMixin):
    """Thread-safe asynchronous session. All public methods are nonblocking
    except start/close. No writes occur until a separate explicit send action.

    Stream freshness and pose age are separate: a live stream must be younger
    than 0.5 s; a candidate may spend up to 2 s in exact geometry validation.
    Those limits serve slow-motion bring-up, not per-frame stabilization.
    A frozen session must be closed and replaced after reviewing its log.
    """

    def __init__(self, calibration=None, *, log_path=None, imu_stale_s=.5,
                 candidate_max_age_s=2., status_stale_s=2.5,
                 accept_timeout_s=5., publish_timeout_s=45., output_timeout_s=12.,
                 geometry_options=None, allow_black=True, serial_factory=None,
                 ports_provider=None, geometry_checker=None, clock=time.monotonic,
                 realtime_pose_age_s=.1, max_pending_outputs=4):
        self.calibration = deepcopy(load_calibration() if calibration is None else calibration)
        self.geometry_options = dict(geometry_options or {})
        forbidden = {"matrix", "raw", "legacy_angle"} & self.geometry_options.keys()
        if forbidden:
            raise ValueError("geometry_options cannot override matrix/raw/legacy_angle")
        self.allow_black = bool(allow_black)
        for name, value in dict(imu_stale_s=imu_stale_s, candidate_max_age_s=candidate_max_age_s,
                                status_stale_s=status_stale_s, accept_timeout_s=accept_timeout_s,
                                publish_timeout_s=publish_timeout_s, output_timeout_s=output_timeout_s,
                                realtime_pose_age_s=realtime_pose_age_s).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
            setattr(self, name, float(value))
        self._serial_factory, self._ports_provider = serial_factory, ports_provider
        self._geometry_checker, self._clock = geometry_checker, clock
        self._drain_timeout_s = 2.
        self.session_id = uuid.uuid4().hex
        self.log_path = Path(log_path) if log_path else ROOT / "logs" / (self.session_id + ".jsonl")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.RLock()
        self._imu_write_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._wake = threading.Event()
        self._threads = []
        self._ports = {}
        self._port_names = {}
        self._connected = {"imu": False, "fpga": False}
        self._started = False
        self._closed = False
        self._decoder = ImuDecoder()
        self._history = PoseHistory()
        self._framer = MixedFramer()
        self._latest_euler = None
        self._status = None
        self._status_at = None
        self._pose_status = None
        self._pose_status_at = None
        self._v5_status = None
        self._v5_at = None
        self._reference = None
        self._mounting = np.eye(3)
        self._mounting_confirmed = False
        self._convention = "imu_to_world"
        self._epoch = 0
        self._request_sequence = 0
        self._identity_test_requested = False
        self._reference_sequence = 0
        self._candidate = None
        self._active = None
        self._last_request = None
        self._last_words = None
        if isinstance(max_pending_outputs, bool) or not isinstance(max_pending_outputs, int) or not 1 <= max_pending_outputs <= 16:
            raise ValueError('max_pending_outputs must be in [1,16]')
        self._max_pending_outputs = max_pending_outputs
        self._pending_outputs = []
        self._realtime = False
        self._publication_times = deque(maxlen=120)
        self._output_times = deque(maxlen=120)
        self._published_count = 0
        self._output_confirmed_count = 0
        self._output_unobserved_count = 0
        self._output_unverifiable_count = 0
        self._latencies = deque(maxlen=128)
        self._events = deque(maxlen=128)
        self._continuous = False
        self._interval_s = 2.
        self._next_due = 0.
        self._frozen = False
        self._fault_sequence = 0
        self._error = None
        self._state = "disconnected"
        self._init_geometry_recovery()
        self._log("session_created", limits=dict(imu_stale_s=self.imu_stale_s,
                  candidate_max_age_s=self.candidate_max_age_s, status_stale_s=self.status_stale_s))

    def _log(self, event, **data):
        with self._lock:
            record = _jsonable(dict(session_id=self.session_id, monotonic_s=self._clock(),
                                    event=event, **data))
            self._events.append(record)
            if self._log_file and not self._log_file.closed:
                self._log_file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def _open(self, name, role):
        available = list_serial_ports(self._ports_provider)
        if not any(p["device"] == name and p["role"] == role for p in available):
            raise ValueError(f"{name} is not a verified {role} USB adapter")
        factory = self._serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        # Set handshake lines while the handle is closed, before any opening.
        port = factory(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                       timeout=.1, write_timeout=2., xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            port.dtr = False
            port.rts = False
            port.port = name
            port.open()
            if hasattr(port, "reset_input_buffer"):
                port.reset_input_buffer()
        except Exception:
            port.close()
            raise
        return port

    def start(self, imu_port=None, fpga_port=None):
        if not imu_port and not fpga_port:
            raise ValueError("Select at least one serial port")
        with self._lock:
            if self._started or self._closed:
                raise RuntimeError("Create a new HostSession for each connection")
            if imu_port and imu_port == fpga_port:
                raise ValueError("IMU and FPGA must use distinct ports")
            try:
                for role, name in (("imu", imu_port), ("fpga", fpga_port)):
                    if name:
                        self._ports[role] = self._open(name, role)
                        self._port_names[role] = name
                        self._connected[role] = True
            except Exception:
                for port in self._ports.values():
                    port.close()
                self._ports.clear()
                self._connected = {"imu": False, "fpga": False}
                raise
            self._started = True
            self._state = "monitor"
            targets = [("imu-reader", self._read_imu)] if imu_port else []
            if fpga_port:
                targets.append(("fpga-reader", self._read_fpga))
            targets += [("session-scheduler", self._schedule), ("geometry-worker", self._geometry_loop)]
            for name, target in targets:
                thread = threading.Thread(target=target, name=name + "-" + self.session_id[:6])
                self._threads.append(thread)
                thread.start()
            self._log("connected_read_only", ports=self._port_names)
        return self

    def _fail(self, reason, *, state="frozen", invalidate=True):
        with self._lock:
            self._cancel_geometry_recovery(str(reason))
            self._fault_sequence += 1
            self._frozen = True
            self._continuous = False
            self._candidate = None
            self._error = str(reason)
            self._state = state
            if invalidate and self._active:
                self._active["invalidated"] = True
            self._log("frozen", reason=str(reason), request_id=(self._active or {}).get("request_id"))

    def _disconnect(self, role, exc):
        with self._lock:
            self._connected[role] = False
            self._fail(f"{role} disconnected: {type(exc).__name__}: {exc}", state="disconnected")

    def _read_imu(self):
        try:
            while not self._shutdown.is_set():
                port = self._ports["imu"]
                chunk = port.read(max(1, min(512, getattr(port, 'in_waiting', 0))))
                if not chunk:
                    continue
                now = self._clock()
                packets = self._decoder.feed(chunk, received_at=now)
                with self._lock:
                    for packet in packets:
                        if packet["kind"] == "quaternion":
                            sample = self._history.add(packet["quaternion"], received_at=packet["received_at"])
                            self._log("imu_sample", sample=_jsonable(sample))
                        elif packet["kind"] == "euler":
                            self._latest_euler = list(packet["euler_rad"])
        except Exception as exc:
            if not self._shutdown.is_set():
                self._disconnect("imu", exc)

    def _read_fpga(self):
        try:
            while not self._shutdown.is_set():
                port = self._ports["fpga"]
                chunk = port.read(max(1, min(1024, getattr(port, 'in_waiting', 0))))
                if not chunk:
                    continue
                now = self._clock()
                for packet in self._framer.feed(chunk):
                    if packet["packet_type"] == "status":
                        self._observe_status(packet, now)
                    elif packet["packet_type"] == "pose_status":
                        self._observe_pose_status(packet, now)
                    else:
                        with self._lock:
                            self._latencies.append((now, packet))
                            self._log("latency", received_at=now, packet=packet)
                            self._confirm_output()
        except Exception as exc:
            if not self._shutdown.is_set():
                self._disconnect("fpga", exc)

    def _observe_status(self, packet, now):
        with self._lock:
            if self._status and packet.get('build_id') != self._status.get('build_id'):
                self._invalidate_published_evidence('Board firmware identity changed')
                self._fail('Board firmware changed during session')
                return
        # C125 POS1 owns transaction state. Periodic V5 snapshots can arrive
        # after newer events and must never roll configuration counters back.
        if packet.get('build_id') in REALTIME_BUILD_IDS:
            with self._lock:
                verify_firmware(packet)
                self._v5_status, self._v5_at = packet, now
                self._log('status_health', received_at=now, status=packet)
                if self._pose_status is None:
                    self._status = dict(packet, **enhancement_status(packet))
                self._status_at = now
                if (self._active or self._pending_outputs or self._continuous) and (packet.get('error_sticky') or packet.get('camera_error')):
                    self._fail('FPGA hardware error during active request')
            return
        self._observe_control_status(packet, now)

    def _observe_pose_status(self, packet, now):
        with self._lock:
            try:
                verify_pose(packet)
            except ValueError as exc:
                if self._status and packet.get('build_id') != self._status.get('build_id'):
                    self._invalidate_published_evidence('Board firmware identity changed')
                self._fail(str(exc))
                return
            self._pose_status = packet
            self._pose_status_at = now
        self._observe_control_status(packet, now)

    def _observe_control_status(self, packet, now):
        with self._lock:
            try:
                verify_firmware(packet)
                packet = dict(packet, **enhancement_status(packet))
            except Exception as exc:
                self._log("incompatible_status", packet=packet)
                self._fail(exc)
                return
            previous = self._status
            automatic_rejection = None
            self._status, self._status_at = packet, now
            self._log("status", received_at=now, status=packet)
            if previous:
                if packet['build_id'] != previous['build_id']:
                    self._invalidate_published_evidence('Board firmware identity changed')
                    self._fail('Board firmware changed during session')
                if 'board_time_us' in packet and 'board_time_us' in previous:
                    if packet['board_time_us'] != previous['board_time_us'] and not _forward(packet['board_time_us'], previous['board_time_us']):
                        self._invalidate_published_evidence('Board microsecond counter decreased/reset')
                        self._fail('Board microsecond counter decreased/reset')
                elif 'sample_ticks' in packet and 'sample_ticks' in previous and packet['sample_ticks'] < previous['sample_ticks']:
                    self._invalidate_published_evidence('Board tick counter decreased/reset')
                    self._fail('Board tick counter decreased/reset')
                for key in ("commits", "captured", "processed", "underflow", "overflow", "dropped"):
                    if key in packet and key in previous and packet[key] < previous[key]:
                        self._invalidate_published_evidence('Board counter decreased/reset: '+key)
                        self._fail("Board counter decreased/reset: " + key)
                if packet["rejected"] != previous["rejected"]:
                    self._fail("FPGA rejection counter changed")
                if packet.get('control_failures', 0) != previous.get('control_failures', 0) and 'control_failures' in previous:
                    base = (self._active or {}).get('before') or {}
                    soft_rejection = bool(self._active and not self._frozen and
                        not self._active.get('invalidated') and
                        packet.get('reject_flags') in (2, 4, 6) and not packet.get('map_building') and
                        not packet.get('error_sticky') and not packet.get('camera_error') and
                        packet.get('commits') == base.get('commits', -2)+1 and
                        packet.get('config_version') == base.get('config_version') and
                        packet.get('rejected') == base.get('rejected') and
                        packet.get('control_failures') == previous.get('control_failures', -2)+1)
                    soft_rejection = soft_rejection and _forward(
                        packet.get('board_time_us', 0), base.get('board_time_us', 0))
                    if self._active:
                        self._active.update(phase='board_rejected', reject_flags=packet.get('reject_flags'), rejected_status=deepcopy(packet))
                    if soft_rejection and self._auto_recover_geometry and self._continuous:
                        automatic_rejection = self._active
                    else:
                        prior_fault = self._error if self._frozen else None
                        self._fail(prior_fault or 'FPGA control/geometry rejection; previous mapping retained; no retransmission')
                        if soft_rejection:
                            self._active['soft_rejection_fault'] = self._fault_sequence
                version_step = (packet["config_version"] - previous["config_version"]) & 0xFFFF
                commit_step = packet["commits"] - previous["commits"]
                if version_step not in (0, 1) or commit_step not in (0, 1):
                    self._fail("Unexpected configuration version/counter jump")
                # A physical button or another writer is not this session's commit.
                if (not self._active or "before" not in self._active) and (version_step or commit_step):
                    self._fail("Configuration changed outside this session")
            if (self._active or self._pending_outputs or self._continuous) and (packet.get("error_sticky") or packet.get("camera_error")):
                self._fail("FPGA hardware error during active request")
            active = self._active
            if active and "before" in active:
                before = active["before"]
                expected = (before["config_version"] + 1) & 0xFFFF
                if packet["commits"] not in (before["commits"], before["commits"] + 1):
                    self._fail("Unexpected commit count for active request")
                if packet["config_version"] not in (before["config_version"], expected):
                    self._fail("Unexpected version for active request")
                if packet["config_version"] == expected and packet["commits"] == before["commits"]:
                    self._fail("Version changed without this request being accepted")
                time_key = 'board_time_us' if 'board_time_us' in packet and 'board_time_us' in before else 'sample_ticks'
                fresh_event = time_key in packet and time_key in before and _forward(packet[time_key], before[time_key])
                if (not active.get("invalidated") and active.get('phase') != 'board_rejected' and fresh_event and
                        packet["rejected"] == before["rejected"] and packet["commits"] == before["commits"] + 1):
                    if "accepted_at" not in active:
                        active["accepted_at"] = now
                        active["phase"] = "await_published"
                        if not self._frozen:
                            self._state = active["phase"]
                        self._log("accepted", request_id=active["request_id"], status=packet)
                    if packet["config_version"] == expected and not packet["map_building"]:
                        if "published_at" not in active:
                            active.update(published_at=now, published_status=deepcopy(packet), phase="await_output")
                            if not self._frozen:
                                self._state = active["phase"]
                            self._log("published", request_id=active["request_id"], status=packet)
                            self._publication_times.append(now)
                            self._published_count += 1
                            if active.get('realtime'):
                                # Publication is the atomic commit boundary.
                                # LAT1 remains separate, bounded evidence.
                                self._last_words = tuple(active['words'])
                                self._pending_outputs.append(active)
                                self._active = None
                                self._last_request = deepcopy(active)
                                self._wake.set()
                        self._confirm_output()
            if automatic_rejection is not None and not self._frozen:
                if not self._hold_geometry_rejection(automatic_rejection, packet, now):
                    self._fail('FPGA control/geometry rejection; automatic recovery is not eligible')
                    automatic_rejection['soft_rejection_fault'] = self._fault_sequence
            if packet["commits"] == MAX_COMMITS:
                # The final allowed count can confirm its preceding request, but
                # cannot provide evidence for a subsequent accepted transaction.
                self._continuous = False
                self._candidate = None
                self._error = "Commit counter saturated; no further submissions allowed"

    def _confirm_output(self):
        for active in list(self._pending_outputs) + ([self._active] if self._active else []):
            self._confirm_one_output(active)

    def _invalidate_published_evidence(self, reason):
        """Close old board-epoch evidence on proven continuity loss only."""
        known = list(self._pending_outputs)
        if self._active and 'published_at' in self._active:
            known.append(self._active)
        for request in known:
            if request.get('published_evidence_invalidated'):
                continue
            request.update(invalidated=True, published_evidence_invalidated=True,
                           phase='published_output_unverifiable', reason=reason,
                           output_evidence_closed_at=self._clock())
            self._output_unverifiable_count += 1
            if request in self._pending_outputs:
                self._pending_outputs.remove(request)
            self._last_request = deepcopy(request)
            self._log('published_output_unverifiable', request=request)
        self._last_words = None

    def _confirm_one_output(self, active):
        if not active or "published_at" not in active or active.get("invalidated"):
            return
        before, version = active["before"], active["published_status"]["config_version"]
        before_us = before.get('board_time_us', (before.get('sample_ticks', 0) * 1_000_000) & 0xFFFFFFFF)
        for received_at, item in list(self._latencies):
            if received_at < active["write_started_at"] or item["config_version"] != version:
                continue
            if not self._accept_output_for_request(active, item):
                continue
            if not _forward(item["warp_start_us"], before_us):
                continue
            analysis = LatencyAnalysis()
            # LAT1 itself validates stage order/flags; POS1 supplies firmware
            # identity and the same free-running microsecond timebase.
            if 'sample_ticks' in before:
                analysis.accept(dict(before, packet_type="status"))
            checked = analysis.accept(item)
            if not checked["accepted"]:
                continue
            active.update(phase="output_confirmed", output_at=received_at, output_sample=checked,
                          source_frame_id=checked["source_frame_id"], output_confirmed_at=self._clock())
            if not active.get('realtime'):
                self._last_words = tuple(active["words"])
            self._last_request = deepcopy(active)
            if active in self._pending_outputs:
                self._pending_outputs.remove(active)
            if self._active is active:
                self._active = None
            self._output_times.append(active['output_confirmed_at'])
            self._output_confirmed_count += 1
            # A valid output of a newer known publication retires older
            # pending evidence. They may never have been used, or their LAT1
            # telemetry may have been lost. Neither is an output success.
            # Request IDs order this session's publications across version wrap.
            for older in list(self._pending_outputs):
                if older['request_id'] < active['request_id']:
                    older.update(phase='superseded_or_output_unobserved',
                                 output_evidence_closed_at=self._clock(),
                                 superseded_by_request_id=active['request_id'],
                                 superseded_by_output_version=version,
                                 reason='Known published; newer known version output observed. This version has no confirmed output.')
                    self._pending_outputs.remove(older)
                    self._output_unobserved_count += 1
                    self._log('superseded_or_output_unobserved', request=older)
            if not self._frozen and self._active is None:
                self._state = "output_confirmed"
            self._log("output_confirmed", request=active)
            self._complete_geometry_recovery(active, 'LAT1 output confirmed')
            self._wake.set()
            return

    def _accept_output_for_request(self, request, item):
        return True

    def _stream_gate(self, now):
        sample = self._history.latest()
        if not self._connected["imu"] or sample is None:
            return "IMU quaternion stream unavailable"
        if now - sample.received_at > self.imu_stale_s:
            return "IMU quaternion stream stale; retained pose is display-only"
        return None

    def _health_status(self):
        # Health readiness may advance in periodic V5 even when there has been
        # no control event. Errors are additionally checked in both snapshots.
        if self._v5_status and (self._pose_status_at is None or self._v5_at > self._pose_status_at):
            return self._v5_status
        return self._status or {}

    def _gate(self, *, require_idle=True, require_pose=True, require_stream=True):
        now = self._clock()
        if self._shutdown.is_set() or self._closed:
            return "Session closed"
        if self._frozen:
            return self._error or "Session frozen"
        if not self._connected["fpga"]:
            return "FPGA disconnected"
        reason = self._stream_gate(now) if require_stream else None
        if reason:
            return reason
        if require_pose and not self._mounting_confirmed:
            return "Installation rotation and quaternion convention are not confirmed"
        if require_pose and self._reference is None:
            return "Capture a software reference first"
        status = self._status
        if status is None or self._status_at is None or now - self._status_at > self.status_stale_s:
            return "FPGA status unavailable or stale"
        if status["commits"] == MAX_COMMITS:
            return "Commit counter saturated; submission blocked"
        health = self._health_status()
        if not all(health.get(key, False) for key in ("ddr_initialized", "camera_config_done", "hdmi_config_done")):
            return "FPGA DDR/camera/HDMI initialization incomplete"
        if status.get("error_sticky") or status.get("camera_error"):
            return "FPGA reports a hardware error"
        if self._v5_status and (self._v5_status.get('error_sticky') or self._v5_status.get('camera_error')):
            return 'FPGA periodic health status reports a hardware error'
        if status.get('build_id') in REALTIME_BUILD_IDS and self._pose_status is None:
            return 'C125 POS1 capability/status not yet received'
        if require_idle and status.get('key_busy'):
            return 'FPGA key configuration is busy'
        if require_idle and status.get("map_building"):
            return "FPGA map is busy"
        return None

    def _identity_gate(self):
        if self._identity_test_requested:
            return "Identity transport test already requested in this session; no retry"
        if self._active or self._candidate or self._continuous:
            return "Wait for pending requests and stop continuous mode before identity test"
        return self._gate(require_pose=False)

    def capture_reference(self):
        with self._lock:
            reason = self._stream_gate(self._clock())
            if reason:
                raise RuntimeError(reason)
            if self._active:
                raise RuntimeError("Wait for the active request before replacing the reference")
            self.stop()
            self._reference_sequence += 1
            self._reference = dict(reference_id=self._reference_sequence, sample=_jsonable(self._history.latest()))
            self._epoch += 1
            self._log("reference_captured", reference=self._reference)
            return deepcopy(self._reference)

    def restore_reference(self, quaternion, provenance):
        """Restore a saved software reference, never a fresh IMU observation.

        Does not zero or configure the sensor. The caller must establish that
        the saved attitude belongs to this unchanged physical assembly.
        """
        quaternion = normalize_quaternion(quaternion)
        if not isinstance(provenance, dict) or not provenance:
            raise ValueError("Saved reference requires nonempty source provenance")
        # Validate and copy before changing session state or logging anything.
        provenance = json.loads(json.dumps(_jsonable(provenance), allow_nan=False))
        with self._lock:
            reason = self._stream_gate(self._clock())
            if reason:
                raise RuntimeError(reason)
            if self._active:
                raise RuntimeError("Wait for the active request before replacing the reference")
            self.stop()
            self._reference_sequence += 1
            self._reference = dict(reference_id=self._reference_sequence,
                source="saved_endpoint", restored_at=self._clock(), provenance=provenance,
                sample=dict(id=None, received_at=None, source="saved_endpoint", quaternion=list(quaternion)),
                meaning="Saved software attitude; not a fresh sample or sensor zero")
            self._epoch += 1
            self._log("reference_restored", reference=self._reference)
            return deepcopy(self._reference)

    def set_mounting(self, matrix, *, convention="imu_to_world", confirmed=False):
        matrix = np.asarray(matrix, dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or
                not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(matrix), 1., atol=1e-6)):
            raise ValueError("Mounting must be a finite proper rotation matrix")
        if convention not in ("imu_to_world", "world_to_imu"):
            raise ValueError("Unknown quaternion convention")
        with self._lock:
            # Revocation must always stop future work, even while an already
            # written transaction is waiting for publication or output.
            if confirmed is not True:
                self.stop()
                self._mounting_confirmed = False
                self._log("mounting_confirmation_revoked")
            if self._active:
                raise RuntimeError("Wait for the active request before changing installation")
            self.stop()
            self._mounting, self._convention = matrix.copy(), convention
            self._mounting_confirmed = confirmed is True
            self._epoch += 1
            self._log("mounting_set", matrix=matrix, convention=convention, confirmed=self._mounting_confirmed)

    def _queue_request(self, *, identity_test=False, realtime=None):
        self._request_sequence += 1
        candidate = dict(request_id=self._request_sequence,
                         request_kind="identity_transport_test" if identity_test else "pose_compensation",
                         reference_id=None if identity_test or self._reference is None else self._reference["reference_id"],
                         sample=_jsonable(self._history.latest()), reference=deepcopy(self._reference),
                         mounting=self._mounting.tolist(), convention=self._convention,
                         queued_at=self._clock(), epoch=self._epoch, phase="queued",
                         realtime=(self._realtime and self._continuous) if realtime is None else bool(realtime))
        if self._candidate:
            self._log("candidate_replaced", request_id=self._candidate["request_id"], by=candidate["request_id"])
        self._candidate = candidate
        self._log("candidate_queued", request=candidate)
        self._wake.set()
        return candidate["request_id"]

    def send_identity_test(self):
        """Explicit one-per-session H=I diagnostic; never asserts physical mounting.

        The current quaternion is logged as an observation and remains subject
        to the same freshness/age checks, but it does not determine this matrix.
        All transport, geometry and confirmation checks are shared with normal
        requests. A queued attempt consumes the allowance even if never written.
        """
        with self._lock:
            reason = self._identity_gate()
            if reason:
                raise RuntimeError(reason)
            self._identity_test_requested = True
            return self._queue_request(identity_test=True, realtime=False)

    def send_once(self):
        with self._lock:
            reason = self._gate(require_idle=self._active is None)
            if reason:
                raise RuntimeError(reason)
            return self._queue_request(realtime=False)

    def start_continuous(self, interval_s=2.):
        if not math.isfinite(interval_s) or interval_s < .5:
            raise ValueError("Low-rate mode requires an interval of at least 0.5 seconds")
        with self._lock:
            reason = self._gate(require_idle=self._active is None)
            if reason:
                raise RuntimeError(reason)
            self._interval_s = float(interval_s)
            self._realtime = False
            self._continuous = True
            self._next_due = self._clock()
            self._log("continuous_started", interval_s=interval_s)
            self._wake.set()

    def start_realtime(self, target_hz=30., *, auto_recover_geometry=False):
        """Offer the latest pose at up to 30 Hz; never promise board throughput.

        Bootstrap uses a complete exactly checked configuration. Only after
        its publication may subsequent H-only poses use board admission.
        """
        if not math.isfinite(target_hz) or not 0 < target_hz <= 30:
            raise ValueError('Target pose frequency must be in (0,30] Hz')
        with self._lock:
            reason = self._gate()
            if reason:
                raise RuntimeError(reason)
            if self._active or self._candidate or self._pending_outputs:
                raise RuntimeError('Wait for previous requests before changing stream mode')
            status = self._pose_status or {}
            if status.get('build_id') not in REALTIME_BUILD_IDS or not status.get('hardware_admission_capable'):
                raise RuntimeError('C125 full-map guard, domain-black and compact-pose capabilities are required')
            if not self.allow_black:
                raise RuntimeError('Realtime board admission explicitly requires allow_black=True; use exact low-rate mode for zero-invalid policy')
            self._realtime = self._continuous = True
            self._auto_recover_geometry = bool(auto_recover_geometry)
            self._geometry_recovery = None
            self._interval_s = 1. / target_hz
            self._next_due = self._clock()
            self._log('realtime_started', target_hz=target_hz,
                      auto_recover_geometry=self._auto_recover_geometry,
                      pose_age_limit_s=self.realtime_pose_age_s,
                      pending_output_limit=self._max_pending_outputs,
                      admission='Exact CPU bootstrap, then full-map FPGA guard; invalid domain becomes black')
            self._wake.set()

    def acknowledge_geometry_rejection(self):
        """Manually release a proven ROI/all-invalid rejection; never send.

        Unknown outcomes, UART/protocol, setup/DDR faults and subsequent faults
        cannot use this action. A separate explicit send/start is required.
        """
        with self._lock:
            active = self._active or {}
            failed = active.get('rejected_status') or {}
            if (not self._frozen or active.get('soft_rejection_fault') != self._fault_sequence or
                    active.get('phase') != 'board_rejected' or 'write_done_at' not in active):
                raise RuntimeError('Only a completed, proven ROI/all-invalid rejection can be acknowledged')
            if self._shutdown.is_set() or not all(self._connected.values()):
                raise RuntimeError('Both links must remain connected')
            reason = self._stream_gate(self._clock())
            if reason:
                raise RuntimeError(reason)
            status = self._status or {}
            if (self._status_at is None or self._clock()-self._status_at > self.status_stale_s or
                    status.get('error_sticky') or status.get('camera_error') or status.get('map_building') or
                    (self._v5_status and (self._v5_status.get('error_sticky') or self._v5_status.get('camera_error'))) or
                    any(status.get(key) != failed.get(key) for key in
                        ('build_id', 'config_version', 'commits', 'rejected', 'control_failures'))):
                raise RuntimeError('Rejection evidence or healthy board state changed; reconnect and review log')
            self.stop()
            active['rejection_acknowledged_at'] = self._clock()
            self._last_request = deepcopy(active)
            self._active = None
            self._frozen = False
            self._error = None
            self._state = 'monitor'
            self._log('geometry_rejection_acknowledged', request=active,
                      meaning='No retry sent; previous mapping retained. Separate send/start required.')
            return '几何拒绝已确认；仍停止提交。退回可用姿态后，请另行选择单次或连续。'

    def _hardware_admission_ready(self, words):
        status = self._pose_status or {}
        return bool(self._realtime and self.allow_black and self._last_words and
                    tuple(words[:18]) == self._last_words[:18] and
                    status.get('hardware_admission_capable') and status.get('saved_valid') and
                    not status.get('raw_active') and not status.get('key_busy'))

    def _pose_age_limit(self, request):
        return self.realtime_pose_age_s if request.get('admission') == 'fpga_full_map_guard' else self.candidate_max_age_s

    def stop(self):
        """Stop future work; finish an already begun atomic transaction."""
        with self._lock:
            self._cancel_geometry_recovery('Stopped or context changed')
            self._continuous = False
            self._candidate = None
            self._epoch += 1  # Invalidates geometry that has not begun writing.
            self._log("submissions_stopped")

    stop_continuous = stop

    def _drain(self, port):
        """Bound physical transmit completion; Win32 Serial.flush is unbounded.

        Once write has begun, closing still permits this bounded drain so a
        normal Quit does not cut an already queued complete transaction short.
        """
        deadline = time.monotonic() + self._drain_timeout_s
        while port.out_waiting:
            if time.monotonic() >= deadline:
                raise TimeoutError("Serial output did not drain; transaction outcome unknown")
            time.sleep(.001 if self._realtime else .01)

    def set_imu_100hz(self):
        with self._imu_write_lock:
            with self._lock:
                if not self._connected["imu"] or self._shutdown.is_set():
                    raise RuntimeError("IMU disconnected")
                port = self._ports["imu"]
            try:
                if port.write(IMU_100HZ_COMMAND) != len(IMU_100HZ_COMMAND):
                    raise IOError("Incomplete IMU rate command")
                self._drain(port)
            except Exception as exc:
                self._disconnect("imu", exc)
                raise
            self._log("imu_100hz_requested", command=IMU_100HZ_COMMAND.hex(),
                      confirmation="No ACK; observe measured quaternion rate")

    def _check_geometry_cancellable(self, words):
        if self._geometry_checker is not None:
            # Test-only dependency; injected checkers must return promptly.
            return self._geometry_checker(tuple(words))
        context = mp.get_context("spawn")
        output = context.Queue(maxsize=1)
        process = context.Process(target=_geometry_process, args=(tuple(words), self.allow_black, output))
        process.start()
        try:
            while not self._shutdown.is_set():
                try:
                    ok, result = output.get(timeout=.05)
                    if not ok:
                        raise ValueError(result)
                    return result
                except queue.Empty:
                    if not process.is_alive():
                        raise RuntimeError("Geometry process exited without a result")
            raise InterruptedError("Session closing")
        finally:
            if process.is_alive():
                process.terminate()
            process.join(timeout=1.)
            if process.is_alive():
                process.kill()
                process.join(timeout=1.)
            output.close()
            output.join_thread()

    def _prepare_matrix_words(self, request):
        h = np.eye(3) if request.get('request_kind') == 'identity_transport_test' else compensation_matrix(
            request['sample']['quaternion'], request['reference']['sample']['quaternion'],
            request['mounting'], convention=request['convention'])
        words = validate_words(configuration(self.calibration, matrix=h, legacy_angle=0,
                                             raw=False, **self.geometry_options))
        return h, words

    def _candidate_age(self, request):
        return self._clock() - request['sample']['received_at']

    def _check_request_geometry(self, words, request):
        return self._check_geometry_cancellable(words)

    def _service_control(self):
        return False

    def _request_packets(self, request, compact):
        """Single-writer transport hook; sources retain distinct wire intent."""
        return (pose_packets(request['words'], self._status['config_version'])
                if compact else configuration_packets(request['words']))

    def _request_gate(self, request):
        return self._gate(require_pose=request.get('request_kind') != 'identity_transport_test')

    def _scheduled_stream_gate(self, now):
        return self._stream_gate(now) if getattr(self, '_pose_mode', 'full') != 'normal' else None

    def _geometry_loop(self):
        while not self._shutdown.is_set():
            with self._lock:
                request = None
                if (self._candidate and self._active is None and not self._frozen and
                        len(self._pending_outputs) < self._max_pending_outputs):
                    request, self._candidate = self._candidate, None
                    if self._continuous and request.get('request_source', 'imu') == 'imu':
                        request["sample"] = _jsonable(self._history.latest())
                        request["selected_at"] = self._clock()
                        self._log("continuous_pose_refreshed", request_id=request["request_id"],
                                  sample=request["sample"])
                    self._active = request
                    request.update(phase="geometry", geometry_started_at=self._clock())
                    self._state = "geometry"
            if request is None:
                self._wake.wait(.05)
                self._wake.clear()
                continue
            try:
                identity_test = request.get("request_kind") == "identity_transport_test"
                h, words = self._prepare_matrix_words(request)
                with self._lock:
                    request.update(H=_jsonable(h), words=list(words))
                    if not identity_test and not request.get('force_full_configuration') and tuple(words) == self._last_words:
                        self._finish_unsent(request, "quantized_unchanged")
                        continue
                with self._lock:
                    hardware_admission = request.get('realtime') and self._hardware_admission_ready(words)
                    request['admission'] = 'fpga_full_map_guard' if hardware_admission else 'cpu_exact'
                if hardware_admission:
                    # Only parameters are transmitted. C125 generates and
                    # validates its own full map before atomic publication.
                    checked = dict(fixed_point_checked=False, admission='fpga_full_map_guard',
                                   domain_policy='invalid pixel becomes black',
                                   publication_requires='setup, ROI, at least one valid sample, DDR checks')
                else:
                    checked = self._check_request_geometry(words, request)
                with self._lock:
                    request.update(geometry=checked, geometry_done_at=self._clock())
                    reason = self._request_gate(request)
                    if request["epoch"] != self._epoch:
                        reason = "Reference/installation changed or stop requested"
                    if self._candidate_age(request) > self._pose_age_limit(request):
                        reason = "Candidate pose expired during geometry validation"
                    if reason:
                        self._finish_unsent(request, "candidate_discarded", reason)
                    else:
                        request["phase"] = "ready"
                        self._state = "ready"
                        self._wake.set()
            except Exception as exc:
                with self._lock:
                    self._finish_unsent(request, "geometry_failed", str(exc))

    def _finish_unsent(self, request, phase, reason=None):
        replace_expired = self._replace_expired_unsent_pose(request, phase, reason)
        request.update(phase=phase, finished_at=self._clock(), reason=reason)
        self._last_request = deepcopy(request)
        if self._active is request:
            self._active = None
        if not self._frozen:
            self._state = phase
            self._error = None if replace_expired else reason
        if reason and not replace_expired:
            self._cancel_geometry_recovery(reason)
            self._continuous = False
            self._candidate = None
        self._log(phase, request=request)
        if phase == 'quantized_unchanged':
            self._complete_geometry_recovery(request, 'Returned to the retained valid mapping')

    def _schedule(self):
        try:
            while not self._shutdown.is_set():
                if self._service_control():
                    self._wake.wait(.01)
                    self._wake.clear()
                    continue
                payload = None
                with self._lock:
                    now = self._clock()
                    reason = self._scheduled_stream_gate(now)
                    if reason and (self._continuous or self._candidate or
                                   (self._active and self._active["phase"] in ("geometry", "ready"))):
                        self._cancel_geometry_recovery(reason)
                        self._continuous = False
                        self._candidate = None
                        self._epoch += 1
                        self._error = reason
                    recovering = self._service_geometry_recovery(now)
                    if self._continuous and not recovering and now >= self._next_due:
                        # Busy publication and bounded output evidence cause
                        # backpressure, not a loss of the latest candidate.
                        reason = self._gate(require_idle=False)
                        if reason:
                            self._cancel_geometry_recovery(reason)
                            self._continuous = False
                            self._candidate = None
                            self._error = reason
                            self._log("continuous_stopped", reason=reason)
                        else:
                            self._queue_request()
                            self._next_due = max(now, self._next_due + self._interval_s)
                    active = self._active
                    if active and active["phase"] == "ready":
                        reason = self._request_gate(active)
                        if active["epoch"] != self._epoch:
                            reason = "Stopped or reference/installation changed"
                        if self._candidate_age(active) > self._pose_age_limit(active):
                            reason = "Candidate pose expired before write"
                        if reason:
                            self._finish_unsent(active, "candidate_discarded", reason)
                        else:
                            compact = active.get('admission') == 'fpga_full_map_guard'
                            if compact and not self._hardware_admission_ready(active['words']):
                                self._finish_unsent(active, 'candidate_discarded', 'Board admission capability/state changed')
                                continue
                            payload = self._request_packets(active, compact)
                            if len(payload) != (88 if compact else 232):
                                raise RuntimeError('Invalid atomic transaction length')
                            visual = active.get('request_source') == 'visual'
                            active.update(transaction=('visual_only' if visual else 'pose_only') if compact else 'full',
                                          candidate_age_at_write_s=self._candidate_age(active))
                            if not visual:
                                active['pose_age_at_write_s'] = self._candidate_age(active)
                            active.update(before=deepcopy(self._status), write_started_at=now, phase="writing")
                            self._state = "writing"
                            self._log("write_started", request=active, byte_count=len(payload))
                    elif active and "write_done_at" in active and active.get('phase') != 'board_rejected':
                        if "published_at" in active:
                            if now - active["published_at"] > self.output_timeout_s and not active.get("output_timeout"):
                                active["output_timeout"] = True
                                self._fail("Mapping published; LAT1 timed out. Read-only output wait; do not resubmit.",
                                           state="published_output_timeout", invalidate=False)
                        elif "accepted_at" in active:
                            if now - active["accepted_at"] > self.publish_timeout_s and not active.get("publish_timeout"):
                                active["publish_timeout"] = True
                                self._fail("Publication timed out; outcome unknown. No retransmission.", invalidate=False)
                        elif now - active["write_done_at"] > self.accept_timeout_s and not active.get("accept_timeout"):
                            active["accept_timeout"] = True
                            self._fail("Acceptance timed out; outcome unknown. No retransmission.", invalidate=False)
                    for pending in self._pending_outputs:
                        if now-pending['published_at'] > self.output_timeout_s and not pending.get('output_timeout'):
                            pending['output_timeout'] = True
                            self._fail('Published version has no LAT1 evidence; bounded stream stopped, no retransmission',
                                       state='published_output_timeout', invalidate=False)
                if payload is not None:
                    # Only this scheduler writes the FPGA. There is exactly one
                    # write call for BEGIN/ordered words/COMMIT, with finite timeout.
                    port = self._ports["fpga"]
                    if port.write(payload) != len(payload):
                        raise IOError("Incomplete FPGA transaction; outcome unknown")
                    self._drain(port)
                    with self._lock:
                        active["write_done_at"] = self._clock()
                        if active["phase"] == "writing":
                            active["phase"] = "await_accepted"
                        if self._active is active and not self._frozen:
                            self._state = active["phase"]
                        if self._last_request and self._last_request["request_id"] == active["request_id"]:
                            self._last_request = deepcopy(active)
                        self._log("write_complete", request_id=active["request_id"],
                                  write_done_at=active["write_done_at"])
                self._wake.wait(.002 if self._realtime else .02)
                self._wake.clear()
        except Exception as exc:
            if not self._shutdown.is_set():
                with self._lock:
                    if self._active:
                        self._active["invalidated"] = True
                    self._fail(f"FPGA transport failure: {exc}")

    def snapshot(self):
        with self._lock:
            now = self._clock()
            sample = self._history.latest()
            age = now - sample.received_at if sample else None
            health = self._health_status()
            def observed_rate(values):
                recent = [value for value in values if now-value <= 5.]
                return (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else None
            imu = dict(connected=self._connected["imu"], port=self._port_names.get("imu"),
                       latest=_jsonable(sample), quaternion=list(sample.quaternion) if sample else None,
                       euler_rad=self._latest_euler, rate_hz=self._history.rate_hz(), age_s=age,
                       stale=age is None or age > self.imu_stale_s,
                       decoder_counters={key: getattr(self._decoder, key, 0) for key in
                           ("accepted_packets", "checksum_errors", "length_errors", "unknown_functions",
                            "invalid_values", "discarded_bytes", "buffered_bytes")})
            return deepcopy(_jsonable(dict(session_id=self.session_id, running=self._started and not self._closed,
                connected=dict(self._connected), state=self._state, error=self._error,
                continuous=self._continuous, frozen=self._frozen, log_path=str(self.log_path),
                geometry_recovery=self._geometry_recovery,
                auto_recover_geometry=self._auto_recover_geometry,
                static_configuration_owned=self._last_words is not None,
                realtime=self._realtime, target_pose_hz=1./self._interval_s if self._continuous else None,
                pending_outputs=self._pending_outputs, max_pending_outputs=self._max_pending_outputs,
                publication_observed_hz=observed_rate(self._publication_times),
                output_confirmation_observed_hz=observed_rate(self._output_times),
                version_evidence=dict(published=self._published_count,
                    output_confirmed=self._output_confirmed_count,
                    superseded_or_output_unobserved=self._output_unobserved_count,
                    published_output_unverifiable=self._output_unverifiable_count,
                    pending=len(self._pending_outputs)+(1 if self._active and 'published_at' in self._active and not self._active.get('published_evidence_invalidated') else 0)),
                latest_pose_status=self._pose_status, latest_v5_status=self._v5_status,
                latest_v5_age_s=now-self._v5_at if self._v5_at is not None else None,
                latest_v5_received_at=self._v5_at,
                latest_latency_packet=self._latencies[-1][1] if self._latencies else None,
                latest_latency_received_at=self._latencies[-1][0] if self._latencies else None,
                latest_latency_age_s=now-self._latencies[-1][0] if self._latencies else None,
                health_status=health,
                imu=imu, fpga=dict(connected=self._connected["fpga"], port=self._port_names.get("fpga"),
                status_age_s=now-self._status_at if self._status_at is not None else None),
                last_status=self._status, reference=self._reference, mounting=self._mounting,
                mounting_confirmed=self._mounting_confirmed, convention=self._convention,
                request=self._active or self._last_request, candidate=self._candidate,
                can_send=self._gate(require_idle=self._active is None) is None,
                send_blocked_reason=self._gate(require_idle=self._active is None),
                can_send_identity_test=self._identity_gate() is None,
                identity_test_blocked_reason=self._identity_gate(),
                identity_test_requested=self._identity_test_requested,
                framing_counters=dict(self._framer.counters), events=list(self._events)[-30:])))

    def close(self):
        """Cancel geometry, finish bounded serial writes, then close all owners."""
        with self._lock:
            if self._closed:
                return
            self.stop()
            self._shutdown.set()
            self._wake.set()
        # Reader timeouts are .1 s. A started FPGA transaction gets its full
        # write_timeout before cancellation; this avoids truncation on Quit.
        deadline = time.monotonic() + 4.5
        for thread in self._threads:
            thread.join(max(0., deadline - time.monotonic()))
        for port in self._ports.values():
            if any(t.is_alive() for t in self._threads):
                for method in ("cancel_read", "cancel_write"):
                    try:
                        if hasattr(port, method):
                            getattr(port, method)()
                    except Exception:
                        pass
            try:
                port.close()
            except Exception:
                pass
        for thread in self._threads:
            if thread.is_alive():
                thread.join(1.)
        with self._lock:
            live = [t.name for t in self._threads if t.is_alive()]
            self._connected = {"imu": False, "fpga": False}
            self._closed = True
            self._state = "closed" if not live else "close_failed"
            self._log("session_closed", remaining_threads=live)
            self._log_file.close()
        if live:
            raise RuntimeError("Workers did not stop: " + ", ".join(live))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
