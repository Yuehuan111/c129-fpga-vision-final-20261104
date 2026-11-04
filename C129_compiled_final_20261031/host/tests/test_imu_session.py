"""Transport/state-machine regression tests. No physical serial port is opened."""
import json
from pathlib import Path
import queue
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from imu_session import HostSession, list_serial_ports, IMU_100HZ_COMMAND
from camera_config import crc16
from complete_geometry import BUILD_ID
from read_board_status import FIELDS_V5
from read_latency import LATENCY_FIELDS, MAGIC
from run_link_test import run_link_test


def imu_frame(q=(1., 0., 0., 0.)):
    data = b"\x7e\x23\x15\x16" + struct.pack("<4f", *q)
    return data + bytes([sum(data) & 255])


def fpga_frame(protocol, words):
    payload = struct.pack("<" + "I" * len(words), *words)
    data = b"\x55\xaa" + bytes([protocol, len(payload)]) + payload
    return data + crc16(data).to_bytes(2, "big")


def status_frame(ticks=100, version=3, commits=7, rejected=0, flags=0x1070, **extras):
    data = dict.fromkeys(FIELDS_V5, 0)
    data.update(build_id=BUILD_ID, version_flags=version | (flags << 16),
                sample_ticks=ticks, commits=commits, rejected=rejected, **extras)
    return fpga_frame(5, [data[k] for k in FIELDS_V5])


def latency_frame(version=4, frame_id=31, base=100_000_100):
    data = dict.fromkeys(LATENCY_FIELDS, 0)
    data.update(magic=MAGIC, source_frame_id=frame_id, version_flags=version | (1 << 16), unit_ns=1000,
                input_first_us=base, capture_done_us=base+100, warp_start_us=base+200,
                warp_done_us=base+300, output_first_us=base+400, output_last_us=base+500)
    return fpga_frame(6, [data[k] for k in LATENCY_FIELDS])


class FakePort:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.port = kwargs.get("port")
        self.dtr = self.rts = True
        self.is_open = False
        self.incoming = queue.Queue()
        self.writes = []
        self.partial_write = False
        self.out_waiting = 0
        self.open_lines = None

    def open(self):
        self.is_open = True
        self.open_lines = self.dtr, self.rts

    def close(self):
        self.is_open = False
        self.incoming.put(b"")

    def reset_input_buffer(self):
        pass

    def read(self, count):
        try:
            value = self.incoming.get(timeout=.02)
        except queue.Empty:
            return b""
        if isinstance(value, Exception):
            raise value
        return value

    def write(self, data):
        if not self.is_open:
            raise OSError("closed")
        self.writes.append(bytes(data))
        return len(data) - 1 if self.partial_write else len(data)

    def flush(self):
        pass


def adapters():
    return [SimpleNamespace(device="IMU", description="CH340", vid=0x1A86, pid=0x7523),
            SimpleNamespace(device="FPGA", description="CP210x", vid=0x10C4, pid=0xEA60),
            SimpleNamespace(device="FTDI", description="Downloader", vid=0x0403, pid=0x6010)]


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.sessions = []
        self.feeders = []

    def tearDown(self):
        for stop, thread in self.feeders:
            stop.set()
            thread.join(1)
        for session in self.sessions:
            session.close()
            self.assertFalse(any(t.is_alive() for t in session._threads))
        self.temp.cleanup()

    def make(self, *, start=True, imu=True, fpga=True, real_geometry=False, **kwargs):
        if not real_geometry:
            kwargs.setdefault("geometry_checker", lambda words: {"fixed_point_checked": True})
        session = HostSession(log_path=Path(self.temp.name) / f"{len(self.sessions)}.jsonl",
                              serial_factory=FakePort, ports_provider=adapters, **kwargs)
        self.sessions.append(session)
        if start:
            session.start("IMU" if imu else None, "FPGA" if fpga else None)
        return session

    def wait(self, predicate, timeout=2.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            value = predicate()
            if value:
                return value
            time.sleep(.01)
        self.fail("Condition did not become true before timeout")

    def feed_imu(self, session, q=(1., 0., 0., 0.)):
        previous = (session.snapshot()["imu"]["latest"] or {}).get("id", 0)
        session._ports["imu"].incoming.put(imu_frame(q))
        self.wait(lambda: (session.snapshot()["imu"]["latest"] or {}).get("id", 0) > previous)

    def streaming(self, session):
        stop = threading.Event()
        def feed():
            while not stop.wait(.01):
                session._ports["imu"].incoming.put(imu_frame())
        thread = threading.Thread(target=feed)
        thread.start()
        self.feeders.append((stop, thread))
        return stop

    def prepare(self, session, **status):
        self.feed_imu(session)
        session._ports["fpga"].incoming.put(status_frame(**status))
        self.wait(lambda: session.snapshot()["last_status"])
        session.capture_reference()
        session.set_mounting(np.eye(3), confirmed=True)

    def written(self, session):
        return self.wait(lambda: session._ports["fpga"].writes)

    def confirm(self, session, version=4, ticks=101, commits=8):
        session._ports["fpga"].incoming.put(latency_frame(version=version))
        session._ports["fpga"].incoming.put(status_frame(ticks=ticks, version=version, commits=commits))
        return self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "output_confirmed")

    def test_monitor_is_read_only_and_handshakes_low_before_open(self):
        session = self.make()
        self.feed_imu(session)
        session._ports["fpga"].incoming.put(status_frame())
        self.wait(lambda: session.snapshot()["last_status"])
        for port in session._ports.values():
            self.assertEqual(port.writes, [])
            self.assertEqual(port.open_lines, (False, False))
            self.assertEqual(port.options["baudrate"], 115200)
            self.assertEqual(port.options["write_timeout"], 2.)
        self.assertGreater(session.snapshot()["imu"]["decoder_counters"]["accepted_packets"], 0)
        with self.assertRaisesRegex(RuntimeError, "confirmed"):
            session.send_once()

    def test_single_device_and_unknown_adapter(self):
        session = self.make(fpga=False)
        self.feed_imu(session)
        session.capture_reference()
        with self.assertRaisesRegex(RuntimeError, "FPGA disconnected"):
            session.send_once()
        self.assertEqual(list_serial_ports(adapters)[2]["role"], None)
        other = self.make(start=False)
        with self.assertRaisesRegex(ValueError, "verified"):
            other.start(fpga_port="FTDI")

    def test_restore_saved_reference_cancels_pending_and_records_honest_source(self):
        session = self.make()
        self.prepare(session)
        provenance = dict(directory="saved/x_start", summary_sha256="example")
        with session._lock:
            session.send_once()
            restored = session.restore_reference([2., 0., 0., 0.], provenance)
            self.assertIsNone(session._candidate)
        self.assertEqual(restored["sample"]["quaternion"], [1., 0., 0., 0.])
        self.assertIsNone(restored["sample"]["id"])
        self.assertIsNone(restored["sample"]["received_at"])
        self.assertEqual(restored["source"], "saved_endpoint")
        provenance["directory"] = "changed"
        self.assertEqual(session.snapshot()["reference"]["provenance"]["directory"], "saved/x_start")
        session.send_once()
        self.written(session)
        with self.assertRaisesRegex(RuntimeError, "active request"):
            session.restore_reference([1, 0, 0, 0], dict(directory="another"))
        self.confirm(session)
        self.assertEqual(session.snapshot()["request"]["reference"], restored)
        self.assertEqual(session._ports["imu"].writes, [])

    def test_restore_reference_requires_valid_finite_inputs_and_fresh_stream(self):
        session = self.make(imu_stale_s=.03)
        for q, provenance in (([float("nan"), 0, 0, 0], dict(file="x")),
                              ([0, 0, 0, 0], dict(file="x")),
                              ([1, 0, 0, 0], {}),
                              ([1, 0, 0, 0], dict(age=float("inf")))):
            with self.assertRaises(ValueError):
                session.restore_reference(q, provenance)
        self.assertIsNone(session.snapshot()["reference"])
        self.feed_imu(session)
        restored = session.restore_reference([1, 0, 0, 0], dict(file="x"))
        self.assertFalse(session.snapshot()["mounting_confirmed"])
        time.sleep(.04)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            session.restore_reference([1, 0, 0, 0], dict(file="y"))
        self.assertEqual(session.snapshot()["reference"], restored)

    def test_complete_transaction_early_latency_and_json_log(self):
        session = self.make()
        self.prepare(session)
        request_id = session.send_once()
        payload = self.written(session)[0]
        self.assertEqual(len(payload), 232)
        self.assertEqual([payload[n+1] for n in range(0, 232, 8)],
                         [0xFE] + list(range(18)) + list(range(0x20, 0x29)) + [0xFF])
        for n in range(0, 232, 8):
            self.assertEqual(crc16(payload[n:n+6]), int.from_bytes(payload[n+6:n+8], "big"))
        self.confirm(session)
        result = session.snapshot()["request"]
        self.assertEqual(result["request_id"], request_id)
        self.assertEqual(result["source_frame_id"], 31)
        self.assertTrue(all(k in result for k in ("accepted_at", "published_at", "output_at", "H", "words")))
        records = [json.loads(line) for line in session.log_path.read_text(encoding="utf-8").splitlines()]
        self.assertTrue(all(r["session_id"] == session.session_id for r in records))
        self.assertIn("output_confirmed", [r["event"] for r in records])

    def test_identity_without_mount_or_reference_uses_exact_wire_identity(self):
        session = self.make()
        self.feed_imu(session, q=(.70710678, .70710678, 0., 0.))
        session._ports["fpga"].incoming.put(status_frame())
        self.wait(lambda: session.snapshot()["can_send_identity_test"])
        for action in (session.send_once, session.start_continuous):
            with self.assertRaisesRegex(RuntimeError, "confirmed"):
                action()
        request_id = session.send_identity_test()
        payload = self.written(session)[0]
        self.assertEqual(len(payload), 232)
        identity_words = [65536, 0, 0, 0, 65536, 0, 0, 0, 65536]
        wire_h = [int.from_bytes(payload[n+2:n+6], "little", signed=True)
                  for n in range(19*8, 28*8, 8)]
        self.assertEqual(wire_h, identity_words)
        self.confirm(session)
        snapshot = session.snapshot()
        request = snapshot["request"]
        self.assertEqual(request["request_id"], request_id)
        self.assertEqual(request["request_kind"], "identity_transport_test")
        self.assertEqual(request["H"], np.eye(3).tolist())
        self.assertEqual(request["words"][18:], identity_words)
        self.assertEqual(request["phase"], "output_confirmed")
        self.assertIsNotNone(request["sample"])
        self.assertIsNone(request["reference"])
        self.assertIsNone(snapshot["reference"])
        self.assertFalse(snapshot["mounting_confirmed"])
        self.assertFalse(snapshot["can_send"])
        self.assertEqual(session._ports["imu"].writes, [])
        with self.assertRaisesRegex(RuntimeError, "already requested"):
            session.send_identity_test()
        self.assertEqual(len(session._ports["fpga"].writes), 1)

    def test_identity_cannot_replace_pending_candidate_or_active_request(self):
        session = self.make()
        self.prepare(session)
        with session._lock:
            first = session.send_once()
            with self.assertRaisesRegex(RuntimeError, "pending"):
                session.send_identity_test()
            self.assertEqual(session._candidate["request_id"], first)
            self.assertFalse(session._identity_test_requested)
        self.written(session)
        with self.assertRaisesRegex(RuntimeError, "pending"):
            session.send_identity_test()
        self.assertEqual(session.snapshot()["request"]["request_id"], first)
        self.assertEqual(len(session._ports["fpga"].writes), 1)

    def test_identity_keeps_hardware_and_freshness_gates(self):
        for flags, message in ((0x1072, "busy"), (0x1030, "initialization"), (0x1074, "hardware error")):
            with self.subTest(flags=flags):
                session = self.make()
                self.feed_imu(session)
                session._ports["fpga"].incoming.put(status_frame(flags=flags))
                self.wait(lambda: session.snapshot()["last_status"])
                with self.assertRaisesRegex(RuntimeError, message):
                    session.send_identity_test()
                self.assertEqual(session._ports["fpga"].writes, [])
        for limits, message in ((dict(imu_stale_s=.03), "stream stale"),
                                (dict(status_stale_s=.03), "status unavailable or stale")):
            session = self.make(**limits)
            self.feed_imu(session)
            session._ports["fpga"].incoming.put(status_frame())
            self.wait(lambda: session.snapshot()["last_status"])
            time.sleep(.04)
            with self.assertRaisesRegex(RuntimeError, message):
                session.send_identity_test()
            self.assertEqual(session._ports["fpga"].writes, [])

    def test_identity_expired_geometry_is_unsent_and_consumes_single_attempt(self):
        entered, release = threading.Event(), threading.Event()
        def checker(words):
            entered.set()
            release.wait(1)
            return {"fixed_point_checked": True}
        session = self.make(candidate_max_age_s=.05, geometry_checker=checker)
        self.feed_imu(session)
        self.streaming(session)
        session._ports["fpga"].incoming.put(status_frame())
        self.wait(lambda: session.snapshot()["can_send_identity_test"])
        session.send_identity_test()
        self.assertTrue(entered.wait(1))
        time.sleep(.08)
        release.set()
        self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "candidate_discarded")
        self.assertIn("expired", session.snapshot()["request"]["reason"])
        with self.assertRaisesRegex(RuntimeError, "already requested"):
            session.send_identity_test()
        self.assertEqual(session._ports["fpga"].writes, [])

    def test_link_cli_waits_for_late_output_and_saves_reports_then_closes(self):
        output = Path(self.temp.name) / "link.json"
        def factory(**kwargs):
            session = HostSession(**kwargs, serial_factory=FakePort, ports_provider=adapters,
                                  geometry_checker=lambda words: {"fixed_point_checked": True},
                                  output_timeout_s=.03)
            self.sessions.append(session)
            stop = threading.Event()
            def feed():
                initial, published, latency_due = False, False, None
                while not stop.wait(.005):
                    if not all(session._connected.values()):
                        continue
                    session._ports["imu"].incoming.put(imu_frame())
                    if not initial:
                        session._ports["fpga"].incoming.put(status_frame())
                        initial = True
                    if session._ports["fpga"].writes and not published:
                        session._ports["fpga"].incoming.put(status_frame(ticks=101, version=4, commits=8))
                        published, latency_due = True, time.monotonic() + .10
                    if latency_due and time.monotonic() >= latency_due:
                        session._ports["fpga"].incoming.put(latency_frame())
                        latency_due = None
            thread = threading.Thread(target=feed)
            thread.start()
            self.feeders.append((stop, thread))
            return session
        result = run_link_test("IMU", "FPGA", output, session_factory=factory,
                               ready_timeout_s=.5, total_timeout_s=1.)
        self.assertEqual(result["result"], "output_confirmed")
        self.assertTrue(result["request"]["output_timeout"])
        self.assertFalse(result["before"]["mounting_confirmed"])
        self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["result"], "output_confirmed")
        records = [json.loads(line) for line in output.with_suffix(".jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertTrue({"identity_test_before", "identity_test_after", "identity_test_result", "session_closed"}
                        <= {r["event"] for r in records})
        session = self.sessions[-1]
        self.assertEqual(len(session._ports["fpga"].writes), 1)
        self.assertEqual(session._ports["imu"].writes, [])
        self.assertTrue(all(not p.is_open for p in session._ports.values()))

    def test_link_cli_readiness_deadline_never_writes(self):
        output = Path(self.temp.name) / "not_ready.json"
        def factory(**kwargs):
            session = HostSession(**kwargs, serial_factory=FakePort, ports_provider=adapters)
            self.sessions.append(session)
            return session
        result = run_link_test("IMU", "FPGA", output, session_factory=factory,
                               ready_timeout_s=.05, total_timeout_s=.1)
        self.assertEqual(result["result"], "not_sent")
        self.assertIsNone(result["request"])
        self.assertIn("unavailable", result["reason"])
        self.assertTrue(output.exists())
        self.assertTrue(output.with_suffix(".jsonl").exists())
        self.assertTrue(all(not p.is_open and not p.writes for p in self.sessions[-1]._ports.values()))

    def test_quantized_identical_skipped_after_confirmed_output(self):
        session = self.make()
        self.prepare(session)
        session.send_once()
        self.written(session)
        self.confirm(session)
        self.feed_imu(session)
        session.send_once()
        self.wait(lambda: session.snapshot()["state"] == "quantized_unchanged")
        self.assertEqual(len(session._ports["fpga"].writes), 1)

    def test_version_wrap_is_valid_and_commit_saturation_blocks(self):
        session = self.make()
        self.prepare(session, version=65535)
        session.send_once()
        self.written(session)
        self.confirm(session, version=0)
        self.assertFalse(session.snapshot()["frozen"])
        saturated = self.make()
        self.prepare(saturated, commits=0xFFFFFFFF)
        with self.assertRaisesRegex(RuntimeError, "saturated"):
            saturated.send_once()
        self.assertEqual(saturated._ports["fpga"].writes, [])

    def test_reset_rejection_and_external_version_jump_freeze(self):
        for changed in (dict(ticks=99), dict(commits=6), dict(rejected=1), dict(version=7), dict(version=4)):
            with self.subTest(changed=changed):
                session = self.make()
                self.prepare(session)
                session._ports["fpga"].incoming.put(status_frame(**changed))
                self.wait(lambda: session.snapshot()["frozen"])
                with self.assertRaises(RuntimeError):
                    session.send_once()
                self.assertEqual(session._ports["fpga"].writes, [])

    def test_published_timeout_remains_read_only_and_late_output_confirms(self):
        session = self.make(output_timeout_s=.06)
        self.prepare(session)
        session.send_once()
        self.written(session)
        session._ports["fpga"].incoming.put(status_frame(ticks=101, version=4, commits=8))
        self.wait(lambda: session.snapshot()["state"] == "published_output_timeout")
        with self.assertRaises(RuntimeError):
            session.send_once()
        session._ports["fpga"].incoming.put(latency_frame())
        self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "output_confirmed")
        self.assertTrue(session.snapshot()["frozen"])
        self.assertEqual(len(session._ports["fpga"].writes), 1)

    def test_accept_timeout_and_partial_write_never_retransmit(self):
        for partial in (False, True):
            with self.subTest(partial=partial):
                session = self.make(accept_timeout_s=.06)
                self.prepare(session)
                session._ports["fpga"].partial_write = partial
                session.send_once()
                self.wait(lambda: session.snapshot()["frozen"])
                self.assertEqual(len(session._ports["fpga"].writes), 1)
                self.assertFalse(session.snapshot()["continuous"])

    def test_geometry_checks_stream_freshness_and_candidate_age(self):
        for stream_continues in (False, True):
            with self.subTest(stream_continues=stream_continues):
                entered, release = threading.Event(), threading.Event()
                def checker(words):
                    entered.set()
                    release.wait(1)
                    return {"fixed_point_checked": True}
                session = self.make(imu_stale_s=.08, candidate_max_age_s=.08, geometry_checker=checker)
                self.prepare(session)
                if stream_continues:
                    self.streaming(session)
                session.send_once()
                self.assertTrue(entered.wait(1))
                time.sleep(.12)
                release.set()
                self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "candidate_discarded")
                self.assertEqual(session._ports["fpga"].writes, [])

    def test_stop_and_mounting_revocation_prevent_later_submission(self):
        session = self.make()
        self.prepare(session)
        session.start_continuous(.5)
        self.written(session)
        session.send_once()
        with self.assertRaisesRegex(RuntimeError, "active request"):
            session.set_mounting(np.eye(3), confirmed=False)
        snapshot = session.snapshot()
        self.assertFalse(snapshot["continuous"])
        self.assertFalse(snapshot["mounting_confirmed"])
        self.assertIsNone(snapshot["candidate"])
        self.confirm(session)
        self.assertEqual(len(session._ports["fpga"].writes), 1)

    def test_disconnect_stops_submissions_and_preserves_pose(self):
        session = self.make()
        self.prepare(session)
        pose = session.snapshot()["imu"]["quaternion"]
        session._ports["imu"].incoming.put(OSError("USB removed"))
        self.wait(lambda: session.snapshot()["frozen"])
        self.assertFalse(session.snapshot()["connected"]["imu"])
        self.assertEqual(session.snapshot()["imu"]["quaternion"], pose)
        self.assertEqual(session._ports["fpga"].writes, [])

    def test_continuous_refreshes_waiting_candidate_when_geometry_starts(self):
        session = self.make()
        self.prepare(session)
        self.streaming(session)
        session.start_continuous(.5)
        self.written(session)
        candidate = self.wait(lambda: session.snapshot()["candidate"], timeout=1.)
        original_sample_id = candidate["sample"]["id"]
        time.sleep(.12)
        session._ports["fpga"].incoming.put(latency_frame())
        session._ports["fpga"].incoming.put(status_frame(ticks=101, version=4, commits=8))
        self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "quantized_unchanged")
        refreshed = session.snapshot()["request"]
        self.assertEqual(refreshed["request_id"], candidate["request_id"])
        self.assertGreater(refreshed["sample"]["id"], original_sample_id)
        self.assertIn("selected_at", refreshed)
        self.assertTrue(session.snapshot()["continuous"])
        session.stop()

    def test_stale_busy_uninitialized_and_error_status_prevent_writes(self):
        for flags, message in ((0x1072, "busy"), (0x1030, "initialization"), (0x1074, "hardware error")):
            with self.subTest(flags=flags):
                session = self.make()
                self.prepare(session, flags=flags)
                with self.assertRaisesRegex(RuntimeError, message):
                    session.send_once()
                self.assertEqual(session._ports["fpga"].writes, [])
        session = self.make(status_stale_s=.04)
        self.prepare(session)
        time.sleep(.05)
        with self.assertRaisesRegex(RuntimeError, "status unavailable or stale"):
            session.send_once()
        self.assertEqual(session._ports["fpga"].writes, [])

    def test_explicit_rate_command_only_touches_imu(self):
        session = self.make()
        session.set_imu_100hz()
        self.assertEqual(session._ports["imu"].writes, [IMU_100HZ_COMMAND])
        self.assertEqual(session._ports["fpga"].writes, [])

    def test_stuck_output_drain_has_deadline_and_no_retransmission(self):
        session = self.make()
        self.prepare(session)
        session._drain_timeout_s = .06
        session._ports["fpga"].out_waiting = 232
        session.send_once()
        self.wait(lambda: session.snapshot()["frozen"])
        self.assertIn("did not drain", session.snapshot()["error"])
        self.assertEqual(len(session._ports["fpga"].writes), 1)
        self.assertTrue(session.snapshot()["request"]["invalidated"])
        other = self.make(imu=True, fpga=False)
        other._drain_timeout_s = .06
        other._ports["imu"].out_waiting = 7
        begin = time.monotonic()
        with self.assertRaisesRegex(TimeoutError, "did not drain"):
            other.set_imu_100hz()
        self.assertLess(time.monotonic() - begin, .5)
        self.assertTrue(other.snapshot()["frozen"])

    def test_close_allows_started_transaction_to_drain(self):
        session = self.make()
        self.prepare(session)
        port = session._ports["fpga"]
        port.out_waiting = 232
        session.send_once()
        self.written(session)
        open_before_drain = []
        def drain_later():
            time.sleep(.06)
            open_before_drain.append(port.is_open)
            port.out_waiting = 0
        draining = threading.Thread(target=drain_later)
        draining.start()
        session.close()
        draining.join(1.)
        self.assertEqual(open_before_drain, [True])
        self.assertEqual(len(port.writes), 1)
        self.assertFalse(port.is_open)
        self.assertFalse(any(t.is_alive() for t in session._threads))

    def test_real_geometry_subprocess_completes_within_candidate_age(self):
        session = self.make(real_geometry=True)
        self.prepare(session)
        self.streaming(session)
        session.send_once()
        self.written(session)
        request = session.snapshot()["request"]
        self.assertTrue(request["geometry"]["fixed_point_checked"])
        self.assertLess(request["geometry_done_at"] - request["sample"]["received_at"], 2.)
        self.assertFalse(session.snapshot()["frozen"])

    def test_close_cancels_real_geometry_process(self):
        session = self.make(real_geometry=True)
        self.prepare(session)
        session.send_once()
        self.wait(lambda: (session.snapshot()["request"] or {}).get("phase") == "geometry")
        begin = time.monotonic()
        session.close()
        self.assertLess(time.monotonic() - begin, 4.)
        self.assertEqual(session._ports["fpga"].writes, [])
        self.assertFalse(any(t.is_alive() for t in session._threads))


if __name__ == "__main__":
    unittest.main(verbosity=2)
