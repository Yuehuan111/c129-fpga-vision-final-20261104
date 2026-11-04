"""Latest-only C128 chessboard observation and small-matrix fitting.

This module never generates a per-pixel map or writes a serial port. Live
input is exclusively the source-frame-tagged raw HDMI inset. Full/static
images are supported only by the offline observation helper for validation.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import binascii
import math
import struct
import threading
import time

import numpy as np

import scan_plane as sp
from complete_geometry import configuration

PATTERN = (10, 5)
INSET_SIZE = (320, 180)
BARCODE_SIZE = (320, 12)
MAGIC = 0xC128


def _forward32(new, old):
    return 0 < ((int(new)-int(old)) & 0xffffffff) < 0x80000000


def decode_inset_metadata(frame):
    """Decode the in-frame 4x4-cell CRC/complement protected C128 footer.

    Only native 1280x720 capture is accepted. A capture-card crop, letterbox,
    scale or mirrored view requires an explicitly verified transform first;
    silently guessing one would bind corners to the wrong raw coordinates.
    """
    image = np.asarray(frame)
    if image.shape[:2] != (720, 1280) or image.ndim not in (2, 3):
        raise ValueError('Live observation requires a native, uncropped 1280x720 HDMI capture')
    gray = image if image.ndim == 2 else sp._cv2().cvtColor(image, sp._cv2().COLOR_BGR2GRAY)
    footer = gray[180:192, :320]
    levels = np.array([np.median(footer[row*4+1:row*4+3, col*4+1:col*4+3])
                       for row in range(3) for col in range(80)])
    low, high = float(np.percentile(levels, 10)), float(np.percentile(levels, 90))
    if high-low < 100:
        raise ValueError('C128 source footer missing or low contrast; scanned output is not a raw observation')
    mid = (high+low)/2
    if np.any(np.abs(levels-mid) < (high-low)*.18):
        raise ValueError('C128 source footer has ambiguous cells')
    bits = levels > mid
    if not np.array_equal(bits[:120], ~bits[120:]):
        raise ValueError('C128 source footer complement mismatch')
    payload = np.packbits(bits[:120].astype(np.uint8), bitorder='little').tobytes()
    magic, source_id, capture_us, version, flags, crc = struct.unpack('<HIIHBH', payload)
    if magic != MAGIC or binascii.crc_hqx(payload[:13], 0xffff) != crc:
        raise ValueError('C128 source footer magic/CRC mismatch')
    if flags != 3:
        raise ValueError('C128 source footer is not a valid corrected-output raw observation')
    return dict(source_frame_id=source_id, source_capture_us=capture_us,
                output_version=version, metadata_crc_valid=True, footer_flags=flags)


@dataclass(frozen=True)
class SourceObservation:
    image: np.ndarray
    metadata: dict

    @classmethod
    def from_capture(cls, packet):
        if (not isinstance(packet, dict) or packet.get('source_kind') != 'camera'
                or packet.get('live') is not True):
            raise ValueError('Live scan accepts only the current capture-card source; files cannot drive the FPGA')
        received = packet.get('captured_at')
        if isinstance(received, bool) or not isinstance(received, (float, int)) or not math.isfinite(received):
            raise ValueError('A monotonic capture receipt time is required')
        frame = np.asarray(packet.get('frame'))
        metadata = decode_inset_metadata(frame)
        metadata.update(source='c128_raw_inset', domain='raw', live=True,
                        source_size=[1280, 720], observation_size=[320, 180],
                        source_transform=[4, 0, 1.5, 0, 4, 1.5], received_at=float(received),
                        capture_frame_id=str(packet.get('frame_id')), capture_generation=packet.get('generation'))
        return cls(frame[:180, :320].copy(), metadata)

    @classmethod
    def offline(cls, image, *, frame_id=1, received_at=0., domain='raw', source_transform=None):
        """Explicit test/reference input; candidates remain live=False."""
        image = np.asarray(image)
        if domain not in ('raw', 'rectified'):
            raise ValueError('Offline domain must be explicitly raw or rectified')
        height, width = image.shape[:2]
        if source_transform is None:
            if (width, height) not in ((1280, 720), (1920, 1080)):
                raise ValueError('Offline downsampling needs its exact pixel-centre transform')
            scale = 1280/width
            source_transform = [scale, 0, (scale-1)/2, 0, scale, (scale-1)/2]
        return cls(image.copy(), dict(source='offline_reference', domain=domain, live=False,
            source_size=[1280, 720], observation_size=[width, height],
            source_transform=list(source_transform), received_at=float(received_at),
            source_frame_id=int(frame_id), source_capture_us=int(frame_id)*33333,
            capture_frame_id='offline:'+str(frame_id), metadata_crc_valid=False, output_version=0))


class ChessboardTracker:
    """Full 50-corner detection, bounded LK tracking, fixed output grid.

    An unmarked chessboard is symmetric under 180 degrees. During continuous
    visibility the preceding ordered corners resolve that ambiguity. After
    a long loss a new stable acquisition establishes orientation again; this
    is reported, never presented as recovered physical corner identity.
    """
    def __init__(self, context, *, cell_pixels=64., max_age_s=.35, redetect_every=5,
                 min_cell_pixels=3., smoothing=.8, clock=time.monotonic):
        self.context = sp.validate_context(context)
        if self.context['reference'] is not None:
            raise ValueError('Visual scanning uses an independent ordinary-lens context')
        if not 16 <= cell_pixels <= 90 or not 0 < smoothing <= 1 or redetect_every < 1:
            raise ValueError('Invalid fixed grid/tracking settings')
        self.cell_pixels, self.max_age_s = float(cell_pixels), float(max_age_s)
        self.redetect_every, self.min_cell_pixels = int(redetect_every), float(min_cell_pixels)
        self.smoothing, self.clock = float(smoothing), clock
        self.target = sp.GRID*self.cell_pixels + [640-4.5*self.cell_pixels, 360-2*self.cell_pixels]
        self._gray = self._corners = self._smoothed = None
        self._last_metadata = None
        self._last_success_time = None
        self._frames_since_detection = 0
        self._detection_scale = 1
        self._acquisition_count = 0
        self._lost = True
        self._last_result = dict(state='acquiring', reason='Waiting for all 50 corners')
        self.stats = dict(observations=0, accepted=0, detected=0, tracked=0, rejected=0, duplicate=0, reacquired=0)

    def snapshot(self):
        return deepcopy(dict(self._last_result, statistics=self.stats))

    def _reject(self, reason, *, state='lost', reset=True):
        self.stats['rejected'] += 1
        if reset:
            self._gray = None
            self._smoothed = None
            self._acquisition_count = 0
            self._lost = True
        self._last_result = dict(state=state, reason=reason, retained_mapping=True)
        return dict(candidate=None, **self.snapshot())

    def _ordered(self, corners, reference):
        c = np.asarray(corners, float).reshape(5, 10, 2)
        horizontal = c[:, -1].mean(axis=0)-c[:, 0].mean(axis=0)
        vertical = c[-1].mean(axis=0)-c[0].mean(axis=0)
        if horizontal[0]*vertical[1]-horizontal[1]*vertical[0] < 0:
            c = c[:, ::-1]
        choices = [c.reshape(-1, 2), c[::-1, ::-1].reshape(-1, 2)]
        if reference is not None:
            costs = [np.mean(np.linalg.norm(value-reference, axis=1)**2) for value in choices]
            return choices[int(np.argmin(costs))]
        delta = choices[0][9]-choices[0][0]
        choose = delta[0] >= 0 if abs(delta[0]) > .15*np.linalg.norm(delta) else delta[1] >= 0
        return choices[0 if choose else 1]

    def _track(self, gray):
        cv2 = sp._cv2()
        prev = np.asarray(self._corners, np.float32).reshape(-1, 1, 2)
        new, ok, error = cv2.calcOpticalFlowPyrLK(self._gray, gray, prev, None,
            winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 20, .02))
        if new is None or not np.all(ok) or np.max(error) > 35:
            return None
        back, bok, _ = cv2.calcOpticalFlowPyrLK(gray, self._gray, new, None,
            winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 20, .02))
        if back is None or not np.all(bok) or np.max(np.linalg.norm(back-prev, axis=2)) > .6:
            return None
        return new.reshape(-1, 2)

    def _corner_quality(self, corners, gray):
        c = corners.reshape(5, 10, 2)
        if (not np.isfinite(c).all() or np.any(c < 3)
                or np.any(c[..., 0] >= gray.shape[1]-3) or np.any(c[..., 1] >= gray.shape[0]-3)):
            raise ValueError('Incomplete board or corners too close to the observation border')
        dx, dy = c[:, 1:]-c[:, :-1], c[1:]-c[:-1]
        distances = np.r_[np.linalg.norm(dx, axis=2).ravel(), np.linalg.norm(dy, axis=2).ravel()]
        if np.percentile(distances, 10) < self.min_cell_pixels or distances.max()/distances.min() > 6:
            raise ValueError('Board is too small or foreshortened for stable 50-corner observation')
        area = dx[:-1, :, 0]*dy[:, :-1, 1]-dx[:-1, :, 1]*dy[:, :-1, 0]
        if np.any(area <= self.min_cell_pixels**2*.18):
            raise ValueError('Corner ordering is folded or locally degenerate')
        # Edge information is checked around the board, not in distant HDMI
        # content. Flat/blurred or occluded observations cannot use old tracks.
        cv2 = sp._cv2()
        scores = cv2.cornerMinEigenVal(gray, blockSize=3)
        strength = [float(scores[max(0, round(y)-2):round(y)+3, max(0, round(x)-2):round(x)+3].max()) for x, y in corners]
        if min(strength) < .00025:
            raise ValueError('One or more tracked corners lacks current image evidence')
        return dict(min_cell_observation_pixels=float(distances.min()),
                    median_cell_observation_pixels=float(np.median(distances)), min_corner_strength=min(strength))

    def process(self, observation, *, now=None):
        if not isinstance(observation, SourceObservation):
            raise TypeError('SourceObservation with an explicit domain is required')
        now = self.clock() if now is None else float(now)
        meta = deepcopy(observation.metadata)
        self.stats['observations'] += 1
        age = now-meta['received_at']
        if not 0 <= age <= self.max_age_s:
            return self._reject('Observation expired before detection', state='stale')
        if self._last_metadata is not None and not _forward32(meta['source_frame_id'], self._last_metadata['source_frame_id']):
            self.stats['duplicate'] += 1
            if now-self._last_metadata['received_at'] > self.max_age_s:
                return self._reject('Source frame ID has stopped advancing; retained mapping is stale', state='stale')
            return dict(candidate=None, **self.snapshot(), skipped='duplicate_or_reordered_source')
        if meta['domain'] not in ('raw', 'rectified') or (meta['live'] and meta['source'] != 'c128_raw_inset'):
            return self._reject('Scanned/main output cannot be used as a raw observation')
        self._last_metadata = meta
        image = np.asarray(observation.image)
        cv2 = sp._cv2()
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if gray.dtype != np.uint8:
            return self._reject('Observation must contain uint8 camera pixels')
        recent = self._last_success_time is not None and now-self._last_success_time <= 1.
        reference = self._corners if recent else None
        corners = None
        method = 'detect'
        if self._gray is not None and self._gray.shape == gray.shape and self._frames_since_detection < self.redetect_every:
            corners = self._track(gray)
            if corners is not None:
                method = 'track'
        if corners is None:
            # Cubic expansion helps SB localize an existing small checker
            # pattern. It creates no details. Points are immediately returned
            # to native observation pixel centres and all quality gates use
            # those coordinates and the actual quantized target matrix.
            detection_scale = 1
            ok, found = cv2.findChessboardCornersSB(gray, PATTERN,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
            if not ok and gray.shape == (180, 320):
                detection_scale = 3
                detector_image = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
                ok, found = cv2.findChessboardCornersSB(detector_image, PATTERN,
                    flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
            if not ok or found is None or len(found) != 50:
                return self._reject('All 50 corners are required; waiting for reacquisition')
            found = (found+.5)/detection_scale-.5
            self._detection_scale = detection_scale
            corners = self._ordered(found, reference)
            self._frames_since_detection = 0
        try:
            quality = self._corner_quality(corners, gray)
            if reference is not None and np.max(np.linalg.norm(corners-reference, axis=1)) > .25*math.hypot(*gray.shape):
                raise ValueError('Corner correspondence jumped; reacquisition is required')
            previous_smoothed = self._smoothed
            smoothed = corners.copy()
            if previous_smoothed is not None and recent:
                # One bounded step of low-pass filtering; no frame queue.
                smoothed = self.smoothing*corners+(1-self.smoothing)*previous_smoothed
            transform = np.asarray(meta['source_transform'], float).reshape(2, 3)
            source_points = np.c_[smoothed, np.ones(50)] @ transform.T
            measured_points = np.c_[corners, np.ones(50)] @ transform.T
            if meta['domain'] == 'raw':
                fitting, roundtrip = sp._joint_unproject(source_points, self.context)
                measured, _ = sp._joint_unproject(measured_points, self.context)
            else:
                k, l = sp._kl(self.context['words18'])
                fitting = sp._project(source_points, l @ np.linalg.inv(k))
                measured = sp._project(measured_points, l @ np.linalg.inv(k))
                roundtrip = 0.
            g = sp._homography(self.target, fitting)
            k, l = sp._kl(self.context['words18'])
            h = sp._positive_scale(g @ k @ np.linalg.inv(l))
            # Evaluate the matrix that will actually be quantized and sent.
            quantized = np.asarray(configuration(self.context['calibration'], matrix=h)[18:], float).reshape(3, 3)/sp.Q
            final_g = quantized @ l @ np.linalg.inv(k)
            error = np.linalg.norm(sp._project(measured, np.linalg.inv(final_g))-self.target, axis=1)
            train = np.indices((5, 10)).sum(axis=0).ravel()%2 == 0
            held = sp._homography(self.target[train], fitting[train])
            held_error = np.linalg.norm(sp._project(measured[~train], np.linalg.inv(held))-self.target[~train], axis=1)
            rms, maximum, held_rms = float(np.sqrt(np.mean(error**2))), float(error.max()), float(np.sqrt(np.mean(held_error**2)))
            if rms > 3 or maximum > 6 or held_rms > 3:
                raise ValueError(f'Plane fit rejected: target RMS {rms:.3f}, max {maximum:.3f}, heldout {held_rms:.3f} px')
            quality.update(accepted=True, corners=50, method=method, rms_target_pixels=rms,
                max_target_pixels=maximum, heldout_rms_target_pixels=held_rms,
                joint_roundtrip_max_source_pixels=roundtrip, optical_validation=False)
            quality['detection_interpolation_scale'] = self._detection_scale
        except (ValueError, np.linalg.LinAlgError) as exc:
            return self._reject(str(exc))
        if self.clock()-meta['received_at'] > self.max_age_s and meta['live']:
            return self._reject('Observation expired during vision processing', state='stale')
        self.stats['detected' if method == 'detect' else 'tracked'] += 1
        self._frames_since_detection += 1
        self._gray, self._corners, self._smoothed = gray.copy(), corners.copy(), smoothed.copy()
        self._last_success_time = now
        self._acquisition_count += 1
        if self._lost and self._acquisition_count < 2:
            self._last_result = dict(state='acquiring', reason='Confirming ordered full-board acquisition', quality=quality)
            return dict(candidate=None, **self.snapshot())
        if self._lost:
            self.stats['reacquired'] += 1
        self._lost = False
        self.stats['accepted'] += 1
        candidate = dict(schema='c128_visual_candidate_v1', context_hash=self.context['context_hash'],
            H=h.tolist(), observation=meta, quality=quality,
            target_grid=dict(pattern_size=[10, 5], inner_corner_span_cells=[9, 4],
                cell_pixels=self.cell_pixels, origin=self.target[0].tolist(),
                centre=[640, 360], canvas=[1280, 720]),
            source_points_720=measured_points.tolist())
        self._last_result = dict(state='tracking', reason=None, quality=quality,
            source_frame_id=meta['source_frame_id'], orientation_note='Continuity while visible; symmetric board orientation re-established after long loss')
        return dict(candidate=candidate, **self.snapshot())


class VisionScanWorker:
    """One detector thread and a single replaceable capture slot, never FIFO."""
    def __init__(self, context, *, submit_candidate, report_state, clock=time.monotonic, **tracker_options):
        self.clock = clock
        self.tracker = ChessboardTracker(context, clock=clock, **tracker_options)
        self.submit_candidate, self.report_state = submit_candidate, report_state
        self._lock = threading.Lock()
        self._wake, self._stop = threading.Event(), threading.Event()
        self._pending = None
        self._last_capture_id = None
        self._thread = None
        self._state = dict(state='stopped')
        self._last_received_at = None
        self.replaced = 0

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='visual-source-latest-only', daemon=True)
        self._thread.start()

    def offer_frame(self, packet):
        if not isinstance(packet, dict):
            return False
        capture_id = (packet.get('generation'), packet.get('frame_id'))
        with self._lock:
            if capture_id == self._last_capture_id:
                return False
            self._last_capture_id = capture_id
            if self._pending is not None:
                self.replaced += 1
            self._pending = dict(packet, frame=np.asarray(packet['frame']).copy())
            self._last_received_at = packet.get('captured_at')
        self._wake.set()
        return True

    def _publish(self, state):
        with self._lock:
            self._state = deepcopy(state)
        self.report_state(state)

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                packet, self._pending = self._pending, None
            if packet is None:
                if self._last_received_at is not None and self.clock()-self._last_received_at > self.tracker.max_age_s:
                    self._publish(dict(state='stale', reason='Capture card has no fresh frame; retained mapping is stale'))
                self._wake.wait(.05)
                self._wake.clear()
                continue
            try:
                observation = SourceObservation.from_capture(packet)
                result = self.tracker.process(observation)
                candidate = result.pop('candidate')
                if self._stop.is_set():
                    break
                if candidate is not None:
                    # Recheck freshness after any concurrent latest replacement.
                    if self.clock()-candidate['observation']['received_at'] > self.tracker.max_age_s:
                        raise ValueError('Candidate expired before offer')
                    with self._lock:
                        newer_waiting = bool(self._pending and self._pending.get('captured_at', -math.inf) > candidate['observation']['received_at'])
                    if not newer_waiting:
                        result['request_id'] = self.submit_candidate(candidate)
                    else:
                        result['skipped'] = 'newer_capture_waiting'
                self._publish(result)
            except Exception as exc:
                self.tracker._reject(str(exc))
                self._publish(dict(state='lost', reason=str(exc), retained_mapping=True))

    def snapshot(self):
        with self._lock:
            return deepcopy(dict(self._state, pending=self._pending is not None, replaced_capture_frames=self.replaced))

    def stop(self):
        self._stop.set()
        self._wake.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.)
        with self._lock:
            self._pending = None
            self._state = dict(state='stopped')
