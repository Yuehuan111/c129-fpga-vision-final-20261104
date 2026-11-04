"""Offline detector/source regressions, including the supplied still photos.

Synthetic raster generation is a test fixture, never a live coordinate-map
upload. The board remains responsible for image mapping and interpolation.
"""
from copy import deepcopy
import binascii
from pathlib import Path
import struct
import sys
import threading
import time
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))
from realtime_scan import ChessboardTracker, SourceObservation, VisionScanWorker, decode_inset_metadata
import scan_plane as sp
from complete_geometry import configuration, load_calibration


def context():
    cal = load_calibration()
    return sp.make_context(cal, configuration(cal)[:18])


def footer(frame, source=1, capture_us=99_900_000, version=3, flags=3):
    frame = np.asarray(frame).copy()
    prefix = struct.pack('<HIIHB', 0xC128, source, capture_us, version, flags)
    payload = prefix+struct.pack('<H', binascii.crc_hqx(prefix, 0xffff))
    bits = np.unpackbits(np.frombuffer(payload, np.uint8), bitorder='little')
    bits = np.r_[bits, 1-bits]
    for index, value in enumerate(bits):
        row, col = divmod(index, 80)
        frame[180+row*4:184+row*4, col*4:col*4+4] = 235 if value else 16
    return frame


def synth_raw(ctx, *, roll=0., shift=(0, 0), tilt=0.):
    """Rasterize only the known board's curved polygon edges under C97."""
    cv2 = sp._cv2()
    image = np.full((720, 1280, 3), 160, np.uint8)
    a = np.deg2rad(roll)
    rotate = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    def project(p):
        p = (np.asarray(p)-[4.5, 2])*.24
        p = p @ rotate.T + shift
        p = p/(1+tilt*p[:, :1])
        return sp._joint_project(p, ctx)
    for row in range(6):
        for col in range(11):
            vertices = np.array([[col-1, row-1], [col, row-1], [col, row], [col-1, row]], float)
            boundary = np.concatenate([vertices[i]+np.linspace(0, 1, 24, endpoint=False)[:, None]*(vertices[(i+1)%4]-vertices[i]) for i in range(4)])
            points = np.rint(project(boundary)*16).astype(np.int32)
            cv2.fillPoly(image, [points], (245,)*3 if (row+col)%2 else (5,)*3, lineType=cv2.LINE_AA, shift=4)
    return image


def capture_packet(raw, source, *, received_at=0., capture_us=None, version=3, source_kind='camera'):
    # Main HDMI pixels are unrelated; only the separately rendered raw inset
    # carries the plane observation. The footer travels with this source.
    hdmi = np.full((720, 1280, 3), 80, np.uint8)
    hdmi[:180, :320] = thumbnail(raw)
    hdmi = footer(hdmi, source, capture_us or 99_900_000+source*1000, version)
    return dict(frame=hdmi, frame_id=f'1:{source}', generation=1, sequence=source,
                captured_at=received_at, source_kind=source_kind, live=source_kind=='camera')


def thumbnail(raw):
    return np.rint((raw[1::4, 1::4].astype(float)+raw[1::4, 2::4]+
                    raw[2::4, 1::4]+raw[2::4, 2::4])/4).astype(np.uint8)


class ObservationTests(unittest.TestCase):
    def test_crc_complement_and_exact_coordinate_contract(self):
        frame = footer(np.zeros((720, 1280, 3), np.uint8), source=0xfffffffe, capture_us=123, version=65535)
        metadata = decode_inset_metadata(frame)
        self.assertEqual(metadata['source_frame_id'], 0xfffffffe)
        self.assertEqual(metadata['source_capture_us'], 123)
        self.assertEqual(metadata['output_version'], 65535)
        packet = dict(frame=frame, captured_at=10., frame_id='g:2', source_kind='camera', live=True)
        obs = SourceObservation.from_capture(packet)
        self.assertEqual(obs.image.shape, (180, 320, 3))
        self.assertEqual(obs.metadata['source_transform'], [4, 0, 1.5, 0, 4, 1.5])
        for index in (0, 119, 120, 239):
            damaged = frame.copy(); row, col = divmod(index, 80)
            damaged[180+row*4:184+row*4, col*4:col*4+4] = 255-damaged[180+row*4:184+row*4, col*4:col*4+4]
            with self.subTest(cell=index), self.assertRaises(ValueError):
                decode_inset_metadata(damaged)

    def test_main_scan_image_scaled_crop_or_file_cannot_become_live_raw(self):
        for frame in (np.zeros((720, 1280, 3), np.uint8), np.zeros((1080, 1920, 3), np.uint8),
                      footer(np.zeros((720, 1280, 3), np.uint8), flags=7)):
            with self.assertRaises(ValueError):
                decode_inset_metadata(frame)
        for kind in ('image', 'video'):
            with self.assertRaisesRegex(ValueError, 'files cannot'):
                SourceObservation.from_capture(dict(source_kind=kind, live=False))

    def test_crc_protects_simultaneous_payload_and_complement_change(self):
        frame = footer(np.zeros((720, 1280, 3), np.uint8))
        for index in (30, 150):
            row, col = divmod(index, 80)
            frame[180+row*4:184+row*4, col*4:col*4+4] = 255-frame[180+row*4:184+row*4, col*4:col*4+4]
        with self.assertRaisesRegex(ValueError, 'CRC'):
            decode_inset_metadata(frame)


class TrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = context()

    def process(self, tracker, image, number, *, domain='raw', small=False):
        kwargs = dict(source_transform=[4, 0, 1.5, 0, 4, 1.5]) if small else {}
        obs = SourceObservation.offline(image, frame_id=number, received_at=number/30, domain=domain, **kwargs)
        return tracker.process(obs, now=number/30)

    def test_full50_corner_raw_inset_motion_uses_fixed_grid(self):
        tracker = ChessboardTracker(self.context, smoothing=1.)
        candidates = []
        for number, angle in enumerate((0, 0, 2, 4, 6, 8, 10), 1):
            raw = synth_raw(self.context, roll=angle, shift=(number*.001, 0), tilt=.08)
            result = self.process(tracker, thumbnail(raw), number, small=True)
            if result['candidate']:
                candidates.append(result['candidate'])
        self.assertGreaterEqual(len(candidates), 4, tracker.snapshot())
        grids = [item['target_grid'] for item in candidates]
        self.assertTrue(all(grid == grids[0] for grid in grids))
        self.assertEqual(grids[0]['inner_corner_span_cells'], [9, 4])
        self.assertNotEqual(candidates[0]['H'], candidates[-1]['H'])
        self.assertTrue(all(item['quality']['corners'] == 50 and item['quality']['rms_target_pixels'] <= 3 for item in candidates))
        self.assertGreater(tracker.stats['tracked'], 0)

    def test_detected_order_reversal_does_not_flip_matrix(self):
        tracker = ChessboardTracker(self.context, redetect_every=1, smoothing=1.)
        image = synth_raw(self.context)
        self.process(tracker, image, 1)
        baseline = self.process(tracker, image, 2)['candidate']
        original = sp._cv2().findChessboardCornersSB
        def reversed_detector(*args, **kwargs):
            ok, corners = original(*args, **kwargs)
            return ok, corners[::-1].copy() if ok else corners
        with patch.object(sp._cv2(), 'findChessboardCornersSB', side_effect=reversed_detector):
            repeated = self.process(tracker, image, 3)['candidate']
        np.testing.assert_allclose(baseline['H'], repeated['H'], atol=2e-4)

    def test_lost_occluded_reacquire_and_stale_source(self):
        tracker = ChessboardTracker(self.context, smoothing=1.)
        image = synth_raw(self.context)
        self.process(tracker, image, 1)
        self.assertIsNotNone(self.process(tracker, image, 2)['candidate'])
        self.assertEqual(self.process(tracker, np.full_like(image, 130), 3)['state'], 'lost')
        self.assertIsNone(self.process(tracker, image, 4)['candidate'])
        self.assertIsNotNone(self.process(tracker, image, 5)['candidate'])
        self.assertEqual(tracker.stats['reacquired'], 2)
        observation = SourceObservation.offline(image, frame_id=5, received_at=.8)
        self.assertEqual(tracker.process(observation, now=.8)['state'], 'stale')
        self.assertIsNone(tracker.process(observation, now=2)['candidate'])

    def test_supplied_photos_use_their_declared_domains_once(self):
        cv2 = sp._cv2()
        for name, domain in (('demo_photo.jpg', 'raw'), ('demo_corrected.jpg', 'rectified')):
            image = cv2.imread(str(HOST/'assets'/name))
            self.assertIsNotNone(image)
            tracker = ChessboardTracker(self.context, smoothing=1.)
            self.process(tracker, image, 1, domain=domain)
            result = self.process(tracker, image, 2, domain=domain)
            self.assertIsNotNone(result['candidate'], result)
            self.assertLess(result['candidate']['quality']['rms_target_pixels'], 3.)
            self.assertFalse(result['candidate']['observation']['live'])
        # A 3x detector-only expansion recovers the historical raw board's
        # small existing corners. Quality is measured after exact centre
        # restoration; this remains an offline still-image regression.
        raw = thumbnail(cv2.resize(cv2.imread(str(HOST/'assets/demo_photo.jpg')), (1280, 720), interpolation=cv2.INTER_AREA))
        tracker = ChessboardTracker(self.context)
        self.process(tracker, raw, 1, small=True)
        result = self.process(tracker, raw, 2, small=True)
        self.assertIsNotNone(result['candidate'], result)
        self.assertLess(result['quality']['rms_target_pixels'], 3)
        self.assertLess(result['quality']['max_target_pixels'], 6)
        self.assertEqual(result['quality']['detection_interpolation_scale'], 3)

    def test_bad_order_or_tiny_board_rejected(self):
        tracker = ChessboardTracker(self.context)
        image = synth_raw(self.context)
        with patch.object(sp._cv2(), 'findChessboardCornersSB', return_value=(True, np.zeros((50, 1, 2), np.float32))):
            result = self.process(tracker, image, 1)
        self.assertIsNone(result['candidate'])


class WorkerTests(unittest.TestCase):
    def test_latest_slot_and_static_file_never_submit(self):
        submitted, states = [], []
        worker = VisionScanWorker(context(), submit_candidate=submitted.append, report_state=states.append)
        for number in range(1, 10):
            worker.offer_frame(dict(frame=np.zeros((720, 1280, 3), np.uint8), captured_at=time.monotonic(),
                frame_id=str(number), generation=1, source_kind='image', live=False))
        self.assertEqual(worker.replaced, 8)
        self.assertEqual(worker._pending['frame_id'], '9')
        worker.start()
        try:
            end = time.monotonic()+2
            while not states and time.monotonic() < end:
                time.sleep(.01)
            self.assertTrue(states)
            self.assertEqual(submitted, [])
            self.assertIn('files cannot', states[-1]['reason'])
        finally:
            worker.stop()

    def test_provided_photo_complete_footer_to_integer_cache_chain(self):
        from validate_visual_scan import run
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path(directory)/'static.json')
        self.assertEqual(len(result['source_points_720']), 50)
        self.assertEqual(len(result['configuration_words27']), 27)
        self.assertTrue(result['geometry']['admitted'])
        self.assertEqual(result['geometry']['tile_budget']['unsupported_tiles'], [])
        self.assertLessEqual(result['geometry']['tile_budget']['max_roi_height'], 64)
        self.assertTrue(result['source_ids_are_synthetic'])
        self.assertFalse(result['hardware_accessed'])
        self.assertFalse(result['map_exported'])


if __name__ == '__main__':
    unittest.main()
