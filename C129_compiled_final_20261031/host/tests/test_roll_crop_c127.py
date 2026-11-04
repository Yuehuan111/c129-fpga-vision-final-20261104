import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'vendor'))
from imu_pose import (camera_relative_roll, compensation_matrix, matrix_to_quaternion,
                      mode_compensation_matrix)
from imu_app import read_capture_metadata, read_mounting, save_mounting
from roll_crop import FixedRollCrop, calibration_digest, verify_crop_profile
from complete_geometry import load_calibration


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class CameraRollTests(unittest.TestCase):
    def test_mixed_rotation_and_inverse_sampling_direction(self):
        reference = ry(.4) @ rx(-.2)
        mounting = rz(.3) @ rx(.8)
        for angle in [-.6, -.1, 0, .25, .7]:
            forward = rz(angle) @ ry(.45) @ rx(-.35)
            now = reference @ mounting @ forward @ mounting.T
            qn, qr = map(matrix_to_quaternion, (now, reference))
            actual = mode_compensation_matrix(qn, qr, mounting, mode='roll_only')
            np.testing.assert_allclose(actual, rz(-angle), atol=1e-12)
            np.testing.assert_allclose(actual @ np.array([1., 0, 1]),
                                       [math.cos(angle), -math.sin(angle), 1], atol=1e-12)
            self.assertAlmostEqual(camera_relative_roll(qn, qr, mounting), angle)

    def test_pitch_yaw_do_not_create_roll_in_defined_decomposition(self):
        for pitch, yaw in [(.6, .3), (-.4, .7), (.8, -.6)]:
            q = matrix_to_quaternion(ry(yaw) @ rx(pitch))
            np.testing.assert_allclose(mode_compensation_matrix(q, [1, 0, 0, 0], np.eye(3),
                                      mode='roll_only'), np.eye(3), atol=1e-12)

    def test_world_to_imu_and_full_preservation(self):
        now, reference = ry(.2) @ rz(.4), rx(.3)
        mounting = rx(-.4)
        forward = [matrix_to_quaternion(r) for r in (now, reference)]
        inverse = [matrix_to_quaternion(r.T) for r in (now, reference)]
        for mode in ['full', 'roll_only']:
            a = mode_compensation_matrix(*forward, mounting, mode=mode)
            b = mode_compensation_matrix(*inverse, mounting, convention='world_to_imu', mode=mode)
            np.testing.assert_allclose(a, b, atol=1e-12)
        np.testing.assert_array_equal(mode_compensation_matrix(*forward, mounting, mode='full'),
                                      compensation_matrix(*forward, mounting))

    def test_normal_needs_no_imu_and_gimbal_degeneracy_rejects(self):
        np.testing.assert_array_equal(mode_compensation_matrix(mode='normal'), np.eye(3))
        with self.assertRaisesRegex(ValueError, 'undefined'):
            mode_compensation_matrix(matrix_to_quaternion(ry(math.pi / 2)), [1, 0, 0, 0],
                                     np.eye(3), mode='roll_only')


class CropAndPersistenceTests(unittest.TestCase):
    def test_fixed_zoom_and_range_rejection(self):
        profile = FixedRollCrop(1.4, 20)
        for degrees in [-20, -5, 0, 7, 20]:
            self.assertEqual(profile.check_angle(math.radians(degrees)), 1.4)
        for angle in [math.radians(20.01), float('nan')]:
            with self.assertRaises(ValueError):
                profile.check_angle(angle)
        for kwargs in [dict(zoom=.9), dict(zoom=True), dict(max_abs_roll_deg=180.01), dict(max_abs_roll_deg=0)]:
            with self.assertRaises(ValueError):
                FixedRollCrop(**kwargs)

    def test_sweep_keeps_same_focal_words_and_labels_finite_evidence(self):
        words_seen = []
        def checker(words, allow_black):
            self.assertFalse(allow_black)
            words_seen.append(words)
            return {'invalid_pixels': 0, 'arithmetic_error_pixels': 0,
                    'fixed_point_checked': True, 'tile_budget': {'unsupported_tiles': []}}
        calibration = load_calibration()
        result = verify_crop_profile(calibration, FixedRollCrop(1.4, 12), step_degrees=5, checker=checker)
        self.assertEqual(len(words_seen), 7)
        self.assertEqual(len({tuple(w[6:8]) for w in words_seen}), 1)
        self.assertFalse(result['continuous_range_proven'])
        self.assertFalse(result['board_verified'])
        self.assertEqual(result['calibration_sha256'], calibration_digest(calibration))

    def test_failed_sample_is_not_certified(self):
        with self.assertRaises(ValueError):
            verify_crop_profile(load_calibration(), FixedRollCrop(), checker=lambda *a, **k: {'invalid_pixels': 1})

    def test_mounting_roundtrip_preserves_provenance_not_confirmation(self):
        mounting = {'R_IC': rx(.4).tolist(), 'convention': 'world_to_imu',
                    'measured_from': 'physical assembly test', 'confirmed': True}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mounting.json'
            save_mounting(path, mounting)
            restored = read_mounting(path)
            np.testing.assert_array_equal(restored['R_IC'], mounting['R_IC'])
            self.assertEqual(restored['measured_from'], mounting['measured_from'])
            self.assertNotIn('confirmed', restored)
            self.assertIn('confirmation_policy', restored)
            bad = dict(mounting, R_IC=np.diag([1, 1, -1]).tolist())
            with self.assertRaises(ValueError):
                save_mounting(Path(directory) / 'bad.json', bad)
            self.assertFalse((Path(directory) / 'bad.json').exists())

    def test_pose_photo_metadata_requires_actual_matching_capture(self):
        data = dict(capture_h=np.eye(3).tolist(), capture_context_hash='frame-context', capture_id='actual-frame-47')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            self.assertEqual(read_capture_metadata(path, 'frame-context'), data)
            with self.assertRaises(ValueError):
                read_capture_metadata(path, 'new-reference-context')
            for key in data:
                path.write_text(json.dumps({k: v for k, v in data.items() if k != key}), encoding='utf-8')
                with self.assertRaises(ValueError):
                    read_capture_metadata(path, 'frame-context')


if __name__ == '__main__':
    unittest.main()
