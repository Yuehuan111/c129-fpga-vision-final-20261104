"""Offline synthetic algebra/point tests and stored-homography ROI regression.

No camera, serial port, PDS process, or downloadable coordinate map is used.
"""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))
import scan_plane as sp
from complete_geometry import configuration, load_calibration, pose_matrix


class ScanPlaneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cal = load_calibration(HOST/'calibration/C97_joint_720p/calibration.json')
        cls.context = sp.make_context(cls.cal, configuration(cls.cal)[:18])

    def test_mirror_uses_actual_noncentred_k_and_nonzero_legacy(self):
        words = configuration(self.cal, legacy_angle=17, zoom=1.3)[:18]
        words[4:8] = [int(v*sp.Q) for v in (617.125, 343.25, 431.25, 397.5)]
        context = sp.make_context(self.cal, words)
        profile = sp.make_mirror_profile(context)
        composed = sp.compose_words(profile, pose_matrix(roll=9, yaw=-2), mirror=True)
        h = np.asarray(composed[18:]).reshape(3, 3)/sp.Q
        hb = np.asarray(configuration(self.cal, matrix=pose_matrix(roll=9, yaw=-2))[18:]).reshape(3, 3)/sp.Q
        k, l = sp._kl(words)
        points = np.array([[0, 0, 1], [1279, 719, 1], [639.5, 359.5, 1]]).T
        expected = hb @ l @ np.linalg.inv(k) @ sp.MIRROR @ points
        actual = h @ l @ np.linalg.inv(k) @ points
        np.testing.assert_allclose(actual[:2]/actual[2], expected[:2]/expected[2], atol=1e-4)
        self.assertLess(np.linalg.det(h), 0)
        self.assertEqual(len(profile['mode_words']), 36)
        self.assertFalse(profile['scan_valid'])
        with self.assertRaisesRegex(ValueError, 'No valid plane'):
            sp.compose_words(profile, scan=True)

    def test_mode00_exact_and_shared_power_of_two_normalization(self):
        profile = sp.make_mirror_profile(self.context)
        hb = pose_matrix(roll=7, pitch=2, yaw=-4)
        self.assertEqual(sp.compose_words(profile, hb)[18:], configuration(self.cal, matrix=hb)[18:])
        # An 8x homogeneous common scale is legal; the composed factor uses
        # a shared positive exponent and retains the exact mirror geometry.
        words = sp.compose_words(profile, 8*np.eye(3), mirror=True)
        self.assertEqual(words[18:], [-sp.Q, 0, 0, 0, sp.Q, 0, 0, 0, sp.Q])
        self.assertEqual(profile['mode_words'][:9], sp.IWORDS)

    def test_source_calibration_and_context_tampering_rejected(self):
        words = configuration(self.cal)[:18]
        words[8] += 1
        with self.assertRaisesRegex(ValueError, 'whole calibration'):
            sp.make_context(self.cal, words)
        changed = deepcopy(self.context)
        changed['output_k'][0][2] += 1
        with self.assertRaisesRegex(ValueError, 'inconsistent'):
            sp.validate_context(changed)
        with self.assertRaises(ValueError):
            sp.make_context(self.cal, configuration(self.cal)[:18], mounting=np.diag([-1, 1, 1]))

    def test_persistence_corruption_and_context_change(self):
        profile = sp.make_mirror_profile(self.context)
        with tempfile.TemporaryDirectory() as directory:
            path = sp.save_profile(profile, Path(directory)/'plane.json')
            self.assertEqual(sp.load_profile(path, expected_context=self.context), profile)
            other = sp.make_context(self.cal, configuration(self.cal, zoom=1.1)[:18])
            with self.assertRaisesRegex(ValueError, 'stale'):
                sp.load_profile(path, expected_context=other)
            data = json.loads(path.read_text())
            data['mode_words'][9] += 1
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'inconsistent'):
                sp.load_profile(path)

    def _known_raw_corners(self, context, cell=64):
        # Stored 50-corner raw fit from the supplied photo1 preview report.
        # This is mathematical regression data, not a new optical measurement.
        a = np.array([[.04143441038939842, -.003383331595408279, -.16066681631377394],
                      [.01591914055655164, .07145313210162207, -.3502884494706377],
                      [-.05903446385246993, .02026712967852193, 1.]])
        rays = sp._project(np.array([[-1, -1], [10, -1], [10, 5], [-1, 5]]), a)
        return sp._joint_project(rays, context)

    def test_raw720_and_1080_pixel_centres_produce_same_plane(self):
        points = self._known_raw_corners(self.context)
        p720 = sp.fit_four_points(points, 11/6, self.context)
        p1080 = sp.fit_four_points((points+.5)*1.5-.5, 11/6, self.context, input_size=(1920, 1080))
        np.testing.assert_allclose(p720['pixel_inverse_h'], p1080['pixel_inverse_h'], atol=1e-10)
        self.assertEqual(p720['mode_words'], p1080['mode_words'])
        self.assertLess(p720['fit']['joint_point_roundtrip_max_source_px'], 1e-5)

    def test_raw_capture_h_reference_binding_with_nonzero_legacy(self):
        words = configuration(self.cal, legacy_angle=13)[:18]
        context = sp.make_context(self.cal, words, reference=dict(reference_id=7, quaternion=[1, 0, 0, 0]))
        points = self._known_raw_corners(context)
        with self.assertRaisesRegex(ValueError, 'actual capture_h'):
            sp.fit_four_points(points, 11/6, context)
        h0 = pose_matrix(roll=9)
        profile = sp.fit_four_points(points, 11/6, context, capture_h=h0,
                                    capture_context_hash=context['context_hash'], capture_id='offline-raw-frame-7')
        p = np.asarray(profile['pixel_inverse_h'])
        k, l = sp._kl(words)
        quant_h0 = np.asarray(configuration(self.cal, matrix=h0)[18:]).reshape(3, 3)/sp.Q
        desired = np.asarray(profile['fit']['target_points'])
        rays = sp._project(desired, quant_h0 @ l @ np.linalg.inv(k) @ p)
        np.testing.assert_allclose(sp._joint_project(rays, context), points, atol=1e-7)

    def test_corrected_points_never_invert_lens_and_require_capture_metadata(self):
        points = [[350, 210], [980, 170], [1020, 560], [300, 590]]
        with self.assertRaisesRegex(ValueError, 'actual capture_h'):
            sp.fit_four_points(points, 11/6, self.context, input_domain='rectified')
        kwargs = dict(input_domain='rectified', capture_h=np.eye(3),
                      capture_context_hash=self.context['context_hash'], capture_id='displayed-frame-23')
        with patch.object(sp, '_joint_unproject', side_effect=AssertionError('double lens inversion')):
            profile = sp.fit_four_points(points, 11/6, self.context, **kwargs)
        np.testing.assert_allclose(sp._project(profile['fit']['target_points'], np.asarray(profile['pixel_inverse_h'])), points, atol=1e-8)
        kwargs['capture_context_hash'] = 'wrong'
        with self.assertRaises(ValueError):
            sp.fit_four_points(points, 11/6, self.context, **kwargs)

    def test_bad_quadrilateral_and_missing_physical_ratio_rejected(self):
        for points, aspect in [([[0, 0], [100, 0], [200, 0], [300, 0]], 1),
                               ([[0, 0], [100, 100], [100, 0], [0, 100]], 1),
                               ([[10, 10], [110, 10], [110, 110], [10, 110]], 0)]:
            with self.assertRaises(ValueError):
                sp.fit_four_points(points, aspect, self.context)

    def test_real_detector_on_synthetic_corrected_checkerboard(self):
        image = np.full((720, 1280, 3), 180, np.uint8)
        for row in range(6):
            for col in range(11):
                image[210+row*40:210+(row+1)*40, 420+col*40:420+(col+1)*40] = 255 if (row+col)%2 else 0
        profile = sp.fit_chessboard(image, self.context, input_domain='rectified',
                                   capture_h=np.eye(3), capture_context_hash=self.context['context_hash'],
                                   capture_id='synthetic-detector-test', pattern_size=(10, 5))
        self.assertEqual(profile['fit']['pattern_size'], [10, 5])
        self.assertEqual(len(profile['fit']['source_points_720']), 50)
        self.assertLess(profile['fit']['heldout_rms_target_pixels'], .2)
        self.assertEqual(profile['layout']['board_size'], [704, 384])

    def test_full_canvas_policy_accepts_black_domain_but_rejects_roi_overflow(self):
        points = self._known_raw_corners(self.context)
        p64 = sp.fit_four_points(points, 11/6, self.context, cell_pixels=64)
        report = sp.validate_modes(p64)
        self.assertTrue(all(m['admitted'] for m in report['modes'].values()))
        self.assertGreater(report['modes']['10']['domain_invalid_pixels'], 0)
        self.assertGreater(report['modes']['10']['invalid_pixels'], 0)
        p48 = sp.fit_four_points(points, 11/6, self.context, cell_pixels=48)
        roll_report = sp.validate_modes(p48, pose_matrix(roll=15))
        for mode in ('10', '11'):
            self.assertFalse(roll_report['modes'][mode]['admitted'])
            self.assertIn('source_roi_overflow', roll_report['modes'][mode]['reasons'])
            self.assertGreater(roll_report['modes'][mode]['tile_budget']['max_roi_height'], 64)


if __name__ == '__main__':
    unittest.main()
