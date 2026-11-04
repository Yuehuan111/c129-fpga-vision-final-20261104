from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

HOST = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(HOST), str(HOST / 'vendor')]
from complete_geometry import load_calibration
from demo_results import ModeResults
from roll_crop import DEMO_CROP_EVIDENCE, DEMO_ROLL_CROP, FixedRollCrop, checked_demo_crop


class CircularCropTests(unittest.TestCase):
    def test_full_circle_seam_and_multiple_turns(self):
        for degrees in [-1080, -540, -180.001, -180, -179.999, -90, 0, 60, 75, 90, 179.999, 180, 180.001, 540, 1080]:
            self.assertEqual(DEMO_ROLL_CROP.check_angle(math.radians(degrees)), 1280/608)
        for value in [True, math.inf, -math.inf, math.nan]:
            with self.assertRaises(ValueError): DEMO_ROLL_CROP.check_angle(value)

    def test_limited_crop_wraps_but_still_rejects_outside_reference_window(self):
        crop = FixedRollCrop(1.4, 20)
        for degrees in [-360, 360, 710, 370]:
            self.assertEqual(crop.check_angle(math.radians(degrees)), 1.4)
        for degrees in [21, 90, 180, 339]:
            with self.assertRaises(ValueError): crop.check_angle(math.radians(degrees))

    def test_packaged_evidence_is_dense_integer_and_not_board_claim(self):
        report = json.loads(DEMO_CROP_EVIDENCE.read_text())
        self.assertGreaterEqual(len(report['samples']), 849)
        self.assertFalse(report['board_verified'])
        self.assertFalse(report['continuous_range_proven'])
        self.assertEqual(checked_demo_crop(load_calibration()), DEMO_ROLL_CROP.to_dict())
        for sample in report['samples']:
            result = sample['geometry']
            self.assertTrue(result['fixed_point_checked'])
            self.assertEqual(result['invalid_pixels'], 0)
            self.assertEqual(result['arithmetic_error_pixels'], 0)
            self.assertFalse(result['tile_budget']['unsupported_tiles'])
        angles = {sample['roll_deg'] for sample in report['samples']}
        for angle in [60.25, 75.75, 89.75, -60.25, -75.75, -89.75, -180.000001, 180.000001]:
            self.assertIn(angle, angles)

    def test_changed_calibration_or_optics_cannot_reuse_preset_evidence(self):
        calibration = deepcopy(load_calibration())
        calibration['camera_matrix'][0][0] += .01
        with self.assertRaises(ValueError): checked_demo_crop(calibration)
        with self.assertRaises(ValueError): checked_demo_crop(load_calibration(), geometry_options={'gain': 1.1})


class ModeResultTests(unittest.TestCase):
    def test_input_copy_raw_enhance_separation_and_staleness(self):
        store = ModeResults(clock=lambda: 10.)
        source = np.zeros((4, 6, 3), np.uint8)
        packet = dict(frame=source, live=True, source_kind='camera', frame_id='1:2', captured_at=9.8)
        data = dict(last_status={'config_version': 4}, scan={'displayed': {'version': 4, 'source_frame_id': 12}})
        self.assertTrue(store.remember('raw', False, packet, data))
        source[:] = 255
        self.assertTrue(store.remember('raw', True, packet, data))
        original = store.snapshot('raw', False, current_mode='raw', live=True)
        enhanced = store.snapshot('raw', True, current_mode='raw', live=True)
        self.assertFalse(original['stale'])
        self.assertTrue(np.all(original['image'] == 0))
        self.assertTrue(np.all(enhanced['image'] == 255))
        self.assertIn('no same-frame', original['association'])
        self.assertTrue(store.snapshot('raw', False, current_mode='normal', live=True)['stale'])
        self.assertTrue(store.snapshot('raw', False, current_mode='raw', live=False)['stale'])
        self.assertFalse(store.remember('normal', False, dict(packet, source_kind='image'), data))
        with self.assertRaises(ValueError): store.remember('normal', True, packet, data)
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(store.export(directory), 2)
            index = json.loads((Path(directory) / 'index.json').read_text(encoding='utf-8'))
            self.assertFalse(index['same_frame_comparison'])
            self.assertEqual({item['image'] for item in index['results']}, {'raw_adaptive_0.png', 'raw_adaptive_1.png'})
            self.assertTrue(all(item['stale'] for item in index['results']))


if __name__ == '__main__': unittest.main()
