"""Portable image/geometry checks; optional real files via C127_TEST_PHOTO_DIR.

No serial transport, camera capture or FPGA build is used by these tests.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

HOST = Path(__file__).resolve().parents[1]
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))
import unified_geometry as g


def write_image(path, image):
    ok, encoded = cv2.imencode('.png', image)
    assert ok
    path.write_bytes(encoded.tobytes())


def make_board(perspective=True):
    """Eleven by six squares: ten by five inner corners, not corner labels."""
    image = np.full((720, 1280, 3), 210, np.uint8)
    cell, left, top = 64, 288, 168
    for row in range(6):
        for column in range(11):
            color = 0 if (row+column)%2 == 0 else 255
            image[top+row*cell:top+(row+1)*cell,
                  left+column*cell:left+(column+1)*cell] = color
    if perspective:
        original = np.array([[left, top], [left+704, top], [left+704, top+384], [left, top+384]], np.float32)
        target = np.array([[390, 170], [1010, 130], [900, 600], [420, 520]], np.float32)
        transform = cv2.getPerspectiveTransform(original, target)
        image = cv2.warpPerspective(image, transform, (1280, 720), borderValue=(210, 210, 210))
    return image


class GeometryPortableTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='unified_geometry_test_')
        cls.folder = Path(cls.temp.name)
        cls.path = cls.folder/'棋盘.png'
        cls.image = make_board()
        write_image(cls.path, cls.image)
        cls.result = g.process_image(cls.path, g.GeometryOptions(input_domain='rectified'))

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_perspective_grid_redetected_in_actual_raster(self):
        report = self.result.report
        self.assertEqual(report['redetected_corner_count'], 50)
        metrics = report['after_metrics']
        self.assertLess(abs(metrics['horizontal_step_mean_px']-64), .3)
        self.assertLess(abs(metrics['vertical_step_mean_px']-64), .3)
        self.assertLess(metrics['horizontal_step_cv_percent'], 1.0)
        self.assertLess(metrics['vertical_step_cv_percent'], 1.0)
        self.assertGreater(report['before_metrics']['horizontal_step_cv_percent'], 5)
        self.assertLess(report['heldout_rms_target_px'], 1)
        self.assertTrue(report['preview_only'])
        self.assertFalse(report['hardware_accessed'])
        self.assertFalse(report['output_geometry']['hardware_matrix_exported'])

    def test_full_canvas_surroundings_and_exact_mirror(self):
        for image in self.result.images.values():
            self.assertEqual(image.shape, (720, 1280, 3))
        self.assertTrue(np.array_equal(self.result.images['mirror'], self.result.images['output'][:, ::-1]))
        # Whole output includes both the central board and transformed gray surroundings.
        self.assertGreater(np.count_nonzero(self.result.images['output'][:, :230]), 10000)

    def test_rectified_input_never_runs_lens_mapper(self):
        with patch.object(g, '_fixed_render', side_effect=AssertionError('double correction')):
            result = g.process_image(self.path, g.GeometryOptions(input_domain='rectified', operation='lens'))
        self.assertTrue(np.array_equal(result.images['output'], self.image))
        self.assertEqual(result.report['lens_geometry']['operation'], 'identity')

    def test_roll_360_identity_and_candidate_limit_unchanged(self):
        result = g.process_image(self.path, g.GeometryOptions(input_domain='rectified', operation='roll', roll_degrees=360))
        self.assertTrue(np.array_equal(result.images['output'], self.image))
        self.assertFalse(result.report['production_roll_limit_changed'])

    def test_unsupported_source_shape_rejected(self):
        path = self.folder/'wrong_shape.png'
        write_image(path, np.zeros((240, 320, 3), np.uint8))
        with self.assertRaises(ValueError):
            g.read_image(path)

    def test_failures_are_not_reported_as_perfect_grid(self):
        real_detector = g.detect_corners
        count = 0
        def fake_after_only(*args, **kwargs):
            nonlocal count
            count += 1
            if count > 1:
                raise ValueError('output genuinely not detected')
            return real_detector(*args, **kwargs)
        with patch.object(g, 'detect_corners', side_effect=fake_after_only):
            result = g.process_image(self.path, g.GeometryOptions(input_domain='rectified'))
        self.assertEqual(result.report['redetected_corner_count'], 0)
        self.assertIsNone(result.report['after_metrics'])
        self.assertIn('output genuinely', result.report['redetection_warning'])

    def test_save_is_explicit_new_folder_with_report(self):
        before = hashlib.sha256(self.path.read_bytes()).hexdigest()
        first, second = g.save_result(self.result, self.folder), g.save_result(self.result, self.folder)
        self.assertNotEqual(first, second)
        self.assertEqual({p.name for p in first.iterdir()}, {'input.png', 'lens.png', 'output.png', 'mirror.png', 'report.json'})
        report = json.loads((first/'report.json').read_text(encoding='utf-8'))
        self.assertEqual(report['source_sha256'], before)
        self.assertEqual(hashlib.sha256(self.path.read_bytes()).hexdigest(), before)
        self.assertEqual(report['redetected_corner_count'], 50)

    def test_generation_rejects_old_source_settings_and_close(self):
        guard = g.ResultGeneration()
        first = guard.next()
        self.assertTrue(guard.accepts(first))
        second = guard.next()
        self.assertFalse(guard.accepts(first))
        self.assertTrue(guard.accepts(second))
        guard.close()
        self.assertFalse(guard.accepts(second))
        with self.assertRaises(RuntimeError):
            guard.next()

    def test_cancelled_job_does_not_read_source(self):
        with patch.object(g, 'read_image', side_effect=AssertionError('must not read')):
            with self.assertRaises(g.GeometryCancelled):
                g.process_image('missing.png', cancelled=lambda: True)

    def test_invalid_options_rejected(self):
        for params in ({'input_domain': 'guess'}, {'columns': True}, {'rows': 1},
                       {'cell_pixels': float('nan')}, {'columns': 20, 'cell_pixels': 90},
                       {'roll_degrees': float('inf')}, {'zoom': .5}):
            with self.subTest(params=params), self.assertRaises(ValueError):
                g.GeometryOptions(**params).validate()

    def test_hidden_tk_background_result_and_stale_generation(self):
        import tkinter as tk
        from unified_geometry_panel import GeometryPanel
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest('Tk display unavailable: ' + str(error))
        root.withdraw()
        panel = None
        try:
            panel = GeometryPanel(root)
            panel.pack(fill='both', expand=True)
            panel.domain.set('已去镜头畸变图')
            panel.load_image(self.path)
            panel.run()
            limit = time.monotonic()+10
            while panel._result is None and time.monotonic() < limit:
                root.update()
                time.sleep(.01)
            self.assertIsNotNone(panel._result, panel.status.get())
            self.assertEqual(panel._result.report['redetected_corner_count'], 50)
            panel.toggle_compare()
            self.assertEqual(panel.view.get(), '输入原图')
            old = panel.guard.generation
            panel.cell.set('63')
            self.assertIsNone(panel._result)
            panel._results.put((old, self.result, None))
            limit = time.monotonic()+2
            while not panel._results.empty() and time.monotonic() < limit:
                root.update()
                time.sleep(.01)
            self.assertIsNone(panel._result, 'old source/settings result was incorrectly adopted')
            panel.run()
            panel.close()
            self.assertTrue(panel.guard.closed)
        finally:
            if panel is not None:
                panel.close()
            root.destroy()


@unittest.skipUnless(os.environ.get('C127_TEST_PHOTO_DIR'), 'Optional real photos not configured')
class GeometryActualPhotoTests(unittest.TestCase):
    def test_two_original_photos(self):
        folder = Path(os.environ['C127_TEST_PHOTO_DIR'])
        cases = [('WIN_20261023_20_45_38_Pro.jpg', 'raw'),
                 ('WIN_20261023_20_45_58_Pro.jpg', 'rectified')]
        for name, domain in cases:
            with self.subTest(photo=name, domain=domain):
                result = g.process_image(folder/name, g.GeometryOptions(input_domain=domain))
                self.assertEqual(result.report['redetected_corner_count'], 50)
                metric = result.report['after_metrics']
                self.assertLess(abs(metric['horizontal_step_mean_px']-64), 1)
                self.assertLess(abs(metric['vertical_step_mean_px']-64), 1)
                self.assertLess(metric['horizontal_step_cv_percent'], 4)
                self.assertLess(metric['vertical_step_cv_percent'], 3)
                self.assertLess(metric['mean_absolute_right_angle_error_deg'], 3)
                self.assertEqual(result.images['output'].shape, (720, 1280, 3))
                if domain == 'raw':
                    self.assertIn('tile_budget', result.report['output_geometry'])
                else:
                    self.assertEqual(result.report['lens_geometry']['operation'], 'identity')
                print(name, json.dumps(metric, ensure_ascii=True))


if __name__ == '__main__':
    unittest.main()
