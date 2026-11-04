"""Local MP4 round-trips and hidden UI lifecycle checks; no hardware access."""
from pathlib import Path
import json
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from preview_recording import PreviewRecorder
import demo_app
import unified_app


class RecordingTests(unittest.TestCase):
    def test_real_mp4_preserves_duration_rotation_size_and_source(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / '中文录像' / 'preview.mp4'
            frame = np.full((96, 128, 3), (40, 70, 90), np.uint8)
            frame[:32, :40] = (250, 10, 10)
            frame[-32:, -40:] = (10, 10, 250)
            original = frame.copy()
            recorder = PreviewRecorder(path, frame, rotate=True, fps=20)
            time.sleep(.32)
            recorder.request_stop()
            self.assertTrue(recorder.done.wait(5))
            self.assertIsNone(recorder.error)
            self.assertTrue(np.array_equal(frame, original))
            capture = cv2.VideoCapture(str(path))
            try:
                self.assertTrue(capture.isOpened())
                decoded = []
                while True:
                    ok, item = capture.read()
                    if not ok:
                        break
                    decoded.append(item)
            finally:
                capture.release()
            self.assertEqual(len(decoded), recorder.written_frames)
            self.assertEqual(decoded[0].shape, frame.shape)
            self.assertLessEqual(abs(len(decoded) / 20 - recorder.elapsed), .075)
            self.assertGreater(float(decoded[0][8:24, 8:24, 2].mean()), 200)
            self.assertLess(float(decoded[0][8:24, 8:24, 0].mean()), 40)
            report = json.loads(path.with_suffix('.json').read_text(encoding='utf-8'))
            self.assertEqual(report['status'], 'saved')
            self.assertEqual(report['preview_submissions'], 1)
            self.assertFalse(report['audio'])

    def test_existing_filename_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'existing.mp4'
            path.write_bytes(b'existing video')
            with self.assertRaises(FileExistsError):
                PreviewRecorder(path, np.zeros((48, 64, 3), np.uint8))
            self.assertEqual(path.read_bytes(), b'existing video')

    def test_encoder_open_failure_is_reported_and_empty_mp4_removed(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'failure.mp4'
            writer = Mock()
            writer.isOpened.return_value = False
            with patch.object(cv2, 'VideoWriter', return_value=writer):
                recorder = PreviewRecorder(path, np.zeros((48, 64, 3), np.uint8))
                self.assertTrue(recorder.done.wait(5))
            self.assertIsNotNone(recorder.error)
            self.assertFalse(path.exists())
            writer.release.assert_called_once()

    def test_resolution_change_rejected_while_original_clip_stays_valid(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'same_size.mp4'
            recorder = PreviewRecorder(path, np.zeros((48, 64, 3), np.uint8))
            try:
                with self.assertRaises(ValueError):
                    recorder.submit(np.zeros((96, 128, 3), np.uint8))
            finally:
                recorder.request_stop()
                self.assertTrue(recorder.done.wait(5))
            self.assertIsNone(recorder.error)


class RecordingUiTests(unittest.TestCase):
    def test_source_change_and_close_finalize_recordings_without_devices(self):
        with tempfile.TemporaryDirectory() as temp, \
                patch.object(demo_app, 'ROOT', Path(temp)), \
                patch.object(unified_app, 'ROOT', Path(temp)), \
                patch('serial.Serial', side_effect=AssertionError('No serial access')), \
                patch('cv2.VideoCapture', side_effect=AssertionError('No camera access')), \
                patch('imu_session.list_serial_ports', side_effect=AssertionError('No enumeration')):
            root = tk.Tk()
            root.withdraw()
            errors = []
            root.report_callback_exception = lambda *args: errors.append(args)
            app = demo_app.DemoControlPanel(root, offline=True)
            try:
                with self.assertRaises(ValueError):
                    app.toggle_preview_recording()
                app.video_panel._frame = np.full((720, 1280, 3), 110, np.uint8)
                app.video_panel._frame_metadata = {
                    'captured_at': time.monotonic(), 'frame_id': '0:1',
                    'generation': 0, 'source_kind': 'video', 'sequence': 1}
                app.video_panel._live = True
                app.record_button.invoke()
                first = app.preview_recorder
                self.assertIsNotNone(first)
                self.assertTrue(root.bind_all('<F9>'))
                for width, height in ((1000, 700), (1360, 860)):
                    root.geometry(f'{width}x{height}')
                    app.tabs.place(x=0, y=0, width=width, height=height - 200)
                    root.update()
                    for button in (app.photo_button, app.photo_folder_button,
                                   app.record_button, app.video_folder_button):
                        left = button.winfo_rootx() - app.demo_page.winfo_rootx()
                        top = button.winfo_rooty() - app.demo_page.winfo_rooty()
                        self.assertGreaterEqual(left, 0)
                        self.assertLessEqual(left + button.winfo_width(), app.demo_page.winfo_width())
                        self.assertGreaterEqual(top, 0)
                        self.assertLessEqual(top + button.winfo_height(), app.demo_page.winfo_height())
                app.photo_button.invoke()
                self.assertEqual(len(list((Path(temp) / 'results/photos').glob('*.png'))), 1)
                app.video_panel._frame_metadata.update(generation=1, frame_id='1:1', captured_at=time.monotonic())
                app.pump_recording()
                self.assertTrue(first.stopping)
                self.assertTrue(first.done.wait(5))
                app.pump_recording()
                self.assertIsNone(app.preview_recorder)
                self.assertEqual(str(app.record_button['state']), 'normal')
                app.video_panel._frame_metadata['captured_at'] = time.monotonic()
                app.toggle_recording()
                second = app.preview_recorder
                self.assertIsNotNone(second)
                app.close()
                self.assertFalse(app.closing)
                self.assertTrue(second.done.wait(5))
                app.pump_recording()
                self.assertTrue(app.closing)
                self.assertIsNone(second.error)
                self.assertGreater(second.path.stat().st_size, 44)
                self.assertEqual(errors, [])
            finally:
                if not app.closing:
                    if app.preview_recorder:
                        app.preview_recorder.request_stop()
                        app.preview_recorder.done.wait(5)
                        app.pump_recording()
                    if not app.closing:
                        app.close()


if __name__ == '__main__':
    unittest.main()
