"""Hidden Tk checks for the demo surface, with hardware access forbidden."""
import sys
from pathlib import Path
import tempfile
from types import SimpleNamespace
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import demo_app
import unified_app


class DemoAppTests(unittest.TestCase):
    def test_demo_has_two_visible_pages_and_no_device_open(self):
        with tempfile.TemporaryDirectory() as temp, \
                patch.object(demo_app, 'ROOT', Path(temp)), \
                patch.object(unified_app, 'ROOT', Path(temp)), \
                patch('serial.Serial', side_effect=AssertionError('No serial access')), \
                patch('cv2.VideoCapture', side_effect=AssertionError('No camera access')), \
                patch('imu_session.list_serial_ports', side_effect=AssertionError('No enumeration')):
            root = tk.Tk(); root.withdraw()
            errors = []
            root.report_callback_exception = lambda *args: errors.append(args)
            app = demo_app.DemoControlPanel(root, offline=True)
            try:
                root.update()
                visible = [app.tabs.tab(p, 'text') for p in app.tabs.tabs() if app.tabs.tab(p, 'state') != 'hidden']
                self.assertEqual(visible, ['演示', '设备与参数'])
                self.assertEqual(set(app.demo_buttons), {'raw', 'normal', 'imu', 'scan', 'enhance'})
                self.assertEqual(app.pose_mode.get(), 'roll_only')
                self.assertTrue(app.crop_enabled.get())
                self.assertAlmostEqual(float(app.crop_zoom.get()), 1280 / 608)
                self.assertEqual(float(app.crop_range.get()), 180)
                self.assertEqual(str(app.adaptive_toggle['state']), 'disabled')
                self.assertFalse(app.detail_enabled.get())
                self.assertEqual(str(app.detail_toggle['state']), 'disabled')
                self.assertIn('同倍率静态对照', app.imu_comparison_button['text'])
                self.assertIs(app.actions, app.demo_controller.sequence)
                self.assertIsNone(app.session)
                app.demo_call('imu')
                self.assertIsNone(app.actions.pending)
                self.assertIsNone(app.video_panel.controller.process)
                self.assertTrue(all(str(button['state']) == 'disabled' for button in app.demo_buttons.values()))
                app.show_advanced(); root.update()
                self.assertTrue(app.advanced_visible)
                app.show_advanced(); root.update()
                self.assertEqual(app.tabs.select(), str(app.demo_page))
                self.assertEqual(errors, [])
            finally:
                app.close()


    def test_settings_controls_reachable_at_minimum_window_size(self):
        with tempfile.TemporaryDirectory() as temp, \
                patch.object(demo_app, 'ROOT', Path(temp)), \
                patch.object(unified_app, 'ROOT', Path(temp)), \
                patch('serial.Serial', side_effect=AssertionError('No serial access')), \
                patch('cv2.VideoCapture', side_effect=AssertionError('No camera access')), \
                patch('imu_session.list_serial_ports', side_effect=AssertionError('No enumeration')):
            root = tk.Tk(); root.withdraw()
            errors = []
            root.report_callback_exception = lambda *args: errors.append(args)
            app = demo_app.DemoControlPanel(root, offline=True)
            try:
                app.tabs.select(app.settings_page)
                widgets = []
                def collect(widget):
                    widgets.append(widget)
                    for child in widget.winfo_children():
                        collect(child)
                collect(app.settings_body)
                for width, height in ((1000, 700),):
                    with self.subTest(window=(width, height)):
                        root.geometry(f'{width}x{height}')
                        # A withdrawn root does not arrange packed children.
                        # Explicit notebook bounds reserve 200 px for app chrome,
                        # without creating or focusing a desktop window.
                        app.tabs.place(x=0, y=0, width=width, height=height - 200)
                        root.update()
                        canvas = app.settings_canvas
                        self.assertEqual(app.settings_body.winfo_width(), canvas.winfo_width())
                        self.assertEqual(app.settings_scrollbar.winfo_height(), canvas.winfo_height())
                        self.assertGreater(app.settings_body.winfo_height(), canvas.winfo_height())
                        for widget in widgets:
                            left = widget.winfo_rootx() - app.settings_body.winfo_rootx()
                            self.assertGreaterEqual(left, 0, str(widget))
                            self.assertLessEqual(left + widget.winfo_width(), canvas.winfo_width(), str(widget))
                            if widget.winfo_class() in ('TButton', 'TCheckbutton', 'TCombobox', 'TSpinbox'):
                                canvas.yview_moveto(0)
                                app.reveal_setting(SimpleNamespace(widget=widget))
                                top = widget.winfo_rooty() - app.settings_body.winfo_rooty()
                                self.assertGreaterEqual(top, canvas.canvasy(0), str(widget))
                                self.assertLessEqual(top + widget.winfo_height(),
                                                     canvas.canvasy(canvas.winfo_height()), str(widget))
                        canvas.yview_moveto(0)
                        self.assertEqual(app.scroll_settings(SimpleNamespace(delta=-120)), 'break')
                        self.assertGreater(canvas.yview()[0], 0)
                        canvas.yview_moveto(1)
                        self.assertGreaterEqual(canvas.yview()[1], .999)
                if hasattr(app, 'detail_toggle'):
                    # Geometry and the existing action remain usable; no serial
                    # request is allowed from this hidden layout check.
                    with patch.object(app, 'demo_call') as call:
                        app.detail_toggle.state(['!disabled'])
                        app.detail_toggle.invoke()
                        call.assert_called_once_with('detail', True)
                self.assertEqual(root.state(), 'withdrawn')
                self.assertEqual(errors, [])
            finally:
                app.close()

if __name__ == '__main__': unittest.main()
