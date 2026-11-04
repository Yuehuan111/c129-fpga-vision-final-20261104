"""Hidden-Tk integration checks for the unified host; no device may be opened."""
from contextlib import contextmanager, ExitStack
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch
import numpy as np

HOST = Path(__file__).resolve().parents[1]
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))
import unified_app


def parameter_hashes():
    files = sorted((HOST/'calibration').rglob('*')) + [HOST/'mounting.example.json']
    return {str(path.relative_to(HOST)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.is_file()}


class UnifiedStatusTests(unittest.TestCase):
    def test_request_or_commit_is_not_confirmed_display(self):
        _, _, board, output = unified_app.format_live_status({
            'state': 'await_output', 'last_status': {'config_version': 42},
            'scan': {'requested_modes': 3, 'committed_modes': 3,
                     'pending': {'version': 42}, 'displayed': None}})
        self.assertIn('42', board)
        self.assertEqual(output, '等待同版本输出确认')
        self.assertNotIn('已确认版本', output)

    def test_display_uses_output_evidence_version_and_source_frame(self):
        output = unified_app.format_live_status({
            'last_status': {'config_version': 43},
            'scan': {'requested_modes': 3, 'committed_modes': 3,
                     'displayed': {'version': 42, 'source_frame_id': 1001}}})[-1]
        self.assertIn('已确认版本 42', output)
        self.assertIn('源帧 1001', output)
        self.assertNotIn('43', output)

    def test_disconnected_status_does_not_claim_output(self):
        self.assertEqual(unified_app.format_live_status(None),
                         ('设备未连接', '等待 IMU', '等待板端状态', '尚无已确认输出'))


class UnifiedAppTkTests(unittest.TestCase):
    @contextmanager
    def app(self, temporary_root, *, offline=True):
        try:
            window = tk.Tk()
        except tk.TclError as error:
            self.skipTest('Tk display unavailable: ' + str(error))
        window.withdraw()
        errors = []
        window.report_callback_exception = lambda kind, value, trace: errors.append((kind, value))
        instance = None
        with ExitStack() as stack:
            # Keep settings reads/writes in this test's temporary directory.
            stack.enter_context(patch.object(unified_app, 'ROOT', Path(temporary_root)))
            serial = stack.enter_context(patch('serial.Serial', side_effect=AssertionError('unexpected serial open')))
            capture = stack.enter_context(patch('cv2.VideoCapture', side_effect=AssertionError('unexpected video open')))
            session = stack.enter_context(patch('c127_session.C127HostSession', side_effect=AssertionError('unexpected session creation')))
            ports = stack.enter_context(patch('imu_session.list_serial_ports', return_value=[]))
            before = parameter_hashes()
            try:
                instance = unified_app.UnifiedControlPanel(window, offline=offline)
                end = time.monotonic()+.35
                while time.monotonic() < end:
                    window.update()
                    time.sleep(.01)
                yield instance, window, ports
            finally:
                if instance is not None:
                    instance.close()
                else:
                    window.destroy()
                # Tcl survives destruction of its root widget. Leftover timers can
                # call deleted Python/Tk commands when another window is created.
                pending = tuple(window.tk.call('after', 'info'))
                for identifier in pending:
                    window.tk.call('after', 'cancel', identifier)
                if instance is not None:
                    self.assertEqual(pending, (), 'app close left pending Tk timers')
                self.assertEqual(errors, [], 'Tk callback error during startup/polling')
                serial.assert_not_called()
                capture.assert_not_called()
                session.assert_not_called()
                self.assertEqual(parameter_hashes(), before, 'bundled calibration or mounting parameters changed')

    def test_offline_startup_seven_tabs_and_close_without_device_access(self):
        with tempfile.TemporaryDirectory(prefix='unified_app_test_') as folder:
            with self.app(folder, offline=True) as (app, window, ports):
                names = [app.tabs.tab(item, 'text') for item in app.tabs.tabs()]
                self.assertEqual(names, ['演示预览', '设备与 IMU', '模式、扫描与镜像',
                                         '离线几何验证', '标定与测试图卡', '板端状态与证据', '日志与安装标定'])
                self.assertEqual(window.state(), 'withdrawn')
                self.assertIsNone(app.session)
                self.assertFalse(app.mount_confirmed.get())
                self.assertFalse(app.video_panel.allow_devices)
                self.assertFalse(app.diagnostics_panel.allow_devices)
                self.assertEqual(str(app.video_panel.open_device_button['state']), 'disabled')
                self.assertEqual(app.connection_badge.get(), '离线文件模式')
                ports.assert_not_called()
                with self.assertRaisesRegex(RuntimeError, '离线'):
                    app.connect()
            preferences = json.loads((Path(folder)/'settings/unified_preferences.json').read_text(encoding='utf-8'))
            self.assertFalse(preferences['auto_connect'])
            self.assertFalse(preferences['auto_apply_mounting'])

    def test_normal_entry_enumerates_only_and_still_never_connects(self):
        with tempfile.TemporaryDirectory(prefix='unified_app_test_') as folder:
            with self.app(folder, offline=False) as (app, _, ports):
                ports.assert_called_once_with()
                self.assertIsNone(app.session)
                self.assertTrue(app.video_panel.allow_devices)
                self.assertFalse(app.mount_confirmed.get())

    def test_mounting_path_persists_but_confirmation_and_session_do_not(self):
        with tempfile.TemporaryDirectory(prefix='unified_app_test_') as folder:
            folder = Path(folder)
            mounting_path = folder/'installation.json'
            mounting = {'R_IC': [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                        'convention': 'imu_to_world', 'confirmed': True}
            mounting_path.write_text(json.dumps(mounting), encoding='utf-8')
            original_bytes = mounting_path.read_bytes()
            with self.app(folder) as (app, _, _):
                app.mount_confirmed.set(True)
                with patch.object(unified_app.filedialog, 'askopenfilename', return_value=str(mounting_path)):
                    app.load_mounting()
                self.assertEqual(Path(app.last_mounting_path), mounting_path.resolve())
                self.assertFalse(app.mount_confirmed.get())
                self.assertIsNone(app.session)
            preferences = json.loads((folder/'settings/unified_preferences.json').read_text(encoding='utf-8'))
            self.assertEqual(Path(preferences['last_mounting_path']), mounting_path.resolve())
            self.assertFalse(preferences['auto_connect'])
            self.assertFalse(preferences['auto_apply_mounting'])
            self.assertNotIn('confirmed', preferences)
            with self.app(folder) as (restored, _, _):
                self.assertEqual(restored.mounting['R_IC'], mounting['R_IC'])
                self.assertEqual(Path(restored.last_mounting_path), mounting_path.resolve())
                self.assertFalse(restored.mount_confirmed.get())
                self.assertIsNone(restored.session)
                self.assertIn('仍需核对确认', restored.mount_note.get())
            self.assertEqual(mounting_path.read_bytes(), original_bytes)

    def test_preview_frame_handoff_accepts_supported_size_and_rejects_wrong_size(self):
        with tempfile.TemporaryDirectory(prefix='unified_app_test_') as folder:
            with self.app(folder) as (app, _, _):
                with patch.object(app.video_panel, 'get_frame_copy', return_value=None):
                    with self.assertRaises(ValueError):
                        app.analyze_preview()
                frame = np.full((720, 1280, 3), 115, dtype=np.uint8)
                with patch.object(app.video_panel, 'get_frame_copy', return_value=frame):
                    app.analyze_preview()
                source = app.geometry_panel._source
                self.assertTrue(source.is_file())
                self.assertEqual(source.parent, Path(folder)/'results'/'preview_frames')
                self.assertEqual(app.tabs.select(), str(app.geometry_panel))
                files_before = list(source.parent.iterdir())
                # Never silently keep the previous geometry source after rejecting
                # a differently sized live/file frame.
                with patch.object(app.video_panel, 'get_frame_copy', return_value=np.zeros((480, 640, 3), np.uint8)), \
                     patch('unified_geometry_panel.messagebox.showerror'):
                    with self.assertRaises(ValueError):
                        app.analyze_preview()
                self.assertEqual(app.geometry_panel._source, source)
                self.assertEqual(list(source.parent.iterdir()), files_before)


if __name__ == '__main__':
    unittest.main()
