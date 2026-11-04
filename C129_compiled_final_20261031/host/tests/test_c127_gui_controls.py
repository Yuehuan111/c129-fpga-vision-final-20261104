"""Hidden Tk controls with fake session; never enumerates or opens hardware."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_app import ControlPanel, scan_status_text


class GuiControlsTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        try:
            self.window = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.window.withdraw()
        with patch.object(ControlPanel, 'refresh_ports', lambda panel: None):
            self.panel = ControlPanel(self.window)
        self.panel.session = Mock()
        self.window.update_idletasks()

    def tearDown(self):
        if hasattr(self, 'panel'):
            self.panel.closing = True
            self.window.destroy()

    def test_defaults_are_roll_only_without_implicit_send(self):
        self.assertEqual(self.panel.pose_mode.get(), 'roll_only')
        self.assertFalse(self.panel.crop_enabled.get())
        self.panel.session.assert_not_called()
        self.assertLessEqual(self.window.winfo_reqheight(), 720)

    def test_apply_only_sets_mode_and_invalidates_local_profile(self):
        self.panel.crop_enabled.set(True)
        self.panel.crop_zoom.set('1.4')
        self.panel.crop_range.set('20')
        self.panel.scan_profile = {'old': 'profile'}
        self.panel.apply_pose_mode()
        self.panel.session.set_pose_mode.assert_called_once_with('roll_only', crop_profile={'zoom': 1.4, 'max_abs_roll_deg': 20.0})
        self.panel.session.request_normal.assert_not_called()
        self.panel.session.request_view_mode.assert_not_called()
        self.assertIsNone(self.panel.scan_profile)

    def test_normal_is_an_explicit_request_and_turns_off_crop(self):
        self.panel.crop_enabled.set(True)
        self.panel.send_normal()
        self.panel.session.set_pose_mode.assert_called_once_with('normal', crop_profile=None)
        self.panel.session.request_normal.assert_called_once_with()
        self.assertFalse(self.panel.crop_enabled.get())

    def test_scan_mirror_request_is_separate_from_profile_upload(self):
        self.panel.scan_enabled.set(True)
        self.panel.mirror_enabled.set(False)
        self.panel.send_view_mode()
        self.panel.session.request_view_mode.assert_called_once_with(scan=True, mirror=False)
        self.panel.session.set_scan_profile.assert_not_called()

    def test_wrong_crop_mode_rejects_before_session_mutation(self):
        self.panel.crop_enabled.set(True)
        self.panel.pose_mode.set('full')
        with self.assertRaises(ValueError):
            self.panel.apply_pose_mode()
        self.panel.session.set_pose_mode.assert_not_called()

    def test_connect_with_loaded_mounting_is_receive_only(self):
        session = Mock()
        self.panel.session = None
        self.panel.mounting = {'R_IC': [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'convention': 'imu_to_world'}
        with patch.dict(sys.modules, {'c127_session': SimpleNamespace(C127HostSession=lambda **kwargs: session)}):
            self.panel.connect()
        session.start.assert_called_once_with(None, None)
        session.set_mounting.assert_not_called()
        session.set_pose_mode.assert_not_called()
        session.invalidate_scan_profile.assert_not_called()
        session.set_scan_profile.assert_not_called()
        session.request_normal.assert_not_called()
        session.request_view_mode.assert_not_called()
        self.assertFalse(self.panel.mount_confirmed.get())

    def test_requested_and_committed_never_claim_displayed_without_evidence(self):
        text = scan_status_text({'scan': {'requested_modes': 3, 'committed_modes': 3, 'displayed': None}})
        self.assertIn('已提交：扫描＋镜像', text)
        self.assertIn('已显示：待同版本输出证据', text)
        text = scan_status_text({'scan': {'requested_modes': 3, 'committed_modes': 3,
            'displayed': {'modes': 0, 'raw': True, 'version': 11, 'source_frame_id': 47}}})
        self.assertIn('已显示：几何原图（附加模式暂停），版本 11，源帧 47', text)

    def test_failed_mode_change_does_not_mark_applied_or_drop_profile(self):
        old = {'old': 'profile'}
        self.panel.scan_profile = old
        self.panel.session.set_pose_mode.side_effect = RuntimeError('in flight')
        self.panel.pose_mode.set('full')
        with self.assertRaises(RuntimeError):
            self.panel.apply_pose_mode()
        self.assertIs(self.panel.scan_profile, old)
        self.assertEqual(self.panel._applied_pose_settings, ('roll_only', None))

    def test_validation_from_old_profile_is_not_applied_to_new_profile(self):
        old = {'old': 'profile'}
        self.panel.scan_profile = {'new': 'profile'}
        self.panel.validation_busy = True
        self.panel.validation_results.put((old, {'modes': {}}, None))
        self.panel.consume_validation_result()
        self.assertFalse(self.panel.validation_busy)
        self.assertIsNone(self.panel.scan_validation)
        self.assertIn('旧检查结果未用于当前参数', self.panel.scan_note.get())

    def test_enhancement_only_sends_after_explicit_action(self):
        self.panel.session.request_enhancement.assert_not_called()
        self.panel.enhancement_mode.set('1 自动')
        self.panel.apply_enhancement()
        self.panel.session.request_enhancement.assert_called_once_with(1)


if __name__ == '__main__':
    unittest.main()
