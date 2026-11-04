"""Stateful C128 target/ACK/output ordering tests; no device access."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from demo_controls import DemoController, display_status, independent_status
from roll_crop import DEMO_ROLL_CROP


class Session:
    def __init__(self):
        self._lock = threading.RLock()
        self._epoch = self._view_epoch = 0
        self._closed = self._frozen = self._view_dirty = False
        self._active = self._candidate = self._view_command = None
        self._pending_outputs = []
        self._shutdown = threading.Event()
        self.data = dict(connected={'fpga': True, 'imu': True}, pose_mode='normal', continuous=False,
            static_configuration_owned=True, mounting_confirmed=True, reference=None, imu={'stale': False},
            fpga={'status_age_s': 0}, enhancement={'enhancement_mode': 0},
            detail={'capable': True, 'active': False},
            last_status={'raw_active': False, 'committed_modes': 0, 'requested_modes': 0,
                         'config_version': 1, 'observation_active': False},
            scan={'displayed': {'version': 1, 'source_frame_id': 1, 'output_first_us': 1}},
            version_evidence={'output_confirmed': 0}, visual_scan={'enabled': False})
        self.calls, self.awaiting_output_kind = [], None

    def snapshot(self): return deepcopy(self.data)
    def stop(self):
        self._epoch += 1
        self.data['continuous'] = False
        self.data['visual_scan']['enabled'] = False
        self.calls.append(('stop',))
    def request_output_route(self, **kw):
        self.calls.append(('route', kw))
        board = self.data['last_status']
        if (board['raw_active'], board['observation_active']) != (kw['raw'], kw['observation']):
            self._view_command = dict(kind='route', **kw)
    def request_view_mode(self, **kw):
        self.calls.append(('view', kw))
        self._view_command = dict(kind='view', **kw)
    def request_normal(self):
        self.calls.append(('normal',))
        self.data['pose_mode'] = 'normal'
        self.data['crop_profile'] = None
        self._active = {'kind': 'normal'}
    def set_pose_mode(self, mode, **kw):
        self.calls.append(('pose', mode, kw))
        self.data.update(pose_mode=mode, **kw)
        self._view_epoch += 1
    def request_static_comparison(self):
        self.calls.append(('static_comparison',))
        self._active = {'kind': 'static_comparison'}
    def capture_reference(self):
        self.calls.append(('reference',))
        self.data['reference'] = {'id': 1}
        self._epoch += 1
    def start_realtime(self, *, auto_recover_geometry=False):
        self.calls.append(('realtime',))
        self.data.update(continuous=True, auto_recover_geometry=auto_recover_geometry)
        self.awaiting_output_kind = 'imu'
    def start_visual_scan(self):
        self.calls.append(('visual',))
        self.data['visual_scan'].update(enabled=True, output_confirmed=False)
        self.awaiting_output_kind = 'visual'
    def request_enhancement(self, mode):
        self.calls.append(('enhancement', mode))
        self._view_command = {'kind': 'enhancement', 'mode': mode}
    def request_detail_enhancement(self, enabled):
        self.calls.append(('detail', enabled))
        if self.data['detail']['active'] != enabled:
            self._view_command = {'kind': 'detail', 'enabled': enabled}
    def finish(self, *, output=True):
        command = self._active or self._view_command
        if not command: return
        self._active = self._view_command = None
        board = self.data['last_status']
        kind = command['kind']
        if kind == 'detail':
            self.data['detail']['active'] = command['enabled']
            self.data['detail_control'] = dict(enabled=command['enabled'], output_confirmed=False)
        elif kind == 'enhancement':
            self.data['enhancement']['enhancement_mode'] = command['mode']
            self.data['enhancement_control'] = dict(mode=command['mode'], output_confirmed=False)
        else:
            board['config_version'] += 1
            if kind == 'route':
                board.update(raw_active=command['raw'], observation_active=command['observation'])
            elif kind == 'view':
                board['committed_modes'] = board['requested_modes'] = int(command['scan'])*2+int(command['mirror'])
            self.awaiting_output_kind = kind
        if output: self.emit_output()
    def emit_output(self):
        old = self.data['scan']['displayed']
        board = self.data['last_status']
        self.data['scan']['displayed'] = dict(version=board['config_version'], raw=board['raw_active'],
            detail_active=self.data['detail']['active'],
            observation=board['observation_active'], source_frame_id=old['source_frame_id']+1,
            output_first_us=old['output_first_us']+100)
        if self.awaiting_output_kind in ('normal', 'imu', 'visual', 'static_comparison'):
            self.data['version_evidence']['output_confirmed'] += 1
        if self.awaiting_output_kind == 'static_comparison':
            self.data['request'] = dict(request_kind='imu_static_comparison', phase='output_confirmed')
        if self.awaiting_output_kind == 'visual':
            self.data['visual_scan']['output_confirmed'] = True
        if self.data.get('enhancement_control'):
            self.data['enhancement_control']['output_confirmed'] = True
        if self.data.get('detail_control'):
            self.data['detail_control']['output_confirmed'] = True
        self.awaiting_output_kind = None


class DemoWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.session, self.notes = Session(), []
        self.clock = 100.
        self.control = DemoController(lambda: self.session, self.notes.append,
            imu_settings=lambda s: ('roll_only', DEMO_ROLL_CROP.to_dict()), clock=lambda: self.clock)

    def ticks(self, count=12):
        for _ in range(count): self.control.tick()

    def drain(self, *, output=True):
        for _ in range(30):
            self.control.tick()
            self.session.finish(output=output)
        self.control.tick()

    def test_normal_explicitly_disables_adaptive_and_clears_scan(self):
        self.session.data['last_status']['committed_modes'] = 2
        self.session.data['enhancement']['enhancement_mode'] = 1
        self.control.normal(); self.ticks()
        self.assertNotIn(('normal',), self.session.calls)
        self.drain()
        self.assertLess(self.session.calls.index(('view', {'scan': False, 'mirror': False})), self.session.calls.index(('normal',)))
        self.assertIn(('enhancement', 0), self.session.calls)
        self.assertEqual(self.control.confirmed_target, 'normal')
        self.assertFalse(self.control.adaptive_allowed)
        with self.assertRaisesRegex(RuntimeError, '始终关闭'): self.control.adaptive(True)

    def test_first_connection_establishes_owned_baseline_before_route(self):
        self.session.data['static_configuration_owned'] = False
        self.control.imu(); self.ticks()
        self.assertIn(('normal',), self.session.calls)
        self.assertFalse(any(call[0] == 'route' for call in self.session.calls))
        self.session.finish()
        self.session.data['static_configuration_owned'] = True
        self.drain()
        self.assertIn(('route', {'raw': False, 'observation': False}), self.session.calls)

    def test_ordinary_does_not_call_old_output_new_submission_success(self):
        self.control.normal(); self.ticks()
        self.session.emit_output()  # Another frame of the old map.
        self.ticks()
        self.assertIsNone(self.control.confirmed_target)
        self.assertTrue(self.control.pending)
        self.drain(output=False)
        self.assertIsNone(self.control.confirmed_target)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.confirmed_target, 'normal')

    def test_raw_is_actual_route_with_output_wait(self):
        self.control.raw(); self.drain(output=False)
        self.assertIn(('route', {'raw': True, 'observation': False}), self.session.calls)
        self.assertTrue(self.control.pending)
        self.assertIsNone(self.control.confirmed_target)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.confirmed_target, 'raw')
        self.assertFalse(any('KEY1' in note for note in self.notes))

    def test_raw_to_normal_explicitly_restores_corrected_and_no_crop(self):
        self.session.data['last_status']['raw_active'] = True
        self.session.data['crop_profile'] = DEMO_ROLL_CROP.to_dict()
        self.control.normal(); self.drain()
        self.assertFalse(self.session.data['last_status']['raw_active'])
        self.assertIsNone(self.session.data['crop_profile'])
        self.assertEqual(self.control.confirmed_target, 'normal')

    def test_imu_uses_roll_crop_and_preserves_auto_recovery(self):
        self.control.imu(); self.drain()
        self.assertIn(('pose', 'roll_only', {'crop_profile': DEMO_ROLL_CROP.to_dict()}), self.session.calls)
        self.assertTrue(self.session.data['auto_recover_geometry'])
        self.assertTrue(self.control.pending)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.confirmed_target, 'imu')
        self.control.imu(); self.drain()
        self.assertEqual(self.session.calls.count(('reference',)), 1)

    def test_full_is_explicit_advanced_choice(self):
        self.control.imu_settings = lambda s: ('full', None)
        self.control.imu(); self.drain()
        self.assertIn(('pose', 'full', {'crop_profile': None}), self.session.calls)

    def test_mounting_or_stale_imu_never_mutates_board(self):
        self.session.data['mounting_confirmed'] = False
        with self.assertRaisesRegex(RuntimeError, '标定'): self.control.imu()
        self.session.data['mounting_confirmed'] = True
        self.session.data['imu']['stale'] = True
        with self.assertRaisesRegex(RuntimeError, 'IMU 数据'): self.control.imu()
        self.assertEqual(self.session.calls, [])

    def test_scan_uses_source_observation_and_visual_api_without_static_profile(self):
        self.control.scan(); self.drain()
        self.assertIn(('route', {'raw': False, 'observation': True}), self.session.calls)
        self.assertIn(('visual',), self.session.calls)
        self.assertFalse(any(c[0] == 'profile' for c in self.session.calls))
        self.assertTrue(self.control.pending)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.confirmed_target, 'scan')

    def test_scan_route_ack_does_not_start_detector_before_output(self):
        self.control.scan()
        for _ in range(20):
            self.control.tick()
            command = self.session._view_command
            if command and command.get('kind') == 'route' and command.get('observation'):
                self.session.finish(output=False)
                break
            self.session.finish()
        self.ticks()
        self.assertNotIn(('visual',), self.session.calls)
        self.session.emit_output(); self.ticks()
        self.assertIn(('visual',), self.session.calls)

    def test_external_key_route_updates_target_without_writes(self):
        self.control.normal(); self.drain()
        self.session.calls.clear()
        self.session.data['last_status'].update(raw_active=True, config_version=20)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.target, 'raw')
        self.assertTrue(self.control.adaptive_allowed)
        self.session.data['last_status'].update(raw_active=False, config_version=21)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.target, 'normal')
        self.assertFalse(self.control.adaptive_allowed)
        self.assertEqual(self.session.calls, [])

    def test_enhance_default_and_preferences_do_not_leak_to_normal(self):
        self.control.enhance(); self.drain()
        self.assertEqual(self.session.data['enhancement']['enhancement_mode'], 1)
        self.control.adaptive(False); self.drain()
        self.assertFalse(self.control.adaptive_preferences['enhance'])
        self.control.normal(); self.drain()
        self.assertEqual(self.session.data['enhancement']['enhancement_mode'], 0)
        self.control.enhance(); self.drain()
        self.assertEqual(self.session.data['enhancement']['enhancement_mode'], 0)

    def test_adaptive_changes_no_geometry_and_waits_for_post_ack_output(self):
        self.control.imu(); self.drain(); self.session.emit_output(); self.ticks()
        before = deepcopy(self.session.data['crop_profile'])
        self.session.calls.clear()
        self.control.adaptive(True); self.drain(output=False)
        self.assertEqual(self.session.data['crop_profile'], before)
        self.assertFalse(any(c[0] in ('normal', 'route', 'pose', 'reference', 'view') for c in self.session.calls))
        self.assertTrue(self.control.pending)
        self.session.emit_output(); self.ticks()
        self.assertEqual(self.control.confirmed_target, 'imu')

    def test_detail_is_optional_default_off_and_normal_gamma_remains_disabled(self):
        self.assertFalse(any(self.control.detail_preferences.values()))
        self.control.normal(); self.drain()
        self.session.calls.clear()
        self.control.detail(True); self.drain()
        self.assertEqual(self.control.confirmed_target, 'normal')
        self.assertTrue(self.session.data['detail']['active'])
        self.assertEqual(self.session.data['enhancement']['enhancement_mode'], 0)
        self.assertFalse(self.control.adaptive_allowed)
        self.assertFalse(any(c[0] in ('normal', 'route', 'pose', 'view', 'enhancement') for c in self.session.calls))
        self.control.raw(); self.drain()
        self.assertFalse(self.session.data['detail']['active'])
        self.assertFalse(self.control.detail_allowed)
        with self.assertRaisesRegex(RuntimeError, '原图'): self.control.detail(True)

    def test_detail_waits_for_output_before_resuming_IMU_and_keeps_crop(self):
        self.control.imu(); self.drain(); self.session.emit_output(); self.ticks()
        saved_crop = deepcopy(self.session.data['crop_profile'])
        self.session.calls.clear()
        self.control.detail(True); self.drain(output=False)
        self.assertNotIn(('realtime',), self.session.calls)
        self.assertTrue(self.control.pending)
        self.assertEqual(saved_crop, self.session.data['crop_profile'])
        self.session.emit_output(); self.ticks()
        self.assertIn(('realtime',), self.session.calls)
        self.assertEqual(self.control.confirmed_target, 'imu')

    def test_detail_preferences_do_not_leak_between_modes(self):
        self.control.normal(); self.drain()
        self.control.detail(True); self.drain()
        self.control.enhance(); self.drain()
        self.assertFalse(self.session.data['detail']['active'])
        self.control.normal(); self.drain()
        self.assertTrue(self.session.data['detail']['active'])
        self.assertEqual(self.session.data['enhancement']['enhancement_mode'], 0)

    def test_static_comparison_preserves_crop_gamma_detail_without_auto_resume(self):
        self.control.imu(); self.drain(); self.session.emit_output(); self.ticks()
        crop = deepcopy(self.session.data['crop_profile'])
        reference = deepcopy(self.session.data['reference'])
        self.control.compare_imu_static(); self.drain()
        self.assertTrue(self.control.imu_static_comparison)
        self.assertFalse(self.session.data['continuous'])
        self.assertIn('同倍率静态对照', display_status(self.session.data, target='imu')[0])
        self.session.calls.clear()
        self.control.detail(True); self.drain()
        self.control.adaptive(True); self.drain()
        self.assertNotIn(('realtime',), self.session.calls)
        self.assertEqual(self.session.data['crop_profile'], crop)
        self.assertEqual(self.session.data['reference'], reference)
        self.control.imu(); self.drain(); self.session.emit_output(); self.ticks()
        self.assertFalse(self.control.imu_static_comparison)
        self.assertTrue(self.session.data['continuous'])

    def test_geometry_hold_and_independent_disconnect_status(self):
        self.session.data['geometry_recovery'] = {'phase': 'waiting_pose'}
        self.assertIn('自动恢复', display_status(self.session.data)[0])
        text = independent_status(self.session.data, dict(has_frame=True, live=False, stale=True, source_kind='camera'))
        self.assertIn('取帧暂停', text)
        self.assertIn('串口已连接', text)
        self.assertIn('几何越界', text)
        self.session.data.update(frozen=True, error='UART lost')
        self.assertEqual(display_status(self.session.data)[1], 'UART lost')

    def test_stop_cancels_future_steps_and_timeout_is_not_success(self):
        self.control.scan(); self.ticks(); self.control.stop()
        self.drain()
        self.assertNotIn(('visual',), self.session.calls)
        self.control.normal(); self.drain(output=False)
        self.clock += 91; self.ticks()
        self.assertIsNone(self.control.confirmed_target)
        self.assertTrue(any('尚未确认' in note for note in self.notes))


if __name__ == '__main__': unittest.main()
