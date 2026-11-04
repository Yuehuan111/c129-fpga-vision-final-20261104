"""C127 auxiliary/context and publication ledger tests. Fake ports only."""
from copy import deepcopy
import math
from pathlib import Path
import struct
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

import test_imu_session as fixtures
from test_imu_session import FakePort, adapters, fpga_frame, latency_frame, status_frame
from c127_session import C127HostSession, profile_packets, _c127_geometry_process
from complete_geometry import configuration
from pose_protocol import POSE_FIELDS, POSE_MAGIC, VIEW_BUILD_ID
from read_latency import MixedFramer
from scan_plane import make_mirror_profile, fit_four_points, validate_profile, compose_words


def view_frame(version=3, commits=7, board_us=100_000_000, context=0,
               matrix=False, plane=False, requested=0, committed=0,
               raw=False, key=False, busy=False, mode_reject=False, flags=0x1070, **extras):
    guard = (0x70100 | (1 << 19) | (int(matrix) << 20) | (int(plane) << 15) |
             (requested << 13) | (committed << 11) | (int(raw) << 9) |
             (int(key) << 22) | (int(busy) << 10) | (int(mode_reject) << 21))
    data = dict.fromkeys(POSE_FIELDS, 0)
    data.update(magic=POSE_MAGIC, build_id=VIEW_BUILD_ID,
                version_flags=version | (flags << 16), commits=commits,
                board_time_us=board_us, guard_state=guard, **extras)
    return fpga_frame(8, [data[k] for k in POSE_FIELDS]+[context])


GOOD_MODES = dict(modes={name: dict(admitted=True) for name in ('00', '01', '10', '11')})


def health_frame(**kwargs):
    words = list(struct.unpack('<51I', status_frame(**kwargs)[4:-2]))
    words[0] = VIEW_BUILD_ID
    return fpga_frame(5, words)


class C127SessionTests(unittest.TestCase):
    setUp = fixtures.SessionTests.setUp
    tearDown = fixtures.SessionTests.tearDown
    wait = fixtures.SessionTests.wait
    feed_imu = fixtures.SessionTests.feed_imu

    def make(self, *, start=True, imu=False, real_geometry=False, **kwargs):
        if not real_geometry:
            kwargs.setdefault('geometry_checker', lambda words: dict(fixed_point_checked=True))
        session = C127HostSession(log_path=Path(self.temp.name)/f'{len(self.sessions)}.jsonl',
                                  serial_factory=FakePort, ports_provider=adapters, **kwargs)
        self.sessions.append(session)
        if start:
            session.start('IMU' if imu else None, 'FPGA')
        return session

    def put(self, session, **kw):
        when = kw.get('board_us', 100_000_000)
        session._ports['fpga'].incoming.put(view_frame(**kw))
        self.wait(lambda: (session.snapshot()['latest_pose_status'] or {}).get('board_time_us') == when)

    def ordinary(self, session, *, own_base=True):
        self.put(session)
        session.set_pose_mode('normal')
        if own_base:
            # These tests focus on subsequent aux commands. Simulate a base
            # already proven by the separate ordinary publication test below.
            session._last_words = tuple(configuration(session.calibration))
        return session

    def install(self, session, profile=None, *, board_us=100_010_000):
        profile = profile or make_mirror_profile(session.scan_context())
        old_count = len(session._ports['fpga'].writes)
        with patch('c127_session.validate_modes', return_value=deepcopy(GOOD_MODES)):
            session.set_scan_profile(profile)
        self.wait(lambda: len(session._ports['fpga'].writes) > old_count)
        token = session._view_command['wire_context']
        self.put(session, board_us=board_us, context=token, matrix=True, plane=profile['scan_valid'],
                 control_failures=(session._pose_status or {}).get('control_failures', 0))
        self.wait(lambda: session._view_command is None)
        return token

    def test_connection_read_only_and_normal_without_imu(self):
        session = self.ordinary(self.make(), own_base=False)
        self.assertEqual(session._ports['fpga'].writes, [])
        session.request_normal()
        self.wait(lambda: session._ports['fpga'].writes)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 232)
        # LAT may arrive before its POS1 publication report.
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_000_100))
        self.put(session, version=4, commits=8, board_us=100_020_000)
        self.wait(lambda: (session.snapshot()['request'] or {}).get('phase') == 'output_confirmed')
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(session.snapshot()['scan']['displayed']['version'], 4)
        self.assertFalse(session.snapshot()['imu']['connected'])

    def test_normal_identity_diagnostic_still_requires_fresh_imu_observation(self):
        normal = self.ordinary(self.make())
        self.assertFalse(normal.snapshot()['can_send_identity_test'])
        self.assertIn('IMU', normal.snapshot()['identity_test_blocked_reason'])
        with self.assertRaisesRegex(RuntimeError, 'IMU'):
            normal.send_identity_test()
        self.assertFalse(normal.snapshot()['identity_test_requested'])
        self.assertEqual(normal._ports['fpga'].writes, [])

        observed = self.ordinary(self.make(imu=True))
        self.feed_imu(observed)
        self.assertTrue(observed.snapshot()['can_send_identity_test'])
        observed.imu_stale_s = 1e-9
        self.assertFalse(observed.snapshot()['can_send_identity_test'])
        self.assertIn('stale', observed.snapshot()['identity_test_blocked_reason'])
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            observed.send_identity_test()
        observed.imu_stale_s = .5
        observed.send_identity_test()
        self.wait(lambda: observed._ports['fpga'].writes)
        self.assertEqual(len(observed._ports['fpga'].writes[0]), 232)
        self.assertFalse(observed.snapshot()['frozen'])

    def test_uninitialized_key_rejection_preserves_session_then_explicit_mirror_initializes(self):
        session = self.ordinary(self.make())
        saved_words = session._last_words
        with session._lock:
            session._continuous = True
            session._next_due = session._clock()+60
        self.put(session, board_us=100_010_000, control_failures=1, mode_reject=True)
        state = session.snapshot()
        self.assertFalse(state['frozen'])
        self.assertFalse(state['continuous'])
        self.assertEqual(session._last_words, saved_words)
        self.assertEqual(state['last_status']['config_version'], 3)
        self.assertEqual(state['version_evidence']['published'], 0)
        self.assertEqual(state['version_evidence']['output_confirmed'], 0)
        self.assertIn('初始化', state['scan']['message'])
        self.assertTrue(any(item['event'] == 'view_rejected' for item in state['events']))
        self.assertEqual(session._ports['fpga'].writes, [])
        token = self.install(session, board_us=100_020_000)
        session.request_view_mode(mirror=True)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.put(session, version=4, board_us=100_030_000, context=token, matrix=True,
                 requested=1, committed=1, key=True, control_failures=1)
        self.wait(lambda: session._view_command is None)
        self.assertFalse(session.snapshot()['frozen'])
        self.assertTrue(session.snapshot()['scan']['owned'])

    def test_uninitialized_rejection_does_not_soften_crc_hardware_or_counter_jump(self):
        for changes in (dict(rejected=1), dict(flags=0x1074), dict(control_failures=2), dict(version=4)):
            with self.subTest(changes=changes):
                session = self.ordinary(self.make())
                packet = dict(board_us=100_010_000, control_failures=1, mode_reject=True)
                packet.update(changes)
                self.put(session, **packet)
                self.assertTrue(session.snapshot()['frozen'])
                self.assertFalse(any(item['event'] == 'view_rejected' for item in session.snapshot()['events']))

    def test_uninitialized_rejection_during_written_pose_or_aux_still_freezes(self):
        pose = self.ordinary(self.make(), own_base=False)
        pose.request_normal()
        self.wait(lambda: pose._ports['fpga'].writes)
        self.put(pose, board_us=100_010_000, control_failures=1, mode_reject=True)
        self.assertTrue(pose.snapshot()['frozen'])
        self.assertEqual(pose.snapshot()['version_evidence']['published'], 0)
        aux = self.ordinary(self.make())
        with patch('c127_session.validate_modes', return_value=deepcopy(GOOD_MODES)):
            aux.set_scan_profile(make_mirror_profile(aux.scan_context()))
        self.wait(lambda: aux._ports['fpga'].writes)
        self.put(aux, board_us=100_010_000, control_failures=1, mode_reject=True)
        self.assertTrue(aux.snapshot()['frozen'])
        self.assertIsNone(aux._scan_profile)

    def test_profile_wire_order_crc_and_fresh_nonce_ack(self):
        session = self.ordinary(self.make())
        first = self.install(session)
        first_payload = session._ports['fpga'].writes[-1]
        self.assertEqual(len(first_payload), 39*8)
        self.assertEqual([first_payload[i+1] for i in range(0, len(first_payload), 8)],
                         [0xE8, 0xE7, *range(0x40, 0x64), 0xE9])
        self.assertEqual(struct.unpack('<I', first_payload[2:6])[0], first)
        profile = fit_four_points([[500, 220], [800, 220], [800, 450], [500, 450]], 11/6,
                                  session.scan_context())
        with patch('c127_session.validate_modes', return_value=deepcopy(GOOD_MODES)):
            session.set_scan_profile(profile)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        second = session._view_command['wire_context']
        self.assertNotEqual(first, second)
        self.put(session, board_us=100_020_000, context=first, matrix=True)
        self.assertIsNotNone(session._view_command)
        self.assertEqual(session._scan_wire_context, first)
        self.put(session, board_us=100_030_000, context=second, matrix=True, plane=True)
        self.wait(lambda: session._view_command is None)
        self.assertEqual(session._scan_profile['profile_hash'], profile['profile_hash'])
        self.assertEqual(session._scan_wire_context, second)

    def test_same_mode_waits_for_new_key_version_and_real_output(self):
        session = self.ordinary(self.make())
        token = self.install(session)
        session.request_view_mode(False, False)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.put(session, board_us=100_020_000, context=token, matrix=True)
        self.assertIsNotNone(session._view_command)
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_020_100))
        self.put(session, version=4, board_us=100_030_000, context=token, matrix=True, key=True)
        self.wait(lambda: session._view_command is None)
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(session.snapshot()['scan']['displayed']['version'], 4)
        self.assertEqual(session.snapshot()['version_evidence']['output_confirmed'], 0)

    def test_mirror_mode_owns_nonce_and_rejects_missing_plane(self):
        session = self.ordinary(self.make())
        token = self.install(session)
        with self.assertRaisesRegex(RuntimeError, '平面'):
            session.request_view_mode(scan=True)
        session.request_view_mode(mirror=True)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.put(session, version=4, board_us=100_020_000, context=token, matrix=True,
                 requested=1, committed=1, key=True)
        self.wait(lambda: session._view_command is None)
        self.assertFalse(session.snapshot()['frozen'])
        self.assertTrue(session.snapshot()['scan']['owned'])

    def test_key_raw_restore_before_any_mirror_profile(self):
        session = self.ordinary(self.make())
        before = session._last_words
        self.put(session, version=4, board_us=100_010_000, raw=True, key=True)
        self.assertFalse(session.snapshot()['frozen'])
        with self.assertRaisesRegex(RuntimeError, 'raw'):
            session.request_normal()
        self.put(session, version=5, board_us=100_020_000, raw=False, key=True)
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(session._last_words, before)
        self.assertEqual(session._ports['fpga'].writes, [])

    def test_key_and_active_pose_collision_never_reports_pose_success(self):
        session = self.ordinary(self.make(), own_base=False)
        session.request_normal()
        self.wait(lambda: session._ports['fpga'].writes)
        self.put(session, version=4, commits=8, board_us=100_010_000, key=True)
        self.assertTrue(session.snapshot()['frozen'])
        self.assertEqual(session.snapshot()['version_evidence']['published'], 0)
        self.assertEqual(session.snapshot()['version_evidence']['output_confirmed'], 0)
        self.assertNotIn('published_at', session._active)
        self.assertTrue(session._active['invalidated'])

    def test_key_after_realtime_publication_retires_missing_output_without_success(self):
        session = self.make(imu=True)
        self.feed_imu(session)
        self.put(session)
        session.capture_reference()
        session.set_mounting(np.eye(3), confirmed=True)
        session.start_realtime()
        self.wait(lambda: session._ports['fpga'].writes)
        self.put(session, version=4, commits=8, board_us=100_010_000)
        self.wait(lambda: len(session.snapshot()['pending_outputs']) == 1)
        session.stop()
        self.put(session, version=5, commits=8, board_us=100_020_000, raw=True, key=True)
        state = session.snapshot()
        self.assertFalse(state['frozen'])
        self.assertEqual(state['pending_outputs'], [])
        self.assertEqual(state['version_evidence']['output_confirmed'], 0)
        self.assertEqual(state['version_evidence']['superseded_or_output_unobserved'], 1)
        self.assertEqual(state['request']['phase'], 'superseded_or_output_unobserved')
        self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_invalidation_waits_for_normal_base_replay_after_context_clear(self):
        session = self.ordinary(self.make())
        token = self.install(session)
        session.request_view_mode(mirror=True)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.put(session, version=4, board_us=100_020_000, context=token, matrix=True,
                 requested=1, committed=1, key=True)
        self.wait(lambda: session._view_command is None)
        session.invalidate_scan_profile()
        self.wait(lambda: len(session._ports['fpga'].writes) == 3)
        self.put(session, version=4, board_us=100_030_000, requested=0, committed=1, busy=True)
        self.assertIsNotNone(session._view_command)
        self.assertTrue(session._view_dirty)
        self.put(session, version=5, board_us=100_040_000, key=True)
        self.wait(lambda: session._view_command is None)
        self.assertFalse(session.snapshot()['frozen'])
        self.assertFalse(session._view_dirty)
        self.assertIsNone(session._scan_profile)

    def test_one_normal_click_defers_until_invalidate_then_sends_once(self):
        session = self.make()
        self.put(session)
        session._last_words = tuple(configuration(session.calibration))
        self.install(session)  # Still in the default roll_only UI mode.
        session.request_normal()
        self.assertTrue(session.snapshot()['scan']['normal_pending'])
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.put(session, version=4, board_us=100_020_000, key=True)
        self.wait(lambda: len(session._ports['fpga'].writes) == 3)
        self.assertEqual(len(session._ports['fpga'].writes[2]), 232)
        self.assertFalse(session.snapshot()['scan']['normal_pending'])
        self.put(session, version=5, commits=8, board_us=100_040_000)
        session._ports['fpga'].incoming.put(latency_frame(version=5, base=100_020_100))
        self.wait(lambda: (session.snapshot()['request'] or {}).get('phase') == 'output_confirmed')
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(len(session._ports['fpga'].writes), 3)

    def test_stop_cancels_deferred_normal_without_aborting_written_invalidation(self):
        session = self.make()
        self.put(session)
        session._last_words = tuple(configuration(session.calibration))
        self.install(session)
        session.request_normal()
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        session.stop_continuous()
        self.put(session, version=4, board_us=100_020_000, key=True)
        self.wait(lambda: session._view_command is None)
        time.sleep(.03)
        self.assertFalse(session.snapshot()['scan']['normal_pending'])
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(len(session._ports['fpga'].writes), 2)

    def test_aux_blocks_geometry_and_partial_write_is_not_retried(self):
        session = self.ordinary(self.make())
        session._ports['fpga'].partial_write = True
        with patch('c127_session.validate_modes', return_value=deepcopy(GOOD_MODES)):
            session.set_scan_profile(make_mirror_profile(session.scan_context()))
        with self.assertRaises(RuntimeError):
            session.request_normal()
        self.wait(lambda: session.snapshot()['frozen'])
        time.sleep(.04)
        self.assertEqual(len(session._ports['fpga'].writes), 1)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 312)
        self.assertIsNone(session._scan_profile)

    def test_unacknowledged_aux_times_out_without_automatic_retry(self):
        session = self.ordinary(self.make(publish_timeout_s=.05))
        with patch('c127_session.validate_modes', return_value=deepcopy(GOOD_MODES)):
            session.set_scan_profile(make_mirror_profile(session.scan_context()))
        self.wait(lambda: session.snapshot()['frozen'])
        self.assertIn('unknown', session.snapshot()['error'])
        self.assertEqual(len(session._ports['fpga'].writes), 1)
        self.assertIsNone(session._scan_profile)

    def test_late_older_latency_cannot_roll_displayed_version_back(self):
        session = self.ordinary(self.make())
        self.put(session, version=4, board_us=100_010_000, key=True)
        self.put(session, version=5, board_us=100_020_000, key=True)
        session._ports['fpga'].incoming.put(latency_frame(version=5, base=100_010_100))
        self.wait(lambda: (session.snapshot()['scan']['displayed'] or {}).get('version') == 5)
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_000_100, frame_id=30))
        time.sleep(.03)
        self.assertEqual(session.snapshot()['scan']['displayed']['version'], 5)

    def test_reset_clears_view_output_and_parameter_ownership(self):
        session = self.ordinary(self.make())
        self.install(session)
        session._view_displayed = dict(version=3, output_first_us=100_000_400)
        self.put(session, board_us=1000)
        self.assertTrue(session.snapshot()['frozen'])
        self.assertIsNone(session.snapshot()['scan']['displayed'])
        self.assertIsNone(session._scan_profile)
        self.assertIsNone(session._scan_wire_context)

    def test_fixed_crop_context_and_identity_use_identical_optics(self):
        session = self.make(start=False)
        crop = dict(zoom=1.3, max_abs_roll_deg=15.)
        session.set_pose_mode('roll_only', crop)
        context = session.scan_context()
        request = dict(request_kind='identity_transport_test', pose_mode='roll_only', crop_profile=crop)
        h, words = session._prepare_matrix_words(request)
        self.assertEqual(words[:18], context['words18'])
        np.testing.assert_array_equal(h, np.eye(3))
        profile = make_mirror_profile(context)
        session.set_pose_mode('roll_only', dict(zoom=1.4, max_abs_roll_deg=15.))
        with self.assertRaisesRegex(ValueError, 'stale'):
            validate_profile(profile, expected_context=session.scan_context())

    def test_enhancement_same_mode_no_write_and_pos_fresh_ack(self):
        session = self.ordinary(self.make())
        session.request_enhancement(1)
        self.assertEqual(session._ports['fpga'].writes, [])
        for bad in (-1, 5, True, 1.5):
            with self.assertRaises(ValueError):
                session.request_enhancement(bad)
        session.request_enhancement(3)
        self.wait(lambda: session._ports['fpga'].writes)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 8)
        self.assertEqual(session._ports['fpga'].writes[0][1], 0xE2)
        with self.assertRaises(RuntimeError):
            session.request_normal()
        self.put(session, board_us=100_010_000, flags=0x2070)
        self.assertIsNotNone(session._view_command)
        self.put(session, board_us=100_020_000, flags=0x3070)
        self.wait(lambda: session._view_command is None)
        self.assertEqual(session.snapshot()['enhancement']['enhancement_mode'], 3)
        self.assertEqual(session.snapshot()['last_status']['config_version'], 3)
        self.assertFalse(session.snapshot()['frozen'])

    def test_enhancement_v5_ack_does_not_rebase_geometry_ledger(self):
        session = self.ordinary(self.make())
        session.request_enhancement(4)
        self.wait(lambda: session._ports['fpga'].writes)
        session._ports['fpga'].incoming.put(health_frame(ticks=100, version=2, flags=0x4070))
        self.wait(lambda: session.snapshot()['latest_v5_status'])
        self.assertIsNotNone(session._view_command)
        session._ports['fpga'].incoming.put(health_frame(ticks=101, version=2, flags=0x4070))
        self.wait(lambda: session._view_command is None)
        self.assertEqual(session.snapshot()['enhancement']['enhancement_mode'], 4)
        self.assertEqual(session.snapshot()['last_status']['config_version'], 3)
        self.assertFalse(session.snapshot()['frozen'])

    def test_c127_geometry_policy_allows_domain_black_and_checks_composed_h(self):
        session = self.ordinary(self.make())
        p = fit_four_points([[530, 145], [810, 95], [910, 500], [510, 475]], 11/6, session.scan_context())
        # This directly invokes the worker function without transport/process
        # scheduling, exercising its real exact integer policy.
        import queue
        output = queue.Queue()
        h = np.array([[.3949950155, -.0322533639, .0240234880],
                      [.1517574671, .6811640556, -.1762661904],
                      [-.5627766570, .1932069294, 1.]])
        words = configuration(session.calibration, matrix=h)
        _c127_geometry_process(words, True, output)
        ok, evidence = output.get()
        self.assertTrue(ok, evidence)
        self.assertGreater(evidence['domain_invalid_pixels'], 0)
        _c127_geometry_process(words, False, output)
        self.assertFalse(output.get()[0])
        seen = []
        session._geometry_checker = lambda checked: seen.append(checked) or dict(ok=True)
        session._scan_profile = p
        session._scan_wire_context = 123
        session._pose_status = dict(session._pose_status, committed_modes=1, matrix_valid=True, view_context=123)
        session._check_request_geometry(configuration(session.calibration), {})
        self.assertEqual(list(seen[0]), compose_words(p, mirror=True))


if __name__ == '__main__':
    unittest.main()
