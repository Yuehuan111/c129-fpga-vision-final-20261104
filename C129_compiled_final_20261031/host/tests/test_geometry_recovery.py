"""Known geometric failures recover; ambiguous transactions stay stopped.

Uses C127 FakePort, the real workers/ledger, and output evidence frames.
"""
import math
import struct
import threading
import time
import unittest

import numpy as np
import test_c127_session as fixtures
from test_imu_session import latency_frame, fpga_frame
from complete_geometry import configuration


class GeometryRecoveryTests(unittest.TestCase):
    setUp = fixtures.C127SessionTests.setUp
    tearDown = fixtures.C127SessionTests.tearDown
    wait = fixtures.C127SessionTests.wait
    feed_imu = fixtures.C127SessionTests.feed_imu
    make = fixtures.C127SessionTests.make

    def put(self, session, **kw):
        guard = kw.pop('guard_state', None)
        wire = fixtures.view_frame(**kw)
        if guard is not None:
            words = list(struct.unpack('<14I', wire[4:-2]))
            words[12] = guard
            wire = fpga_frame(8, words)
        session._ports['fpga'].incoming.put(wire)
        self.wait(lambda: (session.snapshot()['latest_pose_status'] or {}).get('board_time_us') == kw.get('board_us', 100_000_000))

    def pose(self, session, deg):
        a = math.radians(deg)/2
        self.feed_imu(session, (math.cos(a), 0, 0, math.sin(a)))

    def prepared(self, **kwargs):
        kwargs.setdefault('imu_stale_s', 10)
        kwargs.setdefault('status_stale_s', 10)
        session = self.make(imu=True, **kwargs)
        self.put(session)
        self.feed_imu(session)
        session.set_mounting(np.eye(3), confirmed=True)
        session.set_pose_mode('full')
        session.capture_reference()
        # Prior exact baseline is already covered by C127's ordinary test.
        session._last_words = tuple(configuration(session.calibration))
        session._recovery_interval_s = .04
        self.pose(session, 10)
        return session

    def start(self, session, enabled=True):
        session.start_realtime(auto_recover_geometry=enabled)
        self.wait(lambda: session._active and 'write_done_at' in session._active)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 88)

    def reject(self, session, *, commits=8, failures=1, flags=2, **kwargs):
        self.put(session, commits=commits, control_failures=failures,
                 board_us=100_000_000+commits*10_000, guard_state=0xF0100 | flags, **kwargs)

    def held(self, session):
        self.wait(lambda: session.snapshot()['geometry_recovery'] is not None and session._active is None)
        self.assertFalse(session.snapshot()['frozen'])

    def probe(self, session, deg=5):
        count = len(session._ports['fpga'].writes)
        self.pose(session, deg)
        self.wait(lambda: len(session._ports['fpga'].writes) == count+1)
        self.wait(lambda: session._active and 'write_done_at' in session._active)
        self.assertTrue(session._active['geometry_recovery_probe'])

    def test_returned_pose_resumes_only_after_matching_output(self):
        s = self.prepared(); self.start(s)
        reference = s.snapshot()['reference']
        self.reject(s); self.held(s)
        self.probe(s)
        self.put(s, version=4, commits=9, control_failures=1, board_us=100_200_000)
        self.assertIsNotNone(s.snapshot()['geometry_recovery'])
        self.assertEqual(len(s._ports['fpga'].writes), 2)
        s._ports['fpga'].incoming.put(latency_frame(version=4, base=100_100_000))
        self.wait(lambda: s.snapshot()['geometry_recovery'] is None)
        self.assertTrue(s.snapshot()['continuous'])
        self.assertFalse(s.snapshot()['frozen'])
        self.assertEqual(s.snapshot()['reference'], reference)
        self.assertTrue(any(e['event'] == 'geometry_recovery_resumed' for e in s.snapshot()['events']))
        s.stop()

    def test_same_pose_and_quaternion_sign_do_not_repeat_rejected_write(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        q = s._history.latest().quaternion
        self.feed_imu(s, tuple(-x for x in q))
        time.sleep(.12)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertEqual(s.snapshot()['geometry_recovery']['phase'], 'waiting_pose')

    def test_probes_are_rate_limited_and_repeated_rejection_keeps_valid_map(self):
        s = self.prepared(); s._recovery_interval_s = .2
        good = s._last_words
        self.start(s); self.reject(s); self.held(s)
        self.pose(s, 15); time.sleep(.04)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        time.sleep(.18)
        self.pose(s, 15)
        self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        self.wait(lambda: 'write_done_at' in (s._active or {}))
        self.reject(s, commits=9, failures=2); self.held(s)
        self.assertEqual(s._last_words, good)
        self.assertEqual(s.snapshot()['geometry_recovery']['rejection_count'], 2)
        time.sleep(.23)
        self.assertEqual(len(s._ports['fpga'].writes), 2)

    def test_return_to_exact_retained_mapping_needs_no_retransmission(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        self.pose(s, 0)
        self.wait(lambda: s.snapshot()['geometry_recovery'] is None)
        self.assertTrue(s.snapshot()['continuous'])
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertEqual(s.snapshot()['version_evidence']['output_confirmed'], 0)
        s.stop()

    def test_all_invalid_rejection_is_recoverable(self):
        for flags in (4, 6):
            with self.subTest(flags=flags):
                s = self.prepared(); self.start(s); self.reject(s, flags=flags); self.held(s)
                s.stop()

    def test_manual_stop_cancels_recovery(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        s.stop(); self.pose(s, 0); time.sleep(.1)
        self.assertFalse(s.snapshot()['continuous'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])
        self.assertEqual(len(s._ports['fpga'].writes), 1)

    def test_stale_imu_cancels_intent_and_new_samples_do_not_restart(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        s.imu_stale_s = .001
        self.wait(lambda: not s.snapshot()['continuous'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])
        s.imu_stale_s = 10; self.pose(s, 0); time.sleep(.1)
        self.assertEqual(len(s._ports['fpga'].writes), 1)

    def test_setup_ddr_or_combined_errors_never_auto_recover(self):
        for flags in (1, 8, 3, 10):
            with self.subTest(flags=flags):
                s = self.prepared(); self.start(s); self.reject(s, flags=flags)
                self.wait(lambda: s.snapshot()['frozen'])
                self.assertIsNone(s.snapshot()['geometry_recovery'])
                self.assertEqual(len(s._ports['fpga'].writes), 1)

    def test_unmatched_failure_count_never_auto_recovers(self):
        s = self.prepared(); self.start(s); self.reject(s, failures=2)
        self.wait(lambda: s.snapshot()['frozen'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])

    def test_hardware_error_while_held_cancels_recovery(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        self.put(s, commits=8, control_failures=1, board_us=100_100_000,
                 flags=0x1074, guard_state=0xF0102)
        self.wait(lambda: s.snapshot()['frozen'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])

    def test_unknown_write_outcome_remains_stopped(self):
        s = self.prepared(accept_timeout_s=.04); self.start(s)
        self.wait(lambda: s.snapshot()['frozen'])
        self.assertIn('unknown', s.snapshot()['error'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])
        self.assertEqual(len(s._ports['fpga'].writes), 1)

    def test_probe_without_output_confirmation_never_resumes(self):
        s = self.prepared(output_timeout_s=.05); self.start(s); self.reject(s); self.held(s)
        self.probe(s)
        self.put(s, version=4, commits=9, control_failures=1, board_us=100_200_000)
        self.wait(lambda: s.snapshot()['frozen'])
        self.assertIsNone(s.snapshot()['geometry_recovery'])
        self.assertEqual(len(s._ports['fpga'].writes), 2)

    def test_rejection_before_serial_drain_keeps_owner_until_completed(self):
        s = self.prepared()
        release = threading.Event()
        s._drain = lambda port: release.wait(1)
        try:
            s.start_realtime(auto_recover_geometry=True)
            self.wait(lambda: len(s._ports['fpga'].writes) == 1)
            self.reject(s)
            self.assertIsNotNone(s.snapshot()['geometry_recovery'])
            self.assertNotIn('write_done_at', s._active)
            self.pose(s, 5); time.sleep(.08)
            self.assertEqual(len(s._ports['fpga'].writes), 1)
            release.set()
            self.pose(s, 5)
            self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        finally:
            release.set(); s.stop()

    def test_default_manual_rejection_stays_terminal_without_false_timeout(self):
        s = self.prepared(publish_timeout_s=.04, accept_timeout_s=.04)
        self.start(s, enabled=False)
        self.put(s, commits=8, board_us=100_010_000, flags=0x1072)
        self.reject(s)
        self.wait(lambda: s.snapshot()['frozen'])
        time.sleep(.12)
        self.assertEqual(s.snapshot()['request']['phase'], 'board_rejected')
        self.assertIn('geometry rejection', s.snapshot()['error'])
        self.assertEqual(s._fault_sequence, 1)
        s.acknowledge_geometry_rejection()
        self.assertFalse(s.snapshot()['frozen'])
        self.assertFalse(s.snapshot()['continuous'])

    def delay_next_pose(self, session):
        entered, release = threading.Event(), threading.Event()
        original = session._prepare_matrix_words
        delayed = []
        def prepare(request):
            if not delayed:
                delayed.append(request)
                request['sample']['received_at'] -= .2
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('Test did not release delayed preparation')
            return original(request)
        session._prepare_matrix_words = prepare
        self.addCleanup(release.set)
        return entered, release, delayed

    def test_expired_unsent_pose_is_dropped_then_fresh_pose_is_written(self):
        s = self.prepared()
        entered, release, delayed = self.delay_next_pose(s)
        s.start_realtime(auto_recover_geometry=True)
        self.assertTrue(entered.wait(1))
        self.pose(s, 11)
        fresh_id = s._history.latest().id
        release.set()
        self.wait(lambda: s._active and 'write_done_at' in s._active)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertNotIn('write_started_at', delayed[0])
        self.assertEqual(delayed[0]['phase'], 'candidate_discarded')
        self.assertEqual(s._active['sample']['id'], fresh_id)
        self.assertLessEqual(s._active['pose_age_at_write_s'], s.realtime_pose_age_s)
        self.assertTrue(s.snapshot()['continuous'])
        self.assertTrue(s.snapshot()['auto_recover_geometry'])
        s.stop()

    def test_expired_pose_without_fresh_sample_or_opt_in_stays_stopped(self):
        for enabled, new_sample in ((True, False), (False, True)):
            with self.subTest(enabled=enabled, new_sample=new_sample):
                s = self.prepared()
                entered, release, delayed = self.delay_next_pose(s)
                s.start_realtime(auto_recover_geometry=enabled)
                self.assertTrue(entered.wait(1))
                if new_sample:
                    self.pose(s, 11)
                release.set()
                self.wait(lambda: not s.snapshot()['continuous'])
                self.assertEqual(len(s._ports['fpga'].writes), 0)
                self.assertIsNone(s.snapshot()['geometry_recovery'])

    def test_expired_recovery_probe_retains_hold_until_fresh_output(self):
        s = self.prepared(); self.start(s); self.reject(s); self.held(s)
        entered, release, delayed = self.delay_next_pose(s)
        self.pose(s, 5)
        self.assertTrue(entered.wait(1))
        self.pose(s, 4)
        release.set()
        self.wait(lambda: s._active and 'write_done_at' in s._active)
        self.assertEqual(len(s._ports['fpga'].writes), 2)
        self.assertNotIn('write_started_at', delayed[0])
        self.assertIsNotNone(s.snapshot()['geometry_recovery'])
        self.put(s, version=4, commits=9, control_failures=1, board_us=100_200_000)
        s._ports['fpga'].incoming.put(latency_frame(version=4, base=100_100_000))
        self.wait(lambda: s.snapshot()['geometry_recovery'] is None)
        self.assertTrue(s.snapshot()['continuous'])
        s.stop()


if __name__ == '__main__':
    unittest.main()
