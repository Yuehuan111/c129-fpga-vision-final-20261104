"""C125 protocol and scheduler regression; fake transports only."""
import binascii
import math
import struct
import time
import unittest
from unittest.mock import patch

import numpy as np

import test_imu_session as fixtures
from test_imu_session import fpga_frame, latency_frame, status_frame
from complete_geometry import configuration, load_calibration
from pose_protocol import (POSE_FIELDS, POSE_MAGIC, REALTIME_BUILD_ID,
                           pose_packets)
from read_latency import MixedFramer, LatencyAnalysis


def pose_frame(version=3, commits=7, board_us=100_000_000, flags=0x1070,
               guard=0x70100, **kwargs):
    data = dict.fromkeys(POSE_FIELDS, 0)
    data.update(magic=POSE_MAGIC, build_id=REALTIME_BUILD_ID,
                version_flags=version | flags << 16, commits=commits,
                board_time_us=board_us, guard_state=guard, **kwargs)
    return fpga_frame(7, [data[key] for key in POSE_FIELDS])


def v5_c125(**kwargs):
    words = list(struct.unpack('<51I', status_frame(**kwargs)[4:-2]))
    words[0] = REALTIME_BUILD_ID
    return fpga_frame(5, words)


class PoseProtocolTests(unittest.TestCase):
    def test_compact_exact_wire_and_crc(self):
        words = configuration(load_calibration(), yaw=-2)
        wire = pose_packets(words, 65535)
        self.assertEqual(len(wire), 88)
        frames = [wire[i:i+8] for i in range(0, 88, 8)]
        self.assertEqual([f[1] for f in frames], [0xFD, *range(0x20, 0x29), 0xFF])
        self.assertEqual(struct.unpack('<I', frames[0][2:6])[0], 0x504FFFFF)
        self.assertEqual([struct.unpack('<i', f[2:6])[0] for f in frames[1:-1]], words[18:])
        for frame in frames:
            self.assertEqual(int.from_bytes(frame[-2:], 'big'), binascii.crc_hqx(frame[:-2], 65535))
        for invalid in (True, -1, 65536, 3.5):
            with self.assertRaises(ValueError):
                pose_packets(words, invalid)
        with self.assertRaises(ValueError):
            pose_packets(configuration(load_calibration(), raw=True), 1)

    def test_pos1_every_cut_and_crc_recovery(self):
        wire = pose_frame()
        self.assertEqual(len(wire), 58)
        for cut in range(len(wire)+1):
            decoder = MixedFramer()
            records = decoder.feed(wire[:cut]) + decoder.feed(wire[cut:])
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]['packet_type'], 'pose_status')
            self.assertTrue(records[0]['hardware_admission_capable'])
            self.assertEqual(records[0]['config_version'], 3)
        broken = bytearray(wire)
        broken[18] ^= 1
        decoder = MixedFramer()
        records = decoder.feed(bytes(broken)+wire+latency_frame())
        self.assertEqual([x['packet_type'] for x in records], ['pose_status', 'latency'])
        self.assertEqual(decoder.counters['crc_failures'], 1)

    def test_pos1_does_not_contaminate_latency_sample_analysis(self):
        analysis = LatencyAnalysis()
        data = MixedFramer().feed(pose_frame()+latency_frame())
        analysis.accept(data[0])
        self.assertTrue(analysis.accept(data[1])['accepted'])
        self.assertEqual(len(analysis.pose_statuses), 1)
        self.assertEqual(len(analysis.samples), 1)
        self.assertEqual(analysis.rejections, {})


class RealtimeTests(fixtures.SessionTests):
    # Reuse fixture methods without duplicating the inherited baseline tests.
    def prepare125(self, session, **kwargs):
        self.feed_imu(session)
        session._ports['fpga'].incoming.put(pose_frame(**kwargs))
        self.wait(lambda: session.snapshot()['latest_pose_status'])
        session.capture_reference()
        session.set_mounting(np.eye(3), confirmed=True)

    def publish(self, session, version=4, commits=8, board_us=100_020_000):
        session._ports['fpga'].incoming.put(pose_frame(version, commits, board_us))
        self.wait(lambda: (session.snapshot()['last_status'] or {}).get('config_version') == version)

    def change_pose(self, session, angle=.01):
        self.feed_imu(session, (math.cos(angle/2), 0, 0, math.sin(angle/2)))

    def test_125_bootstrap_then_compact_before_lat1(self):
        calls = []
        session = self.make(geometry_checker=lambda w: calls.append(w) or {'fixed_point_checked': True})
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 232)
        self.publish(session)
        self.change_pose(session)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        second = session._ports['fpga'].writes[1]
        self.assertEqual(len(second), 88)
        self.assertEqual(struct.unpack('<I', second[2:6])[0], 0x504F0004)
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(session.snapshot()['pending_outputs']), 1)
        self.assertEqual(session.snapshot()['request']['admission'], 'fpga_full_map_guard')
        session.stop()
        self.publish(session, 5, 9, 100_040_000)
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_000_100))
        session._ports['fpga'].incoming.put(latency_frame(version=5, frame_id=32, base=100_020_100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])
        self.assertEqual(len(session._ports['fpga'].writes), 2)

    def test_delayed_v5_cannot_roll_back_pos1_version(self):
        session = self.make()
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.publish(session)
        session.stop()
        session._ports['fpga'].incoming.put(v5_c125())
        self.wait(lambda: session.snapshot()['latest_v5_status'])
        self.assertEqual(session.snapshot()['last_status']['config_version'], 4)
        self.assertFalse(session.snapshot()['frozen'])

    def test_pending_output_window_applies_backpressure(self):
        session = self.make(max_pending_outputs=1)
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.publish(session)
        self.change_pose(session)
        time.sleep(.1)
        self.assertEqual(len(session._ports['fpga'].writes), 1)
        self.assertTrue(session.snapshot()['continuous'])
        self.change_pose(session)
        session._ports['fpga'].incoming.put(latency_frame(base=100_000_100))
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.assertEqual(len(session._ports['fpga'].writes[-1]), 88)

    def test_failed_guard_stops_without_retransmit(self):
        session = self.make()
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        session._ports['fpga'].incoming.put(pose_frame(commits=8, board_us=100_020_000,
                                                      control_failures=1, guard=0x70102))
        self.wait(lambda: session.snapshot()['frozen'])
        self.assertIn('rejection', session.snapshot()['error'])
        self.assertEqual(session.snapshot()['last_status']['config_version'], 3)
        self.assertEqual(session.snapshot()['request']['reject_flags'], 2)
        time.sleep(.1)
        self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_realtime_requires_caps_and_black_policy(self):
        for options, status in (({}, {'guard': 0x30100}), ({'allow_black': False}, {})):
            with self.subTest(options=options, status=status):
                session = self.make(**options)
                self.prepare125(session, **status)
                with self.assertRaises(RuntimeError):
                    session.start_realtime()
                self.assertEqual(session._ports['fpga'].writes, [])

    def test_precise_clock_wrap_publication(self):
        session = self.make()
        self.prepare125(session, board_us=0xFFFFF000)
        session.start_realtime()
        self.written(session)
        self.publish(session, board_us=2000)
        session.stop()
        self.assertFalse(session.snapshot()['frozen'])
        self.assertEqual(len(session.snapshot()['pending_outputs']), 1)
        session._ports['fpga'].incoming.put(latency_frame(base=0xFFFFF100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])

    def test_lost_output_timeout_is_bounded_and_read_only(self):
        session = self.make(output_timeout_s=.05, max_pending_outputs=1)
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.publish(session)
        self.change_pose(session)
        self.wait(lambda: session.snapshot()['frozen'])
        self.assertEqual(session.snapshot()['state'], 'published_output_timeout')
        self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_c125_single_exact_request_uses_pos1_then_output(self):
        calls = []
        session = self.make(geometry_checker=lambda words: calls.append(words) or {'fixed_point_checked': True})
        self.prepare125(session)
        session.send_once()
        self.written(session)
        self.publish(session)
        self.assertEqual(session.snapshot()['request']['phase'], 'await_output')
        self.assertEqual(session.snapshot()['pending_outputs'], [])
        session._ports['fpga'].incoming.put(latency_frame(base=100_000_100))
        self.wait(lambda: session.snapshot()['request']['phase'] == 'output_confirmed')
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(session._ports['fpga'].writes[0]), 232)

    def test_realtime_age_limit_discards_delayed_computation(self):
        session = self.make(realtime_pose_age_s=.03)
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.publish(session)
        def delayed_configuration(*args, **kwargs):
            time.sleep(.06)
            return configuration(*args, **kwargs)
        with patch('imu_session.configuration', side_effect=delayed_configuration):
            self.change_pose(session)
            self.wait(lambda: not session.snapshot()['continuous'])
        self.assertIn('expired', session.snapshot()['error'])
        self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_pos1_wrong_build_and_reset_fail_closed(self):
        for words_change in ('build', 'clock'):
            with self.subTest(words_change=words_change):
                session = self.make()
                self.prepare125(session)
                words = list(struct.unpack('<13I', pose_frame()[4:-2]))
                words[1 if words_change == 'build' else 6] = 0
                session._ports['fpga'].incoming.put(fpga_frame(7, words))
                self.wait(lambda: session.snapshot()['frozen'])
                self.assertEqual(session._ports['fpga'].writes, [])

    def test_realtime_missing_acceptance_never_retransmits(self):
        session = self.make(accept_timeout_s=.05)
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.wait(lambda: session.snapshot()['frozen'])
        self.assertIn('unknown', session.snapshot()['error'])
        time.sleep(.1)
        self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_v5_readiness_advances_but_error_blocks_idle_send(self):
        session = self.make()
        self.prepare125(session, flags=0x1000)
        self.assertFalse(session.snapshot()['can_send'])
        session._ports['fpga'].incoming.put(v5_c125())
        self.wait(lambda: session.snapshot()['can_send'])
        session._ports['fpga'].incoming.put(v5_c125(flags=0x1074))
        self.wait(lambda: not session.snapshot()['can_send'])
        with self.assertRaises(RuntimeError):
            session.send_once()
        self.assertEqual(session._ports['fpga'].writes, [])

    def reject_active(self, session, guard=0x70102, **kwargs):
        session._ports['fpga'].incoming.put(pose_frame(commits=8, board_us=100_020_000,
            control_failures=1, guard=guard, **kwargs))
        self.wait(lambda: session.snapshot()['frozen'])
        self.wait(lambda: 'write_done_at' in session.snapshot()['request'])

    def test_manual_soft_rejection_release_does_not_retry(self):
        session = self.make()
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.reject_active(session)
        session.acknowledge_geometry_rejection()
        self.assertFalse(session.snapshot()['frozen'])
        self.assertFalse(session.snapshot()['continuous'])
        self.assertEqual(session.snapshot()['last_status']['config_version'], 3)
        time.sleep(.08)
        self.assertEqual(len(session._ports['fpga'].writes), 1)
        # A separate manual action is required to submit anything again.
        session.send_once()
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.assertEqual(len(session._ports['fpga'].writes[-1]), 232)

    def test_fatal_or_uart_rejection_cannot_use_soft_recovery(self):
        for guard, extras in ((0x70101, {}), (0x70108, {}), (0x70102, {'rejected': 1})):
            with self.subTest(guard=guard, extras=extras):
                session = self.make()
                self.prepare125(session)
                session.start_realtime()
                self.written(session)
                self.reject_active(session, guard, **extras)
                with self.assertRaises(RuntimeError):
                    session.acknowledge_geometry_rejection()
                self.assertEqual(len(session._ports['fpga'].writes), 1)

    def test_subsequent_fault_invalidates_soft_recovery(self):
        session = self.make()
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.reject_active(session)
        session._ports['fpga'].incoming.put(pose_frame(commits=8, board_us=100_030_000,
            control_failures=1, guard=0x70102, flags=0x1074))
        self.wait(lambda: 'hardware error' in (session.snapshot()['error'] or ''))
        with self.assertRaises(RuntimeError):
            session.acknowledge_geometry_rejection()

    def test_same_event_reset_cannot_be_masked_as_soft_rejection(self):
        for changes in ({'board_us': 90_000_000, 'captured': 10},
                        {'board_us': 100_020_000, 'captured': 9}):
            with self.subTest(changes=changes):
                session = self.make()
                self.prepare125(session, captured=10)
                session.start_realtime()
                self.written(session)
                session._ports['fpga'].incoming.put(pose_frame(commits=8,
                    control_failures=1, guard=0x70102, **changes))
                self.wait(lambda: session.snapshot()['frozen'])
                with self.assertRaises(RuntimeError):
                    session.acknowledge_geometry_rejection()

    def publish_two(self, session, base_version=3):
        self.prepare125(session, version=base_version)
        session.start_realtime()
        self.written(session)
        self.publish(session, version=(base_version+1) & 65535)
        self.change_pose(session)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        self.publish(session, version=(base_version+2) & 65535, commits=9, board_us=100_040_000)
        session.stop()
        self.assertEqual(len(session.snapshot()['pending_outputs']), 2)

    def test_newer_output_retires_unobserved_older_publication_without_success(self):
        session = self.make()
        self.publish_two(session)
        session._ports['fpga'].incoming.put(latency_frame(version=5, frame_id=32, base=100_020_100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])
        self.assertEqual(session.snapshot()['version_evidence'], dict(
            published=2, output_confirmed=1, superseded_or_output_unobserved=1, published_output_unverifiable=0, pending=0))
        self.assertFalse(session.snapshot()['frozen'])
        closed = [e for e in session.snapshot()['events'] if e['event'] == 'superseded_or_output_unobserved']
        self.assertEqual(closed[0]['request']['published_status']['config_version'], 4)
        self.assertNotIn('output_at', closed[0]['request'])

    def test_older_output_arriving_first_is_confirmed_not_superseded(self):
        session = self.make()
        self.publish_two(session)
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_000_100)+
            latency_frame(version=5, frame_id=32, base=100_020_100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])
        self.assertEqual(session.snapshot()['version_evidence'], dict(
            published=2, output_confirmed=2, superseded_or_output_unobserved=0, published_output_unverifiable=0, pending=0))

    def test_superseded_publications_use_request_order_across_version_wrap(self):
        session = self.make()
        self.publish_two(session, base_version=65534)
        session._ports['fpga'].incoming.put(latency_frame(version=0, frame_id=32, base=100_020_100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])
        self.assertEqual(session.snapshot()['version_evidence']['superseded_or_output_unobserved'], 1)
        self.assertEqual(session.snapshot()['request']['published_status']['config_version'], 0)
        self.assertFalse(session.snapshot()['frozen'])

    def test_board_reset_or_identity_change_invalidates_pending_output_even_near_wrap(self):
        for change in ('reset', 'firmware'):
            with self.subTest(change=change):
                session = self.make()
                self.prepare125(session, board_us=0xFFFFF000)
                session.start_realtime()
                self.written(session)
                self.publish(session, board_us=2000)
                session.stop()
                if change == 'reset':
                    session._ports['fpga'].incoming.put(pose_frame(version=0, commits=0, board_us=100))
                else:
                    session._ports['fpga'].incoming.put(status_frame(version=4, commits=8))
                self.wait(lambda: session.snapshot()['frozen'])
                session._ports['fpga'].incoming.put(latency_frame(version=4, base=300))
                self.wait(lambda: any(e['event']=='latency' for e in session.snapshot()['events']))
                evidence = session.snapshot()['version_evidence']
                self.assertEqual(evidence['published_output_unverifiable'], 1)
                self.assertEqual(evidence['output_confirmed'], 0)
                self.assertEqual(evidence['pending'], 0)

    def test_soft_rejection_keeps_old_published_output_evidence_confirmable(self):
        session = self.make()
        self.prepare125(session)
        session.start_realtime()
        self.written(session)
        self.publish(session)
        self.change_pose(session)
        self.wait(lambda: len(session._ports['fpga'].writes) == 2)
        session._ports['fpga'].incoming.put(pose_frame(version=4, commits=9,
            board_us=100_040_000, control_failures=1, guard=0x70102))
        self.wait(lambda: session.snapshot()['frozen'])
        session._ports['fpga'].incoming.put(latency_frame(version=4, base=100_000_100))
        self.wait(lambda: not session.snapshot()['pending_outputs'])
        self.assertEqual(session.snapshot()['version_evidence']['output_confirmed'], 1)
        self.assertEqual(session.snapshot()['version_evidence']['published_output_unverifiable'], 0)
        self.assertTrue(session.snapshot()['frozen'])


# unittest would otherwise rerun all 35 fixture base tests in this subclass.
for _name in list(vars(fixtures.SessionTests)):
    if _name.startswith('test_'):
        setattr(RealtimeTests, _name, None)

if __name__ == '__main__':
    unittest.main()
