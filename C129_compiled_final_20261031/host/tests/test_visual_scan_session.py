"""C128 source provenance, atomic visual commits and route evidence; fake UART."""
from copy import deepcopy
import struct
import threading
import time
import unittest

import numpy as np

import test_c127_session as fixtures
from test_imu_session import fpga_frame, latency_frame as original_latency_frame
from complete_geometry import configuration
from pose_protocol import SCAN_BUILD_ID, decode_pose, visual_packets, pose_packets


def c128_frame(*, observation=True, guard_extra=0, **kwargs):
    data = list(struct.unpack('<14I', fixtures.view_frame(**kwargs)[4:-2]))
    data[1] = SCAN_BUILD_ID
    data[12] |= (7 << 23) | (int(observation) << 26) | guard_extra
    return fpga_frame(8, data)


def latency_frame(*, observation=True, **kwargs):
    words = list(struct.unpack('<16I', original_latency_frame(**kwargs)[4:-2]))
    words[2] |= int(observation) << 20
    return fpga_frame(6, words)


class VisualSessionTests(unittest.TestCase):
    setUp = fixtures.C127SessionTests.setUp
    tearDown = fixtures.C127SessionTests.tearDown
    wait = fixtures.C127SessionTests.wait
    make = fixtures.C127SessionTests.make

    def put(self, session, **kwargs):
        session._ports['fpga'].incoming.put(c128_frame(**kwargs))
        self.wait(lambda: (session.snapshot()['latest_pose_status'] or {}).get('board_time_us') == kwargs.get('board_us', 100_000_000))

    def prepared(self, **kwargs):
        kwargs.setdefault('status_stale_s', 10)
        session = self.make(**kwargs)
        self.put(session)
        session.set_pose_mode('normal')
        session._last_words = tuple(configuration(session.calibration))
        session._view_versions[3].update(lens_configuration_owned=True, context_hash=session.scan_context()['context_hash'])
        session._view_displayed = dict(raw=False, observation=True, version=3, source_frame_id=30, output_first_us=99_999_000)
        session.start_visual_scan()
        return session

    def candidate(self, session, number=1, offset=.05):
        return dict(schema='c128_visual_candidate_v1', context_hash=session.scan_context()['context_hash'],
            H=[[1., 0, offset], [0, 1., 0], [0, 0, 1.]],
            quality=dict(accepted=True, corners=50, rms_target_pixels=1.),
            target_grid=dict(pattern_size=[10, 5], cell_pixels=64., inner_corner_span_cells=[9, 4]),
            observation=dict(source='c128_raw_inset', domain='raw', live=True,
                source_size=[1280, 720], observation_size=[320, 180], source_transform=[4, 0, 1.5, 0, 4, 1.5],
                source_frame_id=number, source_capture_us=99_900_000+number*1000,
                output_version=3, metadata_crc_valid=True, received_at=session._clock(), capture_frame_id=f'1:{number}'))

    def offer_and_wait(self, session, number=1, offset=.05):
        count = len(session._ports['fpga'].writes)
        request_id = session.offer_visual_scan(self.candidate(session, number, offset))
        self.wait(lambda: len(session._ports['fpga'].writes) > count)
        self.wait(lambda: session._active and 'write_done_at' in session._active)
        return request_id

    def test_explicit_visual_wire_and_no_quaternion_or_repeated_static_writes(self):
        s = self.prepared()
        time.sleep(.06)
        self.assertEqual(s._ports['fpga'].writes, [])
        request_id = self.offer_and_wait(s)
        first = s._ports['fpga'].writes[0]
        self.assertEqual(len(first), 88)
        self.assertEqual(first[1], 0xFC)
        self.assertEqual(struct.unpack('<I', first[2:6])[0], 0x56490003)
        self.assertEqual(s._active['request_kind'], 'visual_scan')
        self.assertEqual(s._active['request_source'], 'visual')
        self.assertEqual(s._active['transaction'], 'visual_only')
        self.assertNotIn('pose_age_at_write_s', s._active)
        for field in ('sample', 'reference', 'quaternion', 'mounting'):
            self.assertNotIn(field, s._active)
        self.assertFalse(s.snapshot()['imu']['connected'])
        self.put(s, version=4, commits=8, board_us=100_020_000)
        self.assertIsNone(s.snapshot()['visual_scan']['output_confirmed'])
        s._ports['fpga'].incoming.put(latency_frame(version=4, frame_id=40, observation=False))
        time.sleep(.02)
        self.assertIsNone(s.snapshot()['visual_scan']['output_confirmed'])
        s._ports['fpga'].incoming.put(latency_frame(version=4, frame_id=41, base=100_000_100))
        self.wait(lambda: s.snapshot()['visual_scan']['output_confirmed'] is not None)
        evidence = s.snapshot()['visual_scan']['output_confirmed']
        self.assertEqual(evidence['request_id'], request_id)
        self.assertEqual(evidence['observed_source_frame_id'], 1)
        self.assertEqual(evidence['rendered_source_frame_id'], 41)
        self.assertNotEqual(evidence['observed_source_frame_id'], evidence['rendered_source_frame_id'])
        time.sleep(.08)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertTrue(s.snapshot()['continuous'])
        s.stop()

    def test_new_source_with_unchanged_quantized_H_does_not_reupload(self):
        s = self.prepared()
        self.offer_and_wait(s)
        self.put(s, version=4, commits=8, board_us=100_020_000)
        s._ports['fpga'].incoming.put(latency_frame(version=4))
        self.wait(lambda: s.snapshot()['visual_scan']['output_confirmed'] is not None)
        request_id = s.offer_visual_scan(self.candidate(s, 2))
        self.wait(lambda: (s.snapshot()['request'] or {}).get('request_id') == request_id and s.snapshot()['request']['phase'] == 'quantized_unchanged')
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertTrue(s.snapshot()['visual_scan']['enabled'])
        self.offer_and_wait(s, 3, .06)
        self.assertEqual(len(s._ports['fpga'].writes), 2)
        s.stop()

    def test_source_age_is_rechecked_before_write(self):
        s = self.prepared()
        request = self.candidate(s)
        # This helper evaluates the real send-time age gate separately from
        # host arrival age: a fresh USB buffer cannot rejuvenate old exposure.
        internal = dict(request_source='visual', observation=request['observation'])
        self.assertLess(s._candidate_age(internal), s._visual_age_s)
        internal['observation']['source_capture_us'] = 90_000_000
        self.assertEqual(s._candidate_age(internal), float('inf'))
        s.stop()

    def test_latest_unsent_candidate_replaces_old(self):
        s = self.prepared()
        self.offer_and_wait(s, 1)
        s.offer_visual_scan(self.candidate(s, 2, .06))
        last_id = s.offer_visual_scan(self.candidate(s, 3, .08))
        self.assertEqual(s._candidate['observation']['source_frame_id'], 3)
        self.assertEqual(s._candidate['request_id'], last_id)
        self.put(s, version=4, commits=8, board_us=100_020_000)
        self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        self.assertEqual(s._active['observation']['source_frame_id'], 3)
        self.assertEqual(s._active['request_id'], last_id)
        s.stop()

    def test_bad_stale_feedback_and_unowned_version_are_rejected(self):
        s = self.prepared()
        changes = [dict(source='scan_output'), dict(domain='rectified'), dict(live=False),
                   dict(metadata_crc_valid=False), dict(received_at=s._clock()-1),
                   dict(source_capture_us=90_000_000), dict(output_version=999),
                   dict(source_transform=[4, 0, 0, 0, 4, 0])]
        for change in changes:
            with self.subTest(change=change):
                candidate = self.candidate(s)
                candidate['observation'].update(change)
                with self.assertRaises(ValueError):
                    s.offer_visual_scan(candidate)
        self.assertEqual(s._ports['fpga'].writes, [])
        route = s._view_versions[3]
        saved_context = route['context_hash']
        route['context_hash'] = 'obsolete-lens-context'
        with self.assertRaisesRegex(ValueError, 'known observation-route'):
            s.offer_visual_scan(self.candidate(s))
        route['context_hash'] = saved_context
        route['lens_configuration_owned'] = False
        with self.assertRaisesRegex(ValueError, 'known observation-route'):
            s.offer_visual_scan(self.candidate(s))
        route['lens_configuration_owned'] = True
        self.offer_and_wait(s)
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            s.offer_visual_scan(self.candidate(s))
        candidate = self.candidate(s, 2)
        candidate['observation']['source_capture_us'] = 99_901_000
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            s.offer_visual_scan(candidate)
        s.stop()

    def test_tracking_loss_drops_pending_unsent_only(self):
        s = self.prepared()
        self.offer_and_wait(s, 1)
        s.offer_visual_scan(self.candidate(s, 2, .1))
        s.report_visual_tracking(dict(state='lost', reason='occluded'))
        self.assertIsNone(s._candidate)
        self.assertEqual(s._active['phase'], 'await_accepted')
        self.put(s, version=4, commits=8, board_us=100_020_000)
        s._ports['fpga'].incoming.put(latency_frame(version=4, frame_id=40))
        self.wait(lambda: s.snapshot()['visual_scan']['output_confirmed'] is not None)
        self.assertEqual(s.snapshot()['visual_scan']['tracking']['state'], 'lost')
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        s.stop()

    def test_proven_geometry_reject_waits_for_new_source_then_lat(self):
        s = self.prepared()
        retained = s._last_words
        self.offer_and_wait(s)
        self.put(s, commits=8, control_failures=1, guard_extra=2, board_us=100_020_000)
        self.wait(lambda: s._active is None and s.snapshot()['geometry_recovery'] is not None)
        self.assertFalse(s.snapshot()['frozen'])
        self.assertEqual(s._last_words, retained)
        time.sleep(.06)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.offer_and_wait(s, 2, .07)
        self.assertTrue(s._active.get('geometry_recovery_probe'))
        self.put(s, version=4, commits=9, control_failures=1, board_us=100_040_000)
        self.assertIsNotNone(s.snapshot()['geometry_recovery'])
        s._ports['fpga'].incoming.put(latency_frame(version=4, frame_id=42, base=100_030_000))
        self.wait(lambda: s.snapshot()['geometry_recovery'] is None)
        self.assertTrue(s.snapshot()['visual_scan']['enabled'])
        s.stop()

    def test_unknown_acceptance_timeout_never_visual_retries(self):
        s = self.prepared(accept_timeout_s=.03)
        self.offer_and_wait(s)
        self.wait(lambda: s.snapshot()['frozen'])
        self.assertFalse(s.snapshot()['visual_scan']['enabled'])
        with self.assertRaisesRegex(RuntimeError, 'not running'):
            s.offer_visual_scan(self.candidate(s, 2))
        self.assertEqual(len(s._ports['fpga'].writes), 1)

    def test_route_requires_publication_and_lat(self):
        s = self.make()
        self.put(s, observation=False)
        s.set_pose_mode('normal')
        s._last_words = tuple(configuration(s.calibration))
        s.request_output_route(raw=False, observation=True)
        self.wait(lambda: s._ports['fpga'].writes)
        self.assertEqual(s._ports['fpga'].writes[0][1], 0xEC)
        self.put(s, observation=True, version=4, key=True, board_us=100_020_000)
        self.wait(lambda: s._view_command is None)
        with self.assertRaisesRegex(RuntimeError, 'LAT'):
            s.start_visual_scan()
        s._ports['fpga'].incoming.put(latency_frame(version=4))
        self.wait(lambda: (s.snapshot()['scan']['displayed'] or {}).get('observation'))
        s.start_visual_scan()
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        s.stop()

    def test_noop_route_without_owned_baseline_and_raw_full_bootstrap(self):
        s = self.make()
        self.put(s, observation=False)
        s.request_output_route(raw=False, observation=False)
        self.assertIsNone(s._view_command)
        self.assertEqual(s._ports['fpga'].writes, [])
        raw = self.make()
        self.put(raw, observation=False, raw=True)
        raw.request_output_route(raw=False, observation=False)
        self.wait(lambda: raw._ports['fpga'].writes)
        self.assertEqual(len(raw._ports['fpga'].writes[0]), 232)
        self.assertEqual(raw._ports['fpga'].writes[0][1], 0xFE)
        self.put(raw, observation=False, raw=False, version=4, commits=8, board_us=100_020_000)
        raw._ports['fpga'].incoming.put(latency_frame(version=4, observation=False))
        self.wait(lambda: (raw.snapshot()['request'] or {}).get('phase') == 'output_confirmed')
        self.assertIsNotNone(raw._last_words)
        self.assertFalse(raw.snapshot()['scan']['displayed']['raw'])

    def test_visual_wire_distinct_and_capabilities_decode(self):
        words = configuration(self.make(start=False).calibration)
        visual, pose = visual_packets(words, 65535), pose_packets(words, 65535)
        self.assertEqual(visual[8:], pose[8:])
        self.assertNotEqual(visual[:8], pose[:8])
        data = struct.unpack('<14I', c128_frame()[4:-2])
        status = decode_pose(data)
        for field in ('software_route_capable', 'observation_capable', 'visual_matrix_capable', 'observation_active'):
            self.assertTrue(status[field])

    def static_unowned(self):
        session = self.make()
        self.put(session, observation=False, context=123, matrix=True, plane=True, requested=3, committed=3)
        self.assertIsNone(session._last_words)
        return session

    def test_initial_static_scan_mirror_EA_is_only_a_full_baseline_prerequisite(self):
        s = self.static_unowned()
        s.request_normal()
        self.wait(lambda: s._view_command and 'write_done_at' in s._view_command)
        self.assertEqual(s._ports['fpga'].writes[0][1], 0xEA)
        self.put(s, observation=False, version=4, key=True, board_us=100_020_000)
        self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        self.assertFalse(s.snapshot()['frozen'])
        self.assertEqual(s._ports['fpga'].writes[1][1], 0xFE)
        self.assertEqual(len(s._ports['fpga'].writes[1]), 232)
        self.assertFalse(s._view_versions[4]['lens_configuration_owned'])
        self.assertIsNone(s._last_words)
        self.assertEqual(s.snapshot()['version_evidence']['published'], 0)
        self.put(s, observation=False, version=5, commits=8, board_us=100_040_000)
        s._ports['fpga'].incoming.put(latency_frame(version=5, observation=False, base=100_030_000))
        self.wait(lambda: (s.snapshot()['request'] or {}).get('phase') == 'output_confirmed')
        self.assertIsNotNone(s._last_words)
        self.assertTrue(s._view_versions[5]['lens_configuration_owned'])
        self.assertEqual(len(s._ports['fpga'].writes), 2)

    def test_initial_loaded_but_disabled_static_table_also_requires_EA_then_FE(self):
        s = self.make()
        self.put(s, observation=False, context=123, matrix=True, plane=True)
        s.request_normal()
        self.wait(lambda: s._view_command and 'write_done_at' in s._view_command)
        self.put(s, observation=False, version=4, key=True, board_us=100_020_000)
        self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        self.assertEqual([wire[1] for wire in s._ports['fpga'].writes], [0xEA, 0xFE])
        self.assertFalse(s.snapshot()['frozen'])
        self.assertIsNone(s._last_words)

    def test_same_scale_static_comparison_uses_identity_and_preserves_context_without_imu(self):
        from roll_crop import DEMO_ROLL_CROP
        s = self.make()
        self.put(s, observation=False)
        s.set_pose_mode('roll_only', crop_profile=DEMO_ROLL_CROP.to_dict())
        s._reference = dict(reference_id=7, sample=dict(quaternion=[1., 0., 0., 0.]))
        s._last_words = tuple(configuration(s.calibration, zoom=DEMO_ROLL_CROP.zoom,
            matrix=[[.9, -.1, 0], [.1, .9, 0], [0, 0, 1]]))
        before = s.snapshot()
        s.request_static_comparison()
        self.wait(lambda: s._active and 'write_done_at' in s._active)
        request = deepcopy(s._active)
        self.assertEqual(request['request_kind'], 'imu_static_comparison')
        self.assertNotIn('sample', request)
        self.assertEqual(request['words'][:18], list(s._last_words[:18]))
        self.assertEqual(request['words'][18:], [65536, 0, 0, 0, 65536, 0, 0, 0, 65536])
        self.assertEqual(len(s._ports['fpga'].writes[0]), 232)
        self.assertFalse(s.snapshot()['continuous'])
        self.assertEqual(s.snapshot()['reference'], before['reference'])
        self.assertEqual(s.snapshot()['crop_profile'], before['crop_profile'])
        self.assertEqual(s.snapshot()['enhancement'], before['enhancement'])
        self.put(s, observation=False, version=4, commits=8, board_us=100_020_000)
        s._ports['fpga'].incoming.put(latency_frame(version=4, observation=False))
        self.wait(lambda: (s.snapshot()['request'] or {}).get('phase') == 'output_confirmed')
        self.assertFalse(s.snapshot()['frozen'])
        self.assertEqual(s.snapshot()['pose_mode'], 'roll_only')

    def test_initial_EA_ack_before_write_drain_waits_without_ownership(self):
        s = self.static_unowned()
        s._ports['fpga'].out_waiting = 1
        try:
            s.request_normal()
            self.wait(lambda: s._ports['fpga'].writes)
            self.put(s, observation=False, version=4, key=True, board_us=100_020_000)
            self.assertFalse(s.snapshot()['frozen'])
            self.assertIn('deferred_bootstrap_publication', s._view_command)
            self.assertEqual(s._status['config_version'], 3)
            self.assertIsNone(s._candidate)
            self.assertIsNone(s._last_words)
        finally:
            s._ports['fpga'].out_waiting = 0
        self.wait(lambda: len(s._ports['fpga'].writes) == 2)
        self.assertEqual(s._ports['fpga'].writes[1][1], 0xFE)
        self.assertFalse(s.snapshot()['frozen'])

    def test_initial_EA_drain_timeout_never_releases_full_baseline(self):
        s = self.static_unowned()
        s._drain_timeout_s = .05
        s._ports['fpga'].out_waiting = 1
        try:
            s.request_normal()
            self.wait(lambda: s._ports['fpga'].writes)
            self.put(s, observation=False, version=4, key=True, board_us=100_020_000)
            self.wait(lambda: s.snapshot()['frozen'])
            self.assertEqual(len(s._ports['fpga'].writes), 1)
            self.assertIsNone(s._last_words)
            self.assertIsNone(s._candidate)
            self.assertEqual(s.snapshot()['version_evidence']['published'], 0)
        finally:
            s._ports['fpga'].out_waiting = 0

    def test_unowned_external_or_mismatched_EA_publications_still_freeze(self):
        external = self.static_unowned()
        self.put(external, observation=False, version=4, key=True, board_us=100_020_000)
        self.assertTrue(external.snapshot()['frozen'])
        self.assertEqual(external._ports['fpga'].writes, [])
        changes = [dict(context=1), dict(requested=1), dict(committed=1), dict(matrix=True),
                   dict(plane=True), dict(raw=True), dict(key=False), dict(commits=8),
                   dict(control_failures=1), dict(version=5), dict(observation=True)]
        for changeset in changes:
            with self.subTest(changeset=changeset):
                s = self.static_unowned()
                s.request_normal()
                self.wait(lambda: s._view_command and 'write_done_at' in s._view_command)
                kwargs = dict(observation=False, version=4, key=True, board_us=100_020_000)
                kwargs.update(changeset)
                self.put(s, **kwargs)
                self.assertTrue(s.snapshot()['frozen'])
                self.assertEqual(len(s._ports['fpga'].writes), 1)
                self.assertIsNone(s._last_words)


if __name__ == '__main__':
    unittest.main()
