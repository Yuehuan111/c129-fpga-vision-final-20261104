"""C129 optional detail protocol and frame-attribution tests; fake UART only."""
from copy import deepcopy
from pathlib import Path
import struct
import sys
import time
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import test_c127_session as fixtures
import test_visual_scan_session as visual
from test_imu_session import fpga_frame
from complete_geometry import configuration
from pose_protocol import DETAIL_BUILD_ID, SCAN_BUILD_IDS, decode_pose, verify_pose


def detail_frame(*, detail=False, capable=True, **kwargs):
    words = list(struct.unpack('<14I', visual.c128_frame(**kwargs)[4:-2]))
    words[1] = DETAIL_BUILD_ID
    words[12] |= (int(capable) << 27) | (int(detail) << 28)
    return fpga_frame(8, words)


def output_frame(*, detail=False, raw=False, **kwargs):
    words = list(struct.unpack('<16I', visual.latency_frame(**kwargs)[4:-2]))
    words[2] |= (int(detail) << 21) | (int(raw) << 22)
    return fpga_frame(6, words)


class DetailSessionTests(unittest.TestCase):
    setUp = fixtures.C127SessionTests.setUp
    tearDown = fixtures.C127SessionTests.tearDown
    wait = fixtures.C127SessionTests.wait
    make = fixtures.C127SessionTests.make
    candidate = visual.VisualSessionTests.candidate

    def put(self, session, **kwargs):
        session._ports['fpga'].incoming.put(detail_frame(**kwargs))
        self.wait(lambda: (session.snapshot()['latest_pose_status'] or {}).get('board_time_us') == kwargs.get('board_us', 100_000_000))

    def ordinary(self, **kwargs):
        session = self.make(status_stale_s=10)
        self.put(session, observation=False, **kwargs)
        session.set_pose_mode('normal')
        session._last_words = tuple(configuration(session.calibration))
        return session

    def test_capability_decode_and_exact_build_acceptance(self):
        status = decode_pose(struct.unpack('<14I', detail_frame(detail=True)[4:-2]))
        verify_pose(status)
        self.assertIn(DETAIL_BUILD_ID, SCAN_BUILD_IDS)
        self.assertTrue(status['detail_capable'])
        self.assertTrue(status['detail_active'])
        status['build_id'] = 0x0720C130
        with self.assertRaises(ValueError): verify_pose(status)

    def test_E4_single_writer_actual_frame_ACK_and_matching_LAT_required(self):
        s = self.ordinary()
        words_before = s._last_words
        s.request_detail_enhancement(True)
        self.wait(lambda: s._view_command and 'write_done_at' in s._view_command)
        wire = s._ports['fpga'].writes[0]
        self.assertEqual(len(wire), 8)
        self.assertEqual(wire[1], 0xE4)
        self.assertEqual(struct.unpack('<I', wire[2:6])[0], 1)
        self.put(s, observation=False, detail=False, board_us=100_010_000)
        self.assertIsNotNone(s._view_command)
        self.put(s, observation=False, detail=True, board_us=100_020_000)
        self.wait(lambda: s._view_command is None)
        self.assertTrue(s.snapshot()['detail']['active'])
        self.assertEqual(s.snapshot()['detail']['output_pipeline_delay_pixclk'], 1656)
        self.assertEqual(s.snapshot()['detail_control']['output_pipeline_delay_pixclk'], 1656)
        self.assertFalse(s.snapshot()['detail_control']['output_confirmed'])
        for index, kw in enumerate([dict(detail=False), dict(detail=True, raw=True), dict(detail=True, observation=True),
                                    dict(detail=True, version=4), dict(detail=True, base=99_000_000)]):
            args = dict(version=3, frame_id=40+index, base=100_030_000, observation=False)
            args.update(kw)
            s._ports['fpga'].incoming.put(output_frame(**args))
        time.sleep(.03)
        self.assertFalse(s.snapshot()['detail_control']['output_confirmed'])
        s._ports['fpga'].incoming.put(output_frame(version=3, frame_id=50, base=100_040_000, observation=False, detail=True))
        self.wait(lambda: s.snapshot()['detail_control']['output_confirmed'])
        self.assertEqual(s.snapshot()['detail_control']['source_frame_id'], 50)
        self.assertEqual(s._last_words, words_before)
        self.assertEqual(s.snapshot()['last_status']['commits'], 7)
        self.assertEqual(s.snapshot()['last_status']['config_version'], 3)
        self.assertEqual(len(s._ports['fpga'].writes), 1)
        self.assertFalse(s.snapshot()['frozen'])

    def test_invalid_unsupported_raw_and_same_value_never_write(self):
        s = self.ordinary()
        for value in [0, 1, -1, None, 'on']:
            with self.assertRaises(ValueError): s.request_detail_enhancement(value)
        s.request_detail_enhancement(False)
        self.assertEqual(s._ports['fpga'].writes, [])
        s2 = self.ordinary(capable=False)
        with self.assertRaisesRegex(RuntimeError, 'capability'): s2.request_detail_enhancement(True)
        s3 = self.ordinary(raw=True)
        with self.assertRaisesRegex(RuntimeError, 'Raw'): s3.request_detail_enhancement(True)
        self.assertEqual(s2._ports['fpga'].writes, [])
        self.assertEqual(s3._ports['fpga'].writes, [])

    def test_inflight_geometry_is_not_interleaved_with_E4(self):
        s = self.ordinary()
        with s._lock:
            s._pending_outputs.append({'request_id': 1})
            with self.assertRaisesRegex(RuntimeError, 'Wait for current'): s.request_detail_enhancement(True)
            s._pending_outputs.clear()
        self.assertEqual(s._ports['fpga'].writes, [])

    def test_failed_drain_cannot_use_early_detail_ACK_as_success(self):
        s = self.ordinary()
        s._drain_timeout_s = .1
        s._ports['fpga'].out_waiting = 1
        try:
            s.request_detail_enhancement(True)
            self.wait(lambda: s._ports['fpga'].writes)
            self.put(s, observation=False, detail=True, board_us=100_020_000)
            self.assertIsNone(s.snapshot()['detail_control'])
            self.wait(lambda: s.snapshot()['frozen'])
            self.assertIsNone(s.snapshot()['detail_control'])
            self.assertEqual(len(s._ports['fpga'].writes), 1)
        finally:
            s._ports['fpga'].out_waiting = 0

    def test_C129_visual_source_and_frame_flags_preserve_C128_contract(self):
        s = self.make(status_stale_s=10)
        self.put(s, observation=True)
        s.set_pose_mode('normal')
        s._last_words = tuple(configuration(s.calibration))
        s._view_versions[3].update(lens_configuration_owned=True, context_hash=s.scan_context()['context_hash'])
        s._view_displayed = dict(raw=False, observation=True, version=3, source_frame_id=30, output_first_us=99_999_000)
        s.start_visual_scan()
        s.offer_visual_scan(self.candidate(s))
        self.wait(lambda: s._active and 'write_done_at' in s._active)
        self.assertEqual(s._ports['fpga'].writes[0][1], 0xFC)
        self.put(s, observation=True, version=4, commits=8, board_us=100_020_000)
        s._ports['fpga'].incoming.put(output_frame(version=4, raw=True))
        time.sleep(.02)
        self.assertIsNone(s.snapshot()['visual_scan']['output_confirmed'])
        s._ports['fpga'].incoming.put(output_frame(version=4, observation=False, frame_id=32))
        time.sleep(.02)
        self.assertIsNone(s.snapshot()['visual_scan']['output_confirmed'])
        s._ports['fpga'].incoming.put(output_frame(version=4, observation=True, frame_id=33))
        self.wait(lambda: s.snapshot()['visual_scan']['output_confirmed'])
        s.stop()

    def test_C129_raw_route_requires_actual_LAT_raw_tag(self):
        s = self.ordinary()
        s.request_raw(True)
        self.wait(lambda: s._view_command and 'write_done_at' in s._view_command)
        self.put(s, observation=False, raw=True, key=True, version=4, board_us=100_020_000)
        s._ports['fpga'].incoming.put(output_frame(version=4, observation=False, raw=False))
        time.sleep(.02)
        self.assertIsNone(s.snapshot()['scan']['displayed'])
        s._ports['fpga'].incoming.put(output_frame(version=4, observation=False, raw=True, frame_id=32))
        self.wait(lambda: s.snapshot()['scan']['displayed'])
        self.assertTrue(s.snapshot()['scan']['displayed']['raw'])
        self.assertFalse(s.snapshot()['scan']['displayed']['detail_active'])


if __name__ == '__main__': unittest.main()
