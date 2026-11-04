"""C124 host wire/geometry contract tests; no serial port is imported or opened."""
import binascii
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
import camera_config
import complete_control as control
import complete_geometry as geometry
from read_latency import LatencyAnalysis, MixedFramer


def telemetry_packet(version, words):
    """Independent encoder using the standard-library CCITT implementation."""
    payload = struct.pack("<" + "I" * len(words), *[word & 0xFFFFFFFF for word in words])
    protected = bytes((0x55, 0xAA, version, len(payload))) + payload
    return protected + binascii.crc_hqx(protected, 0xFFFF).to_bytes(2, "big")


def status_packet(tick=1, version=7, build=0x0720C124):
    # C124: mode=4, direction=2, gamma=5, all three initialization flags set.
    flags = (4 << 12) | (2 << 10) | (5 << 7) | 0x70
    words = [build, (flags << 16) | version, 30 * tick, 29 * tick,
             0, 0, 0, 8, 0, 1234, 30 * tick, tick]
    words += [0] * (51 - len(words))
    return telemetry_packet(5, words)


def latency_packet(frame_id=42, version=7, first=0xFFFFFFE0, flags=1):
    # Stage timestamps cross the 32-bit microsecond wrap boundary.
    words = [0x4C415431, frame_id, (flags << 16) | version, 1000, first]
    words += [(first + delta) & 0xFFFFFFFF for delta in (10, 20, 50, 70, 100)]
    words += [2, 0, 0, 0, 0, 0]
    return telemetry_packet(6, words)


class C124WireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calibration = geometry.load_calibration()

    def test_crc_reference_vector_and_packet_byte_order(self):
        self.assertEqual(camera_config.crc16(b"123456789"), 0x29B1)
        frame = camera_config.packet(0x21, -16384)
        self.assertEqual(frame[:6], b"\xa5\x21\x00\xc0\xff\xff")
        self.assertEqual(int.from_bytes(frame[6:], "big"), binascii.crc_hqx(frame[:6], 0xFFFF))

    def test_full_transaction_has_exact_addresses_scaling_and_signed_values(self):
        matrix = [[1, -0.25, 0], [0.125, 1, 0], [0, 0, 1]]
        words = geometry.configuration(self.calibration, matrix=matrix,
                                       legacy_angle=0, gain=1.5, offset=-3, raw=False)
        payload = control.configuration_packets(words)
        self.assertEqual(len(payload), 232)
        frames = [payload[index:index + 8] for index in range(0, len(payload), 8)]
        expected_addresses = [0xFE, *range(0x12), *range(0x20, 0x29), 0xFF]
        self.assertEqual([frame[1] for frame in frames], expected_addresses)
        self.assertEqual(struct.unpack("<I", frames[0][2:6])[0], 0x434F4D50)
        self.assertEqual(struct.unpack("<I", frames[-1][2:6])[0], 0x434F4D54)
        for address, frame in zip(expected_addresses, frames):
            with self.subTest(address=address):
                self.assertEqual(frame[0], 0xA5)
                self.assertEqual(int.from_bytes(frame[6:], "big"),
                                 binascii.crc_hqx(frame[:6], 0xFFFF))
        decoded = {frame[1]: struct.unpack("<i", frame[2:6])[0] for frame in frames[1:-1]}
        self.assertEqual([decoded[address] for address in expected_addresses[1:-1]], words)
        self.assertEqual([decoded[address] for address in range(8, 12)], [4, 183, 217, -80])
        self.assertEqual([decoded[address] for address in range(12, 18)],
                         [65536, 0, 384, -3, 1, 0x46495348])
        self.assertEqual([decoded[address] for address in range(0x20, 0x29)],
                         [65536, -16384, 0, 8192, 65536, 0, 0, 0, 65536])

    def test_incomplete_words_and_noninteger_values_cannot_be_serialized(self):
        words = geometry.configuration(self.calibration)
        for invalid in (words[:18], words[18:], words[:-1], words + [0],
                        [True] + words[1:], words[:18] + [float("nan")] + words[19:]):
            with self.subTest(length=len(invalid)), self.assertRaises(ValueError):
                control.configuration_packets(invalid)

    def test_c124_enhancement_flags_do_not_use_c121_semantics(self):
        status = MixedFramer().feed(status_packet())[0]
        result = control.enhancement_status(status)
        self.assertEqual(result, {
            "enhancement_mode": 4, "enhancement_mode_name": "histogram",
            "enhancement_direction": 2, "enhancement_direction_name": "darken",
            "gamma_id": 5,
        })
        self.assertTrue(status["ddr_initialized"])
        self.assertTrue(status["camera_config_done"])
        self.assertTrue(status["hdmi_config_done"])
        self.assertFalse(status["map_building"])
        self.assertFalse(status["error_sticky"])
        for mode, direction, gamma in ((0, 0, 3), (1, 1, 2), (2, 1, 0), (3, 2, 5)):
            flags = (mode << 12) | (direction << 10) | (gamma << 7)
            decoded = control.enhancement_status({**status, "flags": flags})
            self.assertEqual((decoded["enhancement_mode"], decoded["enhancement_direction"],
                              decoded["gamma_id"]), (mode, direction, gamma))
        for invalid in ({**status, "build_id": 0x0720C121},
                        {**status, "protocol_version": 4},
                        {**status, "flags": 5 << 12}):
            with self.assertRaises(ValueError):
                control.enhancement_status(invalid)


class TelemetryFramingTests(unittest.TestCase):
    def test_v5_and_lat1_survive_every_two_chunk_cut_and_bytewise_reads(self):
        wire = status_packet() + latency_packet() + status_packet(tick=2)
        self.assertEqual(len(status_packet()), 210)
        self.assertEqual(len(latency_packet()), 70)
        for cut in range(len(wire) + 1):
            framer = MixedFramer()
            records = framer.feed(wire[:cut]) + framer.feed(wire[cut:])
            with self.subTest(cut=cut):
                self.assertEqual([record["packet_type"] for record in records],
                                 ["status", "latency", "status"])
                self.assertEqual(records[0]["build_id"], 0x0720C124)
                self.assertEqual(records[1]["source_frame_id"], 42)
                self.assertEqual(records[1]["config_version"], 7)
                self.assertEqual(records[2]["sample_ticks"], 2)
                self.assertEqual(framer.buffer, b"")
        framer = MixedFramer()
        records = []
        for value in wire:
            records.extend(framer.feed(bytes((value,))))
        self.assertEqual(len(records), 3)

    def test_bad_headers_crc_and_truncated_frames_resynchronize(self):
        bad_status = bytearray(status_packet())
        bad_status[40] ^= 0x80
        bad_latency = bytearray(latency_packet())
        bad_latency[-1] ^= 0x01
        wire = (b"noise\x55\x55\xaa\x06\xff" + bad_status + bad_latency
                + status_packet()[:17] + status_packet(tick=3) + latency_packet(frame_id=43))
        framer = MixedFramer()
        records = []
        for index in range(0, len(wire), 11):
            records.extend(framer.feed(wire[index:index + 11]))
        self.assertEqual([record["packet_type"] for record in records], ["status", "latency"])
        self.assertEqual(records[0]["sample_ticks"], 3)
        self.assertEqual(records[1]["source_frame_id"], 43)
        self.assertGreaterEqual(framer.counters["crc_failures"], 3)
        self.assertGreaterEqual(framer.counters["unsupported_headers"], 1)
        self.assertLessEqual(len(framer.buffer), 1)

    def test_lat1_wrap_is_valid_but_invalid_flags_and_duplicates_are_not(self):
        analysis = LatencyAnalysis()
        framer = MixedFramer()
        analysis.accept(framer.feed(status_packet())[0])
        sample = framer.feed(latency_packet())[0]
        accepted = analysis.accept(sample)
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["complete_frame_latency_us"], 100)
        self.assertEqual(accepted["warp_us"], 30)
        self.assertEqual(analysis.accept(sample)["rejection_reason"], "duplicate_source_frame")
        invalid = framer.feed(latency_packet(frame_id=43, flags=3))[0]
        self.assertEqual(analysis.accept(invalid)["rejection_reason"], "bad_display_frame")


class GeometryAdmissionTests(unittest.TestCase):
    def test_identity_pose_passes_exact_production_cache_admission(self):
        words = geometry.configuration(geometry.load_calibration(),
                                       matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.assertEqual(words[18:], [65536, 0, 0, 0, 65536, 0, 0, 0, 65536])
        report = geometry.check_geometry(tuple(words))
        self.assertEqual((report["width"], report["height"]), (1280, 720))
        self.assertEqual(report["build_id"], "0x0720C124")
        self.assertEqual(report["compiled_tangential_q16"], [-13, 18])
        self.assertTrue(report["fixed_point_checked"])
        self.assertEqual(report["arithmetic_error_pixels"], 0)
        self.assertLess(report["invalid_pixels"], 1280 * 720)
        budget = report["tile_budget"]
        self.assertEqual((budget["tile_width"], budget["tile_height"],
                          budget["cache_width"], budget["cache_height"]), (64, 16, 256, 64))
        self.assertEqual(budget["unsupported_tiles"], [])
        self.assertLessEqual(budget["max_roi_width"], 256)
        self.assertLessEqual(budget["max_roi_height"], 64)
        self.assertEqual(budget["map_read_bytes_per_frame"], 7_372_800)


if __name__ == "__main__":
    unittest.main()
