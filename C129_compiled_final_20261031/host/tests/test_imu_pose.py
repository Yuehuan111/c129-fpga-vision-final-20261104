import math
from pathlib import Path
import struct
import sys
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_pose import (
    ImuDecoder, PoseHistory, compensation_matrix, matrix_to_quaternion,
    normalize_quaternion, quaternion_to_matrix, slerp, validate_mounting_rotation,
)


def packet(function, values):
    payload = struct.pack("<" + "f" * len(values), *values)
    frame = bytes((0x7E, 0x23, len(payload) + 5, function)) + payload
    return frame + bytes((sum(frame) & 0xFF,))


def axis_quaternion(axis, angle):
    result = [math.cos(angle / 2), 0, 0, 0]
    result[axis + 1] = math.sin(angle / 2)
    return tuple(result)


class DecoderTests(unittest.TestCase):
    def test_noise_fragmented_and_glued_packets(self):
        decoder = ImuDecoder()
        q = packet(0x16, [2, 0, 0, 0])
        euler = packet(0x26, [0.1, -0.2, 0.3])
        self.assertEqual(decoder.feed(b"noise\x7e", 1.0), [])
        self.assertEqual(decoder.feed(q[1:8], 1.1), [])
        result = decoder.feed(q[8:] + euler + q, 1.2)
        self.assertEqual([p["kind"] for p in result], ["quaternion", "euler", "quaternion"])
        self.assertEqual(result[0]["quaternion"], (1, 0, 0, 0))
        np.testing.assert_allclose(result[1]["euler_rad"], [0.1, -0.2, 0.3], atol=1e-7)
        self.assertTrue(all(p["received_at"] == 1.2 for p in result))
        self.assertEqual(decoder.buffered_bytes, 0)

    def test_corrupt_length_checksum_unknown_and_truncation_recovery(self):
        decoder = ImuDecoder()
        valid = packet(0x16, [1, 0, 0, 0])
        bad_checksum = valid[:-1] + bytes(((valid[-1] + 1) & 0xFF,))
        malformed = b"\x7e\x23\xff\x16abc\x7e\x23\xff\x99"
        data = malformed + bad_checksum + valid[:8] + valid
        result = decoder.feed(data, 1.0)
        self.assertEqual(len(result), 1)
        self.assertGreaterEqual(decoder.length_errors, 1)
        self.assertGreaterEqual(decoder.unknown_functions, 1)
        self.assertGreaterEqual(decoder.checksum_errors, 2)
        self.assertEqual(decoder.buffered_bytes, 0)

    def test_invalid_float_frames_do_not_poison_next_pose(self):
        decoder = ImuDecoder()
        bad = ([0, 0, 0, 0], [math.inf, 0, 0, 0], [math.nan, 0, 0, 0],
               [1e30, 0, 0, 0], [1e-30, 0, 0, 0])
        stream = b"".join(packet(0x16, q) for q in bad)
        stream += packet(0x26, [0, math.inf, 0]) + packet(0x16, [1, 0, 0, 0])
        self.assertEqual(len(decoder.feed(stream, 2.0)), 1)
        self.assertEqual(decoder.invalid_values, 6)

    def test_bytewise_decode_and_buffer_bound(self):
        decoder = ImuDecoder()
        frame = packet(0x16, axis_quaternion(2, math.pi / 2))
        result = []
        for index, byte in enumerate(b"garbage" + frame):
            result += decoder.feed(bytes((byte,)), float(index))
            self.assertLessEqual(decoder.buffered_bytes, 20)
        self.assertEqual(len(result), 1)
        np.testing.assert_allclose(quaternion_to_matrix(result[0]["quaternion"]) @ [1, 0, 0], [0, 1, 0], atol=1e-6)

    def test_receive_time_validation(self):
        decoder = ImuDecoder()
        decoder.feed(b"", 5)
        for bad_time in (4, math.nan, math.inf):
            with self.assertRaises(ValueError):
                decoder.feed(b"", bad_time)
        with self.assertRaises(TypeError):
            decoder.feed(21, 6)


class RotationTests(unittest.TestCase):
    def test_invalid_quaternion_values_and_extreme_norms(self):
        for q in ([0, 0, 0, 0], [math.inf, 0, 0, 0], [math.nan, 0, 0, 0],
                  [1e308, 1e308, 0, 0], [1e-300, 0, 0, 0], [1, 0, 0], [[1, 0, 0, 0]]):
            with self.subTest(q=q), self.assertRaises(ValueError):
                normalize_quaternion(q)
        self.assertEqual(normalize_quaternion([3, 0, 0, 0]), (1, 0, 0, 0))

    def test_axis_positive_and_negative_compensation(self):
        reference = [1, 0, 0, 0]
        positive_rotations = (
            np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
            np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
            np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
        )
        for axis, expected in enumerate(positive_rotations):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    q = axis_quaternion(axis, sign * math.pi / 2)
                    r = expected if sign == 1 else expected.T
                    np.testing.assert_allclose(quaternion_to_matrix(q), r, atol=1e-14)
                    np.testing.assert_allclose(compensation_matrix(q, reference, np.eye(3)), r.T, atol=1e-14)
                    np.testing.assert_allclose(compensation_matrix(q, reference, np.eye(3), "world_to_imu"), r, atol=1e-14)

    def test_mounting_conjugation_and_reference_identity(self):
        mounting = quaternion_to_matrix(axis_quaternion(0, math.pi / 2))
        now = axis_quaternion(2, 0.7)
        reference = axis_quaternion(1, -0.4)
        expected = mounting.T @ quaternion_to_matrix(now).T @ quaternion_to_matrix(reference) @ mounting
        np.testing.assert_allclose(compensation_matrix(now, reference, mounting), expected, atol=1e-14)
        np.testing.assert_allclose(compensation_matrix(now, now, mounting), np.eye(3), atol=1e-14)
        # +IMU Z with this mounting is +camera Y, so compensation is -camera Y.
        np.testing.assert_allclose(compensation_matrix(now, [1, 0, 0, 0], mounting),
                                   quaternion_to_matrix(axis_quaternion(1, -0.7)), atol=1e-14)

    def test_bad_mounting_and_convention_rejected(self):
        invalid = (np.eye(2), np.diag([1, 1, -1]), np.diag([1, 2, 1]),
                   [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]], np.full((3, 3), math.inf),
                   np.eye(3) * 1e308)
        for mounting in invalid:
            with self.subTest(mounting=mounting), self.assertRaises(ValueError):
                validate_mounting_rotation(mounting)
        with self.assertRaises(ValueError):
            compensation_matrix([1, 0, 0, 0], [1, 0, 0, 0], np.eye(3), "unclear")

    def test_double_cover_and_conversion_near_pi(self):
        for axis in range(3):
            for angle in (0, 0.3, math.pi - 1e-8, math.pi, math.pi + 1e-8):
                q = axis_quaternion(axis, angle)
                r = quaternion_to_matrix(q)
                np.testing.assert_allclose(quaternion_to_matrix([-v for v in q]), r, atol=1e-14)
                np.testing.assert_allclose(quaternion_to_matrix(matrix_to_quaternion(r)), r, atol=1e-14)

    def test_slerp_shortest_arc_and_no_extrapolation(self):
        q = axis_quaternion(2, 0.8)
        np.testing.assert_allclose(slerp(q, [-v for v in q], 0.4), q, atol=1e-14)
        q0 = axis_quaternion(2, math.radians(170))
        q1 = axis_quaternion(2, math.radians(-170))
        np.testing.assert_allclose(quaternion_to_matrix(slerp(q0, q1, 0.5)),
                                   quaternion_to_matrix(axis_quaternion(2, math.pi)), atol=1e-14)
        for amount in (-0.1, 1.1, math.nan):
            with self.assertRaises(ValueError):
                slerp(q0, q1, amount)


class HistoryTests(unittest.TestCase):
    def test_bounded_history_ids_sign_continuity_and_rate(self):
        history = PoseHistory(maxlen=3)
        self.assertIsNone(history.latest())
        self.assertEqual(history.rate_hz(), 0)
        for index in range(5):
            q = axis_quaternion(0, 0.1 * index)
            if index % 2:
                q = [-v for v in q]
            sample = history.add(q, 10 + index * 0.01)
            self.assertEqual(sample.id, index + 1)
        samples = history.snapshot()
        self.assertEqual([s.id for s in samples], [3, 4, 5])
        self.assertEqual(len(history), 3)
        self.assertTrue(all(np.dot(a.quaternion, b.quaternion) >= 0 for a, b in zip(samples, samples[1:])))
        self.assertAlmostEqual(history.rate_hz(), 100)
        self.assertAlmostEqual(history.rate_hz(0.015), 100)
        self.assertIsNone(history.interpolate(10))

    def test_history_slerp_no_extrapolation_and_exact_endpoints(self):
        history = PoseHistory()
        first = history.add(axis_quaternion(2, 0), 10)
        last = history.add(axis_quaternion(2, math.pi / 2), 12)
        self.assertEqual(history.interpolate(10), first)
        self.assertEqual(history.interpolate(12), last)
        mid = history.interpolate(11)
        self.assertEqual(mid.id, first.id)
        self.assertEqual(mid.received_at, 11)
        np.testing.assert_allclose(mid.quaternion, axis_quaternion(2, math.pi / 4), atol=1e-14)
        self.assertIsNone(history.interpolate(9.99))
        self.assertIsNone(history.interpolate(12.01))
        np.testing.assert_allclose(compensation_matrix(last, first, np.eye(3)),
                                   quaternion_to_matrix(last.quaternion).T, atol=1e-14)

    def test_duplicate_times_choose_latest_and_invalid_samples_do_not_consume_id(self):
        history = PoseHistory()
        history.add([1, 0, 0, 0], 1)
        latest = history.add(axis_quaternion(1, 0.2), 1)
        self.assertEqual(history.interpolate(1), latest)
        self.assertEqual(history.rate_hz(), 0)
        for q, timestamp in (([1, 0, 0, 0], 0), ([0, 0, 0, 0], 2), ([1, 0, 0, 0], math.inf)):
            with self.assertRaises(ValueError):
                history.add(q, timestamp)
        self.assertEqual(history.add([1, 0, 0, 0], 2).id, 3)
        for window in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                history.rate_hz(window)

    def test_concurrent_producers_and_readers(self):
        history = PoseHistory(maxlen=100)
        start = threading.Barrier(4)

        def writer():
            start.wait()
            for index in range(150):
                history.add(axis_quaternion(0, index * 0.001))

        def reader():
            start.wait()
            for _ in range(300):
                snapshot = history.snapshot()
                if snapshot:
                    self.assertTrue(all(a.id < b.id and a.received_at <= b.received_at
                                        for a, b in zip(snapshot, snapshot[1:])))
                    history.interpolate(snapshot[-1].received_at)
                history.latest()
                history.rate_hz()

        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(writer), executor.submit(writer),
                       executor.submit(reader), executor.submit(reader)]
            for future in futures:
                future.result(timeout=10)
        self.assertEqual(history.latest().id, 300)
        self.assertEqual(len(history), 100)


if __name__ == "__main__":
    unittest.main()
