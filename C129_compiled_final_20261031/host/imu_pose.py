"""IMU byte decoding and camera-ray rotation, with no hardware access.

Quaternion order is [w, x, y, z]. A quaternion normally represents R_WI,
the active rotation from IMU coordinates into world coordinates. R_IC is
the fixed rotation from camera coordinates into IMU coordinates; it must
be measured/confirmed for the actual assembly. An identity R_IC is only
an assumption and does not establish a physical calibration.

Host monotonic receive times include USB/scheduling latency. They are not
sensor exposure or IMU acquisition timestamps. No time extrapolation is
performed here.
"""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections import deque
from dataclasses import dataclass
import math
import struct
import threading
import time
from typing import Iterable

import numpy as np


Quaternion = tuple[float, float, float, float]
MIN_QUATERNION_NORM = 1e-6
MAX_QUATERNION_NORM = 1e6
SO3_TOLERANCE = 1e-6


def _finite_time(value: float | None) -> float:
    result = time.monotonic() if value is None else float(value)
    if not math.isfinite(result):
        raise ValueError("Receive time must be finite and use a monotonic clock")
    return result


def normalize_quaternion(quaternion: Iterable[float]) -> Quaternion:
    """Normalize a finite, nondegenerate wxyz quaternion.

    Norms outside [1e-6, 1e6] are treated as corrupt sensor data, rather
    than silently converting underflow/overflow-scale numbers into a pose.
    Scaling before the norm also avoids overflow during this validation.
    """
    try:
        q = np.asarray(quaternion, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Quaternion must contain four real numbers") from exc
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("Quaternion must be a finite [w, x, y, z] vector")
    scale = float(np.max(np.abs(q)))
    if scale == 0.0 or scale > MAX_QUATERNION_NORM:
        raise ValueError("Quaternion norm is outside the valid range")
    scaled = q / scale
    scaled_norm = float(np.linalg.norm(scaled))
    norm = scale * scaled_norm
    if not MIN_QUATERNION_NORM <= norm <= MAX_QUATERNION_NORM:
        raise ValueError("Quaternion norm is outside the valid range")
    return tuple(float(v) for v in scaled / scaled_norm)


def quaternion_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    """Return the right-handed active 3x3 rotation of a wxyz quaternion."""
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ], dtype=float)


def validate_mounting_rotation(mounting: Iterable[Iterable[float]]) -> np.ndarray:
    """Copy and validate R_IC as SO(3); never repair scale, shear or reflection."""
    try:
        matrix = np.array(mounting, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("R_IC must be a finite 3x3 rotation matrix") from exc
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("R_IC must be a finite 3x3 rotation matrix")
    if np.max(np.abs(matrix)) > 1 + SO3_TOLERANCE:
        raise ValueError("R_IC must be orthonormal with determinant +1")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=SO3_TOLERANCE, rtol=0):
        raise ValueError("R_IC must be orthonormal with determinant +1")
    if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=SO3_TOLERANCE, rel_tol=0):
        raise ValueError("R_IC must be a proper rotation (determinant +1)")
    return matrix


def matrix_to_quaternion(matrix: Iterable[Iterable[float]]) -> Quaternion:
    """Convert an SO(3) active rotation to a normalized wxyz quaternion."""
    r = validate_mounting_rotation(matrix)
    trace = float(np.trace(r))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = (s / 4, (r[2, 1] - r[1, 2]) / s,
             (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    else:
        axis = int(np.argmax(np.diag(r)))
        j, k = (axis + 1) % 3, (axis + 2) % 3
        s = 2 * math.sqrt(max(0.0, 1 + r[axis, axis] - r[j, j] - r[k, k]))
        xyz = [0.0, 0.0, 0.0]
        xyz[axis] = s / 4
        xyz[j] = (r[j, axis] + r[axis, j]) / s
        xyz[k] = (r[k, axis] + r[axis, k]) / s
        q = ((r[k, j] - r[j, k]) / s, *xyz)
    result = normalize_quaternion(q)
    return tuple(-v for v in result) if result[0] < 0 else result


def slerp(first: Iterable[float], second: Iterable[float], fraction: float) -> Quaternion:
    """Interpolate along the shortest quaternion arc; extrapolation is rejected."""
    amount = float(fraction)
    if not math.isfinite(amount) or not 0 <= amount <= 1:
        raise ValueError("SLERP fraction must be finite and in [0, 1]")
    a = np.asarray(normalize_quaternion(first))
    b = np.asarray(normalize_quaternion(second))
    dot = float(a @ b)
    if dot < 0:
        b = -b
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(a + amount * (b - a))
    angle = math.acos(dot)
    result = (math.sin((1 - amount) * angle) * a + math.sin(amount * angle) * b) / math.sin(angle)
    return normalize_quaternion(result)


@dataclass(frozen=True)
class PoseSample:
    """An immutable received pose, or a history interpolation result.

    For an interpolated result, id identifies its left bracketing sample;
    received_at is the requested host time. It is not a new sensor sample.
    """

    id: int
    received_at: float
    quaternion: Quaternion


def compensation_matrix(
    now: Iterable[float] | PoseSample,
    reference: Iterable[float] | PoseSample,
    mounting: Iterable[Iterable[float]],
    convention: str = "imu_to_world",
) -> np.ndarray:
    """Compute H = R_WC_now.T @ R_WC_ref for output-to-source camera rays.

    R_WC = R_WI @ R_IC. For world_to_imu sensor quaternions, transpose
    their rotations first to obtain R_WI. H uses normalized camera rays,
    not pixels; do not premultiply by camera intrinsics in this function.
    A reference pose is software state, not a sensor zero/calibration.
    """
    if convention not in ("imu_to_world", "world_to_imu"):
        raise ValueError("convention must be imu_to_world or world_to_imu")
    r_ic = validate_mounting_rotation(mounting)
    q_now = now.quaternion if isinstance(now, PoseSample) else now
    q_ref = reference.quaternion if isinstance(reference, PoseSample) else reference
    r_now = quaternion_to_matrix(q_now)
    r_ref = quaternion_to_matrix(q_ref)
    if convention == "world_to_imu":
        r_now = r_now.T
        r_ref = r_ref.T
    r_wc_now = r_now @ r_ic
    r_wc_ref = r_ref @ r_ic
    return r_wc_now.T @ r_wc_ref


def camera_relative_roll(now, reference, mounting, convention="imu_to_world") -> float:
    """Camera-relative roll in radians, with forward D = Rz(roll) Ry(yaw) Rx(pitch).

    D maps the current camera basis into the reference camera basis. Extraction
    occurs after the measured R_IC change of basis, never on an arbitrary IMU
    Euler field. This convention ignores pitch/yaw in that decomposition; it
    is reference leveling, not an absolute gravity/horizon measurement.
    At +/-90 degree yaw the roll is unobservable and a new request is rejected.
    """
    forward = compensation_matrix(now, reference, mounting, convention).T
    if math.hypot(float(forward[0, 0]), float(forward[1, 0])) < 1e-6:
        raise ValueError("Camera-relative roll is undefined near 90 degree yaw")
    return math.atan2(float(forward[1, 0]), float(forward[0, 0]))


def mode_compensation_matrix(now=None, reference=None, mounting=None,
                             convention="imu_to_world", mode="full") -> np.ndarray:
    """Return output-to-source rays for normal, roll_only, or full compensation.

    normal uses identity H while retaining the lens model in configuration().
    roll_only uses only inverse camera optical-axis roll. full preserves C125.
    """
    if mode == "normal":
        return np.eye(3)
    if mode == "full":
        return compensation_matrix(now, reference, mounting, convention)
    if mode != "roll_only":
        raise ValueError("Pose mode must be normal, roll_only, or full")
    angle = -camera_relative_roll(now, reference, mounting, convention)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class ImuDecoder:
    """Incremental 7E 23 / total length / function / payload / sum8 decoder.

    0x16 is exactly 21 bytes (four float32 LE wxyz values); 0x26 is
    exactly 17 bytes (three float32 LE radians). Unknown functions and
    malformed lengths are skipped immediately so a corrupt length cannot
    hide the next valid known frame. On checksum errors, rescan one byte
    later, including headers within the rejected candidate.

    One owner should call feed. All packets completed in a feed share its
    receive time; burst packets must not be assigned invented sample times.
    """

    _LENGTHS = {0x16: 21, 0x26: 17}

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._last_received_at: float | None = None
        self.accepted_packets = 0
        self.checksum_errors = 0
        self.length_errors = 0
        self.unknown_functions = 0
        self.invalid_values = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, data: bytes | bytearray | memoryview, received_at: float | None = None) -> list[dict]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("IMU input must be bytes-like")
        timestamp = _finite_time(received_at)
        if self._last_received_at is not None and timestamp < self._last_received_at:
            raise ValueError("Receive times must be nondecreasing")
        self._last_received_at = timestamp
        self._buffer.extend(data)
        decoded = []
        while self._buffer:
            header = self._buffer.find(b"\x7e\x23")
            if header < 0:
                keep = 1 if self._buffer[-1] == 0x7E else 0
                count = len(self._buffer) - keep
                self.discarded_bytes += count
                del self._buffer[:count]
                break
            if header:
                self.discarded_bytes += header
                del self._buffer[:header]
            if len(self._buffer) < 4:
                break
            length, function = self._buffer[2:4]
            expected = self._LENGTHS.get(function)
            if expected is None or length != expected:
                if expected is None:
                    self.unknown_functions += 1
                else:
                    self.length_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < length:
                break
            packet = self._buffer[:length]
            if sum(packet[:-1]) & 0xFF != packet[-1]:
                self.checksum_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            del self._buffer[:length]
            try:
                if function == 0x16:
                    quaternion = normalize_quaternion(struct.unpack("<4f", packet[4:-1]))
                    value = {"kind": "quaternion", "quaternion": quaternion, "received_at": timestamp}
                else:
                    euler = struct.unpack("<3f", packet[4:-1])
                    if not all(math.isfinite(angle) for angle in euler):
                        raise ValueError("Euler angles must be finite")
                    value = {"kind": "euler", "euler_rad": euler, "received_at": timestamp}
            except ValueError:
                self.invalid_values += 1
                continue
            decoded.append(value)
            self.accepted_packets += 1
        return decoded


class PoseHistory:
    """Thread-safe finite pose history with consecutive IDs starting at 1.

    Equal receive times are allowed for USB bursts; regressing times are
    rejected. Quaternion signs are chosen continuously on insertion.
    """

    def __init__(self, maxlen: int = 2000) -> None:
        if isinstance(maxlen, bool) or not isinstance(maxlen, int) or maxlen < 2:
            raise ValueError("maxlen must be an integer of at least 2")
        self._samples: deque[PoseSample] = deque(maxlen=maxlen)
        self._lock = threading.RLock()
        self._next_id = 1

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)

    def add(self, quaternion: Iterable[float], received_at: float | None = None) -> PoseSample:
        q = normalize_quaternion(quaternion)
        with self._lock:
            timestamp = _finite_time(received_at)
            if self._samples:
                previous = self._samples[-1]
                if timestamp < previous.received_at:
                    raise ValueError("Receive times must be nondecreasing")
                if float(np.dot(previous.quaternion, q)) < 0:
                    q = tuple(-v for v in q)
            sample = PoseSample(self._next_id, timestamp, q)
            self._samples.append(sample)
            self._next_id += 1
            return sample

    def latest(self) -> PoseSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def snapshot(self) -> tuple[PoseSample, ...]:
        with self._lock:
            return tuple(self._samples)

    def interpolate(self, timestamp: float) -> PoseSample | None:
        """SLERP at a host receive time; return None outside retained history.

        Exact timestamps return the newest sample at that time, making
        duplicate receive times deterministic without division by zero.
        """
        timestamp = _finite_time(timestamp)
        samples = self.snapshot()
        if not samples or timestamp < samples[0].received_at or timestamp > samples[-1].received_at:
            return None
        times = [sample.received_at for sample in samples]
        right = bisect_right(times, timestamp)
        left_sample = samples[right - 1]
        if left_sample.received_at == timestamp:
            return left_sample
        right_sample = samples[right]
        fraction = (timestamp - left_sample.received_at) / (right_sample.received_at - left_sample.received_at)
        return PoseSample(left_sample.id, timestamp, slerp(left_sample.quaternion, right_sample.quaternion, fraction))

    def rate_hz(self, window_seconds: float | None = None) -> float:
        """Estimate received quaternion rate, not the IMU's acquisition rate."""
        if window_seconds is not None:
            window_seconds = float(window_seconds)
            if not math.isfinite(window_seconds) or window_seconds <= 0:
                raise ValueError("Rate window must be finite and positive")
        samples = self.snapshot()
        if window_seconds is not None and samples:
            cutoff = samples[-1].received_at - window_seconds
            start = bisect_left([sample.received_at for sample in samples], cutoff)
            samples = samples[start:]
        if len(samples) < 2:
            return 0.0
        span = samples[-1].received_at - samples[0].received_at
        return (len(samples) - 1) / span if span > 0 else 0.0
