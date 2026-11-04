"""Fixed-FOV roll crop settings and offline checks; never sends coordinates.

A finite angle sweep establishes evidence at those angles only. The actual
candidate still needs its existing integer geometry/cache admission check.
This is not an optical recalibration or an arbitrary-angle no-black guarantee.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class FixedRollCrop:
    zoom: float = 1.3
    max_abs_roll_deg: float = 15.0

    def __post_init__(self):
        if isinstance(self.zoom, bool) or not math.isfinite(self.zoom) or not 1 <= self.zoom <= 8:
            raise ValueError("Fixed crop zoom must be in [1, 8]")
        if (isinstance(self.max_abs_roll_deg, bool) or not math.isfinite(self.max_abs_roll_deg)
                or not 0 < self.max_abs_roll_deg <= 180):
            raise ValueError("Roll crop circular half-range must be in (0, 180] degrees")

    def check_angle(self, roll_radians):
        # Roll is an orientation on a circle, not an unbounded Euler counter.
        # A half-range of 180 covers the full circle including both spellings
        # of its seam. Smaller windows retain their reference-centred limit.
        if isinstance(roll_radians, bool) or not math.isfinite(roll_radians):
            raise ValueError("Roll must be a finite angle")
        angle = math.degrees(math.atan2(math.sin(roll_radians), math.cos(roll_radians)))
        if abs(angle) > self.max_abs_roll_deg + 1e-9:
            raise ValueError("Roll is outside the selected fixed-crop range; retain the last valid output")
        return self.zoom

    def to_dict(self):
        return asdict(self)

    @property
    def full_circle(self):
        return self.max_abs_roll_deg == 180


DEMO_ROLL_CROP = FixedRollCrop(1280 / 608, 180)
DEMO_CROP_EVIDENCE = Path(__file__).resolve().parent / 'validation_c128' / 'fixed_roll_360.json'


def checked_demo_crop(calibration, *, geometry_options=None, evidence_path=DEMO_CROP_EVIDENCE):
    """Select the demo preset only for its checked integer optical context.

    This gate binds the packaged finite sweep to the untouched C97 parameters.
    Per-candidate FPGA admission and geometry auto recovery remain mandatory.
    """
    report = json.loads(Path(evidence_path).read_text(encoding='utf-8'))
    if (report.get('schema') != 'c128_fixed_roll_crop_checks_v2'
            or report.get('profile') != DEMO_ROLL_CROP.to_dict()
            or report.get('calibration_sha256') != calibration_digest(calibration)
            or report.get('sample_count', 0) < 721
            or report.get('max_sample_gap_degrees', 360) > .5
            or not report.get('all_samples_passed')):
        raise ValueError('固定整圈裁剪的离线证据与当前镜头参数不匹配；请先重新验证。')
    vendor = str(Path(__file__).resolve().parent / 'vendor')
    if vendor not in sys.path:
        sys.path.insert(0, vendor)
    from complete_geometry import configuration
    options = dict(geometry_options or {})
    options['zoom'] = DEMO_ROLL_CROP.zoom
    words = configuration(calibration, matrix=np.eye(3), legacy_angle=0, **options)
    if list(words[:16]) != report.get('optical_words16'):
        raise ValueError('固定整圈裁剪的中心／视野已变化；请重新执行整数映射检查。')
    return DEMO_ROLL_CROP.to_dict()


def calibration_digest(calibration):
    return hashlib.sha256(json.dumps(calibration, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode("utf-8")).hexdigest()


def verify_crop_profile(calibration, profile, *, step_degrees=5., checker=None):
    """Check the real integer fisheye model at a reproducible finite angle grid.

    Never interpolate a success claim between grid angles. Fail immediately if
    any sample has invalid pixels, arithmetic errors, or unsupported tile ROI.
    The same single zoom is used at every angle, avoiding per-frame FOV breathing.
    """
    if not isinstance(profile, FixedRollCrop):
        raise TypeError("profile must be FixedRollCrop")
    if isinstance(step_degrees, bool) or not math.isfinite(step_degrees) or step_degrees <= 0:
        raise ValueError("Angle step must be positive and finite")
    vendor = str(Path(__file__).resolve().parent / "vendor")
    if vendor not in sys.path:
        sys.path.insert(0, vendor)
    from complete_geometry import configuration, check_geometry
    checker = checker or check_geometry
    count = math.ceil(profile.max_abs_roll_deg / step_degrees)
    angles = np.linspace(-profile.max_abs_roll_deg, profile.max_abs_roll_deg, 2 * count + 1)
    checked = []
    for angle in angles:
        words = tuple(configuration(calibration, roll=float(angle), zoom=profile.zoom, legacy_angle=0))
        evidence = checker(words, allow_black=False)
        if (evidence.get("invalid_pixels", 0) or evidence.get("arithmetic_error_pixels", 0)
                or evidence.get("tile_budget", {}).get("unsupported_tiles")):
            raise ValueError("Crop sample did not pass full integer geometry/cache admission")
        checked.append({"roll_deg": float(angle), "geometry": evidence})
    return {"schema": "c127_fixed_roll_crop_checks_v1", "profile": profile.to_dict(),
            "calibration_sha256": calibration_digest(calibration), "samples": checked,
            "continuous_range_proven": False, "board_verified": False,
            "meaning": "Exact integer checks at listed angles only; check each actual candidate. "
                       "No guarantee for other poses, scan homographies, optical changes, or arbitrary angles."}
