"""Plane/mirror parameter preparation, persistence, and OFFLINE validation.

No serial transport, coordinate-map export, or hardware access exists here.
The FPGA receives four 3x3 Q16.16 configuration factors and generates its maps.
Raw points invert the compiled fisheye4-then-tangential2 model exactly once;
rectified-view points never go through that inverse lens model again.

Point inversion and chessboard ordering are adapted from the supplied offline
plane_rectification_preview.py (SHA256
2105757a0aef2d06d8c9ed5ebbf36d973a05f9a2749b65f827156587687ede29).
The source was documentation/evidence, not a production implementation.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np

VENDOR = Path(__file__).resolve().parent / "vendor"
if str(VENDOR) not in sys.path:
    sys.path.insert(0, str(VENDOR))
from complete_geometry import (configuration, normalize_joint_calibration,
                               quantize_matrix, validate_words)
from project_profile import SENSOR_MODE, TANGENTIAL_Q16
from imu_pose import normalize_quaternion, validate_mounting_rotation

SCHEMA = "fpga_plane_scan_v1"
CONTEXT_SCHEMA = "fpga_plane_context_v1"
WIDTH, HEIGHT = 1280, 720
Q = 65536
IWORDS = [Q, 0, 0, 0, Q, 0, 0, 0, Q]
MIRROR = np.array([[-1., 0, WIDTH-1], [0, 1, 0], [0, 0, 1]])
GRID = np.mgrid[:5, :10][::-1].transpose(1, 2, 0).reshape(-1, 2).astype(float)


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False)


def _hash(value):
    return hashlib.sha256(_json(value).encode("utf-8")).hexdigest()


def _matrix(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (3, 3) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if abs(float(np.linalg.det(a))) < 1e-10:
        raise ValueError(f"{name} must be nonsingular")
    return a


def _positive_scale(a):
    """Only a positive common scale; never turn backward rays forward."""
    a = _matrix(a, "Projective matrix")
    return a / float(np.max(np.abs(a)))


def _kl(words):
    w = np.asarray(words, float) / Q
    k = np.array([[w[6], 0, w[4]], [0, w[7], w[5]], [0, 0, 1]])
    l = np.array([[w[12], -w[13], 0], [w[13], w[12], 0], [0, 0, 1]])
    _matrix(l, "Quantized legacy roll")
    return k, l


def make_context(calibration, words18, *, reference=None, mounting=None,
                 convention="imu_to_world", sensor_mode=SENSOR_MODE, epoch=0):
    """Bind a plane to immutable geometry/reference state, not changing poses.

    ``reference`` accepts the session snapshot's reference dict (or None for
    ordinary mode). A changed reference, mounting, crop/zoom, or session epoch
    produces a different hash. Actual displayed-frame H is separate metadata.
    """
    cal = normalize_joint_calibration(deepcopy(calibration))
    words = validate_words(list(words18) + IWORDS)[:18]
    if words[16] != 1:
        raise ValueError("Plane/mirror factors require corrected geometry mode")
    expected = configuration(cal)[:18]
    if any(words[i] != expected[i] for i in (*range(4), *range(8, 12), 17)):
        raise ValueError("Source K/D/model words do not match the whole calibration")
    k, l = _kl(words)
    mounting = validate_mounting_rotation(np.eye(3) if mounting is None else mounting)
    if convention not in ("imu_to_world", "world_to_imu"):
        raise ValueError("Unknown quaternion convention")
    if not isinstance(sensor_mode, str) or not sensor_mode.strip():
        raise ValueError("A named sensor mode is required")
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
        raise ValueError("Context epoch must be a nonnegative integer")
    ref = None
    if reference is not None:
        if not isinstance(reference, dict) or "reference_id" not in reference:
            raise ValueError("Reference requires reference_id and quaternion")
        quaternion = reference.get("quaternion")
        if quaternion is None:
            quaternion = reference.get("sample", {}).get("quaternion")
        q = normalize_quaternion(quaternion)
        if q[0] < 0:
            q = tuple(-v for v in q)
        ref = dict(reference_id=reference["reference_id"], quaternion=list(q))
        if isinstance(ref["reference_id"], bool) or not isinstance(ref["reference_id"], (int, str)):
            raise ValueError("Reference id must be an integer or string")
    context = dict(schema=CONTEXT_SCHEMA, width=WIDTH, height=HEIGHT,
                   calibration=cal, calibration_sha256=_hash(cal), words18=words,
                   output_k=k.tolist(), legacy_l=l.tolist(), reference=ref,
                   mounting=mounting.tolist(), convention=convention,
                   sensor_mode=sensor_mode, epoch=epoch,
                   source_coordinate_definition="1280x720 same-FOV pixel centres; 1920x1080 uses (p+0.5)*2/3-0.5",
                   optical_mode_match="Existing project assumption; not newly measured optical calibration")
    context["context_hash"] = _hash(context)
    context["context_id"] = int(context["context_hash"][:8], 16)
    return context


def validate_context(context):
    if not isinstance(context, dict) or context.get("schema") != CONTEXT_SCHEMA:
        raise ValueError("Unknown plane context schema")
    expected = make_context(context["calibration"], context["words18"],
                            reference=context["reference"], mounting=context["mounting"],
                            convention=context["convention"], sensor_mode=context["sensor_mode"],
                            epoch=context["epoch"])
    if _json(expected) != _json(context):
        raise ValueError("Plane context contents/hash are inconsistent")
    return deepcopy(expected)


def _factors(context, p):
    k, l = _kl(context["words18"])
    ki, li = np.linalg.inv(k), np.linalg.inv(l)
    words = []
    for scan, mirror in ((False, False), (False, True), (True, False), (True, True)):
        if not scan and not mirror:
            part = IWORDS.copy()  # Preserve mode00 without extra quantization.
        else:
            c = l @ ki @ (p if scan else np.eye(3)) @ (MIRROR if mirror else np.eye(3)) @ k @ li
            part = quantize_matrix(_positive_scale(c))
        words.extend(part)
    return words


def _profile(context, p, *, scan_valid, fit, layout):
    context = validate_context(context)
    p = _matrix(p, "Pixel inverse H")
    profile = dict(schema=SCHEMA, context=context, context_hash=context["context_hash"],
                   context_id=context["context_id"], scan_valid=bool(scan_valid),
                   pixel_inverse_h=p.tolist(), mode_order=["00", "01", "10", "11"],
                   mode_words=_factors(context, p), fit=fit, layout=layout)
    profile["profile_hash"] = _hash(profile)
    return profile


def make_mirror_profile(context):
    """Explicitly initialize horizontal mirror without assuming any plane."""
    return _profile(context, np.eye(3), scan_valid=False,
                    fit=dict(kind="no_plane", input_domain=None), layout=None)


def validate_profile(profile, *, expected_context=None):
    if not isinstance(profile, dict) or profile.get("schema") != SCHEMA:
        raise ValueError("Unknown plane profile schema")
    context = validate_context(profile["context"])
    if expected_context is not None and context["context_hash"] != validate_context(expected_context)["context_hash"]:
        raise ValueError("Plane is stale: calibration, view, reference, mounting, or session changed")
    if type(profile.get("scan_valid")) is not bool:
        raise ValueError("scan_valid must be Boolean")
    p = _matrix(profile["pixel_inverse_h"], "Pixel inverse H")
    if not profile["scan_valid"] and not np.array_equal(p, np.eye(3)):
        raise ValueError("A no-plane profile must use identity scan factors")
    expected = _profile(context, p, scan_valid=profile["scan_valid"],
                        fit=profile["fit"], layout=profile["layout"])
    if _json(expected) != _json(profile):
        raise ValueError("Profile checksum, context, or quantized factors are inconsistent")
    return deepcopy(profile)


def save_profile(profile, path):
    """Atomically save only a small parameter document, never coordinate maps."""
    profile = validate_profile(profile)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(profile, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return path


def load_profile(path, *, expected_context=None):
    path = Path(path)
    if path.stat().st_size > 1024*1024:
        raise ValueError("Plane parameter file is unexpectedly large")
    return validate_profile(json.loads(path.read_text(encoding="utf-8-sig")), expected_context=expected_context)


def compose_words(profile, base_h=None, *, scan=False, mirror=False):
    """Match config-stage full-sum multiplication and common power-of-two scale.

    Mode00 returns the original base words. Overflow/singular/unsupported scale
    is rejected; no clipping or wrapping and no hidden sign normalization.
    """
    profile = validate_profile(profile)
    if scan and not profile["scan_valid"]:
        raise ValueError("No valid plane; scanning cannot be enabled")
    hb = np.eye(3) if base_h is None else _matrix(base_h, "Base H")
    hwords = quantize_matrix(hb)
    mode = 2*bool(scan) + bool(mirror)
    if mode == 0:
        final = hwords
    else:
        cwords = profile["mode_words"][9*mode:9*(mode+1)]
        sums = [sum(hwords[3*r+i]*cwords[3*i+c] for i in range(3))
                for r in range(3) for c in range(3)]
        maximum = max(abs(v) for v in sums)
        if maximum == 0:
            raise ValueError("Composed projection is zero")
        shift = maximum.bit_length()-1-16
        final = [int((v+(1 << (shift-1))) >> shift) if shift > 0 else int(v << -shift)
                 for v in sums]
    return validate_words(profile["context"]["words18"] + final)


def validate_modes(profile, base_h=None, *, require_no_black=False):
    """Offline full-canvas integer audit using the C125 FPGA guard policy.

    Domain-invalid samples count as black. Setup failure, unsupported 256x64
    source ROI for any 64x16 output tile, and all-black maps reject the mode.
    This is no hardware/DDR/timing test. No generated map is saved or returned.
    """
    from complete_geometry_reference import vector_fixed_map
    from tile_budget import tile_budget
    profile = validate_profile(profile)
    modes = {}
    for scan, mirror in ((False, False), (False, True), (True, False), (True, True)):
        name = f"{int(scan)}{int(mirror)}"
        if scan and not profile["scan_valid"]:
            modes[name] = dict(admitted=False, reason="no_valid_plane")
            continue
        try:
            words = compose_words(profile, base_h, scan=scan, mirror=mirror)
            entries, domain = vector_fixed_map(WIDTH, HEIGHT, words[:18], words[18:])
            invalid = int(np.count_nonzero((entries >> 38) == 0))
            budget = tile_budget(entries, WIDTH, HEIGHT, 64, 16, 256, 64)
            reasons = []
            if budget["unsupported_tiles"]:
                reasons.append("source_roi_overflow")
            if invalid == WIDTH*HEIGHT:
                reasons.append("all_black")
            if require_no_black and invalid:
                reasons.append("black_pixels_remain")
            modes[name] = dict(admitted=not reasons, reasons=reasons, words=words,
                               invalid_pixels=invalid, domain_invalid_pixels=int(domain),
                               black_fraction=invalid/(WIDTH*HEIGHT), tile_budget=budget)
        except ValueError as exc:
            modes[name] = dict(admitted=False, reason="setup_or_matrix_invalid", detail=str(exc))
    return dict(scope="Offline exact-integer geometry/ROI only; not board validation",
                context_hash=profile["context_hash"], domain_policy="invalid samples become black",
                no_black_required=bool(require_no_black), modes=modes)


def _cv2():
    try:
        import cv2
        return cv2
    except ImportError as exc:
        raise RuntimeError("Point/image fitting requires OpenCV; no hardware access is needed") from exc


def _project(points, h):
    homogeneous = np.c_[np.asarray(points, float), np.ones(len(points))] @ h.T
    if np.any(np.abs(homogeneous[:, 2]) < 1e-10):
        raise ValueError("Point lies on the projective horizon")
    return homogeneous[:, :2]/homogeneous[:, 2:]


def _homography(output_points, input_points):
    a, b = np.asarray(output_points, float), np.asarray(input_points, float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 2 or len(a) < 4 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("At least four finite point correspondences are required")
    def norm(p):
        mean = p.mean(axis=0)
        radius = np.linalg.norm(p-mean, axis=1).mean()
        if radius < 1e-9:
            raise ValueError("Degenerate point set")
        s = math.sqrt(2)/radius
        t = np.array([[s, 0, -s*mean[0]], [0, s, -s*mean[1]], [0, 0, 1]])
        return _project(p, t), t
    an, ta = norm(a)
    bn, tb = norm(b)
    rows = []
    for (x, y), (u, v) in zip(an, bn):
        rows += [[-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                 [0, 0, 0, -x, -y, -1, v*x, v*y, v]]
    _, singular, vh = np.linalg.svd(rows, full_matrices=True)
    if singular[7] < singular[0]*1e-10:
        raise ValueError("Degenerate or collinear plane points")
    h = np.linalg.inv(tb) @ vh[-1].reshape(3, 3) @ ta
    centre_z = float((h @ np.array([WIDTH/2, HEIGHT/2, 1]))[2])
    if abs(centre_z) < 1e-10:
        raise ValueError("Canvas centre lies on the projective horizon")
    return h/centre_z


def _joint_project(rays, context):
    cv2 = _cv2()
    cal = context["calibration"]
    k = np.asarray(cal["camera_matrix"], float)
    xy = cv2.fisheye.distortPoints(np.asarray(rays, float)[:, None, :], np.eye(3),
                                  np.asarray(cal["distortion"], float)).reshape(-1, 2)
    x, y = xy.T
    p1, p2 = np.array(TANGENTIAL_Q16)/Q
    delta = np.c_[3*p2*x*x+p2*y*y+2*p1*x*y, p1*x*x+3*p1*y*y+2*p2*x*y]
    return (xy+delta)*[k[0, 0], k[1, 1]]+k[:2, 2]


def _joint_unproject(pixels, context):
    cv2 = _cv2()
    cal = context["calibration"]
    k = np.asarray(cal["camera_matrix"], float)
    distorted = (pixels-k[:2, 2])/[k[0, 0], k[1, 1]]
    xy = distorted.copy()
    p1, p2 = np.array(TANGENTIAL_Q16)/Q
    for _ in range(40):
        x, y = xy.T
        delta = np.c_[3*p2*x*x+p2*y*y+2*p1*x*y, p1*x*x+3*p1*y*y+2*p2*x*y]
        update = distorted-delta
        if np.max(np.abs(update-xy)) < 1e-13:
            xy = update
            break
        xy = update
    rays = cv2.fisheye.undistortPoints(xy[:, None, :], np.eye(3), np.asarray(cal["distortion"], float),
                                      criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)).reshape(-1, 2)
    if not np.isfinite(rays).all():
        raise ValueError("Joint inverse produced invalid rays")
    error = float(np.linalg.norm(_joint_project(rays, context)-pixels, axis=1).max())
    if error > 1e-5:
        raise ValueError(f"Joint point inversion failed roundtrip: {error} pixels")
    return rays, error


def _fit(points, desired, context, *, input_domain, input_size, capture_h,
         capture_context_hash, capture_id, fit, layout):
    context = validate_context(context)
    domain = "rectified" if input_domain == "already_rectified" else input_domain
    if domain not in ("raw", "rectified"):
        raise ValueError("input_domain must be raw or rectified")
    if tuple(input_size) not in ((WIDTH, HEIGHT), (1920, 1080)):
        raise ValueError("Only full same-FOV 1280x720 or 1920x1080 input is defined")
    points = np.asarray(points, float)
    if not np.isfinite(points).all() or np.any(points < 0) or np.any(points > np.asarray(input_size)-1):
        raise ValueError("Selected points lie outside the supplied image")
    points = (points+.5)*(WIDTH/input_size[0])-.5
    if domain == "rectified" or context["reference"] is not None:
        if capture_h is None or capture_context_hash != context["context_hash"] or not isinstance(capture_id, str) or not capture_id.strip():
            raise ValueError("A corrected/pose-bound frame needs its actual capture_h, capture_context_hash, and capture_id")
    if capture_context_hash is not None and capture_context_hash != context["context_hash"]:
        raise ValueError("Selected frame context differs from the active geometry")
    h0 = np.eye(3) if capture_h is None else np.asarray(quantize_matrix(_matrix(capture_h, "Capture H")), float).reshape(3, 3)/Q
    k, l = _kl(context["words18"])
    if domain == "raw":
        fitting, inverse_error = _joint_unproject(points, context)
        g = _homography(desired, fitting)
        p = k @ np.linalg.inv(l) @ np.linalg.inv(h0) @ g
        fit["joint_point_roundtrip_max_source_px"] = inverse_error
    else:
        fitting = points
        p = _homography(desired, fitting)
        g = p
    residual = np.linalg.norm(_project(fitting, np.linalg.inv(g))-desired, axis=1)
    fit.update(input_domain=domain, input_size=list(input_size), source_points_720=points.tolist(),
               target_points=np.asarray(desired).tolist(), rms_target_pixels=float(np.sqrt(np.mean(residual**2))),
               max_target_pixels=float(residual.max()), capture_h=h0.tolist(),
               capture_id=capture_id, capture_context_hash=context["context_hash"],
               input_view_requirement="Base corrected view must be unmirrored and uncropped relative to this context; remove UI letterbox/scale before selecting points",
               optical_validation=False)
    if len(points) >= 8:
        pattern = fit.get("pattern_size")
        if pattern:
            train = np.indices((pattern[1], pattern[0])).sum(axis=0).ravel()%2 == 0
        else:
            train = np.arange(len(points))%2 == 0
        heldout_h = _homography(desired[train], fitting[train])
        heldout_error = np.linalg.norm(_project(fitting[~train], np.linalg.inv(heldout_h))-desired[~train], axis=1)
        fit["heldout_points"] = int((~train).sum())
        fit["heldout_rms_target_pixels"] = float(np.sqrt(np.mean(heldout_error**2)))
        if fit["heldout_rms_target_pixels"] > 3.0:
            raise ValueError("Plane fit heldout RMS exceeds 3 target pixels; inspect points, model, and input domain")
    profile = _profile(context, p, scan_valid=True, fit=fit, layout=layout)
    # Validation is exposed separately so callers can show rejection statistics
    # and select an appropriate crop without silently replacing their fit.
    return profile


def fit_chessboard(image_or_path, context, *, input_domain="raw", capture_h=None,
                   capture_context_hash=None, capture_id=None, cell_pixels=64,
                   pattern_size=(10, 5)):
    cv2 = _cv2()
    if not isinstance(cell_pixels, (int, float)) or isinstance(cell_pixels, bool) or not math.isfinite(cell_pixels) or not 16 <= cell_pixels <= 90:
        raise ValueError("cell_pixels must be finite in [16,90]")
    if not isinstance(pattern_size, (list, tuple)) or len(pattern_size) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or not 2 <= v <= 20 for v in pattern_size):
        raise ValueError("pattern_size must contain internal corner columns/rows in [2,20]")
    cols, rows = pattern_size
    if (cols+2)*cell_pixels > WIDTH or (rows+2)*cell_pixels > HEIGHT:
        raise ValueError("Board plus surrounding margin does not fit; reduce cell_pixels")
    source_hash = None
    if isinstance(image_or_path, (str, Path)):
        data = Path(image_or_path).read_bytes()
        source_hash = hashlib.sha256(data).hexdigest()
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    else:
        image = np.asarray(image_or_path)
    if image is None or image.ndim not in (2, 3):
        raise ValueError("Cannot read the selected image")
    size = (image.shape[1], image.shape[0])
    if size not in ((1280, 720), (1920, 1080)):
        raise ValueError("Image must be a full 1280x720 or 1920x1080 frame")
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        raise ValueError(f"All {cols*rows} internal corners of the {cols}x{rows} board are required")
    c = corners.reshape(rows, cols, 2)
    if c[:, :, 0].mean(axis=0)[0] > c[:, :, 0].mean(axis=0)[-1]:
        c = c[:, ::-1]
    if c[:, :, 1].mean(axis=1)[0] > c[:, :, 1].mean(axis=1)[-1]:
        c = c[::-1]
    origin = np.array([(WIDTH-(cols+1)*cell_pixels)/2+cell_pixels, (HEIGHT-(rows+1)*cell_pixels)/2+cell_pixels])
    grid = np.mgrid[:rows, :cols][::-1].transpose(1, 2, 0).reshape(-1, 2).astype(float)
    desired = grid*cell_pixels+origin
    profile = _fit(c.reshape(-1, 2), desired, context, input_domain=input_domain,
                   input_size=size, capture_h=capture_h, capture_context_hash=capture_context_hash,
                   capture_id=capture_id, fit=dict(kind="chessboard", pattern_size=[cols, rows], source_sha256=source_hash),
                   layout=dict(cell_pixels=cell_pixels, board_squares=[cols+1, rows+1],
                               board_size=[(cols+1)*cell_pixels, (rows+1)*cell_pixels], origin=origin.tolist(),
                               canvas=[WIDTH, HEIGHT]))
    return profile


def fit_four_points(points, aspect_ratio, context, *, input_domain="raw", input_size=(1280, 720),
                    capture_h=None, capture_context_hash=None, capture_id=None, cell_pixels=64):
    """Fit ordered TL,TR,BR,BL corners with a required physical width/height."""
    points = np.asarray(points, float)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError("Exactly four finite TL,TR,BR,BL points are required")
    if isinstance(aspect_ratio, bool) or not isinstance(aspect_ratio, (int, float)) or not math.isfinite(aspect_ratio) or aspect_ratio <= 0:
        raise ValueError("A finite positive physical width/height ratio is required")
    if isinstance(cell_pixels, bool) or not isinstance(cell_pixels, (int, float)) or not math.isfinite(cell_pixels) or not 16 <= cell_pixels <= 90:
        raise ValueError("cell_pixels must be finite in [16,90]")
    edges = np.roll(points, -1, axis=0)-points
    crosses = edges[:, 0]*np.roll(edges[:, 1], -1)-edges[:, 1]*np.roll(edges[:, 0], -1)
    if np.any(crosses <= 1e-6):
        raise ValueError("Corners must form a nondegenerate convex TL,TR,BR,BL polygon")
    w = min(11*cell_pixels, WIDTH-2*cell_pixels, (HEIGHT-2*cell_pixels)*aspect_ratio)
    h = w/aspect_ratio
    left, top = (WIDTH-w)/2, (HEIGHT-h)/2
    desired = np.array([[left, top], [left+w, top], [left+w, top+h], [left, top+h]])
    return _fit(points, desired, context, input_domain=input_domain, input_size=input_size,
                capture_h=capture_h, capture_context_hash=capture_context_hash, capture_id=capture_id,
                fit=dict(kind="four_points", aspect_ratio=float(aspect_ratio)),
                layout=dict(cell_pixels=cell_pixels, board_size=[w, h], canvas=[WIDTH, HEIGHT]))
