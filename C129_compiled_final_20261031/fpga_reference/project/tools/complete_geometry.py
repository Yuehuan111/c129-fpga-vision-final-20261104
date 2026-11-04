"""Complete FPGA geometry parameters and offline validation only.

The serial transaction contains 27 words, never a pixel/map array. Coordinates
are output-to-source rays. X points right, Y down, Z forward. A homography is
valid for the chosen plane or a known camera rotation; it cannot automatically
frontally restore arbitrary 3-D scenes. C97 k1..k4 remain fisheye coefficients;
this build applies its separate compiled p1,p2 after fisheye and before source K.
"""
from dataclasses import dataclass
from functools import lru_cache
import json
import math
from numbers import Integral
from pathlib import Path
import numpy as np

from fisheye_config import configuration as fisheye_configuration
from fisheye_reference import validate_words as validate_fisheye, MODEL
from project_profile import (BUILD_ID, DEFAULT_CALIBRATION, HEIGHT, JOINT_MODEL,
                             ROOT, TANGENTIAL_Q16, WIDTH)
from tile_budget import tile_budget

IDENTITY_MATRIX = (65536, 0, 0, 0, 65536, 0, 0, 0, 65536)
ADDRESSES = tuple(range(18)) + tuple(range(0x20, 0x29))


@dataclass(frozen=True)
class BudgetProfile:
    tile_width: int = 64
    tile_height: int = 16
    cache_width: int = 256
    cache_height: int = 64

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0
               for v in (self.tile_width, self.tile_height, self.cache_width, self.cache_height)):
            raise ValueError('Tile/cache dimensions must be positive integers.')
        if self.tile_width % 16:
            raise ValueError('Tile width must be a multiple of the 16-pixel burst.')


PRODUCTION_BUDGET = BudgetProfile()


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite.')
    return float(value)


def quantize_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('Inverse projection must contain nine finite values in a 3x3 matrix.')
    maximum = float(np.max(np.abs(matrix)))
    if 0 < maximum < .5:
        # A positive common scale preserves the forward-facing rays while
        # keeping homogeneous setup away from poorly resolved tiny values.
        matrix = matrix / maximum
    if np.max(np.abs(matrix)) > 8:
        raise ValueError('Every inverse-projective coefficient must be in [-8,8].')
    words = [int(math.floor(v * 65536 + .5)) for v in matrix.flat]
    restored = np.asarray(words, dtype=float).reshape(3, 3) / 65536
    if abs(np.linalg.det(restored)) < 1e-8:
        raise ValueError('Projection is singular after Q16.16 quantization.')
    return words


def pose_matrix(roll=0., pitch=0., yaw=0., tx=0., ty=0.):
    """Return inverse-sampling T(tx,ty) Rz(roll) Ry(yaw) Rx(pitch).

    Angles are degrees about the explicit ray axes; translation is in normalized
    source-plane units, not metres. The caller supplies the required correction
    rotation from known pose. This function does not estimate physical pose.
    """
    roll, pitch, yaw, tx, ty = [_finite(v, n) for v, n in
        zip((roll, pitch, yaw, tx, ty), ('roll', 'pitch', 'yaw', 'tx', 'ty'))]
    r, p, y = map(math.radians, (roll, pitch, yaw))
    rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    return np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]]) @ rz @ ry @ rx


def four_point_matrix(output_points, source_points):
    """Solve H mapping four output-plane points to source undistorted-plane points.

    Both inputs are normalized coordinates. The output points are after the
    legacy cos/sin stage. Raw fisheye image coordinates are not valid inputs.
    All four correspondences must refer to one target plane and be nondegenerate.
    """
    dst, src = np.asarray(output_points, float), np.asarray(source_points, float)
    if dst.shape != (4, 2) or src.shape != (4, 2) or not np.isfinite(dst).all() or not np.isfinite(src).all():
        raise ValueError('Exactly four finite [x,y] output and source points are required.')
    def normalize_points(points):
        centered = points - points.mean(axis=0)
        length = np.sqrt((centered * centered).sum(axis=1)).mean()
        if length < 1e-10:
            raise ValueError('Repeated or degenerate four-point geometry.')
        scale = math.sqrt(2) / length
        transform = np.array([[scale, 0, -scale * points[:, 0].mean()],
                              [0, scale, -scale * points[:, 1].mean()], [0, 0, 1]])
        return centered * scale, transform
    nd, td = normalize_points(dst)
    ns, ts = normalize_points(src)
    rows = []
    for (x, y), (u, v) in zip(nd, ns):
        rows.extend([[-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                     [0, 0, 0, -x, -y, -1, v*x, v*y, v]])
    a = np.asarray(rows)
    _, singular, vh = np.linalg.svd(a, full_matrices=True)
    if singular[-1] < singular[0] * 1e-10:
        raise ValueError('Four points must not be collinear or repeated.')
    h = np.linalg.inv(ts) @ vh[-1].reshape(3, 3) @ td
    if abs(h[2, 2]) < 1e-10:
        raise ValueError('Homography cannot use the supported forward-facing normalization.')
    h /= h[2, 2]
    quantize_matrix(h)
    return h


def normalize_joint_calibration(data):
    """Admit only the complete joint model matching this build's compiled p.

    The historical fisheye loader remains strict: old radial-only calibrations
    cannot silently acquire this build's p. No Brown coefficient reinterpretation
    is allowed. The original candidate Q16 JSON can be read without conversion.
    """
    if data.get('model') != JOINT_MODEL:
        raise ValueError('C121 requires the whole joint fisheye4-then-tangential2 calibration; old K/D cannot be mixed with compiled p.')
    data = dict(data)
    if isinstance(data.get('parameters'), dict):
        if data.get('fractional_bits') != 16 or data.get('total_bits') != 32 or data.get('all_parameters_fit') is not True:
            raise ValueError('Joint parameters must be signed32 Q16.16 without overflow.')
        def get(key):
            item = data['parameters'][key]
            value = item['decimal']
            if isinstance(value, bool) or not isinstance(value, int) or not -(1<<31) <= value < (1<<31):
                raise ValueError('Invalid signed joint parameter word: ' + key)
            if item.get('restored_float') != value / 65536:
                raise ValueError('Joint decimal/restored value mismatch: ' + key)
            return value / 65536
        dimensions = data.get('image_size')
        if dimensions != [WIDTH, HEIGHT]:
            raise ValueError('Joint calibration requires 1280x720 source coordinates.')
        canonical=dict(width=WIDTH, height=HEIGHT,
                    camera_matrix=[[get('fx_src'),0,get('cx_src')],[0,get('fy_src'),get('cy_src')],[0,0,1]],
                    output_camera_matrix=[[get('fx_out'),0,get('cx_out')],[0,get('fy_out'),get('cy_out')],[0,0,1]],
                    distortion=[get('fish_k'+str(i)) for i in range(1,5)],
                    tangential_q16=[int(get('fish_p'+str(i))*65536) for i in range(1,3)])
        for key,value in canonical.items():
            if key in data and data[key] != value:
                raise ValueError('Joint normalized field conflicts with Q16 source: ' + key)
        data.update(canonical)
    p = data.get('tangential_q16')
    if not isinstance(p, (list,tuple)) or len(p) != 2 or any(isinstance(v,bool) or not isinstance(v,Integral) for v in p) or tuple(p) != TANGENTIAL_Q16:
        raise ValueError('Joint p must match compiled Q16 constants [-13,18]; changing p requires a new FPGA build.')
    if (data.get('width'), data.get('height')) != (WIDTH, HEIGHT):
        raise ValueError('C121 joint firmware requires 1280x720 calibration.')
    if data.get('identity_mode', False) is not False or data.get('roi_cropped',False) is not False:
        raise ValueError('Joint calibration cannot imply raw identity or cropped coordinates.')
    if data.get('rectification_rotation', [[1,0,0],[0,1,0],[0,0,1]]) != [[1,0,0],[0,1,0],[0,0,1]]:
        raise ValueError('Calibration rectification must be identity; use the explicit projective controls.')
    if data.get('synthetic') is not False:
        raise ValueError('Board parameters must come from real calibration (synthetic=false).')
    return data


def _radial_parameters(calibration):
    data = normalize_joint_calibration(calibration)
    # Adapt only the validated radial portion to the historical 18-word encoder.
    # Keep joint p outside the UART words; FPGA/complete oracle apply it later.
    return {key:data[key] for key in ('width','height','synthetic','camera_matrix',
                                     'output_camera_matrix','distortion')} | {'model':MODEL}


def load_calibration(path=DEFAULT_CALIBRATION):
    data = normalize_joint_calibration(json.loads(Path(path).read_text(encoding='utf-8-sig')))
    fisheye_configuration(_radial_parameters(data))
    return data


def configuration(calibration, *, matrix=None, roll=0., pitch=0., yaw=0.,
                  zoom=1., tx=0., ty=0., legacy_angle=0., gain=1., offset=0, raw=False):
    base = fisheye_configuration(_radial_parameters(calibration), angle=legacy_angle, zoom=zoom, gain=gain, offset=offset)
    if raw:
        base[16] = 0
        projection = list(IDENTITY_MATRIX)
    else:
        base[16] = 1
        projection = quantize_matrix(pose_matrix(roll, pitch, yaw, tx, ty) if matrix is None else matrix)
    return validate_words(base + projection)


def validate_words(words):
    if not isinstance(words, (list, tuple)) or len(words) != 27:
        raise ValueError('A complete transaction needs all 18 fisheye and 9 projective words.')
    if any(isinstance(v, bool) or not isinstance(v, Integral) or not -(1 << 31) <= v < (1 << 31) for v in words):
        raise ValueError('Every word must be a signed 32-bit integer.')
    base = validate_fisheye(list(words[:18]))
    if max(base[0:2]) > 4096 * 65536:
        raise ValueError('Source focal length must not exceed 4096 pixels.')
    matrix = list(map(int, words[18:]))
    if max(abs(v) for v in matrix) < 32768:
        raise ValueError('Serialized matrix scale is too small; normalize the float matrix before Q16.16 export.')
    if any(abs(v) > 8 * 65536 for v in matrix):
        raise ValueError('Inverse-projective coefficients must lie in [-8,8].')
    quantize_matrix(np.asarray(matrix).reshape(3, 3) / 65536)
    if base[16] == 0 and tuple(matrix) != IDENTITY_MATRIX:
        raise ValueError('Raw mode uses an explicit identity projection.')
    return base + matrix


def fixed_map(width, height, words):
    """Offline only: use the exact integer oracle supplied with the FPGA mapper."""
    words = validate_words(words)
    # Deliberately no floating-point fallback: an approximate envelope is not a
    # valid basis for admitting a configuration to the physical tile cache.
    from complete_geometry_reference import vector_fixed_map
    return vector_fixed_map(width, height, words[:18], words[18:], tangential_q16=TANGENTIAL_Q16)


@lru_cache(maxsize=12)
def check_geometry(words, profile=PRODUCTION_BUDGET, allow_black=True):
    words = validate_words(words)
    entries, errors = fixed_map(WIDTH, HEIGHT, words)
    invalid = int(np.count_nonzero((entries >> 38) == 0))
    budget = tile_budget(entries, WIDTH, HEIGHT, profile.tile_width, profile.tile_height,
                         profile.cache_width, profile.cache_height)
    result = dict(width=WIDTH, height=HEIGHT, invalid_pixels=invalid,
                  invalid_fraction=invalid / (WIDTH * HEIGHT), arithmetic_error_pixels=int(errors),
                  tile_budget=budget, fixed_point_checked=True,
                  model=JOINT_MODEL, compiled_tangential_q16=list(TANGENTIAL_Q16),
                  build_id=f'0x{BUILD_ID:08X}',
                  meaning='Offline exact-integer geometry/cache check; not optical calibration or board verification.')
    if errors:
        raise ValueError(f'FPGA arithmetic domain fails for {errors} pixels; configuration not admissible.')
    if invalid == WIDTH * HEIGHT:
        raise ValueError('All source samples are invalid.')
    if budget['unsupported_tiles']:
        raise ValueError('FPGA source-cache envelope exceeded: ' + str(budget['unsupported_tiles'][:3]))
    if invalid and not allow_black:
        raise ValueError(f'{invalid} invalid pixels remain; no automatic black-border removal is performed.')
    return result


FOUR_POINT_OUTPUT = [[-1., -.5], [1., -.5], [1., .5], [-1., .5]]
FOUR_POINT_SOURCE = [[-.90, -.46], [.94, -.49], [1.00, .48], [-.96, .51]]
PRESETS = {
    'c97_joint': dict(zoom=1.),
    # Historical preset name remains an alias for this build's whole joint set.
    'c97_original': dict(zoom=1.),
    'safe_pose': dict(roll=3., pitch=3., yaw=-3., zoom=1.3, tx=.02, ty=-.01),
    'four_point_demo': dict(zoom=1.3, matrix=four_point_matrix(FOUR_POINT_OUTPUT, FOUR_POINT_SOURCE).tolist()),
    'raw': dict(raw=True),
}


def preset(name, calibration=None):
    if name not in PRESETS:
        raise ValueError('Unknown geometry preset.')
    return configuration(load_calibration() if calibration is None else calibration, **PRESETS[name])

