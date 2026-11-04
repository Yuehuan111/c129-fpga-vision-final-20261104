"""File-only geometry workbench. No transport or live-device access.

Point inversion and ordering follow C127 scan_plane.py, itself adapted from
plane_rectification_preview.py (SHA256
2105757a0aef2d06d8c9ed5ebbf36d973a05f9a2749b65f827156587687ede29).
Raw previews reuse the shipped integer mapper; RGB interpolation is OpenCV BGR,
not RTL RGB565 verification. Rectified input never repeats lens correction.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Callable

import cv2
import numpy as np

HOST = Path(__file__).resolve().parent
if str(HOST / 'vendor') not in sys.path:
    sys.path.insert(0, str(HOST / 'vendor'))
from complete_geometry import configuration, load_calibration
from complete_geometry_reference import vector_fixed_map
from project_profile import DEFAULT_CALIBRATION
from scan_plane import _joint_unproject, make_context
from tile_budget import tile_budget

WIDTH, HEIGHT = 1280, 720


class GeometryCancelled(RuntimeError):
    pass


@dataclass(frozen=True)
class GeometryOptions:
    input_domain: str = 'raw'
    operation: str = 'scan'
    columns: int = 10
    rows: int = 5
    cell_pixels: float = 64.0
    roll_degrees: float = 0.0
    zoom: float = 1.0

    def validate(self):
        if self.input_domain not in ('raw', 'rectified'):
            raise ValueError('请选择原始鱼眼图或已去镜头畸变图。')
        if self.operation not in ('lens', 'scan', 'roll'):
            raise ValueError('未知离线处理模式。')
        for value in (self.columns, self.rows):
            if isinstance(value, bool) or not isinstance(value, int) or not 2 <= value <= 20:
                raise ValueError('棋盘内角点列/行必须是 2～20 的整数。')
        if not math.isfinite(self.cell_pixels) or not 16 <= self.cell_pixels <= 90:
            raise ValueError('输出格距应为 16～90 像素。')
        if ((self.columns+2)*self.cell_pixels > WIDTH or
                (self.rows+2)*self.cell_pixels > HEIGHT):
            raise ValueError('棋盘和周边范围超出画布，请减小输出格距。')
        if not math.isfinite(self.roll_degrees) or abs(self.roll_degrees) > 3600:
            raise ValueError('离线角度应是 -3600～3600 度的有限数值。')
        if not math.isfinite(self.zoom) or not 1 <= self.zoom <= 8:
            raise ValueError('固定放大倍率应为 1～8。')
        return self


@dataclass
class GeometryResult:
    images: dict[str, np.ndarray]
    report: dict


class ResultGeneration:
    """UI-owned generation counter; old work cannot replace a new source/result."""
    def __init__(self):
        self.generation = 0
        self.closed = False

    def next(self):
        if self.closed:
            raise RuntimeError('工作区已关闭。')
        self.generation += 1
        return self.generation

    def accepts(self, token):
        return not self.closed and token == self.generation

    def close(self):
        self.generation += 1
        self.closed = True


def read_image(path):
    path = Path(path)
    data = path.read_bytes()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('无法读取所选图片。')
    if (image.shape[1], image.shape[0]) not in ((1280, 720), (1920, 1080)):
        raise ValueError('本工具只接受完整的 1280×720 或同视野 1920×1080 图片；不自动处理裁剪截图。')
    return image, hashlib.sha256(data).hexdigest()


def canonical_corners(corners, columns, rows):
    points = np.asarray(corners, dtype=float).reshape(rows, columns, 2)
    if points[:, :, 0].mean(axis=0)[0] > points[:, :, 0].mean(axis=0)[-1]:
        points = points[:, ::-1]
    if points[:, :, 1].mean(axis=1)[0] > points[:, :, 1].mean(axis=1)[-1]:
        points = points[::-1]
    return points.reshape(-1, 2).copy()


def detect_corners(image, columns=10, rows=5):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    attempts = []
    for name, flags in (
            ('SB_NORMALIZE', cv2.CALIB_CB_NORMALIZE_IMAGE),
            ('SB_EXHAUSTIVE_ACCURACY', cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)):
        ok, corners = cv2.findChessboardCornersSB(gray, (columns, rows), flags=flags)
        attempts.append({'method': name, 'found': bool(ok)})
        if ok:
            return canonical_corners(corners, columns, rows), attempts
    raise ValueError(f'未检测到完整 {columns}×{rows} 内角点棋盘；请核对角点数、清晰度及完整边界。')


def shape_metrics(corners, columns, rows):
    points = np.asarray(corners).reshape(rows, columns, 2)
    dx, dy = np.diff(points, axis=1), np.diff(points, axis=0)
    horizontal, vertical = np.linalg.norm(dx, axis=2), np.linalg.norm(dy, axis=2)
    a, b = dx[:-1], dy[:, :-1]
    cosine = np.sum(a*b, axis=2)/(np.linalg.norm(a, axis=2)*np.linalg.norm(b, axis=2))
    angles = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
    return {'horizontal_step_mean_px': float(horizontal.mean()),
            'vertical_step_mean_px': float(vertical.mean()),
            'horizontal_step_cv_percent': float(100*horizontal.std()/horizontal.mean()),
            'vertical_step_cv_percent': float(100*vertical.std()/vertical.mean()),
            'mean_absolute_right_angle_error_deg': float(np.abs(angles-90).mean())}


def _transform(points, matrix):
    return cv2.perspectiveTransform(np.asarray(points, float)[:, None, :], matrix).reshape(-1, 2)


def _fixed_render(source, calibration, matrix=None, roll=0.0, zoom=1.0):
    words = configuration(calibration, matrix=matrix, roll=roll, zoom=zoom, legacy_angle=0)
    entries, domain = vector_fixed_map(WIDTH, HEIGHT, words[:18], words[18:])
    valid = (entries >> 38) != 0
    mapx = ((entries >> 19) & ((1 << 19)-1)).astype(np.float32)/256
    mapy = (entries & ((1 << 19)-1)).astype(np.float32)/256
    result = cv2.remap(source, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    result[~valid] = 0
    budget = tile_budget(entries, WIDTH, HEIGHT, 64, 16, 256, 64)
    invalid = int(np.count_nonzero(~valid))
    info = {'coordinate_model': 'C127 shipped integer mapper; OpenCV BGR bilinear sampling',
            'invalid_pixels': invalid, 'domain_invalid_pixels': int(domain),
            'valid_fraction': float(valid.mean()), 'tile_budget': budget,
            'map_guard_admitted': invalid < WIDTH*HEIGHT and not budget['unsupported_tiles'],
            'hardware_matrix_exported': False,
            'meaning': 'Static coordinate/ROI evidence only; not live admission, timing or board validation'}
    return result, info


def _pixel_render(source, inverse):
    result = cv2.warpPerspective(source, inverse, (WIDTH, HEIGHT),
                                 flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                 borderMode=cv2.BORDER_CONSTANT)
    y, x = np.indices((HEIGHT, WIDTH), dtype=float)
    z = inverse[2, 0]*x+inverse[2, 1]*y+inverse[2, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        sx = (inverse[0, 0]*x+inverse[0, 1]*y+inverse[0, 2])/z
        sy = (inverse[1, 0]*x+inverse[1, 1]*y+inverse[1, 2])/z
    valid = np.isfinite(sx) & np.isfinite(sy) & (np.abs(z) > 1e-12)
    valid &= (sx >= 0) & (sx < WIDTH-1) & (sy >= 0) & (sy < HEIGHT-1)
    return result, {'coordinate_model': 'Pixel homography of already corrected input; no second lens operation',
                    'valid_fraction': float(valid.mean()),
                    'validity_rule': 'Full bilinear source footprint; conservative at image boundary',
                    'hardware_matrix_exported': False,
                    'meaning': 'Prior optical correction configuration is unknown; image-only result'}


def process_image(path, options=None, *, calibration=None, cancelled: Callable[[], bool] | None = None):
    options = (options or GeometryOptions()).validate()
    def checkpoint():
        if cancelled is not None and cancelled():
            raise GeometryCancelled('旧任务已取消。')
    checkpoint()
    original, source_hash = read_image(path)
    used_default_calibration = calibration is None
    calibration = load_calibration() if used_default_calibration else calibration
    source = cv2.resize(original, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    report = {'schema': 'unified_geometry_preview_v1', 'source': str(Path(path)),
              'source_sha256': source_hash, 'source_size': [original.shape[1], original.shape[0]],
              'output_size': [WIDTH, HEIGHT], 'options': asdict(options),
              'calibration_file_sha256': hashlib.sha256(Path(DEFAULT_CALIBRATION).read_bytes()).hexdigest()
              if used_default_calibration else None,
              'normalized_calibration_sha256': hashlib.sha256(json.dumps(calibration, sort_keys=True,
                  separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')).hexdigest(),
              'input_domain_is_user_selected': True, 'preview_only': True,
              'hardware_accessed': False, 'parameter_upload': False,
              'limitations': ['Software BGR interpolation is not RTL RGB565 pixel verification.',
                              'Existing calibration optical-mode assumptions remain.',
                              'Only the selected plane is rectified; background may stretch.',
                              'No live IMU, throughput, timing or continuous 360-degree validation.'],
              'provenance': 'C127 scan_plane.py joint point inversion; existing complete_geometry_reference.py mapper; original plane preview SHA256 2105757a0aef2d06d8c9ed5ebbf36d973a05f9a2749b65f827156587687ede29'}
    if options.input_domain == 'raw':
        lens, report['lens_geometry'] = _fixed_render(source, calibration)
    else:
        lens = source.copy()
        report['lens_geometry'] = {'operation': 'identity', 'reason': 'Input already has lens correction'}
    checkpoint()
    if options.operation == 'lens':
        result = lens.copy()
        report['output_geometry'] = report['lens_geometry']
    elif options.operation == 'roll':
        if options.input_domain == 'raw':
            result, info = _fixed_render(source, calibration, roll=options.roll_degrees, zoom=options.zoom)
        else:
            angle = math.radians(options.roll_degrees)
            c, s = math.cos(angle), math.sin(angle)
            k = np.array([[1., 0, (WIDTH-1)/2], [0, 1., (HEIGHT-1)/2], [0, 0, 1.]])
            rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
            inverse = k @ rz @ np.diag([1/options.zoom, 1/options.zoom, 1.]) @ np.linalg.inv(k)
            result, info = _pixel_render(source, inverse)
        report['output_geometry'] = info
        report['roll_convention'] = 'Inverse-sampling positive angle; image content rotates in the opposite visual direction'
        report['production_roll_limit_changed'] = False
    else:
        corners, attempts = detect_corners(original, options.columns, options.rows)
        points = (corners+.5)*(WIDTH/original.shape[1])-.5
        report['input_detection'] = attempts
        report['before_metrics'] = shape_metrics(points, options.columns, options.rows)
        checkpoint()
        grid = np.mgrid[:options.rows, :options.columns][::-1].transpose(1, 2, 0).reshape(-1, 2).astype(float)
        cell = options.cell_pixels
        origin = np.array([(WIDTH-(options.columns+1)*cell)/2+cell,
                           (HEIGHT-(options.rows+1)*cell)/2+cell])
        target = grid*cell+origin
        pixel_to_grid = np.array([[1/cell, 0, -origin[0]/cell], [0, 1/cell, -origin[1]/cell], [0, 0, 1.]])
        if options.input_domain == 'raw':
            context = make_context(calibration, configuration(calibration)[:18])
            fitting, inverse_error = _joint_unproject(points, context)
            report['point_roundtrip_max_source_px'] = inverse_error
        else:
            fitting = points
        grid_to_source, _ = cv2.findHomography(grid, fitting, method=0)
        if grid_to_source is None or abs(np.linalg.det(grid_to_source)) < 1e-12:
            raise ValueError('棋盘单应矩阵退化，请更换清晰且完整的图像。')
        train = np.indices((options.rows, options.columns)).sum(axis=0).ravel()%2 == 0
        if np.count_nonzero(train) >= 4 and np.count_nonzero(~train) >= 4:
            held, _ = cv2.findHomography(grid[train], fitting[train], method=0)
            if held is not None and abs(np.linalg.det(held)) > 1e-12:
                residual = _transform(fitting[~train], np.linalg.inv(held))-grid[~train]
                report['heldout_rms_target_px'] = float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))*cell)
        checkpoint()
        if options.input_domain == 'raw':
            inverse = grid_to_source @ pixel_to_grid @ np.asarray(calibration['output_camera_matrix'])
            if abs(inverse[2, 2]) < 1e-10:
                raise ValueError('画面中心接近透视奇点，请重新取景。')
            inverse /= inverse[2, 2]
            result, info = _fixed_render(source, calibration, matrix=inverse)
        else:
            inverse = grid_to_source @ pixel_to_grid
            result, info = _pixel_render(source, inverse)
        report['output_geometry'] = info
        report['layout'] = {'cell_pixels': cell, 'board_size': [(options.columns+1)*cell, (options.rows+1)*cell]}
        # Re-detect actual raster output; never substitute target or transformed fit points.
        lo = np.floor(origin-cell-cell/4).astype(int)
        hi = np.ceil(origin+[options.columns*cell, options.rows*cell]+cell/4).astype(int)
        lo = np.maximum(lo, 0); hi = np.minimum(hi, [WIDTH, HEIGHT])
        checkpoint()
        try:
            actual, output_attempts = detect_corners(result[lo[1]:hi[1], lo[0]:hi[0]], options.columns, options.rows)
            report['output_detection'] = output_attempts
            report['redetected_corner_count'] = len(actual)
            report['after_metrics'] = shape_metrics(actual, options.columns, options.rows)
            report['redetected_target_rms_px'] = float(np.sqrt(np.mean(np.sum((actual+lo-target)**2, axis=1))))
        except ValueError as error:
            report['redetected_corner_count'] = 0
            report['after_metrics'] = None
            report['redetection_warning'] = str(error)
    checkpoint()
    report['mirror_method'] = 'Horizontal pixel index reversal after single interpolation; not a second warp or RTL compositor verification'
    return GeometryResult({'input': source, 'lens': lens, 'output': result,
                           'mirror': np.ascontiguousarray(result[:, ::-1])}, report)


def save_result(result, directory):
    """Explicit export into a fresh subdirectory; never overwrite source/calibration."""
    parent = Path(directory)
    target = parent / ('geometry_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    target.mkdir(parents=True, exist_ok=False)
    for key, image in result.images.items():
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            raise ValueError('图片导出失败。')
        (target/(key+'.png')).write_bytes(encoded.tobytes())
    (target/'report.json').write_text(json.dumps(result.report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    return target
