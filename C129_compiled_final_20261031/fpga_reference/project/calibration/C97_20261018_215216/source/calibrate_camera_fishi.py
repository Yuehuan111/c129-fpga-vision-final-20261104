#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""棋盘格相机标定：在电脑上运行，无需烧录到摄像头。

安装：python -m pip install opencv-python numpy
运行：python calibrate_camera_fishi.py
图片：本文件旁的 calib_images 文件夹
默认：1920×1080，OpenCV fisheye鱼眼模型（与旧Brown FPGA参数不兼容）。
鱼眼输出：居中透视画面，水平视场122°；不再使用balance自动估计K_out。
旧模型：python calibrate_camera_fishi.py --model brown，固定 k3=0。
"""

from pathlib import Path
from datetime import datetime
import csv
import json
import math
import sys
import argparse

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print(f"缺少依赖或依赖加载失败：{exc}")
    print("请在同一个Python环境中执行：python -m pip install opencv-python numpy")
    raise SystemExit(1)


# ==================== 按实物和最终输入模式修改 ====================
BOARD = (10, 5)               # 内部角点：(列数, 行数)，不是方格数量
SQUARE_MM = 20.0              # 单个方格实测边长，单位毫米
EXPECTED_SIZE = (1920, 1080)  # 图片：(宽, 高)，严格检查，不自动缩放
MIN_VIEWS = 25                # 操作门槛；建议20～30张不同姿态的有效照片
FRAC_BITS = 16               # 团队标准：32位有符号Q16.16，1.0对应65536
OUTPUT_HFOV_DEG = 122.0      # 鱼眼矫正输出的水平视场，单位度；应在0～180之间
OUTPUT_ALPHA = 1.0           # 仅旧Brown模型使用；鱼眼输出不使用alpha或balance
# =================================================================

ROOT = Path(__file__).resolve().parent
INPUT_DIR = ROOT / "calib_images"
VALIDATION_DIR = ROOT / "validation_images"
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def image_paths(folder):
    return [p for p in sorted(folder.iterdir())
            if p.is_file() and p.suffix.lower() in IMAGE_EXTS]


def read_image(path):
    """支持Windows中文路径，并应用EXIF方向，与看图软件中的显示方向一致。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    # JPG旋转可能只记录在EXIF中；忽略它会使JPG与已旋转像素的PNG方向不一致。
    # 不额外强制旋转180度，避免对已旋转像素的图片重复旋转。
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def save_png(path, image):
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"图片编码失败：{path}")
    encoded.tofile(str(path))


def save_json(path, data):
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8"
    )


def fixed_parameters(parameters):
    result = {"total_bits": 32, "fractional_bits": FRAC_BITS,
              "fixed_point_format": f"Q{32 - FRAC_BITS}.{FRAC_BITS}",
              "rounding": "floor(value * 2^fractional_bits + 0.5)",
              "all_parameters_fit": True, "parameters": {}}
    for name, value in parameters.items():
        integer = math.floor(value * (1 << FRAC_BITS) + 0.5)
        if not -(1 << 31) <= integer <= (1 << 31) - 1:
            result["all_parameters_fit"] = False
            result["parameters"][name] = {
                "float_value": value,
                "error": "超出32位有符号范围，请调整位宽或小数位数"
            }
        else:
            restored = integer / float(1 << FRAC_BITS)
            result["parameters"][name] = {
                "float_value": value,
                "decimal": integer,
                "verilog_hex": f"32'sh{integer & 0xffffffff:08X}",
                "restored_float": restored,
                "quantization_error": restored - value
            }
    return result


def calibrate_fisheye(object_points, image_points, size):
    """Fit every supplied view from two focal-length guesses; keep the best fit."""
    if len(object_points) == 0 or len(object_points) != len(image_points):
        raise ValueError("物点和像点必须非空且视图数一致")
    # OpenCV 5.x on Windows expects each fisheye view as 1xNx3/1xNx2.
    objects = [np.ascontiguousarray(p, dtype=np.float64).reshape(1, -1, 3)
               for p in object_points]
    images = [np.ascontiguousarray(p, dtype=np.float64).reshape(1, -1, 2)
              for p in image_points]
    if any(len(op) != len(ip) or len(op) == 0 for op, ip in zip(objects, images)):
        raise ValueError("每张图片的物点和像点数量必须一致且非空")
    # OpenCV 5 exposes fisheye calibration flags at the cv2 top level.
    flags = (cv2.CALIB_FIX_SKEW | cv2.CALIB_RECOMPUTE_EXTRINSIC
             | cv2.CALIB_CHECK_COND | cv2.CALIB_USE_INTRINSIC_GUESS)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10)
    candidates, attempts = [], []
    for focal in dict.fromkeys((0.55 * max(size), 0.40 * size[0])):
        initial_K = np.array([[focal, 0, (size[0] - 1) / 2],
                              [0, focal, (size[1] - 1) / 2],
                              [0, 0, 1]], dtype=np.float64)
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objects, images, size, initial_K, np.zeros((4, 1), np.float64),
                flags=flags, criteria=criteria)
            if (not np.isfinite(rms) or not np.isfinite(K).all()
                    or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0
                    or len(rvecs) != len(objects) or len(tvecs) != len(objects)
                    or not all(np.isfinite(v).all() for v in (*rvecs, *tvecs))):
                raise RuntimeError("鱼眼拟合返回异常参数或不完整的视图姿态")
        except (cv2.error, RuntimeError) as exc:
            attempts.append({"initial_focal_px": float(focal),
                             "status": "failed", "error": str(exc)})
            continue
        attempts.append({"initial_focal_px": float(focal), "status": "success",
                         "rms_px": float(rms)})
        candidates.append({"rms": float(rms), "K": K, "D": D,
                           "rvecs": rvecs, "tvecs": tvecs, "flags": int(flags),
                           "criteria": criteria, "initial_focal_px": float(focal)})
    if not candidates:
        reasons = "\n".join(f"f={a['initial_focal_px']}: {a['error']}" for a in attempts)
        raise RuntimeError("鱼眼模型的全部初值均拟合失败；未删除任何视图。\n" + reasons)
    result = min(candidates, key=lambda candidate: candidate["rms"])
    result["initialization_attempts"] = attempts
    return result


def fit_model(object_points, image_points, size, model="fisheye"):
    """Return fit parameters and optimizer metadata without reading/writing files."""
    if model == "fisheye":
        return calibrate_fisheye(object_points, image_points, size)
    if model != "brown":
        raise ValueError(f"不支持的模型：{model}")
    # 保留旧Brown拟合调用；CALIB_FIX_K3将k3固定为零。
    flags = cv2.CALIB_FIX_K3
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, size, None, None, flags=flags)
    return {"rms": float(rms), "K": K, "D": D, "rvecs": rvecs, "tvecs": tvecs,
            "flags": int(flags), "criteria": None, "initialization_attempts": []}


def build_undistortion_maps(K, D, size, model="fisheye"):
    # K描述真实相机，K_out描述矫正图的视野；不要用K_out替换标定K。
    if model == "fisheye":
        if not np.isfinite(OUTPUT_HFOV_DEG) or not 0 < OUTPUT_HFOV_DEG < 180:
            raise ValueError("OUTPUT_HFOV_DEG应大于0且小于180，单位为度")
        width, height = size
        # 用画布外边界定义水平视场；主点居中，横纵方向使用相同的像素焦距。
        focal = (width / 2) / math.tan(math.radians(OUTPUT_HFOV_DEG / 2))
        K_out = np.array([[focal, 0, (width - 1) / 2],
                          [0, focal, (height - 1) / 2],
                          [0, 0, 1]], dtype=np.float64)
        valid_roi = None  # fisheye API不提供Brown式有效矩形；不虚构ROI。
        map_builder = cv2.fisheye.initUndistortRectifyMap
    elif model == "brown":
        K_out, valid_roi = cv2.getOptimalNewCameraMatrix(
            K, D, size, OUTPUT_ALPHA, size)
        valid_roi = tuple(int(v) for v in valid_roi)
        map_builder = cv2.initUndistortRectifyMap
    else:
        raise ValueError(f"不支持的模型：{model}")
    if (not np.isfinite(K_out).all() or K_out[0, 0] <= 0 or K_out[1, 1] <= 0):
        raise RuntimeError("输出视场计算异常，请检查标定模型和参数")
    map_x, map_y = map_builder(
        K, D, np.eye(3), K_out, size, cv2.CV_32FC1)
    if not np.isfinite(map_x).all() or not np.isfinite(map_y).all():
        raise RuntimeError("矫正映射包含异常坐标，请检查标定参数")
    # valid_roi只作记录；按它裁剪会再次丢失外围内容。
    return K_out, valid_roi, map_x, map_y


def valid_grid(corners, board, size):
    """Reject incomplete, non-finite, out-of-frame or folded grids."""
    if corners is None or corners.size != board[0] * board[1] * 2:
        return False
    pts = corners.reshape(board[1], board[0], 2)
    if not np.isfinite(pts).all():
        return False
    if (pts[..., 0].min() < 0 or pts[..., 0].max() >= size[0]
            or pts[..., 1].min() < 0 or pts[..., 1].max() >= size[1]):
        return False
    horizontal = pts[:-1, 1:] - pts[:-1, :-1]
    vertical = pts[1:, :-1] - pts[:-1, :-1]
    cross = horizontal[..., 0] * vertical[..., 1] - horizontal[..., 1] * vertical[..., 0]
    return bool(np.all(cross > 1) or np.all(cross < -1))


def detect_corners(gray, board):
    """Try complementary detectors; always return ORIGINAL image coordinates.

    Resizing is only for detection, not for calibration image geometry.
    No synthetic corners, rectification or partial grids are used.
    """
    size = (gray.shape[1], gray.shape[0])
    normal = cv2.CALIB_CB_NORMALIZE_IMAGE
    robust = normal | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    for method in ('sb_original', 'sb_half_exhaustive', 'classic_subpixel', 'sb_full_exhaustive'):
        if method == 'sb_half_exhaustive':
            small = cv2.resize(gray, (max(1, size[0] // 2), max(1, size[1] // 2)),
                               interpolation=cv2.INTER_AREA)
            found, corners = cv2.findChessboardCornersSB(small, board, robust)
            if found:
                # OpenCV resize maps pixel centres, including odd image dimensions.
                scale = np.array([size[0] / small.shape[1], size[1] / small.shape[0]], np.float32)
                corners = ((corners + 0.5) * scale - 0.5).astype(np.float32)
        elif method == 'classic_subpixel':
            found, corners = cv2.findChessboardCorners(
                gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH | normal)
        else:
            found, corners = cv2.findChessboardCornersSB(
                gray, board, normal if method == 'sb_original' else robust)
        if not found:
            continue
        if method in ('sb_half_exhaustive', 'classic_subpixel'):
            grid = corners.reshape(board[1], board[0], 2)
            spacing = min(np.linalg.norm(np.diff(grid, axis=0), axis=2).min(),
                          np.linalg.norm(np.diff(grid, axis=1), axis=2).min())
            radius = max(2, min(7, int(spacing / 4)))
            corners = cv2.cornerSubPix(gray, corners, (radius, radius), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
        if valid_grid(corners, board, size):
            return True, corners, method
    return False, None, 'not_found'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detect-only', action='store_true',
                        help='只检查棋盘格并保存角点预览，不拟合或导出畸变参数')
    parser.add_argument('--model', choices=('brown', 'fisheye'), default='fisheye',
                        help='默认fisheye鱼眼模型；brown保留原针孔模型')
    args = parser.parse_args()
    cv2.setNumThreads(min(4, cv2.getNumThreads()))
    if not hasattr(cv2, "findChessboardCornersSB"):
        raise RuntimeError("需要支持findChessboardCornersSB的OpenCV 4.x，请升级")
    if len(BOARD) != 2 or any(not isinstance(n, int) or n < 3 for n in BOARD):
        raise ValueError("BOARD应为两个不小于3的整数：(内部角点列数, 行数)")
    if not np.isfinite(SQUARE_MM) or SQUARE_MM <= 0:
        raise ValueError("SQUARE_MM应为实测方格边长，且大于零")
    if (len(EXPECTED_SIZE) != 2
            or any(not isinstance(n, int) or n <= 0 for n in EXPECTED_SIZE)):
        raise ValueError("EXPECTED_SIZE应为正整数的(宽, 高)")
    if not isinstance(MIN_VIEWS, int) or MIN_VIEWS < 3:
        raise ValueError("MIN_VIEWS至少为3，当前建议25张以上不同姿态")
    if not isinstance(FRAC_BITS, int) or not 0 <= FRAC_BITS <= 30:
        raise ValueError("FRAC_BITS应是0～30之间的整数")
    if args.model == "fisheye" and (
            not np.isfinite(OUTPUT_HFOV_DEG) or not 0 < OUTPUT_HFOV_DEG < 180):
        raise ValueError("OUTPUT_HFOV_DEG应大于0且小于180，单位为度")
    if args.model == "brown" and (
            not np.isfinite(OUTPUT_ALPHA) or not 0 <= OUTPUT_ALPHA <= 1):
        raise ValueError("OUTPUT_ALPHA应在0～1之间")

    INPUT_DIR.mkdir(parents=True, exist_ok=True)
    VALIDATION_DIR.mkdir(parents=True, exist_ok=True)
    paths = image_paths(INPUT_DIR)
    if not paths:
        print("还没有标定图片。请把20～30张棋盘格图片放入：")
        print(INPUT_DIR)
        print(f"当前配置：内部角点={BOARD}，方格边长={SQUARE_MM}mm，尺寸={EXPECTED_SIZE}")
        print("按实物修改脚本开头配置后再次运行。")
        return 2

    # 每次生成独立目录，不混入旧参数或旧预览。
    output = ROOT / "results" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    corners_dir = output / "corners"
    if args.detect_only:
        corners_dir.mkdir()
    print("图片目录：", INPUT_DIR)
    print("本次结果目录：", output)
    print("标定模型：", args.model)
    print(f"内部角点={BOARD}，方格边长={SQUARE_MM}mm，尺寸={EXPECTED_SIZE}")

    # 平面棋盘格：Z=0，单位mm；角点按行排列。
    template = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    template[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2)
    template *= SQUARE_MM
    object_points, image_points, accepted = [], [], []
    rejected = []
    detections = []

    for index, path in enumerate(paths, 1):
        image = read_image(path)
        if image is None:
            rejected.append({"file": path.name, "reason": "无法解码"})
            print(f"[{index}/{len(paths)}] 跳过：{path.name}，无法解码")
            continue
        size = (image.shape[1], image.shape[0])
        if size != EXPECTED_SIZE:
            raise ValueError(
                f"{path.name}尺寸为{size}，期望{EXPECTED_SIZE}。"
                "检查相机和采集软件设置；最终模式不同则修改EXPECTED_SIZE。"
                "不要仅为凑尺寸而随意缩放图片。"
            )
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners, method = detect_corners(gray, BOARD)
        detections.append({'file': path.name, 'found': bool(found), 'method': method})
        if not found:
            rejected.append({"file": path.name, "reason": "未找到完整棋盘格"})
            print(f"[{index}/{len(paths)}] 跳过：{path.name}，未找到完整棋盘格")
            continue
        # detect_corners返回原图坐标；完整原图SB结果不额外做cornerSubPix。
        object_points.append(template.copy())
        image_points.append(corners)
        accepted.append(path)
        if args.detect_only:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, BOARD, corners, True)
            save_png(corners_dir / f"{index:03d}_{path.stem}_corners.png", preview)
        print(f"[{index}/{len(paths)}] 检测成功：{path.name} ({method})")

    save_json(output / "image_detection.json", {
        "board_inner_corners": list(BOARD),
        "expected_image_size": list(EXPECTED_SIZE),
        "accepted_images": [p.name for p in accepted],
        "rejected_images": rejected,
        "detections": detections,
        "accepted_count": len(accepted), "total_count": len(paths),
        "note": "检测成功不代表标定合格；必须使用平整硬板，检查角点和独立验证。"
    })
    if image_points:
        np.savez_compressed(output / 'detected_points.npz',
                            corners=np.asarray(image_points),
                            filenames=np.asarray([p.name for p in accepted]),
                            image_size=np.asarray(EXPECTED_SIZE), board=np.asarray(BOARD))
    print(f"\n角点检测：{len(accepted)}/{len(paths)} 张。识别失败清单见 image_detection.json。")
    if args.detect_only:
        print("仅检测完成，没有生成畸变参数。角点预览：", corners_dir)
        print("手持弯曲纸张即使识别成功也不适合最终标定，请贴平在硬板上重拍。")
        return 0
    if len(accepted) < MIN_VIEWS:
        raise RuntimeError(
            f"有效图片只有{len(accepted)}张，至少需要{MIN_VIEWS}张。"
            "使用 --detect-only 检查角点预览，并补拍不同位置和倾斜角度的照片。"
        )

    fit = fit_model(object_points, image_points, EXPECTED_SIZE, args.model)
    rms, K, D = fit["rms"], fit["K"], fit["D"]
    rvecs, tvecs, flags = fit["rvecs"], fit["tvecs"], fit["flags"]
    is_fisheye = args.model == "fisheye"
    model_name = "opencv_fisheye_k1_k2_k3_k4" if is_fisheye else "pinhole_k1_k2_p1_p2_k3_fixed_zero"
    coeff_order = ["fish_k1", "fish_k2", "fish_k3", "fish_k4"] if is_fisheye else ["k1", "k2", "p1", "p2", "k3"]
    distortion = D.ravel()
    if (not np.isfinite(rms) or not np.isfinite(K).all()
            or not np.isfinite(distortion).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise RuntimeError("标定结果异常，请检查角点、标定板和平面倾斜姿态")
    if is_fisheye and distortion.size != 4:
        raise RuntimeError("鱼眼模型应为4项角度多项式系数")
    if not is_fisheye and (distortion.size != 5 or abs(float(distortion[4])) > 1e-12):
        raise RuntimeError("模型不符合预期：应为5项畸变系数且k3=0")
    if len(rvecs) != len(accepted) or len(tvecs) != len(accepted):
        raise RuntimeError("拟合返回的视图姿态数量与全部有效图片不一致")

    views = []
    squared_error_sum, point_count = 0.0, 0
    for path, op, ip, rv, tv in zip(
        accepted, object_points, image_points, rvecs, tvecs
    ):
        if is_fisheye:
            projected, _ = cv2.fisheye.projectPoints(
                np.ascontiguousarray(op, dtype=np.float64).reshape(-1, 1, 3), rv, tv, K, D)
        else:
            projected, _ = cv2.projectPoints(op, rv, tv, K, D)
        delta = ip.reshape(-1, 2) - projected.reshape(-1, 2)
        squared_errors = np.sum(delta ** 2, axis=1)
        squared_error_sum += float(np.sum(squared_errors))
        point_count += len(squared_errors)
        error = float(np.sqrt(np.mean(squared_errors)))
        if not np.isfinite(error):
            raise RuntimeError(f"{path.name}的重投影误差异常")
        views.append({"file": path.name, "rms_px": error,
                      "rvec": rv.ravel().tolist(), "tvec_mm": tv.ravel().tolist()})
    training_rms = float(np.sqrt(squared_error_sum / point_count))

    parameters = {
        "fx": float(K[0, 0]), "fy": float(K[1, 1]),
        "cx": float(K[0, 2]), "cy": float(K[1, 2])
    }
    parameters.update({name: float(value) for name, value in zip(coeff_order[:4], distortion[:4])})
    warnings = []
    if is_fisheye:
        warnings.append("本结果使用OpenCV fisheye角度模型；fish_k1..fish_k4不能写入旧Brown FPGA的k1/k2/p1/p2寄存器。")
    else:
        warnings.append("本结果使用Brown针孔模型（k3=0），不是OpenCV fisheye模型；鱼眼镜头需检查模型适用性。")
    warnings.append("参数对应当前图片尺寸与采集链路；若与FPGA输入分辨率/裁剪不同，不能直接套用。")
    if len(accepted) < 20:
        warnings.append("有效图片少于20张，建议补充不同姿态和边缘位置。")
    if not (0 <= K[0, 2] < EXPECTED_SIZE[0] and 0 <= K[1, 2] < EXPECTED_SIZE[1]):
        warnings.append("主点位于图像范围之外，请检查数据覆盖和拟合稳定性。")

    K_out, valid_roi, map_x, map_y = build_undistortion_maps(K, D, EXPECTED_SIZE, args.model)
    output_parameters = {
        "fx_out": float(K_out[0, 0]), "fy_out": float(K_out[1, 1]),
        "cx_out": float(K_out[0, 2]), "cy_out": float(K_out[1, 2])
    }
    warnings.append("矫正图使用独立输出内参K_out；FPGA复现时也须使用output_camera_matrix，不能继续假定输出内参等于K。")
    fov_parameter = "horizontal_fov_deg" if is_fisheye else "alpha"
    fov_value = float(OUTPUT_HFOV_DEG if is_fisheye else OUTPUT_ALPHA)
    output_view_selection = "centered_horizontal_fov" if is_fisheye else "opencv_optimal_alpha"
    width, height = EXPECTED_SIZE
    horizontal_fov_deg = math.degrees(
        math.atan((width - 0.5 - K_out[0, 2]) / K_out[0, 0])
        - math.atan((-0.5 - K_out[0, 2]) / K_out[0, 0]))
    vertical_fov_deg = math.degrees(
        math.atan((height - 0.5 - K_out[1, 2]) / K_out[1, 1])
        - math.atan((-0.5 - K_out[1, 2]) / K_out[1, 1]))
    in_bounds = ((map_x >= 0) & (map_x <= width - 1)
                 & (map_y >= 0) & (map_y <= height - 1))
    map_in_bounds_fraction = float(np.mean(in_bounds))
    # 只说明采样是否超出原图，不作为标定精度或画面质量验收。
    output_view_metadata = {
        "output_view_selection": output_view_selection,
        "output_fov_parameter": fov_parameter, "output_fov_value": fov_value,
        "output_horizontal_fov_deg": horizontal_fov_deg,
        "output_vertical_fov_deg": vertical_fov_deg,
        "map_in_bounds_fraction": map_in_bounds_fraction,
    }
    fixed_format_tag = f"q{32 - FRAC_BITS}_{FRAC_BITS}"
    fixed_filename = (f"parameters_fisheye_{fixed_format_tag}.json" if is_fisheye
                      else f"parameters_{fixed_format_tag}.json")

    save_json(output / "calibration.json", {
        "created_at": datetime.now().astimezone().isoformat(),
        "opencv_version": cv2.__version__,
        "model": model_name,
        "compatible_with_legacy_brown_fpga": not is_fisheye,
        "image_size": list(EXPECTED_SIZE),
        "board_inner_corners": list(BOARD), "square_size_mm": SQUARE_MM,
        "camera_matrix": K.tolist(),
        "dist_coeff_order": coeff_order,
        "dist_coeffs": distortion.tolist(), "parameters": parameters,
        "output_camera_matrix": K_out.tolist(),
        "output_parameters": output_parameters,
        "output_image_size": list(EXPECTED_SIZE),
        "output_alpha": None if is_fisheye else float(OUTPUT_ALPHA),
        "output_balance": None,  # 固定水平视场模式不使用balance。
        "output_fov_scale": None,
        **output_view_metadata,
        "valid_pixel_roi": None if valid_roi is None else list(valid_roi), "roi_cropped": False,
        "rectification_rotation": np.eye(3).tolist(),
        "calibration_flags": int(flags), "used_view_count": len(accepted),
        "optimizer_criteria": fit["criteria"],
        "initialization_attempts": fit["initialization_attempts"],
        "selected_initial_focal_px": fit.get("initial_focal_px"),
        "rms_px": float(rms), "views": views,
        "training_rms_px": training_rms,
        "training_rms_definition": "sqrt(sum((observed_x-projected_x)^2 + (observed_y-projected_y)^2) / all_detected_point_count); fitted training views, not held-out validation",
        "training_point_count": point_count,
        "all_detected_views_used": True, "fit_excluded_images": [],
        "quality_note": "训练RMS按原始像素报告，不设强制达标阈值；角点检测成功不等于标定质量合格，需独立验证。",
        "reference_map_direction": "output_pixel_to_input_sampling_coordinate",
        "reference_interpolation": "INTER_LINEAR",
        "reference_border_mode": "BORDER_CONSTANT", "reference_border_value": [0, 0, 0],
        "rejected_images": rejected, "warnings": warnings
    })
    fixed = fixed_parameters(parameters)
    # 鱼眼不输出旧Brown文件名，避免旧FPGA自动参数链读取不同数学模型。
    fixed["model"] = model_name
    fixed["opencv_version"] = cv2.__version__
    fixed["dist_coeff_order"] = coeff_order
    fixed["compatible_with_legacy_brown_fpga"] = not is_fisheye
    fixed["input_image_size"] = list(EXPECTED_SIZE)
    fixed["output_image_size"] = list(EXPECTED_SIZE)
    fixed.update(output_view_metadata)
    fixed["output_balance"] = None
    fixed["roi_cropped"] = False
    fixed["parameter_usage"] = (
        "Normalize output pixels with K_out, apply this model's distortion, "
        f"then project with input K; Q{32 - FRAC_BITS}.{FRAC_BITS} parameters alone "
        "do not define intermediate arithmetic.")
    output_fixed = fixed_parameters(output_parameters)
    fixed["output_parameters"] = output_fixed["parameters"]
    fixed["all_parameters_fit"] = fixed["all_parameters_fit"] and output_fixed["all_parameters_fit"]
    save_json(output / fixed_filename, fixed)
    # 字符串和数值数组均可使用allow_pickle=False读取；无ROI用空数组和显式bool表示。
    map_metadata = {
        "K": K, "D": D, "K_out": K_out, "R": np.eye(3),
        "model": np.asarray(model_name), "dist_coeff_order": np.asarray(coeff_order),
        "opencv_version": np.asarray(cv2.__version__),
        "compatible_with_legacy_brown_fpga": np.asarray(not is_fisheye),
        "image_size": np.asarray(EXPECTED_SIZE, dtype=np.int32),
        "output_image_size": np.asarray(EXPECTED_SIZE, dtype=np.int32),
        **{name: np.asarray(value) for name, value in output_view_metadata.items()},
        "valid_pixel_roi": np.asarray([] if valid_roi is None else valid_roi, dtype=np.int32),
        "valid_pixel_roi_available": valid_roi is not None, "roi_cropped": False,
    }
    if not is_fisheye:
        map_metadata["output_alpha"] = OUTPUT_ALPHA
    np.savez_compressed(output / "calibration.npz", **map_metadata,
                        training_rms_px=training_rms, all_detected_views_used=True)
    with (output / "reprojection_errors.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["image", "rms_px"])
        writer.writerows((v["file"], v["rms_px"]) for v in views)

    np.savez_compressed(output / "reference_map.npz", map_x=map_x, map_y=map_y,
                        **map_metadata,
                        map_direction="output_pixel_to_input_sampling_coordinate",
                        map_units="pixels", interpolation="INTER_LINEAR",
                        border_mode="BORDER_CONSTANT", border_value=np.zeros(3, dtype=np.uint8))

    def correct(image):
        return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

    comparisons_dir = output / "comparisons"
    comparisons_dir.mkdir()
    for index, path in enumerate(accepted, 1):
        image = read_image(path)
        if image is None or (image.shape[1], image.shape[0]) != EXPECTED_SIZE:
            raise RuntimeError(f"输出对比图时无法读取原尺寸图片：{path.name}")
        save_png(comparisons_dir / f"{index:03d}_{path.stem}_comparison.png",
                 np.hstack((image, correct(image))))

    validation_output = output / "validation"
    validation_count = 0
    for index, path in enumerate(image_paths(VALIDATION_DIR), 1):
        image = read_image(path)
        if image is None:
            print("跳过验证图片：无法解码", path.name)
            continue
        if (image.shape[1], image.shape[0]) != EXPECTED_SIZE:
            print("跳过验证图片：尺寸不一致", path.name)
            continue
        validation_output.mkdir(exist_ok=True)
        result = correct(image)
        save_png(validation_output / f"{index:03d}_{path.stem}_comparison.png",
                 np.hstack((image, result)))
        validation_count += 1

    print("\n标定计算完成。")
    print("有效标定图片数：", len(accepted))
    print("全部标定对比图：", comparisons_dir)
    print("独立验证图片数：", validation_count)
    print("相机内参矩阵K：\n", K)
    print(f"畸变向量{coeff_order}：", distortion)
    print(f"训练重投影RMS：{training_rms:.6f}像素（全部{len(accepted)}张、{point_count}个角点）")
    print("保留全部检测成功的视图；RMS不设强制达标阈值，此值不是独立验证误差。")
    print("误差最高的5张照片：")
    for view in sorted(views, key=lambda v: v["rms_px"], reverse=True)[:5]:
        print(f"  {view['file']}: {view['rms_px']:.6f} px")
    for warning in warnings:
        print("检查提示：", warning)
    if not fixed["all_parameters_fit"]:
        print(f"定点参数有溢出，请查看{fixed_filename}；浮点标定结果仍已保存。")
    print("定点参数文件：", fixed_filename)
    print("请人工检查角点和独立图片矫正效果，数值输出不等于质量验收通过。")
    print(f"对比图左原图、右矫正图；{fov_parameter}={fov_value}，未按ROI裁剪。")
    print(f"输出视场：水平{horizontal_fov_deg:.3f}°，垂直{vertical_fov_deg:.3f}°。")
    if is_fisheye:
        print("输出K_out按指定水平视场计算，主点居中；balance不参与计算。")
    print(f"原图范围内的采样位置占比：{map_in_bounds_fraction:.6%}；此值不代表矫正精度。")
    print("输出目录：", output)
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except (OSError, ValueError, RuntimeError, cv2.error) as exc:
        print(f"\n标定未完成：{exc}", file=sys.stderr)
        exit_code = 1
    raise SystemExit(exit_code)
