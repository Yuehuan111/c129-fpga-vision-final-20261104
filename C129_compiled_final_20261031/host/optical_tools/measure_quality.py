"""Auditable approximate edge SFR/MTF50 and manually sampled line straightness.

Not an ISO 12233 implementation or certification. Edge measurement requires an
explicit ROI, transfer assumption and data provenance. It does not find chart
edges automatically, infer camera gamma, or claim synthetic charts are captures.
Dependencies: NumPy, Pillow, Matplotlib. OpenCV/SciPy are not required.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


def _linearize(values: np.ndarray, transfer: str, gamma: float = 2.2) -> np.ndarray:
    if transfer == "linear":
        return values
    if transfer == "srgb":
        return np.where(values <= 0.04045, values / 12.92,
                        ((values + 0.055) / 1.055) ** 2.4)
    if transfer == "gamma":
        if not math.isfinite(gamma) or gamma <= 0:
            raise ValueError("gamma must be finite and positive")
        return values ** gamma
    raise ValueError("transfer must be linear, srgb, or gamma")


def load_gray_roi(image_path: Path, roi: list[int], transfer: str,
                  gamma: float = 2.2) -> np.ndarray:
    """Load an explicit half-open x,y,width,height ROI; no resize or auto crop."""
    with Image.open(image_path) as im:
        if im.mode not in ("L", "RGB", "RGBA"):
            raise ValueError("Use an 8-bit grayscale or RGB PNG/TIFF image (L/RGB/RGBA)")
        x, y, width, height = roi
        if min(x, y) < 0 or min(width, height) <= 0:
            raise ValueError("ROI needs nonnegative x/y and positive width/height")
        if x + width > im.width or y + height > im.height:
            raise ValueError("ROI extends outside the image; it will not be silently clipped")
        crop = np.asarray(im.crop((x, y, x + width, y + height)))
        if im.mode == "RGBA":
            if not np.all(crop[..., 3] == 255):
                raise ValueError("Transparent pixels are not valid chart measurements")
            crop = crop[..., :3]
    values = _linearize(crop.astype(np.float64) / 255.0, transfer, gamma)
    if values.ndim == 3:
        # Linear-light Rec.709 luminance weights, applied after the chosen EOTF.
        values = values @ np.array([0.2126, 0.7152, 0.0722])
    return values


def analyze_edge(gray: np.ndarray, axis: str = "vertical", oversample: int = 4):
    """Fit one nearly vertical/horizontal straight edge, bin ESF, FFT its LSF.

    Distances and frequencies are along the fitted edge normal, in source pixels.
    Only the 0..0.5 cycles/pixel band is reported. The central-difference transfer
    is corrected; finite bin width, window bias, chart, ISP and noise are not.
    """
    image = np.asarray(gray, dtype=np.float64)
    if image.ndim != 2 or not np.isfinite(image).all():
        raise ValueError("gray must be a finite 2-D array")
    if image.min() < 0 or image.max() > 1:
        raise ValueError("gray must be normalized to [0,1]")
    if axis not in ("vertical", "horizontal") or oversample not in (4, 8):
        raise ValueError("axis must be vertical/horizontal; oversample must be 4 or 8")
    oriented = image.T.copy() if axis == "horizontal" else image.copy()
    height, width = oriented.shape
    if width < 32 or height < 48:
        raise ValueError("ROI needs >=32 pixels across and >=48 pixels along the edge")
    band = max(3, width // 10)
    left = float(np.median(oriented[:, :band]))
    right = float(np.median(oriented[:, -band:]))
    reverse = right < left
    if reverse:
        oriented = 1.0 - oriented
        left, right = 1.0 - left, 1.0 - right
    contrast = right - left
    if contrast < 0.05:
        raise ValueError("Insufficient edge contrast, wrong axis, or unsuitable ROI")

    row_y, row_x = [], []
    for y, row in enumerate(oriented):
        low, high = np.median(row[:band]), np.median(row[-band:])
        if high - low < contrast * 0.5:
            continue
        # A small filter stabilizes edge POSITION only; the ESF uses raw samples.
        smooth = np.convolve(np.pad(row, (1, 1), mode="edge"), [0.25, 0.5, 0.25], "valid")
        level = (low + high) * 0.5
        crosses = np.flatnonzero((smooth[:-1] <= level) & (smooth[1:] > level))
        if len(crosses) != 1:
            continue
        x = int(crosses[0])
        if x < band or x >= width - band - 1:
            continue
        row_y.append(float(y))
        row_x.append(x + (level - smooth[x]) / (smooth[x + 1] - smooth[x]))
    yy, xx = np.asarray(row_y), np.asarray(row_x)
    if len(xx) < 0.8 * height:
        raise ValueError("At least 80% of rows must contain exactly one isolated edge crossing")

    selected = np.ones(len(xx), dtype=bool)
    for _ in range(3):
        slope, intercept = np.polyfit(yy[selected], xx[selected], 1)
        residual = xx - (slope * yy + intercept)
        center = np.median(residual[selected])
        mad = np.median(np.abs(residual[selected] - center))
        proposed = np.abs(residual - center) <= max(0.12, 4.5 * 1.4826 * mad)
        if proposed.sum() < 0.8 * height:
            raise ValueError("Edge position is inconsistent; inspect noise, curvature, or multiple edges")
        selected = proposed
    slope, intercept = np.polyfit(yy[selected], xx[selected], 1)
    fit_rms = float(np.sqrt(np.mean((xx[selected] - slope * yy[selected] - intercept) ** 2)))
    angle = math.degrees(math.atan(slope))
    if not 1.0 <= abs(angle) <= 15.0:
        raise ValueError("Choose an edge 1..15 degrees from the selected axis; 3..7 is preferable")
    if fit_rms > 0.35:
        raise ValueError("Straight-edge fit RMS exceeds 0.35 pixel; this approximation is unsuitable")

    gy, gx = np.mgrid[:height, :width]
    normalizer = math.sqrt(1.0 + slope * slope)
    distance = (gx - slope * gy - intercept) / normalizer
    edges = slope * np.arange(height) + intercept
    half_width = min(32.0, float(edges.min()) / normalizer,
                     float((width - 1 - edges).min()) / normalizer) - 1.0
    half_bins = int(math.floor(half_width * oversample))
    if half_bins < 8 * oversample:
        raise ValueError("Leave at least 9 pixels of flat background on both sides of the edge")
    n = 2 * half_bins
    half_width = half_bins / oversample
    indices = np.floor((distance + half_width) * oversample).astype(np.int64)
    inside = (indices >= 0) & (indices < n)
    accepted_rows = np.zeros(height, dtype=bool)
    accepted_rows[yy[selected].astype(np.int64)] = True
    inside &= accepted_rows[:, None]
    counts = np.bincount(indices[inside], minlength=n)
    totals = np.bincount(indices[inside], weights=oriented[inside], minlength=n)
    populated = counts > 0
    empty_fraction = float(np.mean(~populated))
    if empty_fraction > 0.05:
        raise ValueError("Subpixel phases leave >5% ESF bins empty; change edge angle or length")
    centers = (np.arange(n) + 0.5) / oversample - half_width
    esf = np.interp(centers, centers[populated], totals[populated] / counts[populated])
    esf = (esf - left) / contrast
    delta = 1.0 / oversample
    lsf = np.gradient(esf, delta, edge_order=2)
    window = np.hanning(n)
    windowed = lsf * window
    fft_size = 1 << int(math.ceil(math.log2(n * 8)))
    spectrum = np.abs(np.fft.rfft(windowed, n=fft_size))
    if spectrum[0] < 1e-12:
        raise ValueError("LSF has negligible DC integral")
    frequencies = np.fft.rfftfreq(fft_size, d=delta)
    band_mask = frequencies <= 0.5
    frequencies = frequencies[band_mask]
    mtf = spectrum[band_mask] / spectrum[0]
    # derivative sin(2*pi*f*delta)/delta versus ideal derivative 2*pi*f
    z = 2.0 * np.pi * frequencies * delta
    correction = np.ones_like(z)
    correction[1:] = z[1:] / np.sin(z[1:])
    mtf *= correction
    below = np.flatnonzero(mtf <= 0.5)
    mtf50 = None
    if below.size:
        k = int(below[0])
        if k > 0:
            mtf50 = float(frequencies[k - 1] + (0.5 - mtf[k - 1]) *
                          (frequencies[k] - frequencies[k - 1]) / (mtf[k] - mtf[k - 1]))
    warnings = ["Approximate edge SFR, not ISO-certified; chart/ISP/noise/window biases remain."]
    if mtf50 is None:
        warnings.append("No 50% crossing at or below sensor Nyquist; MTF50 is not reported.")
    if empty_fraction:
        warnings.append("Some empty ESF bins were linearly interpolated.")
    if float(mtf.max()) > 1.05:
        warnings.append("SFR exceeds 1.05; inspect sharpening, ringing, noise, or ROI suitability.")
    result = {
        "measurement": "approximate_slanted_edge_sfr", "iso_certified": False,
        "axis": axis, "angle_from_axis_degrees": angle, "fit_rms_pixels": fit_rms,
        "rows_used": int(selected.sum()), "rows_total": height,
        "polarity_reversed": bool(reverse), "linear_contrast": contrast,
        "oversample": oversample, "esf_half_width_pixels": half_width,
        "empty_bin_fraction": empty_fraction, "frequency_units": "cycles_per_pixel_along_edge_normal",
        "mtf50_cycles_per_pixel": mtf50,
        "mtf50_status": "measured" if mtf50 is not None else "not_crossed_below_nyquist",
        "mtf_at_nyquist": float(mtf[-1]), "warnings": warnings,
    }
    curves = {"gray": image, "esf_x": centers, "esf": esf, "lsf": lsf,
              "windowed_lsf": windowed, "frequency": frequencies, "mtf": mtf}
    return result, curves


def analyze_line(points: np.ndarray, units: str = "pixels"):
    """Total least squares: orthogonal residuals, valid for vertical lines too."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 3 or not np.isfinite(pts).all():
        raise ValueError("Need at least 3 finite, manually selected x,y points")
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, singular, vh = np.linalg.svd(centered, full_matrices=False)
    if singular[0] < 1e-12:
        raise ValueError("Points must not all coincide")
    if singular[1] >= 0.5 * singular[0]:
        raise ValueError("Points do not describe a reasonably elongated line")
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]])
    residual = centered @ normal
    result = {"measurement": "manual_point_line_straightness", "point_count": len(pts),
              "units": units, "fit": "orthogonal_total_least_squares",
              "centroid": centroid.tolist(), "unit_direction": direction.tolist(),
              "line_abc": [float(normal[0]), float(normal[1]), -float(normal @ centroid)],
              "rms_residual": float(np.sqrt(np.mean(residual ** 2))),
              "max_absolute_residual": float(np.max(np.abs(residual))),
              "residuals": residual.tolist(),
              "note": "Residual to the best fitted line; includes manual picking noise, not absolute camera accuracy."}
    return result


def read_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.reader(handle))
    if rows and [x.strip().lower() for x in rows[0]] == ["x", "y"]:
        rows = rows[1:]
    if any(len(row) != 2 for row in rows):
        raise ValueError("Point CSV must have exactly x,y columns, with an optional x,y header")
    try:
        return np.asarray([[float(x), float(y)] for x, y in rows], dtype=np.float64)
    except ValueError as exc:
        raise ValueError("Every point coordinate must be numeric") from exc


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def save_edge_outputs(prefix: Path, result: dict, curves: dict):
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                                   allow_nan=False), encoding="utf-8")
    np.savetxt(prefix.with_suffix(".csv"), np.column_stack((curves["frequency"], curves["mtf"])),
               delimiter=",", header="cycles_per_pixel_normal,estimated_sfr", comments="")
    plt = _pyplot()
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
    axes[0, 0].imshow(curves["gray"], cmap="gray", vmin=0, vmax=1)
    axes[0, 0].set_title("User-selected linear-light ROI")
    axes[0, 1].plot(curves["esf_x"], curves["esf"])
    axes[0, 1].set(title="Binned edge spread function", xlabel="Normal distance (pixels)")
    axes[1, 0].plot(curves["esf_x"], curves["lsf"], label="Derivative")
    axes[1, 0].plot(curves["esf_x"], curves["windowed_lsf"], label="Hann-windowed")
    axes[1, 0].legend(); axes[1, 0].set(title="Line spread function", xlabel="Normal distance (pixels)")
    axes[1, 1].plot(curves["frequency"], curves["mtf"])
    axes[1, 1].axhline(0.5, color="gray", linestyle="--")
    value = result["mtf50_cycles_per_pixel"]
    if value is not None:
        axes[1, 1].axvline(value, color="red", linestyle="--", label=f"MTF50={value:.4f} cyc/px")
        axes[1, 1].legend()
    axes[1, 1].set(title="Approximate edge SFR (not ISO-certified)",
                   xlabel="Cycles/pixel along edge normal", ylabel="Normalized SFR", xlim=(0, 0.5))
    fig.suptitle(f"Source: {result.get('source', 'unspecified')} | Transfer: {result.get('transfer', 'array input')}")
    fig.savefig(prefix.with_suffix(".png"), dpi=150)
    plt.close(fig)


def save_line_outputs(prefix: Path, result: dict, points: np.ndarray):
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                                   allow_nan=False), encoding="utf-8")
    np.savetxt(prefix.with_suffix(".csv"), np.column_stack((points, result["residuals"])),
               delimiter=",", header="x,y,signed_normal_residual", comments="")
    plt = _pyplot()
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    center, direction = np.asarray(result["centroid"]), np.asarray(result["unit_direction"])
    along = (points - center) @ direction
    fit = center + np.array([along.min(), along.max()])[:, None] * direction
    axes[0].scatter(points[:, 0], points[:, 1], s=12)
    axes[0].plot(fit[:, 0], fit[:, 1], color="red")
    axes[0].axis("equal"); axes[0].set(title="Manual points and TLS line", xlabel="x", ylabel="y")
    axes[1].plot(along, result["residuals"], ".")
    axes[1].axhline(0, color="gray")
    axes[1].set(title=f"RMS={result['rms_residual']:.6g}; max={result['max_absolute_residual']:.6g}",
                xlabel=f"Along fitted line ({result['units']})", ylabel=f"Normal residual ({result['units']})")
    fig.savefig(prefix.with_suffix(".png"), dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    edge = sub.add_parser("edge", help="Approximate SFR of one explicitly selected chart edge")
    edge.add_argument("--image", type=Path, required=True)
    edge.add_argument("--roi", nargs=4, type=int, required=True, metavar=("X", "Y", "WIDTH", "HEIGHT"))
    edge.add_argument("--axis", choices=("vertical", "horizontal"), required=True)
    edge.add_argument("--transfer", choices=("linear", "srgb", "gamma"), required=True)
    edge.add_argument("--gamma", type=float, default=2.2)
    edge.add_argument("--source", choices=("real-chart", "synthetic-test"), required=True)
    edge.add_argument("--oversample", type=int, choices=(4, 8), default=4)
    edge.add_argument("--out", type=Path, required=True, help="Output prefix, without extension")
    line = sub.add_parser("line", help="Orthogonal straightness residuals of manually selected points")
    line.add_argument("--points", type=Path, required=True)
    line.add_argument("--units", default="pixels")
    line.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.suffix:
        raise ValueError("--out is a filename prefix without an extension")
    if args.command == "edge":
        gray = load_gray_roi(args.image, args.roi, args.transfer, args.gamma)
        result, curves = analyze_edge(gray, args.axis, args.oversample)
        result.update(image=str(args.image.resolve()), roi=args.roi, source=args.source,
                      transfer=args.transfer, gamma=args.gamma if args.transfer == "gamma" else None)
        save_edge_outputs(args.out, result, curves)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        points = read_points(args.points)
        result = analyze_line(points, args.units)
        result["points_file"] = str(args.points.resolve())
        save_line_outputs(args.out, result, points)
        print(json.dumps({k: v for k, v in result.items() if k != "residuals"},
                         ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as exc:
        raise SystemExit(f"Quality measurement rejected: {exc}")
