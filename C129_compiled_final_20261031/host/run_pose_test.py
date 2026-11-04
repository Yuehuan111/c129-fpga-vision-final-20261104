"""Explicit one-shot pose compensation with a verified temporary installation.

Requires --confirm-temporary-mount after the operator checks all three physical
motion directions. Files stay unconfirmed; final visual validation is pending.
No rate, sensor-zero, enhancement, or other auxiliary commands are sent.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from imu_pose import normalize_quaternion, validate_mounting_rotation
from imu_session import HostSession
from mounting_calibration import LABELS, WINDOW_PROFILES, analyze_window, fit_mounting
from run_link_test import run_single_submission


def _read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _endpoint(directory):
    summary, metadata = _read(directory / "summary.json"), _read(directory / "metadata.json")
    if directory.name not in LABELS or summary["label"] != directory.name or metadata["label"] != directory.name:
        raise ValueError("Reference endpoint label mismatch")
    if (summary["quality"]["passed"] is not True or summary["capture_complete"] is not True
            or summary.get("error") or not summary["port"] or summary["port"] != metadata["port"]):
        raise ValueError("Endpoint did not pass capture validation")
    fields = [metadata.get("profile"), summary.get("profile"), summary["quality"].get("profile")]
    if all(value is None for value in fields):
        profile = "static"
    elif len(set(fields)) == 1 and fields[0] in WINDOW_PROFILES:
        profile = fields[0]
    else:
        raise ValueError("Endpoint profile mismatch")
    for name in ("metadata.json", "samples.jsonl", "raw.bin"):
        if _sha(directory / name) != summary["sha256"][name]:
            raise ValueError("Endpoint hash mismatch: " + name)
    samples = [json.loads(line) for line in (directory / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
    quality = analyze_window(samples, profile=profile)
    if not quality["passed"]:
        raise ValueError("Saved endpoint no longer passes window validation")
    saved_q = normalize_quaternion(summary["quality"]["mean_quaternion_wxyz"])
    if not np.isclose(abs(np.dot(saved_q, quality["mean_quaternion_wxyz"])), 1., atol=1e-12, rtol=0):
        raise ValueError("Saved endpoint mean does not match its samples")
    return dict(directory=str(directory), label=directory.name, metadata=metadata,
                summary_sha256=_sha(directory / "summary.json"), files_sha256=summary["sha256"],
                quality=quality, port=summary["port"])


def load_pose_inputs(mounting, reference_endpoint, *, confirm_temporary_mount=False):
    """Validate all inputs before any port is opened; never modifies a capture."""
    if confirm_temporary_mount is not True:
        raise ValueError("Explicit --confirm-temporary-mount is required after physical direction checks")
    mounting_path, reference_path = Path(mounting).resolve(), Path(reference_endpoint).resolve()
    mounting_data = _read(mounting_path)
    validate_mounting_rotation(mounting_data["R_IC"])
    if mounting_data.get("convention") not in ("imu_to_world", "world_to_imu"):
        raise ValueError("Invalid quaternion convention")
    if mounting_data.get("confirmed") is not False:
        raise ValueError("Expected unchanged temporary mounting file with confirmed=false")
    report_path = mounting_path.parent / "quality_report.json"
    report = _read(report_path)
    if report.get("passed") is not True or report.get("confirmed") is not False:
        raise ValueError("Mounting quality report did not pass")
    if report.get("mounting") != mounting_data:
        raise ValueError("Mounting file does not match its quality report")
    session_directory = mounting_path.parent.parent
    if reference_path != session_directory / reference_path.name or reference_path.name not in LABELS:
        raise ValueError("Reference must be an endpoint from this mounting fit session")
    endpoints, sources, port, last_time = {}, {}, None, None
    for label in LABELS:
        source = _endpoint(session_directory / label)
        recorded = report["sources"][label]
        if (source["summary_sha256"] != recorded["summary_sha256"]
                or source["files_sha256"] != recorded["files_sha256"]):
            raise ValueError("Endpoint changed since mounting fit: " + label)
        if port is not None and source["port"] != port:
            raise ValueError("Mounting endpoints came from different IMU ports")
        quality = source["quality"]
        if last_time is not None and quality["first_received_at"] <= last_time:
            raise ValueError("Mounting endpoint order mismatch")
        port, last_time = source["port"], quality["last_received_at"]
        endpoints[label], sources[label] = quality, source
    checked_fit = fit_mounting(endpoints, min_angle_deg=min(5., report["limits"]["min_angle_deg"]))
    if (not checked_fit["passed"] or checked_fit["mounting"]["convention"] != mounting_data["convention"]
            or not np.allclose(checked_fit["mounting"]["R_IC"], mounting_data["R_IC"], atol=1e-10, rtol=0)):
        raise ValueError("Mounting fit cannot be reproduced from saved endpoints")
    reference = sources[reference_path.name]
    return dict(mounting=mounting_data, mounting_path=str(mounting_path),
                mounting_sha256=_sha(mounting_path), quality_report_path=str(report_path),
                quality_report_sha256=_sha(report_path), operator_confirmed_temporary_mount=True,
                reference_quaternion=reference["quality"]["mean_quaternion_wxyz"],
                reference_provenance=reference,
                meaning="Temporary coarse installation for a single test; visual validation pending")


def run_pose_test(imu_port, fpga_port, out, *, mounting, reference_endpoint,
                  confirm_temporary_mount=False, session_factory=HostSession,
                  ready_timeout_s=8., total_timeout_s=60.):
    def preflight():
        inputs = load_pose_inputs(mounting, reference_endpoint,
                                  confirm_temporary_mount=confirm_temporary_mount)
        if inputs["reference_provenance"]["port"] != imu_port:
            raise ValueError("Selected IMU port differs from the saved installation capture")
        return inputs

    def prepare(session, inputs):
        session.set_mounting(inputs["mounting"]["R_IC"],
                             convention=inputs["mounting"]["convention"], confirmed=True)
        session.restore_reference(inputs["reference_quaternion"], inputs["reference_provenance"])
        session._log("temporary_mount_operator_confirmation", inputs=inputs,
                     visual_validation="pending", source_file_confirmed=False)

    return run_single_submission(imu_port, fpga_port, out, session_factory=session_factory,
        ready_timeout_s=ready_timeout_s, total_timeout_s=total_timeout_s,
        test_kind="temporary_mount_pose_test", preflight=preflight, prepare=prepare)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mounting", required=True)
    parser.add_argument("--reference-endpoint", required=True)
    parser.add_argument("--imu-port", required=True)
    parser.add_argument("--fpga-port", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--confirm-temporary-mount", action="store_true")
    args = parser.parse_args()
    result = run_pose_test(args.imu_port, args.fpga_port, args.out, mounting=args.mounting,
                          reference_endpoint=args.reference_endpoint,
                          confirm_temporary_mount=args.confirm_temporary_mount)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result["result"] == "output_confirmed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
