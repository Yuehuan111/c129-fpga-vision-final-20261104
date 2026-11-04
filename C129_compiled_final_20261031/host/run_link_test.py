"""Explicit single H=I transport diagnostic. Never changes IMU rate or mounting.

Example: python run_link_test.py --imu-port COM7 --fpga-port COM8 --out logs/link.json
The sibling .jsonl contains stream observations and every request transition.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from imu_session import HostSession


def run_link_test(imu_port, fpga_port, out, *, session_factory=HostSession,
                  ready_timeout_s=8., total_timeout_s=60.):
    return run_single_submission(imu_port, fpga_port, out, session_factory=session_factory,
                                 ready_timeout_s=ready_timeout_s, total_timeout_s=total_timeout_s)


def run_single_submission(imu_port, fpga_port, out, *, session_factory=HostSession,
                          ready_timeout_s=8., total_timeout_s=60.,
                          test_kind="identity_transport_test", preflight=None, prepare=None):
    """Wait for complete hardware readiness, submit once, observe until deadline.

    Timeout overrides exist for fake-port tests; callers cannot extend the
    eight-second readiness or sixty-second total observation limits.
    """
    if not 0 < ready_timeout_s <= 8 or not 0 < total_timeout_s <= 60:
        raise ValueError("Readiness must be <=8 seconds and total wait <=60 seconds")
    output = Path(out).resolve()
    if output.suffix.lower() != ".json":
        output = output / "result.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    log_path = output.with_suffix(".jsonl")
    # Avoid appending an independent hardware attempt to an earlier report.
    if output.exists() or log_path.exists():
        raise FileExistsError(f"Use a new output path: {output}")
    started = time.monotonic()
    deadline = started + total_timeout_s
    if test_kind not in ("identity_transport_test", "temporary_mount_pose_test"):
        raise ValueError("Unknown single-submission test kind")
    identity = test_kind == "identity_transport_test"
    prefix = "identity_test" if identity else "pose_test"
    result = dict(test=test_kind, imu_port=imu_port, fpga_port=fpga_port,
                  result="not_sent", before=None, after=None, request=None,
                  json_path=str(output), jsonl_path=str(log_path))
    session = None
    try:
        inputs = preflight() if preflight else None
        if inputs is not None:
            result["inputs"] = inputs
        if not identity:
            result["visual_validation"] = "pending"
            if prepare is None:
                raise ValueError("Pose test requires explicit installation/reference preparation")
        session = session_factory(log_path=log_path)
        session.start(imu_port=imu_port, fpga_port=fpga_port)
        ready_deadline = min(deadline, started + ready_timeout_s)
        prepared = identity
        while True:
            snapshot = session.snapshot()
            blocked_key = "identity_test_blocked_reason" if identity else "send_blocked_reason"
            if snapshot["frozen"] or time.monotonic() >= ready_deadline:
                result["reason"] = snapshot[blocked_key] or "Readiness deadline reached"
                result["before"] = snapshot
                return result
            if not prepared and snapshot["imu"]["connected"] and not snapshot["imu"]["stale"]:
                prepare(session, inputs)
                prepared = True
                snapshot = session.snapshot()
            if prepared and snapshot["can_send_identity_test" if identity else "can_send"]:
                break
            time.sleep(min(.025, max(0., ready_deadline - time.monotonic())))
        result["before"] = snapshot
        session._log(prefix + "_before", snapshot=snapshot)
        result["request_id"] = session.send_identity_test() if identity else session.send_once()
        result["result"] = "pending"
        while True:
            snapshot = session.snapshot()
            request = snapshot["request"] or snapshot["candidate"]
            if request and request.get("request_id") == result["request_id"]:
                result["request"] = request
                if request["phase"] == "output_confirmed":
                    result["result"] = "output_confirmed"
                    break
                if request["phase"] in ("geometry_failed", "candidate_discarded", "quantized_unchanged"):
                    result.update(result="not_sent", reason=request.get("reason") or request["phase"])
                    break
                if request.get("invalidated"):
                    result.update(result="outcome_unknown", reason=snapshot["error"])
                    break
            # Publication/output/acceptance timeout is never permission to retry.
            # Keep the readers alive: late status and LAT1 can still confirm it.
            if time.monotonic() >= deadline:
                written = request and "write_started_at" in request
                result.update(result="outcome_unknown" if written else "not_sent",
                              reason=snapshot["error"] or "Total observation deadline reached")
                break
            time.sleep(min(.025, max(0., deadline - time.monotonic())))
        return result
    except Exception as exc:
        result.update(result="error", reason=f"{type(exc).__name__}: {exc}")
        return result
    finally:
        if session is not None:
            result["after"] = session.snapshot()
            result["request"] = result["after"]["request"] or result["after"]["candidate"]
            session._log(prefix + "_after", result=result["result"],
                         snapshot=result["after"], reason=result.get("reason"))
            try:
                session.close()
            except Exception as exc:
                result["close_error"] = f"{type(exc).__name__}: {exc}"
                result["result"] = "error"
        result["elapsed_s"] = time.monotonic() - started
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                          encoding="utf-8")
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(event=prefix + "_result", result=result),
                                    ensure_ascii=False, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-port", required=True)
    parser.add_argument("--fpga-port", required=True)
    parser.add_argument("--out", required=True, help="New .json path or new output directory")
    args = parser.parse_args()
    result = run_link_test(args.imu_port, args.fpga_port, args.out)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result["result"] == "output_confirmed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
