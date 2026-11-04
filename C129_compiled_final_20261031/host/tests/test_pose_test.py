"""Saved-reference and one-shot runner checks; all serial ports are fakes."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_session import HostSession
from mounting_calibration import LABELS, fit_session
from run_pose_test import load_pose_inputs, run_pose_test
import test_mounting_calibration as mounting_tests
from test_imu_session import FakePort, adapters, imu_frame, status_frame, latency_frame


class PoseTestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        data = mounting_tests.endpoints(np.eye(3), angle_deg=20.)
        maker = mounting_tests.CaptureSessionTests()
        for label in LABELS:
            maker.fake_capture(self.root, label, data[label]["mean_quaternion_wxyz"], handheld=True)
        report = fit_session(self.root)
        self.assertTrue(report["passed"])
        self.mounting = Path(report["output_directory"]) / "mounting.temporary.json"
        self.reference = self.root / "x_start"

    def tearDown(self):
        self.temp.cleanup()

    def test_preflight_recomputes_reference_and_rejects_changed_sources(self):
        inputs = load_pose_inputs(self.mounting, self.reference, confirm_temporary_mount=True)
        self.assertFalse(inputs["mounting"]["confirmed"])
        self.assertEqual(inputs["reference_provenance"]["label"], "x_start")
        self.assertTrue(inputs["reference_provenance"]["quality"]["passed"])
        path = self.root / "y_end" / "samples.jsonl"
        path.write_text(path.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            load_pose_inputs(self.mounting, self.reference, confirm_temporary_mount=True)

    def test_missing_confirmation_fails_before_open_and_saves_failure(self):
        def never_open(**kwargs):
            self.fail("Preflight failure must not open a serial session")
        output = self.root / "rejected.json"
        result = run_pose_test("COM_FAKE", "FPGA", output, mounting=self.mounting,
                               reference_endpoint=self.reference, session_factory=never_open)
        self.assertEqual(result["result"], "error")
        self.assertIn("confirm-temporary-mount", result["reason"])
        self.assertTrue(output.exists())
        self.assertTrue(output.with_suffix(".jsonl").exists())

    def test_one_normal_pose_transaction_from_saved_reference_and_late_output(self):
        sessions, threads = [], []
        stop = threading.Event()
        original_mounting = self.mounting.read_bytes()
        def ports():
            result = adapters()
            result[0].device = "COM_FAKE"
            return result
        def factory(**kwargs):
            session = HostSession(**kwargs, serial_factory=FakePort, ports_provider=ports,
                                  geometry_checker=lambda words: {"fixed_point_checked": True},
                                  output_timeout_s=.03)
            sessions.append(session)
            def feed():
                initial, published, due = False, False, None
                while not stop.wait(.005):
                    if not all(session._connected.values()):
                        continue
                    session._ports["imu"].incoming.put(imu_frame((.9961947, 0., .0871557, 0.)))
                    if not initial:
                        session._ports["fpga"].incoming.put(status_frame())
                        initial = True
                    if session._ports["fpga"].writes and not published:
                        session._ports["fpga"].incoming.put(status_frame(ticks=101, version=4, commits=8))
                        published, due = True, time.monotonic() + .1
                    if due and time.monotonic() >= due:
                        session._ports["fpga"].incoming.put(latency_frame())
                        due = None
            thread = threading.Thread(target=feed)
            threads.append(thread)
            thread.start()
            return session
        try:
            result = run_pose_test("COM_FAKE", "FPGA", self.root / "pose.json", mounting=self.mounting,
                reference_endpoint=self.reference, confirm_temporary_mount=True,
                session_factory=factory, ready_timeout_s=2., total_timeout_s=3.)
        finally:
            stop.set()
            for thread in threads:
                thread.join(1.)
        self.assertEqual(result["result"], "output_confirmed", result.get("reason"))
        request = result["request"]
        self.assertEqual(request["request_kind"], "pose_compensation")
        self.assertFalse(np.allclose(request["H"], np.eye(3)))
        self.assertTrue(request["output_timeout"])
        self.assertEqual(result["visual_validation"], "pending")
        self.assertIsNone(request["reference"]["sample"]["received_at"])
        self.assertFalse(result["after"]["identity_test_requested"])
        self.assertEqual(self.mounting.read_bytes(), original_mounting)
        self.assertEqual([len(x) for x in sessions[0]._ports["fpga"].writes], [232])
        self.assertEqual(sessions[0]._ports["imu"].writes, [])
        self.assertTrue(all(not p.is_open for p in sessions[0]._ports.values()))


if __name__ == "__main__":
    unittest.main()
