"""Action sequencing against the real C127 ledger, without devices or threads.

The injected serial factory and port provider fail if ever used. Board events
below are deliberately simulated ledger transitions, not hardware evidence.
"""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))

from c127_session import C127HostSession
from pose_protocol import VIEW_BUILD_ID
from unified_actions import ActionSequencer


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class ActionSequencerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="_test_actions_", dir=HOST)
        self.addCleanup(self.temp.cleanup)
        self.clock = FakeClock()
        self.serial_factory = Mock(side_effect=AssertionError("No device access permitted"))
        self.ports_provider = Mock(side_effect=AssertionError("No port enumeration permitted"))
        self.sessions = []
        self.session = self.make_session()
        self.current_session = self.session
        self.messages = []
        self.actions = ActionSequencer(
            lambda: self.current_session, self.messages.append,
            clock=self.clock, timeout=25.0)

    def make_session(self):
        session = C127HostSession(
            log_path=Path(self.temp.name) / f"session_{len(self.sessions)}.jsonl",
            serial_factory=self.serial_factory, ports_provider=self.ports_provider,
            clock=self.clock)
        self.sessions.append(session)
        self.addCleanup(session.close)
        return session

    def tearDown(self):
        self.serial_factory.assert_not_called()
        self.ports_provider.assert_not_called()
        for session in self.sessions:
            self.assertEqual(session._ports, {})
            self.assertEqual(session._threads, [])

    def ticks(self, count=5):
        for _ in range(count):
            self.actions.tick()

    def test_construction_neither_starts_session_nor_stops_it(self):
        self.assertIsNone(self.actions.pending)
        self.assertFalse(self.session._started)
        self.assertEqual(self.session._epoch, 0)
        self.assertEqual(self.messages, [])

    def test_begin_without_session_is_rejected_without_pending_work(self):
        self.current_session = None
        with self.assertRaisesRegex(RuntimeError, "连接 FPGA"):
            self.actions.begin("test", [Mock()])
        self.assertIsNone(self.actions.pending)

    def test_frozen_session_is_rejected_without_an_extra_stop(self):
        self.session._frozen = True
        self.session._error = "simulated fault"
        with patch.object(self.session, "stop", wraps=self.session.stop) as stop:
            with self.assertRaisesRegex(RuntimeError, "simulated fault"):
                self.actions.begin("test", [Mock()])
            stop.assert_not_called()
        self.assertIsNone(self.actions.pending)

    def test_begin_and_step_hold_real_reentrant_session_lock(self):
        old_snapshot = self.session.snapshot
        old_stop = self.session.stop
        observed = []

        def snapshot():
            self.assertTrue(self.session._lock._is_owned())
            return old_snapshot()

        def stop():
            self.assertTrue(self.session._lock._is_owned())
            return old_stop()

        def step(session):
            self.assertTrue(session._lock._is_owned())
            observed.append(session)

        with patch.object(self.session, "snapshot", side_effect=snapshot), \
                patch.object(self.session, "stop", side_effect=stop):
            self.actions.begin("locked", [step])
            self.actions.tick()
        self.assertEqual(observed, [self.session])

    def test_stop_called_once_while_waiting_for_atomic_transaction(self):
        active = {"request_id": 17, "phase": "writing"}
        pending = [{"request_id": 16, "phase": "published"}]
        self.session._continuous = True
        self.session._candidate = {"request_id": 18}
        self.session._active = active
        self.session._pending_outputs = pending
        step = Mock()
        with patch.object(self.session, "stop", wraps=self.session.stop) as stop:
            self.actions.begin("wait", [step])
            self.ticks(10)
            stop.assert_called_once_with()
            self.assertFalse(self.session._continuous)
            self.assertIsNone(self.session._candidate)
            self.assertIs(self.session._active, active)
            self.assertIs(self.session._pending_outputs, pending)
            step.assert_not_called()
            self.session._active = None
            self.ticks()
            step.assert_not_called()
            self.session._pending_outputs = []
            self.ticks()
            stop.assert_called_once_with()
            step.assert_called_once_with(self.session)

    def test_every_existing_ledger_busy_marker_blocks_a_step(self):
        markers = {
            "_active": {"request_id": 1},
            "_candidate": {"request_id": 2},
            "_pending_outputs": [{"request_id": 3}],
            "_view_command": {"kind": "mode", "phase": "queued"},
            "_view_dirty": True,
        }
        for name, value in markers.items():
            with self.subTest(marker=name):
                step = Mock()
                self.actions.begin(name, [step])
                with self.session._lock:
                    setattr(self.session, name, value)
                self.ticks()
                step.assert_not_called()
                self.assertIsNotNone(self.actions.pending)
                with self.session._lock:
                    setattr(self.session, name, [] if name == "_pending_outputs" else
                            False if name == "_view_dirty" else None)
                self.ticks()
                step.assert_called_once_with(self.session)
                self.assertIsNone(self.actions.pending)

    def test_second_begin_does_not_replace_an_existing_sequence(self):
        first, second = Mock(), Mock()
        self.actions.begin("first", [first])
        original = self.actions.pending
        with patch.object(self.session, "stop", wraps=self.session.stop) as stop:
            with self.assertRaisesRegex(RuntimeError, "上一项操作"):
                self.actions.begin("second", [second])
            stop.assert_not_called()
        self.assertIs(self.actions.pending, original)
        self.ticks()
        first.assert_called_once()
        second.assert_not_called()

    def test_session_replacement_or_disconnect_cancels_without_stopping_new_owner(self):
        for replacement in (None, self.make_session()):
            with self.subTest(replacement=replacement is not None):
                self.current_session = self.session
                step = Mock()
                self.actions.begin("old owner", [step])
                self.current_session = replacement
                replacement_epoch = replacement._epoch if replacement else None
                self.ticks()
                self.assertIsNone(self.actions.pending)
                step.assert_not_called()
                if replacement:
                    self.assertEqual(replacement._epoch, replacement_epoch)
                self.assertIn("连接已改变", self.messages[-1])

    def test_external_stop_epoch_cancels_old_sequence(self):
        step = Mock()
        self.actions.begin("old request", [step])
        view_epoch = self.session._view_epoch
        self.session.stop()
        self.assertEqual(self.session._view_epoch, view_epoch)
        self.ticks()
        step.assert_not_called()
        self.assertIsNone(self.actions.pending)
        self.assertIn("取消旧操作", self.messages[-1])

    def test_view_epoch_change_cancels_old_sequence(self):
        step = Mock()
        self.actions.begin("old context", [step])
        with self.session._lock:
            self.session._invalidate_view()
        self.ticks()
        step.assert_not_called()
        self.assertIsNone(self.actions.pending)
        self.assertIn("上下文已改变", self.messages[-1])

    def test_freeze_close_or_shutdown_cancels_without_submission(self):
        for field in ("_frozen", "_closed", "_shutdown"):
            with self.subTest(field=field):
                step = Mock()
                self.actions.begin(field, [step])
                if field == "_shutdown":
                    self.session._shutdown.set()
                else:
                    setattr(self.session, field, True)
                self.ticks()
                step.assert_not_called()
                self.assertIsNone(self.actions.pending)
                self.assertIn("保护停止", self.messages[-1])
                if field == "_shutdown":
                    self.session._shutdown.clear()
                else:
                    setattr(self.session, field, False)

    def test_timeout_at_deadline_cancels_and_never_retries(self):
        step = Mock()
        self.actions.begin("timeout", [step])
        self.session._active = {"request_id": 1}
        self.clock.now += 24.99
        self.actions.tick()
        self.assertIsNotNone(self.actions.pending)
        self.clock.now += .01
        self.actions.tick()
        self.assertIsNone(self.actions.pending)
        self.session._active = None
        self.ticks()
        step.assert_not_called()
        self.assertIn("超时", self.messages[-1])

    def test_manual_cancel_never_resumes_or_submits(self):
        step = Mock()
        self.session._continuous = True
        self.actions.begin("manual", [step])
        self.actions.cancel()
        self.ticks()
        self.assertFalse(self.session._continuous)
        self.assertIsNone(self.actions.pending)
        step.assert_not_called()

    def test_two_steps_do_not_repeat_while_first_view_command_waits(self):
        second = Mock()
        with patch.object(self.session, "_view_gate", return_value={}), \
                patch.object(self.session, "request_view_mode",
                             wraps=self.session.request_view_mode) as first:
            self.actions.begin("two steps", [
                lambda session: session.request_view_mode(scan=False, mirror=False), second])
            self.actions.tick()
            first.assert_called_once_with(scan=False, mirror=False)
            self.assertEqual(self.session._view_command["kind"], "mode")
            self.ticks(20)
            first.assert_called_once()
            second.assert_not_called()
            # Simulate only completion of the first command ledger.
            with self.session._lock:
                self.session._view_command = None
            self.ticks(20)
            first.assert_called_once()
            second.assert_called_once_with(self.session)
        self.assertIsNone(self.actions.pending)
        self.assertIn("等待板端确认", self.messages[-1])

    def test_own_context_change_is_accepted_then_waits_for_invalidation(self):
        self.session._pose_mode = "normal"
        self.session._pose_status = {"build_id": VIEW_BUILD_ID, "matrix_valid": True}
        submitted = Mock()
        self.actions.begin("restore roll", [
            lambda session: session.set_pose_mode("roll_only"), submitted])
        self.actions.tick()
        self.assertEqual(self.session._pose_mode, "roll_only")
        self.assertTrue(self.session._view_dirty)
        self.assertEqual(self.session._view_command["kind"], "invalidate")
        self.assertEqual(self.actions.pending["epoch"],
                         (self.session._epoch, self.session._view_epoch))
        self.ticks(10)
        submitted.assert_not_called()
        with self.session._lock:
            self.session._view_command = None
            self.session._view_dirty = False
        self.ticks(10)
        submitted.assert_called_once_with(self.session)

    def test_first_step_exception_cancels_remaining_steps_without_retry(self):
        first = Mock(side_effect=RuntimeError("simulated refusal"))
        second = Mock()
        self.actions.begin("refused", [first, second])
        self.ticks(10)
        first.assert_called_once_with(self.session)
        second.assert_not_called()
        self.assertIsNone(self.actions.pending)
        self.assertIn("simulated refusal", self.messages[-1])


if __name__ == "__main__":
    unittest.main()
