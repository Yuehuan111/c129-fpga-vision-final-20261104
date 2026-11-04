"""Fake captures plus a generated local clip; never open physical video devices."""
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unified_video import CaptureController, VideoPanel, VideoSource, _capture_worker, _put_latest


class FakeCapture:
    def __init__(self, *args):
        self.items = [np.full((12, 20, 3), n, np.uint8) for n in range(3)]
        self.released = False

    def isOpened(self):
        return True

    def get(self, prop):
        return 240.0

    def read(self):
        return (True, self.items.pop(0)) if self.items else (False, None)

    def release(self):
        self.released = True


class FakeProcess:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.alive = False
        self.exitcode = None
        self.terminated = False
        self.closed = False

    def start(self):
        self.alive = True

    def join(self, timeout=None):
        pass  # Simulates a blocked camera backend.

    def is_alive(self):
        return self.alive

    def terminate(self):
        self.terminated = True
        self.alive = False
        self.exitcode = -15

    def close(self):
        self.closed = True


class FakeContext:
    def __init__(self):
        self.processes = []

    def Queue(self, maxsize):
        return queue.Queue(maxsize=maxsize)

    def Event(self):
        return threading.Event()

    def Process(self, **kwargs):
        process = FakeProcess(**kwargs)
        self.processes.append(process)
        return process


class VideoWorkerTests(unittest.TestCase):
    def test_native_size_request_never_resizes_or_accepts_driver_size_as_proof(self):
        capture = FakeCapture()
        capture.set = Mock(return_value=True)
        stop = threading.Event()
        wrong_size = np.zeros((12, 20, 3), np.uint8)
        reads = iter([(True, wrong_size)])
        def read():
            try: return next(reads)
            except StopIteration:
                stop.set()
                return False, None
        capture.read = read
        frames, events = queue.Queue(maxsize=1), queue.Queue(maxsize=8)
        _capture_worker(VideoSource('camera', 1, requested_size=(1280, 720)), 1,
                        frames, events, stop, lambda *_: capture)
        self.assertEqual(capture.set.call_args_list[0].args, (cv2.CAP_PROP_FRAME_WIDTH, 1280))
        self.assertEqual(capture.set.call_args_list[1].args, (cv2.CAP_PROP_FRAME_HEIGHT, 720))
        self.assertEqual(frames.get_nowait()['frame'].shape, (12, 20, 3))
        self.assertIn('format_warning', [events.get_nowait()['kind'] for _ in range(events.qsize())])

    def test_camera_transient_read_failure_recovers_without_reopening(self):
        capture = FakeCapture()
        stop = threading.Event()
        frame = np.zeros((12, 20, 3), np.uint8)
        reads = iter([(False, None), (True, frame), (False, None), (True, frame)])
        def read():
            try:
                return next(reads)
            except StopIteration:
                stop.set()
                return False, None
        capture.read = read
        factory = Mock(return_value=capture)
        frames, events = queue.Queue(maxsize=1), queue.Queue(maxsize=8)
        _capture_worker(VideoSource('camera', 1, 'DSHOW'), 1, frames, events, stop, factory)
        factory.assert_called_once()
        self.assertTrue(capture.released)
        self.assertEqual(frames.get_nowait()['sequence'], 2)
        kinds = [events.get_nowait()['kind'] for _ in range(events.qsize())]
        self.assertEqual(kinds, ['opened', 'read_retry', 'read_recovered', 'read_retry', 'read_recovered'])

    def test_persistent_camera_read_failure_is_bounded_and_not_called_serial_disconnect(self):
        capture = FakeCapture()
        capture.read = Mock(return_value=(False, None))
        stop = threading.Event()
        frames, events = queue.Queue(maxsize=1), queue.Queue(maxsize=8)
        _capture_worker(VideoSource('camera', 1), 1, frames, events, stop, lambda *_: capture)
        self.assertEqual(capture.read.call_count, 10)
        self.assertTrue(capture.released)
        messages = [events.get_nowait() for _ in range(events.qsize())]
        self.assertEqual(messages[-1]['kind'], 'error')
        self.assertIn('这不表示 FPGA／IMU 串口断开', messages[-1]['message'])

    def test_manual_stop_interrupts_camera_read_retry(self):
        capture = FakeCapture()
        stop = threading.Event()
        frames, events = queue.Queue(maxsize=1), queue.Queue(maxsize=8)
        def read():
            stop.set()
            return False, None
        capture.read = Mock(side_effect=read)
        _capture_worker(VideoSource('camera', 1), 1, frames, events, stop, lambda *_: capture)
        self.assertEqual(capture.read.call_count, 1)
        self.assertTrue(capture.released)
        self.assertEqual([events.get_nowait()['kind'] for _ in range(events.qsize())], ['opened'])

    def test_constructor_does_not_spawn_or_open_capture(self):
        context = FakeContext()
        with patch.object(cv2, "VideoCapture") as capture:
            controller = CaptureController(context=context)
            self.assertEqual(context.processes, [])
            self.assertIsNone(controller.process)
            self.assertEqual(controller.poll(), (None, []))
            capture.assert_not_called()

    def test_source_validation_rejects_implicit_urls_and_bad_camera_indices(self):
        for value in [-1, True, "0", 256]:
            with self.assertRaises(ValueError):
                VideoSource("camera", value)
        with self.assertRaises(ValueError):
            VideoSource("network", "example")

    def test_latest_queue_drops_old_frame(self):
        channel = queue.Queue(maxsize=1)
        for n in range(10):
            _put_latest(channel, n)
        self.assertEqual(channel.qsize(), 1)
        self.assertEqual(channel.get_nowait(), 9)

    def test_fake_file_worker_releases_and_reports_end(self):
        capture = FakeCapture()
        frames, events = queue.Queue(maxsize=1), queue.Queue(maxsize=8)
        with patch.object(cv2, "VideoCapture") as real_capture:
            _capture_worker(VideoSource("file", "fake.avi"), 7, frames, events,
                            threading.Event(), capture_factory=lambda *_: capture)
            real_capture.assert_not_called()
        self.assertTrue(capture.released)
        packet = frames.get_nowait()
        self.assertEqual(packet["generation"], 7)
        self.assertEqual(packet["sequence"], 3)
        self.assertTrue(np.all(packet["frame"] == 2))
        kinds = [events.get_nowait()["kind"] for _ in range(events.qsize())]
        self.assertEqual(kinds, ["opened", "ended"])

    def test_switch_terminates_only_owned_worker_and_discards_old_generation(self):
        context = FakeContext()
        controller = CaptureController(context=context)
        controller.start(VideoSource("file", "first.avi"))
        first, old_frames, old_generation = controller.process, controller.frames, controller.generation
        old_frames.put({"generation": old_generation, "frame": np.ones((2, 2, 3), np.uint8)})
        controller.start(VideoSource("file", "second.avi"))
        self.assertTrue(first.terminated)
        self.assertTrue(first.closed)
        self.assertGreater(controller.generation, old_generation)
        self.assertIsNot(controller.frames, old_frames)
        controller.frames.put({"generation": old_generation, "frame": np.ones((2, 2, 3), np.uint8)})
        self.assertIsNone(controller.poll()[0])
        controller.frames.put({"generation": controller.generation, "frame": np.zeros((2, 2, 3), np.uint8)})
        self.assertFalse(controller.poll()[0]["frame"].any())
        second = controller.process
        controller.stop()
        self.assertTrue(second.terminated)
        self.assertIsNone(controller.poll()[0])

    def test_local_video_uses_real_child_and_stops(self):
        # This verifies spawn/IPC/OpenCV using a local generated file, not a camera.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"offline.avi"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 12.0, (80, 48))
            if not writer.isOpened():
                self.skipTest("Local MJPG encoder unavailable")
            for n in range(12):
                writer.write(np.full((48, 80, 3), (n*10, 50, 120), dtype=np.uint8))
            writer.release()
            controller = CaptureController()
            try:
                controller.start(VideoSource("file", str(path), loop=True))
                worker = controller.process
                received = None
                errors = []
                deadline = time.monotonic()+12
                while time.monotonic() < deadline:
                    received, events = controller.poll()
                    errors.extend(event for event in events if event['kind'] == 'error')
                    if received is not None or errors:
                        break
                    time.sleep(.03)
                self.assertFalse(errors)
                self.assertIsNotNone(received)
                self.assertEqual(received['frame'].shape, (48, 80, 3))
                self.assertTrue(worker.is_alive())
            finally:
                controller.stop()
            self.assertIsNone(controller.process)
            self.assertEqual(controller.poll(), (None, []))

    def test_spawn_failure_cleans_up_without_joining_unstarted_process(self):
        context = FakeContext()
        controller = CaptureController(context=context)
        original = context.Process
        def failing_process(**kwargs):
            process = original(**kwargs)
            process.start = Mock(side_effect=OSError('spawn unavailable'))
            process.join = Mock(side_effect=AssertionError('not started'))
            return process
        context.Process = failing_process
        with self.assertRaisesRegex(OSError, 'spawn unavailable'):
            controller.start(VideoSource('file', 'fake.avi'))
        context.processes[0].join.assert_not_called()
        self.assertIsNone(controller.process)
        self.assertIsNone(controller.frames)

    def test_720p_file_end_flushes_large_frame_without_stale_live_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'720p.avi'
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 10.0, (1280, 720))
            if not writer.isOpened():
                self.skipTest('Local MJPG encoder unavailable')
            for value in (40, 80, 120):
                writer.write(np.full((720, 1280, 3), value, np.uint8))
            writer.release()
            controller = CaptureController()
            got_frame, ended, exited = False, False, False
            try:
                controller.start(VideoSource('file', str(path), loop=False))
                deadline = time.monotonic()+12
                while time.monotonic() < deadline and not exited:
                    frame, events = controller.poll()
                    if frame is not None:
                        self.assertEqual(frame['frame'].shape, (720, 1280, 3))
                        got_frame = True
                    self.assertFalse([e for e in events if e['kind'] == 'error'])
                    ended |= any(e['kind'] == 'ended' for e in events)
                    exited |= any(e['kind'] == 'worker_exit' for e in events)
                    time.sleep(.01)
                self.assertTrue(got_frame)
                self.assertTrue(ended)
                self.assertTrue(exited)
                self.assertFalse(controller.process.is_alive())
            finally:
                controller.stop()


class VideoPanelTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        try:
            self.window = tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        self.window.withdraw()

    def tearDown(self):
        if hasattr(self, 'panel'):
            self.panel.close()
        if hasattr(self, 'window'):
            self.window.destroy()

    def test_panel_startup_and_image_load_never_open_camera(self):
        with patch.object(cv2, "VideoCapture") as capture:
            self.panel = VideoPanel(self.window)
            self.window.update()
            self.assertIsNone(self.panel.controller.process)
            self.assertIsNone(self.panel.get_frame_copy())
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"棋盘输入.png"
                pixels = np.full((36, 64, 3), (12, 100, 200), np.uint8)
                path.write_bytes(cv2.imencode('.png', pixels)[1].tobytes())
                self.panel.load_image(path)
                copy = self.panel.get_frame_copy()
                self.assertTrue(np.array_equal(copy, pixels))
                copy[:] = 0
                self.assertTrue(np.array_equal(self.panel.get_frame_copy(), pixels))
                self.assertIn('离线静态图', self.panel.status.get())
                self.panel.stop()
                self.assertIsNone(self.panel.get_frame_copy())
            capture.assert_not_called()

    def test_close_cancels_tick_and_stops_controller(self):
        self.panel = VideoPanel(self.window)
        with patch.object(self.panel.controller, 'stop') as stop:
            self.panel.close()
            self.panel.close()
            self.assertTrue(self.panel._closed)
            self.assertIsNone(self.panel._after_id)
            stop.assert_called_once()

    def test_offline_panel_blocks_camera_even_if_callback_is_called(self):
        with patch.object(cv2, 'VideoCapture') as capture:
            self.panel = VideoPanel(self.window, allow_devices=False)
            self.assertIn('disabled', self.panel.open_device_button.state())
            self.panel.open_camera()
            with self.assertRaisesRegex(RuntimeError, '离线'):
                self.panel._begin(VideoSource('camera', 0), 'invalid offline source')
            self.assertIsNone(self.panel.controller.process)
            capture.assert_not_called()


if __name__ == '__main__':
    unittest.main()
