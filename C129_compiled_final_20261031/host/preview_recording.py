"""Bounded, asynchronous recording of the existing preview; no device access."""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

import numpy as np


class PreviewRecorder:
    """Write a silent MP4 on a wall-clock timeline, holding the latest preview.

    The 30 FPS output is a playback timebase, not a measured FPGA frame rate.
    A single immutable pending frame bounds memory. Encoding never runs in Tk.
    """

    max_encode_lag_seconds = 2.0

    def __init__(self, path, frame, *, rotate=False, fps=30.0):
        self.path = Path(path)
        self.fps = float(fps)
        if not math.isfinite(self.fps) or not 1 <= self.fps <= 60:
            raise ValueError('录像帧率设置无效。')
        self._validate_frame(frame)
        self.size = (frame.shape[1], frame.shape[0])
        if any(dimension % 2 for dimension in self.size):
            raise ValueError('MP4 录像需要偶数宽高，请使用原生 1280×720 采集画面。')
        self._lock = threading.Lock()
        self._latest = (frame.copy(), bool(rotate), 1)
        self.submitted_frames = 1
        self.written_frames = 0
        self.stop_at = None
        self.stop_reason = ''
        self.error = None
        self.done = threading.Event()
        self._stop_event = threading.Event()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Reserve this filename without allowing the encoder to overwrite a file.
        with self.path.open('xb'):
            pass
        self.started_at = time.monotonic()
        self.thread = threading.Thread(target=self._encode, name='PreviewMP4', daemon=True)
        try:
            self.thread.start()
        except Exception:
            self.path.unlink(missing_ok=True)
            raise

    @staticmethod
    def _validate_frame(frame):
        if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or
                frame.ndim != 3 or frame.shape[2] != 3 or min(frame.shape[:2]) < 2):
            raise ValueError('录像需要有效的三通道视频画面。')

    @property
    def stopping(self):
        return self._stop_event.is_set()

    @property
    def elapsed(self):
        return max(0.0, (self.stop_at or time.monotonic()) - self.started_at)

    def submit(self, frame, *, rotate=False):
        self._validate_frame(frame)
        if (frame.shape[1], frame.shape[0]) != self.size:
            raise ValueError('视频分辨率已改变，本段录像将停止保存。')
        with self._lock:
            if self.stopping or self.done.is_set():
                return
            self.submitted_frames += 1
            self._latest = (frame.copy(), bool(rotate), self.submitted_frames)

    def request_stop(self, reason='手动停止'):
        with self._lock:
            if not self.stopping:
                self.stop_at = time.monotonic()
                self.stop_reason = str(reason)
                self._stop_event.set()

    def _encode(self):
        import cv2
        writer = None
        try:
            writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'),
                                     self.fps, self.size)
            if not writer.isOpened():
                raise RuntimeError('MP4 编码器无法打开，请检查保存位置和 OpenCV 编码支持。')
            cached_sequence, encoded_frame = None, None
            while True:
                with self._lock:
                    frame, rotate, sequence = self._latest
                    stop_at = self.stop_at
                elapsed = max(0.0, (stop_at if stop_at is not None else time.monotonic()) - self.started_at)
                target_count = max(1, math.ceil(elapsed * self.fps))
                if (target_count - self.written_frames) / self.fps > self.max_encode_lag_seconds:
                    raise RuntimeError('录像编码未能跟上实时画面，已结束本段；请检查磁盘速度或降低电脑负载。')
                if sequence != cached_sequence:
                    encoded_frame = cv2.rotate(frame, cv2.ROTATE_180) if rotate else frame
                    cached_sequence = sequence
                # Duplicate held preview frames to preserve elapsed time rather
                # than speeding up the video when preview updates are slower.
                while self.written_frames < target_count:
                    writer.write(encoded_frame)
                    self.written_frames += 1
                if stop_at is not None:
                    break
                wait = self.written_frames / self.fps - (time.monotonic() - self.started_at)
                self._stop_event.wait(max(0.001, wait))
        except Exception as error:
            self.error = str(error)
        finally:
            if writer is not None:
                try:
                    writer.release()
                except Exception as error:
                    self.error = self.error or str(error)
            self.request_stop(self.error or '录像结束')
            try:
                if not self.written_frames:
                    self.path.unlink(missing_ok=True)
                elif not self.path.is_file() or self.path.stat().st_size <= 44:
                    self.error = self.error or '录像文件未正常写入，请检查磁盘空间。'
                report = {
                    'video': self.path.name, 'status': 'error' if self.error else 'saved',
                    'error': self.error, 'stop_reason': self.stop_reason,
                    'width': self.size[0], 'height': self.size[1], 'audio': False,
                    'codec': 'mp4v', 'file_fps': self.fps,
                    'elapsed_seconds': self.elapsed,
                    'video_duration_seconds': self.written_frames / self.fps,
                    'written_frames': self.written_frames,
                    'preview_submissions': self.submitted_frames,
                    'note': 'Preview recording; held frames preserve wall time. File FPS is not FPGA throughput.',
                    'orientation': 'Follows preview rotation; original capture pixels are unchanged.',
                }
                self.path.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            except Exception as error:
                self.error = self.error or str(error)
            self.done.set()
