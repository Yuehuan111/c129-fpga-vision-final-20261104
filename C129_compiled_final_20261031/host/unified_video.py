"""Single-stream preview for the unified desktop host.

Constructing this module or a VideoPanel never opens a camera. VideoCapture is
created only in a task-owned child process after an explicit source-open action.
The preview never claims to be a second, synchronized FPGA before/after stream.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import multiprocessing as mp
from pathlib import Path
import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from PIL import Image, ImageTk


@dataclass(frozen=True)
class VideoSource:
    kind: str
    value: int | str
    backend: str = "AUTO"
    loop: bool = False
    requested_size: tuple[int, int] | None = None

    def __post_init__(self):
        if self.kind not in ("camera", "file"):
            raise ValueError("Unsupported video source")
        if self.backend not in ("AUTO", "DSHOW", "MSMF"):
            raise ValueError("Unsupported video backend")
        if self.kind == "camera":
            if isinstance(self.value, bool) or not isinstance(self.value, int) or not 0 <= self.value <= 255:
                raise ValueError("设备索引应为 0 至 255 的整数。")
        elif not isinstance(self.value, str) or not self.value:
            raise ValueError("请选择本地视频文件。")
        if self.requested_size is not None:
            if (self.kind != 'camera' or not isinstance(self.requested_size, tuple)
                    or len(self.requested_size) != 2 or
                    any(type(value) is not int or not 1 <= value <= 8192 for value in self.requested_size)):
                raise ValueError('Capture size must be an explicit camera width/height tuple')


def _put_latest(channel, item):
    """A single pending frame bounds memory and avoids an accumulating delay."""
    try:
        channel.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        channel.get_nowait()
    except queue.Empty:
        pass  # Multiprocessing's feeder may still be transferring the old item.
    try:
        channel.put_nowait(item)
    except queue.Full:
        pass  # Dropping a preview frame is preferable to blocking the producer.


def _emit(channel, generation, kind, **data):
    try:
        channel.put_nowait({"generation": generation, "kind": kind, **data})
    except queue.Full:
        pass


def _capture_worker(source, generation, frames, events, stop_event, capture_factory=None):
    """All potentially blocking OpenCV operations live outside Tk's process.

    capture_factory exists for fake-capture tests; production uses VideoCapture.
    The parent may terminate ONLY this process when a backend ignores stop.
    """
    capture = None
    try:
        import cv2

        factory = capture_factory or cv2.VideoCapture
        if source.kind == "camera":
            api = {"AUTO": cv2.CAP_ANY, "DSHOW": cv2.CAP_DSHOW, "MSMF": cv2.CAP_MSMF}[source.backend]
            capture = factory(source.value, api)
        else:
            capture = factory(source.value)
        if not capture.isOpened():
            raise RuntimeError("视频源打开失败；请检查设备索引、后端或文件。")
        if stop_event.is_set():
            return
        if source.kind == 'camera' and source.requested_size is not None:
            # Ask the device for native HDMI resolution, never resize a frame
            # on the CPU and then claim its source geometry is unchanged.
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, source.requested_size[0])
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, source.requested_size[1])
        reported_fps = float(capture.get(cv2.CAP_PROP_FPS))
        file_fps = reported_fps if math.isfinite(reported_fps) and 0.1 <= reported_fps <= 240 else 30.0
        _emit(events, generation, "opened", source_kind=source.kind,
              source_fps=reported_fps if math.isfinite(reported_fps) else None)
        sequence = 0
        read_failures = 0
        deadline = time.monotonic()
        while not stop_event.is_set():
            ok, frame = capture.read()
            if stop_event.is_set():
                break
            if not ok or frame is None:
                if source.kind == "file" and source.loop and sequence:
                    if not capture.set(cv2.CAP_PROP_POS_FRAMES, 0):
                        raise RuntimeError("视频无法回到开头；请关闭循环播放后重试。")
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError("视频循环读取失败。")
                    deadline = time.monotonic()
                elif source.kind == "file":
                    _emit(events, generation, "ended", message="本地视频播放结束，当前保留最后一帧。")
                    break
                else:
                    read_failures += 1
                    if read_failures == 1:
                        _emit(events, generation, 'read_retry', message='采集卡暂未返回视频帧，正在继续取帧；串口连接独立。')
                    if read_failures >= 10:
                        raise RuntimeError('采集卡连续取帧失败，预览已停止并保留最后一帧；这不表示 FPGA／IMU 串口断开。请重新打开实时画面。')
                    stop_event.wait(.1)
                    continue
            if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
                raise RuntimeError("预览需要三通道 BGR 视频帧。")
            if not sequence and source.requested_size and (frame.shape[1], frame.shape[0]) != source.requested_size:
                _emit(events, generation, 'format_warning', message=
                    f'采集卡实际返回 {frame.shape[1]}×{frame.shape[0]}；实时扫描需要原生1280×720，未对预览做几何伪装。')
            if read_failures:
                _emit(events, generation, 'read_recovered')
                read_failures = 0
            sequence += 1
            _put_latest(frames, {"generation": generation, "sequence": sequence,
                                 "timestamp": time.monotonic(), "frame": frame})
            if source.kind == "file":
                deadline += 1.0 / file_fps
                delay = deadline - time.monotonic()
                if delay > 0:
                    stop_event.wait(delay)
                elif delay < -1:
                    deadline = time.monotonic()
    except Exception as error:
        _emit(events, generation, "error", message=str(error))
    finally:
        if capture is not None:
            capture.release()
        # Keep the frame feeder alive until normal delivery finishes. Cancelling
        # its join here can truncate a large IPC message and block the reader.
        # An explicit parent stop can terminate this owned process after first
        # invalidating and abandoning its queues.


class CaptureController:
    """Owns one preview worker and generation-tagged, bounded IPC queues."""

    def __init__(self, context=None):
        self.context = context or mp.get_context("spawn")
        self.generation = 0
        self.process = None
        self.frames = None
        self.events = None
        self.stop_event = None
        self.source = None
        self._exit_reported = False
        self._process_started = False

    def start(self, source):
        if not isinstance(source, VideoSource):
            raise TypeError("VideoSource required")
        self.stop()
        self.source = source
        self.frames = self.context.Queue(maxsize=1)
        self.events = self.context.Queue(maxsize=8)
        self.stop_event = self.context.Event()
        self._exit_reported = False
        self._process_started = False
        self.process = self.context.Process(
            target=_capture_worker,
            args=(source, self.generation, self.frames, self.events, self.stop_event),
            daemon=True,
            name=f"UnifiedVideo-{self.generation}",
        )
        try:
            self.process.start()
            self._process_started = True
        except Exception:
            self.stop()
            raise

    def poll(self):
        latest = None
        messages = []
        if self.frames is not None:
            while True:
                try:
                    item = self.frames.get_nowait()
                except (queue.Empty, EOFError, OSError, ValueError):
                    break
                if item.get("generation") == self.generation:
                    latest = item
        if self.events is not None:
            while True:
                try:
                    item = self.events.get_nowait()
                except (queue.Empty, EOFError, OSError, ValueError):
                    break
                if item.get("generation") == self.generation:
                    messages.append(item)
        if self.process is not None and not self.process.is_alive() and not self._exit_reported:
            self._exit_reported = True
            messages.append({"generation": self.generation, "kind": "worker_exit",
                             "exitcode": self.process.exitcode})
        return latest, messages

    def stop(self):
        # Invalidate old frames before attempting a possibly blocked shutdown.
        self.generation += 1
        process, stop_event, started = self.process, self.stop_event, self._process_started
        channels = (self.frames, self.events)
        self.process = self.frames = self.events = self.stop_event = None
        self.source = None
        self._process_started = False
        if stop_event is not None:
            stop_event.set()
        if process is not None:
            try:
                if started:
                    process.join(timeout=0.15)
                    if process.is_alive():
                        process.terminate()
                        process.join(timeout=0.35)
                    if process.is_alive() and hasattr(process, "kill"):
                        process.kill()
                        process.join(timeout=0.35)
            finally:
                if hasattr(process, "close") and not process.is_alive():
                    process.close()
        for channel in channels:
            if channel is not None:
                if hasattr(channel, "cancel_join_thread"):
                    channel.cancel_join_thread()
                if hasattr(channel, "close"):
                    channel.close()


class VideoPanel(ttk.Frame):
    """Embeddable single-stream Tk preview, with opt-in capture and local files."""

    def __init__(self, master, on_status=None, *, allow_devices=True):
        super().__init__(master, padding=0)
        self.on_status = on_status
        self.allow_devices = bool(allow_devices)
        self.controller = CaptureController()
        self._closed = False
        self._after_id = None
        self._frame = None
        self._frame_metadata = None
        self._photo = None
        self._frame_times = deque(maxlen=60)
        self.preview_rotate_180 = tk.BooleanVar(master=self, value=False)
        self.preview_full_rotation_crop = False
        self._fullscreen_window = None
        self._fullscreen_canvas = None
        self._live = False
        self._source_label = "未打开视频源"
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(bar, text="设备索引").grid(row=0, column=0, sticky="w")
        self.device_index = tk.StringVar(value="0")
        ttk.Spinbox(bar, textvariable=self.device_index, from_=0, to=255, width=4).grid(row=0, column=1, padx=(5, 8))
        self.backend = tk.StringVar(value="DSHOW")
        ttk.Combobox(bar, textvariable=self.backend, values=("DSHOW", "MSMF", "AUTO"),
                     state="readonly", width=7).grid(row=0, column=2, padx=(0, 8))
        self.open_device_button = ttk.Button(bar, text="打开采集卡", command=self.open_camera)
        self.open_device_button.grid(row=0, column=3, padx=(0, 5))
        if not self.allow_devices:
            self.open_device_button.configure(state="disabled")
        ttk.Button(bar, text="关闭视频源", command=self.stop).grid(row=0, column=4)
        ttk.Button(bar, text="打开图片…", command=self.choose_image).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Button(bar, text="打开视频…", command=self.choose_video).grid(row=1, column=2, columnspan=2, sticky="w", pady=(6, 0))
        self.loop_file = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="本地视频循环", variable=self.loop_file).grid(row=1, column=4, sticky="w", pady=(6, 0))

        self.canvas = tk.Canvas(self, background="#111c2c", highlightthickness=0,
                                width=640, height=360)
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        self.canvas.bind("<Double-Button-1>", self.toggle_fullscreen)
        self.status = tk.StringVar(value="尚未打开视频源；选择设备索引后点击打开，或先载入本地图片。")
        if not self.allow_devices:
            self.status.set("离线模式：可载入本地图片或视频；设备采集已禁用。")
        ttk.Label(self, textvariable=self.status, wraplength=740).grid(row=2, column=0, sticky="ew", pady=(7, 0))
        self.statistics = tk.StringVar(value="单路预览；采集预览 FPS 不等于 FPGA 每秒新矫正帧。")
        ttk.Label(self, textvariable=self.statistics, wraplength=740).grid(row=3, column=0, sticky="ew", pady=(3, 0))
        self.bind("<Destroy>", self._destroyed, add="+")
        self._after_id = self.after(33, self._tick)
        self._redraw()

    def _notify(self, message):
        self.status.set(message)
        if self.on_status is not None:
            self.on_status(message)

    def _error(self, error):
        self._notify(str(error))
        messagebox.showerror("视频预览", str(error), parent=self.winfo_toplevel())

    def open_camera(self):
        if self._closed:
            return
        if not self.allow_devices:
            self._notify("离线模式禁止打开设备；请使用本地图片或视频。")
            return
        try:
            source = VideoSource("camera", int(self.device_index.get()), self.backend.get(), requested_size=(1280, 720))
            self._begin(source, f"采集设备 {source.value} / {source.backend}")
        except Exception as error:
            self._error(error)

    def choose_image(self):
        path = filedialog.askopenfilename(parent=self.winfo_toplevel(), title="打开离线图片",
                                         filetypes=[("图片", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp"), ("全部文件", "*.*")])
        if path:
            try:
                self.load_image(path)
            except Exception as error:
                self._error(error)

    def choose_video(self):
        path = filedialog.askopenfilename(parent=self.winfo_toplevel(), title="打开本地演示视频",
                                         filetypes=[("视频", "*.mp4 *.avi *.mkv *.mov *.wmv"), ("全部文件", "*.*")])
        if path:
            try:
                self.load_video(path, loop=self.loop_file.get())
            except Exception as error:
                self._error(error)

    def load_video(self, path, *, loop=False):
        path = Path(path).expanduser().resolve()
        if not path.is_file():
            raise ValueError("视频文件不存在。")
        self._begin(VideoSource("file", str(path), loop=bool(loop)), "本地视频 · " + path.name)

    def _begin(self, source, label):
        if self._closed:
            raise RuntimeError("视频面板已关闭。")
        if source.kind == "camera" and not self.allow_devices:
            raise RuntimeError("离线模式禁止打开视频设备。")
        self._frame = None
        self._frame_metadata = None
        self._photo = None
        self._frame_times.clear()
        self._source_label = label
        self._live = False
        self.controller.start(source)
        self.statistics.set("正在等待首帧；视频采集不发送任何 FPGA 配置。")
        self._notify("正在打开 " + label + "…")
        self._redraw()

    def load_image(self, path):
        if self._closed:
            raise RuntimeError("视频面板已关闭。")
        import cv2

        path = Path(path)
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("不能读取这张图片。")
        self.controller.stop()
        self._frame_times.clear()
        self._live = False
        self._frame = image
        self._frame_metadata = {'captured_at': time.monotonic(), 'frame_id': 'image:' + str(path.resolve()),
                                'source_kind': 'image', 'generation': self.controller.generation, 'sequence': 0}
        self._source_label = "本地图片 · " + path.name
        self._notify(self._source_label + "（离线静态图）")
        self.statistics.set(f"{image.shape[1]}×{image.shape[0]} · 静态图片；不代表实时 FPGA 输出。")
        self._redraw()

    def get_frame_copy(self):
        return None if self._frame is None else self._frame.copy()

    def get_frame_observation(self):
        """Copy one capture packet; time and ID belong to the producer frame.

        The local capture ID is not the FPGA source ID. C128's in-band source
        footer is decoded separately by the visual tracker before submission.
        """
        if self._frame is None or self._frame_metadata is None:
            return None
        packet = dict(self._frame_metadata)
        packet.update(frame=self._frame.copy(), live=bool(self._live and
                      packet['source_kind'] == 'camera' and time.monotonic() - packet['captured_at'] < 1.5))
        return packet

    def capture_status(self):
        metadata = self._frame_metadata or {}
        age = None if not metadata else time.monotonic() - metadata['captured_at']
        return {'source_kind': metadata.get('source_kind'), 'has_frame': self._frame is not None,
                'live': self._live, 'age_s': age, 'stale': age is None or age > 1.5,
                'frame_id': metadata.get('frame_id')}

    def stop(self):
        self.controller.stop()
        self._frame = None
        self._frame_metadata = None
        self._photo = None
        self._frame_times.clear()
        self._live = False
        self._source_label = "未打开视频源"
        if not self._closed:
            self._notify("视频源已关闭；串口会话和 FPGA 配置不受此按钮影响。")
            self.statistics.set("单路预览；采集预览 FPS 不等于 FPGA 每秒新矫正帧。")
            self._redraw()

    def _tick(self):
        self._after_id = None
        if self._closed:
            return
        frame, events = self.controller.poll()
        for event in events:
            kind = event["kind"]
            if kind == "opened":
                self._live = True
                self._notify(self._source_label + " · 已打开")
            elif kind == 'read_retry':
                self._live = False
                self._notify(event['message'])
            elif kind == 'read_recovered':
                self._live = True
                self._notify(self._source_label + ' · 取帧已恢复')
            elif kind == 'format_warning':
                self._notify(event['message'])
            elif kind in ("error", "ended"):
                self._live = False
                self._notify(event["message"])
            elif kind == "worker_exit" and self._live:
                self._live = False
                self._notify("视频工作进程已结束，当前保留最后一帧。")
        if frame is not None:
            self._frame = frame["frame"]
            source = self.controller.source
            self._frame_metadata = {'captured_at': frame['timestamp'],
                'frame_id': f"{frame['generation']}:{frame['sequence']}",
                'generation': frame['generation'], 'sequence': frame['sequence'],
                'source_kind': 'camera' if source and source.kind == 'camera' else 'video'}
            self._frame_times.append(time.monotonic())
            self._redraw()
        if self._frame is not None and self.controller.source is not None:
            fps = 0.0
            if self._live and len(self._frame_times) > 1:
                span = self._frame_times[-1] - self._frame_times[0]
                if span > 0 and time.monotonic() - self._frame_times[-1] < 1.5:
                    fps = (len(self._frame_times)-1)/span
            suffix = "更新中" if self._live else "已停止／最后一帧"
            self.statistics.set(f"{self._frame.shape[1]}×{self._frame.shape[0]} · 预览更新 {fps:.1f} FPS · {suffix}；不是 FPGA 新帧率。")
        self._after_id = self.after(33, self._tick)

    def toggle_fullscreen(self, _event=None):
        """A second display surface for the same capture; never open a device."""
        if self._closed:
            return "break"
        if self._fullscreen_window is not None:
            return self.exit_fullscreen()
        window = tk.Toplevel(self.winfo_toplevel())
        self._fullscreen_window = window
        window.title("视频全屏 · Esc 或双击返回")
        window.configure(background="#111c2c")
        canvas = self._fullscreen_canvas = tk.Canvas(
            window, background="#111c2c", highlightthickness=0)
        canvas.pack(fill="both", expand=True)
        canvas.bind("<Configure>", lambda _event: self._redraw())
        canvas.bind("<Double-Button-1>", self.exit_fullscreen)
        window.bind("<Escape>", self.exit_fullscreen)
        window.protocol("WM_DELETE_WINDOW", self.exit_fullscreen)
        hint = tk.Label(window, text="Esc / 双击返回", background="#111c2c",
                        foreground="#dce6f2", padx=10, pady=5)
        hint.place(relx=1.0, x=-16, y=16, anchor="ne")
        hint.bind("<Double-Button-1>", self.exit_fullscreen)
        window.attributes("-fullscreen", True)
        window.lift()
        canvas.focus_set()
        self._redraw()
        return "break"

    def exit_fullscreen(self, _event=None):
        window = self._fullscreen_window
        self._fullscreen_window = self._fullscreen_canvas = None
        if window is not None:
            window.destroy()
        if not self._closed:
            self._redraw()
            self.canvas.focus_set()
        return "break"

    def _redraw(self):
        if self._closed:
            return
        canvas = self._fullscreen_canvas if self._fullscreen_canvas is not None else self.canvas
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        canvas.delete("all")
        if self._frame is None:
            canvas.create_text(width/2, height/2, text="HDMI 视频预览\n\n打开采集卡或载入本地图片 / 视频",
                                    fill="#a8bbd1", font=("Microsoft YaHei UI", 15), justify="center",
                                    width=max(100, width-32))
            return
        frame = self._frame
        factor = min(width/frame.shape[1], height/frame.shape[0])
        size = (max(1, round(frame.shape[1]*factor)), max(1, round(frame.shape[0]*factor)))
        image = Image.fromarray(frame[:, :, ::-1]).resize(size, Image.Resampling.BILINEAR)
        # Rotate only the rendered preview. _frame and its source metadata stay
        # native so scanning, CRC decoding and saved evidence use original pixels.
        if self.preview_rotate_180.get():
            image = image.transpose(Image.Transpose.ROTATE_180)
        self._photo = ImageTk.PhotoImage(image=image, master=canvas)
        canvas.create_image(width/2, height/2, image=self._photo, anchor="center")

    def _destroyed(self, event):
        if event.widget is self:
            self.close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.exit_fullscreen()
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        self.controller.stop()
        self._frame = None
        self._photo = None

