"""C129 extended five-mode UI; optional FPGA detail enhancement defaults off."""
from __future__ import annotations
import argparse
import json
import multiprocessing
import os
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog

from unified_app import UnifiedControlPanel, ROOT
from demo_controls import DemoController, display_status, independent_status, board_status, MODE_LABELS
from demo_results import ModeResults
from demo_metrics import DemoMetrics
from demo_theme import apply_demo_theme, confirmed_mode
from preview_recording import PreviewRecorder
from roll_crop import DEMO_ROLL_CROP, checked_demo_crop


class DemoControlPanel(UnifiedControlPanel):
    def __init__(self, window, *, offline=False):
        self.demo_controller = None
        self.advanced_visible = False
        self.imported_scan_path = None
        self.visual_worker = None
        self._vision_frame_id = None
        self.mode_results = ModeResults()
        self.demo_metrics = DemoMetrics(ROOT.parent)
        self.metrics_recording = False
        self.metrics_visible = False
        self._history_frame_id = None
        self.preview_recorder = None
        self._recording_source = None
        self._recording_frame_key = None
        self._close_after_recording = False
        super().__init__(window, offline=offline)
        self.settings_path = ROOT/'settings/demo_preferences.json'
        self.demo_controller = DemoController(lambda: self.session, self.notify,
                                             imu_settings=self.demo_imu_settings,
                                             start_scan=self.start_visual_worker,
                                             stop_scan=self.stop_visual_worker)
        self.pose_mode.set('roll_only')
        self.crop_enabled.set(True)
        self.crop_zoom.set(str(DEMO_ROLL_CROP.zoom))
        self.crop_range.set(str(DEMO_ROLL_CROP.max_abs_roll_deg))
        # Demo and advanced controls share one queue and the original single
        # serial owner. A second UI queue must never race a mode change.
        self.actions = self.demo_controller.sequence
        self.advanced_pages = [page for page in self.tabs.tabs() if page != str(self.demo_page)]
        self.settings_page = ttk.Frame(self.tabs)
        self.tabs.insert(1, self.settings_page, text='设备与参数')
        self.build_settings()
        for container in window.winfo_children():
            for widget in container.winfo_children():
                if isinstance(widget, ttk.Label) and widget.cget('text') == '统一视觉工作台  /  C127':
                    widget.configure(text='五模式演示台  /  C129 扩展')
        for page in self.advanced_pages:
            self.tabs.hide(page)
        self.tabs.tab(self.demo_page, text='演示')
        self.tabs.select(self.demo_page)
        window.title('VISION LAB · 五模式演示台 · C129 扩展' + (' · 离线界面预览' if offline else ''))
        window.minsize(1000, 700)
        window.geometry(f'{min(1360, window.winfo_screenwidth()-60)}x{min(860, window.winfo_screenheight()-90)}')
        window.bind('<F1>', lambda e: self.tabs.select(self.demo_page))
        window.bind('<F2>', lambda e: self.tabs.select(self.settings_page))
        window.bind('<F3>', lambda e: self.show_advanced())
        # Tk's application-wide binding also reaches the fullscreen Toplevel.
        window.bind_all('<F8>', self.take_photo, add='+')
        window.bind_all('<F9>', self.toggle_recording, add='+')
        self.notify('先连接设备并导入所需参数；主界面直接切换演示模式。' if not offline
                    else '离线界面预览：未连接串口或采集卡，照片仅用于查看界面布局。')
        self.window.after(33, self.pump_visual_frames)

    def setup_style(self):
        super().setup_style()
        apply_demo_theme(self.window)

    def build_demo(self):
        from unified_video import VideoPanel
        page = self.demo_page
        page.columnconfigure(0, weight=1)
        page.rowconfigure(3, weight=1)
        title = ttk.Frame(page)
        title.grid(row=0, column=0, sticky='ew', pady=(0, 12))
        ttk.Label(title, text='视觉矫正演示', style='DemoTitle.TLabel').pack(side='left')
        ttk.Button(title, text='高级调试', command=self.show_advanced).pack(side='right')
        self.advanced_button = title.winfo_children()[-1]
        self.metrics_toggle_button = ttk.Button(title, text='实测指标', command=self.toggle_metrics_panel)
        self.metrics_toggle_button.pack(side='right', padx=8)
        modes = ttk.Frame(page)
        modes.grid(row=1, column=0, sticky='ew')
        self.demo_buttons = {}
        for index, (key, label, method) in enumerate([
                ('raw', '原图', 'raw'),
                ('normal', '普通矫正', 'normal'),
                ('imu', 'IMU 动态矫正', 'imu'),
                ('scan', '平面扫描', 'scan'),
                ('enhance', '自适应增强', 'enhance')]):
            modes.columnconfigure(index, weight=1)
            button = ttk.Button(modes, text=label, style='Mode.TButton',
                                state='disabled', command=lambda name=method: self.demo_call(name))
            button.grid(row=0, column=index, sticky='ew', padx=(0, 8 if index < 4 else 0))
            self.demo_buttons[key] = button
        info = ttk.Frame(page)
        info.grid(row=2, column=0, sticky='ew', pady=(12, 10))
        self.demo_mode = tk.StringVar(value='未连接 FPGA')
        self.demo_evidence = tk.StringVar(value='在“设备与参数”中连接设备。')
        controls = ttk.Frame(info)
        controls.pack(fill='x')
        ttk.Label(controls, textvariable=self.demo_mode, style='Status.TLabel').pack(side='left')
        ttk.Label(info, textvariable=self.demo_evidence, style='Subtle.TLabel', wraplength=1160).pack(anchor='w', pady=(5, 0))
        self.adaptive_enabled = tk.BooleanVar(value=False)
        self.adaptive_toggle = ttk.Checkbutton(controls, text='自适应亮度', variable=self.adaptive_enabled,
                                               state='disabled', command=lambda: self.demo_call('adaptive', self.adaptive_enabled.get()))
        self.adaptive_toggle.pack(side='right')
        self.reference_button = ttk.Button(controls, text='记录正向', command=lambda: self.demo_call('reference'))
        self.imu_comparison_button = ttk.Button(controls, text='同倍率静态对照（非同帧）',
            command=lambda: self.demo_call('compare_imu_static'))
        self.reacquire_button = ttk.Button(controls, text='重新识别棋盘', command=lambda: self.act(self.reacquire_scan))
        self.stop_button = ttk.Button(controls, text='停止更新', command=self.stop_all_requests)
        self.stop_button.pack(side='right', padx=8)
        video_row = ttk.Frame(page)
        video_row.grid(row=3, column=0, sticky='nsew')
        video_row.columnconfigure(0, weight=1)
        video_row.rowconfigure(0, weight=1)
        self.video_panel = VideoPanel(video_row, on_status=self.notify, allow_devices=not self.offline)
        self.video_panel.grid(row=0, column=0, sticky='nsew')
        self.build_metrics_panel(video_row)
        ttk.Checkbutton(title, text='观看旋转 180°（仅显示）',
                        variable=self.video_panel.preview_rotate_180,
                        command=lambda: self.act(self.change_preview_rotation)).pack(side='right', padx=18)
        # Keep the existing capture pipeline; move only its user controls to
        # the setup page, leaving a clean full-width preview on the demo page.
        for row in (0, 3):
            for widget in self.video_panel.grid_slaves(row=row):
                widget.grid_remove()
        self.panels.append(self.video_panel)
        photo_bar = ttk.Frame(page)
        photo_bar.grid(row=4, column=0, sticky='ew', pady=(10, 2))
        photo_bar.columnconfigure(4, weight=1)
        ttk.Label(photo_bar, text='双击全屏，Esc / 双击返回 · F8 拍照 · F9 开始 / 停止录像\nIMU：固定测试卡，相机与 IMU 一起转动；自适应：改变照明',
                  style='Subtle.TLabel', wraplength=900).grid(row=1, column=0, columnspan=5, sticky='w', pady=(5, 0))
        self.photo_button = ttk.Button(photo_bar, text='拍照 (F8)',
                                       command=self.take_photo)
        self.photo_button.grid(row=0, column=0, padx=(0, 8))
        self.photo_folder_button = ttk.Button(photo_bar, text='照片文件夹',
            command=lambda: self.act(self.open_photo_directory))
        self.photo_folder_button.grid(row=0, column=1, padx=(0, 16))
        self.record_button = ttk.Button(photo_bar, text='开始录像 (F9)', command=self.toggle_recording)
        self.record_button.grid(row=0, column=2, padx=(0, 8))
        self.video_folder_button = ttk.Button(photo_bar, text='录像文件夹',
            command=lambda: self.act(self.open_video_directory))
        self.video_folder_button.grid(row=0, column=3)
        self.recording_status = tk.StringVar(value='MP4 · 仅画面，无声音')
        ttk.Label(photo_bar, textvariable=self.recording_status,
                  style='Subtle.TLabel', wraplength=300).grid(row=0, column=4, sticky='e', padx=(10, 0))
        self.device_status = tk.StringVar(value='视频：未打开   |   FPGA：串口未连接   |   IMU：未连接')
        ttk.Label(page, textvariable=self.device_status, style='Subtle.TLabel').grid(row=5, column=0, sticky='w')
        self.history_status = tk.StringVar(value='单路 HDMI；各模式历史快照来自不同时间，不是同帧五路输出。')
        ttk.Label(page, textvariable=self.history_status, style='Subtle.TLabel').grid(row=6, column=0, sticky='w')
        # Parent update() retains its diagnostic strings without putting them
        # on the presentation surface.
        self.device_text = tk.StringVar()
        self.imu_text = tk.StringVar()
        self.board_text = tk.StringVar()
        self.output_text = tk.StringVar()

    def take_photo(self, _event=None):
        self.act(self.save_preview_photo)
        return 'break'

    def save_preview_photo(self):
        """Save native pixels in the displayed orientation; keep capture data intact."""
        import cv2
        frame = self.video_panel.get_frame_copy()
        if frame is None:
            raise ValueError('当前没有可拍照的画面，请先打开实时画面或本地照片。')
        if self.video_panel.preview_rotate_180.get():
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        ok, encoded = cv2.imencode('.png', frame)
        if not ok:
            raise RuntimeError('照片编码失败，请重试。')
        directory = ROOT / 'results' / 'photos'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / datetime.now().strftime('photo_%Y%m%d_%H%M%S_%f.png')
        # Exclusive creation protects an existing photo even if the clock repeats.
        with path.open('xb') as output:
            output.write(encoded.tobytes())
        return f'照片已保存（{frame.shape[1]}×{frame.shape[0]}，方向与预览一致）：{path}'

    def open_photo_directory(self):
        directory = ROOT / 'results' / 'photos'
        directory.mkdir(parents=True, exist_ok=True)
        os.startfile(str(directory))

    def toggle_recording(self, _event=None):
        self.act(self.toggle_preview_recording)
        return 'break'

    def toggle_preview_recording(self):
        if self.preview_recorder is not None:
            self.preview_recorder.request_stop()
            self.record_button.configure(text='正在保存…', state='disabled')
            self.recording_status.set('正在保存录像…')
            return '正在完成 MP4 文件，请稍候。'
        status = self.video_panel.capture_status()
        packet = self.video_panel.get_frame_observation()
        if (not packet or not status['live'] or status['stale'] or
                packet['source_kind'] not in ('camera', 'video')):
            raise ValueError('请先打开正在更新的实时画面或本地视频，再开始录像。')
        rotate = bool(self.video_panel.preview_rotate_180.get())
        path = ROOT / 'results' / 'videos' / datetime.now().strftime('video_%Y%m%d_%H%M%S_%f.mp4')
        self.preview_recorder = PreviewRecorder(path, packet['frame'], rotate=rotate)
        self._recording_source = (packet['generation'], packet['source_kind'])
        self._recording_frame_key = (packet['frame_id'], rotate)
        self.record_button.configure(text='停止录像 (F9)')
        self.recording_status.set('录像中 00:00 · 无声音')
        return '录像已开始；按 F9 或“停止录像”保存。'

    def open_video_directory(self):
        directory = ROOT / 'results' / 'videos'
        directory.mkdir(parents=True, exist_ok=True)
        os.startfile(str(directory))

    def pump_recording(self):
        recorder = self.preview_recorder
        if recorder is None:
            return
        if recorder.done.is_set():
            self.preview_recorder = None
            self.record_button.configure(text='开始录像 (F9)', state='normal')
            if recorder.error:
                self.recording_status.set('录像异常，请查看底部提示')
                self.notify(f'录像异常：{recorder.error}；保存位置：{recorder.path.parent}')
            else:
                self.recording_status.set('录像已保存 · 无声音')
                self.notify(f'录像已保存：{recorder.path}（{recorder.elapsed:.1f} 秒；{recorder.stop_reason}）')
            if self._close_after_recording:
                self._close_after_recording = False
                if not recorder.error:
                    self.close()
            return
        if recorder.stopping:
            self.recording_status.set('正在保存录像…')
            self.record_button.configure(text='正在保存…', state='disabled')
            return
        seconds = int(recorder.elapsed)
        self.recording_status.set(f'录像中 {seconds // 60:02d}:{seconds % 60:02d} · 无声音')
        status = self.video_panel.capture_status()
        packet = self.video_panel.get_frame_observation()
        if (not packet or not status['live'] or status['stale'] or
                (packet['generation'], packet['source_kind']) != self._recording_source):
            recorder.request_stop('视频源停止、切换或超过 1.5 秒没有新帧')
            return
        rotate = bool(self.video_panel.preview_rotate_180.get())
        key = (packet['frame_id'], rotate)
        if key != self._recording_frame_key:
            try:
                recorder.submit(packet['frame'], rotate=rotate)
                self._recording_frame_key = key
            except Exception as error:
                recorder.request_stop(str(error))

    def build_metrics_panel(self, parent):
        # Presentation-only view of the existing session; no device access.
        self.metrics_panel = ttk.Frame(parent, width=310, padding=(10, 0, 0, 0))
        self.metrics_panel.grid(row=0, column=1, sticky='nsew')
        self.metrics_panel.grid_propagate(False)
        self.metrics_panel.columnconfigure(0, weight=1)
        self.metrics_panel.rowconfigure(2, weight=1)
        self.metrics_title = tk.StringVar(value='实测指标 · 等待连接')
        ttk.Label(self.metrics_panel, textvariable=self.metrics_title,
                  font=('Microsoft YaHei UI', 11, 'bold')).grid(row=0, column=0,
                    columnspan=2, sticky='w', pady=(0, 8))
        buttons = ttk.Frame(self.metrics_panel)
        buttons.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(0, 8))
        self.metrics_start_button = ttk.Button(buttons, text='开始本段记录', state='disabled',
            command=lambda: self.act(self.start_metrics_recording))
        self.metrics_start_button.pack(side='left')
        self.metrics_export_button = ttk.Button(buttons, text='导出本段…', state='disabled',
            command=lambda: self.act(self.export_metrics_recording))
        self.metrics_export_button.pack(side='left', padx=(6, 0))
        self.metrics_canvas = tk.Canvas(self.metrics_panel, highlightthickness=0,
                                        background='#eef2f6', yscrollincrement=20)
        scroll = ttk.Scrollbar(self.metrics_panel, orient='vertical',
                               command=self.metrics_canvas.yview)
        self.metrics_canvas.configure(yscrollcommand=scroll.set)
        self.metrics_canvas.grid(row=2, column=0, sticky='nsew')
        scroll.grid(row=2, column=1, sticky='ns')
        self.metrics_body = ttk.Frame(self.metrics_canvas, padding=(0, 0, 6, 8))
        self.metrics_body.columnconfigure(0, weight=1)
        self.metrics_window = self.metrics_canvas.create_window(0, 0, anchor='nw',
                                                               window=self.metrics_body)
        self.metrics_body.bind('<Configure>', lambda event: self.metrics_canvas.configure(
            scrollregion=self.metrics_canvas.bbox('all')))
        self.metrics_canvas.bind('<Configure>', lambda event: self.metrics_canvas.itemconfigure(
            self.metrics_window, width=event.width))
        self.metrics_canvas.bind('<MouseWheel>', self.scroll_metrics)
        self.metrics_body.bind('<MouseWheel>', self.scroll_metrics)
        self.metrics_widgets = []
        self.metrics_note = tk.StringVar(value='只显示现有遥测与离线编译信息；缺失值保留为待测。')
        self.metrics_note_label = ttk.Label(self.metrics_body, textvariable=self.metrics_note,
            style='Subtle.TLabel', wraplength=265, justify='left')
        self.metrics_note_label.bind('<MouseWheel>', self.scroll_metrics)
        self.metrics_quality_button = ttk.Menubutton(self.metrics_panel, text='载入画质结果…')
        quality_menu = tk.Menu(self.metrics_quality_button, tearoff=False)
        quality_menu.add_command(label='矫正后结果…', command=lambda: self.act(
            lambda: self.load_quality_metrics('corrected')))
        quality_menu.add_command(label='原图结果…', command=lambda: self.act(
            lambda: self.load_quality_metrics('original')))
        self.metrics_quality_button.configure(menu=quality_menu)
        self.metrics_quality_button.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.metrics_panel.grid_remove()

    def scroll_metrics(self, event):
        if event.delta:
            steps = max(1, abs(event.delta) // 120)
            self.metrics_canvas.yview_scroll(-steps if event.delta > 0 else steps, 'units')
        return 'break'

    def toggle_metrics_panel(self):
        self.metrics_visible = not self.metrics_visible
        if self.metrics_visible:
            self.metrics_panel.grid()
        else:
            self.metrics_panel.grid_remove()
        self.metrics_toggle_button.configure(text='收起指标' if self.metrics_visible else '实测指标')

    def refresh_metrics_panel(self, data, video, target, waiting):
        frame = self.video_panel._frame
        video = dict(video)
        if frame is not None:
            video.update(width=int(frame.shape[1]), height=int(frame.shape[0]))
        view = self.demo_metrics.observe(data, video, mode=target, waiting=waiting)
        self.metrics_title.set(view['title'])
        self.metrics_note.set('')
        self.metrics_note_label.grid_remove()
        # Hide unfilled display rows; keep measurements and exports unchanged.
        rows = [(name, text) for name, text in view['rows']
                if text not in ('—', '等待有效同版本样本', '未找到已通过编译的结果报告')
                and not str(text).startswith('待测')]
        if len(rows) != len(self.metrics_widgets):
            for label, value in self.metrics_widgets:
                label.destroy()
                value.destroy()
            self.metrics_widgets = []
            for index in range(len(rows)):
                label = ttk.Label(self.metrics_body, style='Subtle.TLabel', wraplength=265)
                value = ttk.Label(self.metrics_body, font=('Microsoft YaHei UI', 10, 'bold'),
                                  wraplength=265, justify='left')
                label.grid(row=index * 2, column=0, sticky='w', pady=(4, 0))
                value.grid(row=index * 2 + 1, column=0, sticky='w', pady=(0, 7))
                label.bind('<MouseWheel>', self.scroll_metrics)
                value.bind('<MouseWheel>', self.scroll_metrics)
                self.metrics_widgets.append((label, value))
        for (label, value), (name, text) in zip(self.metrics_widgets, rows):
            label.configure(text=str(name))
            value.configure(text=str(text))
        connected = bool(data and (data.get('connected') or {}).get('fpga'))
        self.metrics_start_button.configure(state='normal' if connected and not self.offline
            and not self.metrics_recording else 'disabled',
            text='正在记录…' if self.metrics_recording else '开始本段记录')
        self.metrics_export_button.configure(state='normal' if self.metrics_recording else 'disabled')

    def load_quality_metrics(self, role='corrected'):
        label = '矫正后' if role == 'corrected' else '原图'
        path = filedialog.askopenfilename(parent=self.window, title=f'载入{label}的画质测量 JSON',
            filetypes=[('画质结果 JSON', '*.json')])
        if not path:
            return None
        return self.demo_metrics.load_quality_report(path, role=role)

    def start_metrics_recording(self):
        if self.offline:
            raise ValueError('离线界面不能记录实板测量。')
        mode = self.demo_controller.confirmed_target if self.demo_controller else None
        message = self.demo_metrics.start_recording(self.snapshot_or_none(), mode=mode)
        self.metrics_recording = True
        self.metrics_start_button.configure(state='disabled', text='正在记录…')
        self.metrics_export_button.configure(state='normal')
        return message

    def export_metrics_recording(self):
        directory = filedialog.askdirectory(parent=self.window, title='选择本段实测证据的保存位置')
        if not directory:
            return None
        path = self.demo_metrics.export(directory, data=self.snapshot_or_none())
        self.metrics_recording = False
        self.metrics_export_button.configure(state='disabled')
        return f'本段记录已保存：{path}；包含原始日志、JSON 汇总和 CSV。'

    def build_settings(self):
        self.settings_page.columnconfigure(0, weight=1)
        self.settings_page.rowconfigure(0, weight=1)
        self.settings_canvas = tk.Canvas(self.settings_page, highlightthickness=0,
                                         background='#eef2f6', yscrollincrement=24)
        self.settings_scrollbar = ttk.Scrollbar(self.settings_page, orient='vertical',
                                                command=self.settings_canvas.yview)
        self.settings_canvas.configure(yscrollcommand=self.settings_scrollbar.set)
        self.settings_canvas.grid(row=0, column=0, sticky='nsew')
        self.settings_scrollbar.grid(row=0, column=1, sticky='ns')
        page = self.settings_body = ttk.Frame(self.settings_canvas, padding=20)
        self.settings_window = self.settings_canvas.create_window(0, 0, anchor='nw', window=page)
        page.bind('<Configure>', lambda event: self.settings_canvas.configure(
            scrollregion=self.settings_canvas.bbox('all')))
        self.settings_canvas.bind('<Configure>', lambda event: self.settings_canvas.itemconfigure(
            self.settings_window, width=event.width))
        page.columnconfigure(0, weight=1, uniform='settings')
        page.columnconfigure(1, weight=1, uniform='settings')
        ttk.Label(page, text='演示前设置一次', style='DemoTitle.TLabel').grid(row=0, column=0, columnspan=2, sticky='w', pady=(0, 16))
        devices = ttk.LabelFrame(page, text='设备连接', padding=16)
        devices.grid(row=1, column=0, columnspan=2, sticky='ew')
        ttk.Label(devices, text='FPGA').grid(row=0, column=0)
        self.demo_fpga_combo = ttk.Combobox(devices, textvariable=self.fpga_port, values=self.fpga_combo['values'], state='readonly', width=12)
        self.demo_fpga_combo.grid(row=0, column=1, padx=(8, 24))
        ttk.Label(devices, text='IMU').grid(row=0, column=2)
        self.demo_imu_combo = ttk.Combobox(devices, textvariable=self.imu_port, values=self.imu_combo['values'], state='readonly', width=12)
        self.demo_imu_combo.grid(row=0, column=3, padx=8)
        ttk.Button(devices, text='刷新', command=lambda: self.act(self.refresh_ports)).grid(row=0, column=4, padx=8)
        self.connect_button = ttk.Button(devices, text='连接设备', command=lambda: self.act(self.connect_or_disconnect))
        self.connect_button.grid(row=0, column=5)
        if self.offline:
            self.connect_button.configure(state='disabled')
        ttk.Label(devices, textvariable=self.port_note, wraplength=1150, style='Subtle.TLabel').grid(row=1, column=0, columnspan=6, sticky='w', pady=(10, 0))
        video = ttk.LabelFrame(page, text='画面来源', padding=16)
        video.grid(row=2, column=0, columnspan=2, sticky='ew', pady=12)
        ttk.Label(video, text='采集卡编号').grid(row=0, column=0)
        ttk.Spinbox(video, textvariable=self.video_panel.device_index, from_=0, to=255, width=5).grid(row=0, column=1, padx=8)
        self.preview_button = ttk.Button(video, text='打开实时画面', command=self.video_panel.open_camera)
        self.preview_button.grid(row=0, column=2, padx=8)
        if self.offline:
            self.preview_button.configure(state='disabled')
        ttk.Button(video, text='关闭画面', command=self.video_panel.stop).grid(row=0, column=3, padx=8)
        ttk.Button(video, text='本地照片…', command=self.video_panel.choose_image).grid(row=0, column=4, padx=8)
        ttk.Label(video, text='打开时请求原生 1280×720；实际帧与源图标识仍需校验。若 Windows“相机”占用采集卡，请先关闭它。', wraplength=1150, style='Subtle.TLabel').grid(row=1, column=0, columnspan=6, sticky='w', pady=(10, 0))
        imu = ttk.LabelFrame(page, text='IMU 标定', padding=16)
        imu.grid(row=3, column=0, sticky='nsew', padx=(0, 8))
        ttk.Button(imu, text='导入 IMU 标定…', command=lambda: self.act(self.load_mounting)).pack(anchor='w')
        self.demo_mount_note = tk.StringVar(value='尚未导入 IMU 标定。')
        ttk.Label(imu, textvariable=self.demo_mount_note, wraplength=530, style='Subtle.TLabel').pack(anchor='w', pady=8)
        ttk.Checkbutton(imu, text='当前安装与标定一致（相机与 IMU 固定在一起）', variable=self.mount_confirmed,
                        command=lambda: self.act(self.apply_mounting)).pack(anchor='w')
        ttk.Label(imu, text='首次开启 IMU 时，把相机摆正；程序记录此时方向并开始连续补偿。', wraplength=530).pack(anchor='w', pady=8)
        ttk.Button(imu, text='把当前方向重新设为正向', command=lambda: self.demo_call('reference')).pack(anchor='w')
        ttk.Label(imu, text='演示默认：纯 roll、中心 608×342 对应固定倍率 2.105263。\n整圈为离线整数抽样验证，实际候选仍由 FPGA 准入；full 三轴保留在高级设置。',
                  wraplength=530, style='Subtle.TLabel').pack(anchor='w', pady=8)
        scan = ttk.LabelFrame(page, text='自动平面扫描', padding=16)
        scan.grid(row=3, column=1, sticky='nsew', padx=(8, 0))
        self.demo_scan_note = tk.StringVar(value='10×5 内角点棋盘；固定目标位置、方向和格距。开启实时画面后选择“平面扫描”。')
        ttk.Label(scan, textvariable=self.demo_scan_note, wraplength=530, style='Subtle.TLabel').pack(anchor='w', pady=8)
        ttk.Label(scan, text='从 FPGA 源图小窗读取同帧标识并自动识别；持续提交最新有效矩阵。主输出不作为普通矫正图重复拟合。', wraplength=530).pack(anchor='w', pady=8)
        ttk.Label(scan, text='采集卡必须保留完整 HDMI 视野。失去棋盘或源图标识时保留上一映射并显示等待；静态参数导入保留在高级调试。', wraplength=530, style='Subtle.TLabel').pack(anchor='w')
        ttk.Button(page, text='返回演示', style='Primary.TButton', command=lambda: self.tabs.select(self.demo_page)).grid(row=4, column=0, sticky='w', pady=18)
        ttk.Button(page, text='导出各模式历史快照…', command=lambda: self.act(self.export_mode_results)).grid(row=4, column=1, sticky='e', pady=18)
        detail = ttk.LabelFrame(page, text='可选实验：FPGA 细节增强', padding=10)
        detail.grid(row=5, column=0, columnspan=2, sticky='ew')
        self.detail_enabled = tk.BooleanVar(value=False)
        self.detail_toggle = ttk.Checkbutton(detail, text='当前模式启用细节增强', variable=self.detail_enabled,
            state='disabled', command=lambda: self.demo_call('detail', self.detail_enabled.get()))
        self.detail_toggle.pack(anchor='w')
        self.detail_note = tk.StringVar(value='默认关闭；原图保持关闭。独立于自适应亮度，普通矫正的自适应仍禁用。实拍收益待验证。')
        ttk.Label(detail, textvariable=self.detail_note, wraplength=790, style='Subtle.TLabel').pack(anchor='w', pady=(6, 0))
        self.bind_settings_scrolling((devices, video, imu, scan, detail))

    def bind_settings_scrolling(self, sections):
        for section in sections:
            section.bind('<Configure>', self.wrap_settings_text)
        def bind_children(widget):
            # Bind only this page: wheel events elsewhere keep their own behavior.
            widget.bind('<MouseWheel>', self.scroll_settings)
            if widget is not self.settings_canvas:
                widget.bind('<FocusIn>', self.reveal_setting)
            for child in widget.winfo_children():
                bind_children(child)
        bind_children(self.settings_canvas)
        self.settings_canvas.bind('<Prior>', lambda event: self.settings_canvas.yview_scroll(-1, 'pages'))
        self.settings_canvas.bind('<Next>', lambda event: self.settings_canvas.yview_scroll(1, 'pages'))

    def wrap_settings_text(self, event):
        for child in event.widget.winfo_children():
            if isinstance(child, ttk.Label) and int(child.cget('wraplength') or 0) > 0:
                child.configure(wraplength=max(160, event.width - 36))

    def scroll_settings(self, event):
        canvas = self.settings_canvas
        if self.settings_body.winfo_height() > canvas.winfo_height() and event.delta:
            steps = max(1, abs(event.delta) // 120)
            canvas.yview_scroll(-steps if event.delta > 0 else steps, 'units')
        return 'break'

    def reveal_setting(self, event):
        # Keyboard navigation must also reach controls outside the current viewport.
        canvas = self.settings_canvas
        top = event.widget.winfo_rooty() - self.settings_body.winfo_rooty()
        bottom = top + event.widget.winfo_height()
        visible_top = canvas.canvasy(0)
        height = canvas.winfo_height()
        content_height = max(1, self.settings_body.winfo_height())
        if top < visible_top:
            canvas.yview_moveto(max(0, top - 12) / content_height)
        elif bottom > visible_top + height:
            canvas.yview_moveto((bottom - height + 12) / content_height)

    def show_advanced(self):
        if not hasattr(self, 'advanced_pages'):
            return
        self.advanced_visible = not self.advanced_visible
        for page in self.advanced_pages:
            if self.advanced_visible:
                self.tabs.add(page)
            else:
                self.tabs.hide(page)
        self.advanced_button.configure(text='收起高级调试' if self.advanced_visible else '高级调试')
        if not self.advanced_visible:
            self.tabs.select(self.demo_page)

    def load_preferences(self):
        self.settings_path = ROOT/'settings/demo_preferences.json'
        super().load_preferences()
        try:
            data = json.loads(self.settings_path.read_text(encoding='utf-8'))
            self.video_panel.preview_rotate_180.set(data.get('preview_rotate_180') is True)
        except (OSError, ValueError, TypeError, AttributeError):
            pass

    def save_preferences(self):
        super().save_preferences()
        data = json.loads(self.settings_path.read_text(encoding='utf-8'))
        data['preview_rotate_180'] = bool(self.video_panel.preview_rotate_180.get())
        temp = self.settings_path.with_suffix('.tmp')
        temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        temp.replace(self.settings_path)

    def change_preview_rotation(self):
        self.video_panel._redraw()
        self.save_preferences()
        direction = '旋转 180°' if self.video_panel.preview_rotate_180.get() else '原始方向'
        self.notify(f'观看方向：{direction}；扫描识别继续使用原始采集帧。')

    def demo_call(self, method, *args):
        if self.offline:
            self.notify('当前是离线界面预览；未连接设备，不发送控制指令。')
            return
        self.act(lambda: getattr(self.demo_controller, method)(*args))

    def demo_imu_settings(self, session):
        mode = self.pose_mode.get()
        if mode == 'full':
            return 'full', None
        self.pose_mode.set('roll_only')
        crop = self.crop_profile()
        if crop == DEMO_ROLL_CROP.to_dict():
            crop = checked_demo_crop(session.calibration, geometry_options=session.geometry_options)
        return 'roll_only', crop

    def start_visual_worker(self, session):
        from realtime_scan import VisionScanWorker
        self.stop_visual_worker()
        self.visual_worker = VisionScanWorker(session.scan_context(),
            submit_candidate=session.offer_visual_scan, report_state=session.report_visual_tracking)
        self.visual_worker.start()

    def stop_visual_worker(self):
        worker, self.visual_worker = self.visual_worker, None
        if worker is not None:
            worker.stop()
        self._vision_frame_id = None

    def reacquire_scan(self):
        if not self.session or not (self.session.snapshot().get('visual_scan') or {}).get('enabled'):
            raise RuntimeError('请先选择平面扫描模式。')
        self.start_visual_worker(self.session)
        self.notify('正在重新识别棋盘；保留上一有效映射，等待新的源图观测。')

    def export_mode_results(self):
        if not self.mode_results.results:
            raise RuntimeError('尚无已确认模式的实时采集快照。')
        path = filedialog.askdirectory(parent=self.window, title='保存各模式历史图像及来源记录')
        if path:
            count = self.mode_results.export(path)
            self.notify(f'已保存 {count} 组历史快照及来源记录；不是同帧五路对比。')

    def pump_visual_frames(self):
        if self.closing:
            return
        self.pump_recording()
        if self.closing:
            return
        if self.visual_worker is not None:
            packet = self.video_panel.get_frame_observation()
            if packet and packet['frame_id'] != self._vision_frame_id:
                self._vision_frame_id = packet['frame_id']
                self.visual_worker.offer_frame(packet)
        self.window.after(33, self.pump_visual_frames)

    def connect_or_disconnect(self):
        if self.session is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        # A selected file is not an initialized device profile. Preserve it
        # for later full context validation, never mark it already active.
        imported_profile = self.scan_profile if self.imported_scan_path else None
        super().connect()
        self.crop_enabled.set(True)
        self.crop_zoom.set(str(DEMO_ROLL_CROP.zoom))
        self.crop_range.set(str(DEMO_ROLL_CROP.max_abs_roll_deg))
        self.demo_controller.target = self.demo_controller.confirmed_target = None
        if imported_profile is not None:
            self.scan_profile = imported_profile

    def refresh_ports(self):
        super().refresh_ports()
        if hasattr(self, 'demo_fpga_combo'):
            self.demo_fpga_combo.configure(values=self.fpga_combo['values'])
            self.demo_imu_combo.configure(values=self.imu_combo['values'])

    def disconnect(self):
        if self.demo_controller is not None:
            self.demo_controller.stop()
        super().disconnect()
        if self.demo_controller is not None:
            self.demo_controller.target = self.demo_controller.confirmed_target = None

    def import_scan(self):
        from scan_plane import load_profile
        path = filedialog.askopenfilename(parent=self.window, title='导入已拟合的固定平面参数', filetypes=[('JSON 参数', '*.json')])
        if not path:
            return
        profile = load_profile(path)
        if not profile.get('scan_valid'):
            raise ValueError('此文件只有镜像参数，没有有效的扫描平面。')
        self.scan_profile = profile
        self.scan_validation = None
        self.imported_scan_path = str(Path(path).resolve())
        self.demo_scan_note.set(Path(path).name + ' · 已导入；开启扫描时校验并上传。')
        self.notify('已导入平面参数；尚未发送到 FPGA。')

    def notify(self, text):
        super().notify(text)

    def update(self):
        if self.closing:
            return
        super().update()
        if not hasattr(self, 'demo_mode'):
            return
        data = self.snapshot_or_none()
        actual_view = board_status(data)
        preview_crop = bool((data or {}).get('pose_mode') == 'full'
                            and not actual_view.get('raw_active')
                            and not actual_view.get('observation_active')
                            and not actual_view.get('committed_modes', 0))
        if self.video_panel.preview_full_rotation_crop != preview_crop:
            self.video_panel.preview_full_rotation_crop = preview_crop
            self.video_panel._redraw()
        if self.demo_controller is not None:
            self.demo_controller.observe()
        target = self.demo_controller.target if self.demo_controller else None
        waiting = bool(self.demo_controller and self.demo_controller.pending)
        name, evidence = display_status(data, target=(self.demo_controller.confirmed_target if self.demo_controller else None), waiting=waiting)
        if self.offline:
            name, evidence = '离线界面预览', '照片展示 · 未连接硬件'
        self.demo_mode.set(name)
        self.demo_evidence.set(evidence)
        video = self.video_panel.capture_status()
        self.device_status.set(independent_status(data, video))
        self.refresh_metrics_panel(data, video, target, waiting)
        busy = bool(self.demo_controller and self.demo_controller.pending) or bool(getattr(self, 'actions', None) and self.actions.pending)
        connected = bool(data and (data.get('connected') or {}).get('fpga'))
        state = 'normal' if connected and not busy and not self.offline else 'disabled'
        highlighted = confirmed_mode(data, self.demo_controller.confirmed_target
            if self.demo_controller else None, waiting=busy)
        for key, button in self.demo_buttons.items():
            button.configure(state=state, style='Confirmed.Mode.TButton'
                if key == highlighted else 'Mode.TButton')
        self.adaptive_toggle.configure(state=state if self.demo_controller and self.demo_controller.adaptive_allowed else 'disabled')
        if hasattr(self, 'detail_toggle'):
            self.detail_toggle.configure(state=state if self.demo_controller and self.demo_controller.detail_allowed else 'disabled')
            detail = (data or {}).get('detail') or {}
            detail_control = (data or {}).get('detail_control') or {}
            if not busy:
                self.detail_enabled.set(detail.get('active', False))
            actual = '开' if detail.get('active') else '关'
            shown = ((data or {}).get('scan') or {}).get('displayed') or {}
            current_tag = bool(shown and shown.get('version') == board_status(data).get('config_version')
                               and bool(shown.get('detail_active')) == bool(detail.get('active')))
            proof = ('当前 LAT 帧状态已确认' if current_tag else '以实际帧边界和 LAT 为准')
            self.detail_note.set(f'实际状态：{actual} · {proof}；固定增加 1656 像素时钟。独立于亮度增强，实拍收益待验证。'
                if detail.get('capable') else '默认关闭；需 C129 固件。原图保持关闭，普通矫正的自适应仍禁用。实拍收益待验证。')
        self.stop_button.configure(state='normal' if connected and not self.offline else 'disabled')
        if not busy:
            enhancement = (data or {}).get('enhancement') or {}
            self.adaptive_enabled.set(False if target == 'normal' else enhancement.get('enhancement_mode') == 1)
        self.reference_button.pack_forget()
        self.imu_comparison_button.pack_forget()
        self.reacquire_button.pack_forget()
        if target == 'imu':
            self.reference_button.configure(state=state)
            self.reference_button.pack(side='right', padx=12)
            self.imu_comparison_button.configure(state=state)
            self.imu_comparison_button.pack(side='right', padx=6)
        elif target == 'scan':
            self.reacquire_button.configure(state='normal' if connected and not self.offline else 'disabled')
            self.reacquire_button.pack(side='right', padx=12)
        if self.demo_controller and self.demo_controller.confirmed_target and not busy:
            actual = board_status(data)
            if (bool(actual.get('raw_active')) == (target == 'raw') and
                    bool(actual.get('observation_active')) == (target == 'scan')):
                packet = self.video_panel.get_frame_observation()
                adaptive = (data.get('enhancement') or {}).get('enhancement_mode') == 1
                if packet and packet['frame_id'] != self._history_frame_id and not (target == 'normal' and adaptive):
                    history_mode = ('imu_static_comparison' if not data.get('continuous') and
                        (data.get('request') or {}).get('request_kind') == 'imu_static_comparison' and
                        (data.get('request') or {}).get('phase') == 'output_confirmed' else target)
                    if self.mode_results.remember(history_mode, adaptive, packet, data):
                        self._history_frame_id = packet['frame_id']
        saved = len(self.mode_results.results)
        self.history_status.set(f'已保留 {saved} 组模式／增强快照 · 异步采集与串口状态，不是同帧五路输出；切走或取帧停止后为历史结果。')
        visual = (data or {}).get('visual_scan') or {}
        if visual.get('enabled'):
            tracking = visual.get('tracking') or {}
            self.demo_scan_note.set('连续扫描：' + str(tracking.get('state', '等待可追溯源图')) + '；' + str(tracking.get('reason', '等待棋盘观测与对应输出。')))
        if hasattr(self, 'connect_button'):
            self.connect_button.configure(text='断开设备' if self.session is not None else '连接设备')
            selector = 'disabled' if self.session is not None else 'readonly'
            self.demo_fpga_combo.configure(state=selector)
            self.demo_imu_combo.configure(state=selector)
            if self.last_mounting_path:
                confirmation = '安装已确认' if (data or {}).get('mounting_confirmed') else '请核对下方安装关系'
                self.demo_mount_note.set(Path(self.last_mounting_path).name + ' · ' + confirmation)
            else:
                self.demo_mount_note.set('尚未导入 IMU 标定。')

    def stop_all_requests(self):
        if self.demo_controller is not None:
            self.demo_controller.stop()
        super().stop_all_requests()

    def close(self):
        if self.preview_recorder is not None:
            self._close_after_recording = True
            self.preview_recorder.request_stop('关闭上位机，自动保存')
            self.recording_status.set('正在保存录像，完成后关闭…')
            return
        if self.metrics_recording:
            try:
                path = self.demo_metrics.export(ROOT / 'results' / 'demo_evidence', data=self.snapshot_or_none())
                self.metrics_recording = False
                self.notify(f'未导出的本段记录已自动保存：{path}')
            except Exception as error:
                from tkinter import messagebox
                messagebox.showerror('记录未保存', f'请先导出本段记录后再关闭：{error}', parent=self.window)
                return
        self.stop_visual_worker()
        super().close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--demo-image', type=Path)
    args = parser.parse_args(argv)
    root = tk.Tk()
    app = DemoControlPanel(root, offline=args.offline)
    if args.demo_image:
        root.after(150, lambda: app.act(lambda: app.video_panel.load_image(args.demo_image)))
    root.mainloop()


if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
