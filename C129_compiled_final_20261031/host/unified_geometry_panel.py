"""Embedded Tk file-only geometry workspace for the unified C127 application."""
from __future__ import annotations

from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk

from unified_geometry import (GeometryCancelled, GeometryOptions, ResultGeneration,
                              process_image, read_image, save_result)


class GeometryPanel(ttk.Frame):
    def __init__(self, master, on_status=None):
        super().__init__(master, padding=12)
        self.on_status = on_status
        self.guard = ResultGeneration()
        self._source = None
        self._result = None
        self._display = None
        self._photo = None
        self._cancel = threading.Event()
        self._stop = threading.Event()
        self._jobs = queue.Queue(maxsize=1)
        self._results = queue.Queue()
        self._worker = None
        self._after_id = None
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)
        ttk.Label(self, text='离线几何工作区', style='Heading.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(self, text='只读取图片文件；软件预览不上传参数，不改变当前 FPGA 画面。',
                  wraplength=1050).grid(row=1, column=0, sticky='w', pady=(2, 8))

        controls = ttk.Frame(self)
        controls.grid(row=2, column=0, sticky='ew')
        controls.columnconfigure(1, weight=1)
        ttk.Button(controls, text='打开图片…', command=self._choose).grid(row=0, column=0, sticky='w')
        self.source_text = tk.StringVar(value='支持完整 1280×720 或同视野 1920×1080 图片')
        ttk.Label(controls, textvariable=self.source_text, wraplength=880).grid(row=0, column=1, columnspan=7, sticky='w', padx=10)

        self.domain = tk.StringVar(value='原始鱼眼图')
        self.operation = tk.StringVar(value='棋盘平面转正')
        self.columns = tk.StringVar(value='10')
        self.rows = tk.StringVar(value='5')
        self.cell = tk.StringVar(value='64')
        self.roll = tk.StringVar(value='0')
        self.zoom = tk.StringVar(value='1.0')
        ttk.Label(controls, text='输入类型').grid(row=1, column=0, sticky='w', pady=8)
        ttk.Combobox(controls, textvariable=self.domain, values=('原始鱼眼图', '已去镜头畸变图'),
                     state='readonly', width=20).grid(row=1, column=1, sticky='w')
        ttk.Label(controls, text='处理').grid(row=1, column=2, padx=(16, 5))
        ttk.Combobox(controls, textvariable=self.operation,
                     values=('棋盘平面转正', '镜头去畸变', '离线旋转与固定裁边'),
                     state='readonly', width=24).grid(row=1, column=3, columnspan=3, sticky='w')
        ttk.Label(controls, text='内角点 列×行').grid(row=2, column=0, sticky='w')
        pattern = ttk.Frame(controls)
        pattern.grid(row=2, column=1, sticky='w')
        ttk.Spinbox(pattern, from_=2, to=20, textvariable=self.columns, width=4).pack(side='left')
        ttk.Label(pattern, text=' × ').pack(side='left')
        ttk.Spinbox(pattern, from_=2, to=20, textvariable=self.rows, width=4).pack(side='left')
        ttk.Label(controls, text='输出格距 px').grid(row=2, column=2, padx=(16, 5))
        ttk.Entry(controls, textvariable=self.cell, width=7).grid(row=2, column=3, sticky='w')
        ttk.Label(controls, text='旋转角度 °').grid(row=2, column=4, padx=(16, 5))
        ttk.Entry(controls, textvariable=self.roll, width=8).grid(row=2, column=5, sticky='w')
        ttk.Label(controls, text='固定倍率').grid(row=2, column=6, padx=(16, 5))
        ttk.Entry(controls, textvariable=self.zoom, width=7).grid(row=2, column=7, sticky='w')

        actions = ttk.Frame(self)
        actions.grid(row=3, column=0, sticky='ew', pady=9)
        ttk.Button(actions, text='运行离线预览', command=self.run).pack(side='left')
        ttk.Button(actions, text='取消', command=self.cancel).pack(side='left', padx=6)
        self.view = tk.StringVar(value='处理后')
        selector = ttk.Combobox(actions, state='readonly', width=18, textvariable=self.view,
                                values=('输入原图', '镜头矫正后', '处理后', '处理后＋左右镜像'))
        selector.pack(side='left', padx=8)
        selector.bind('<<ComboboxSelected>>', lambda event: self._show_selected())
        ttk.Button(actions, text='前／后对比', command=self.toggle_compare).pack(side='left')
        ttk.Button(actions, text='保存图片与报告…', command=self.save).pack(side='right')
        self.canvas = tk.Canvas(self, background='#111c2b', highlightthickness=0, height=320)
        self.canvas.grid(row=4, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', lambda event: self._redraw())
        self.status = tk.StringVar(value='尚未加载图片。镜头参数沿用 C127 的 C97_joint_720p。')
        ttk.Label(self, textvariable=self.status, wraplength=1080).grid(row=5, column=0, sticky='ew', pady=(8, 4))
        self.details = tk.Text(self, height=5, wrap='word', state='disabled', font=('Microsoft YaHei UI', 10))
        self.details.grid(row=6, column=0, sticky='ew')
        self._set_details('棋盘转正会保留整张 1280×720 画布，非共面背景可能拉伸。\n'
                          '已去畸变的图片只做像素域变换；离线旋转候选不表示现有 C127 已支持连续 360°。')
        for variable in (self.domain, self.operation, self.columns, self.rows, self.cell, self.roll, self.zoom):
            variable.trace_add('write', self._settings_changed)
        self._after_id = self.after(100, self._poll)

    def _notify(self, text):
        self.status.set(text)
        if self.on_status is not None:
            self.on_status(text)

    def _set_details(self, text):
        self.details.configure(state='normal')
        self.details.delete('1.0', 'end')
        self.details.insert('1.0', text)
        self.details.configure(state='disabled')

    def _invalidate(self):
        self._cancel.set()
        token = self.guard.next()
        self._result = None
        while True:
            try:
                self._jobs.get_nowait()
            except queue.Empty:
                break
        return token

    def _settings_changed(self, *_):
        if self.guard.closed:
            return
        self._invalidate()
        if self._source is not None:
            self._notify('参数已改变，请重新运行离线预览。')
            self._set_details('当前显示保留上次图像；结果与导出已失效，重新运行后再比较。')

    def _choose(self):
        path = filedialog.askopenfilename(parent=self.winfo_toplevel(), title='打开完整画面',
                                          filetypes=[('图片', '*.png *.jpg *.jpeg *.bmp *.tif *.tiff'), ('所有文件', '*.*')])
        if path:
            self.load_image(path)

    def load_image(self, path):
        try:
            image, _ = read_image(path)
            self._invalidate()
            self._source = Path(path)
            self._display = image
            self.source_text.set(str(self._source))
            self.view.set('输入原图')
            self._redraw()
            self._notify('图片已加载。请选择它是否已经去过镜头畸变，再运行预览。')
            self._set_details('没有访问设备或上传参数。请确认图片来自同一镜头、同一完整视野。')
        except Exception as error:
            self._notify(str(error))
            messagebox.showerror('图片未加载', str(error), parent=self.winfo_toplevel())

    def _geometry_options(self):
        return GeometryOptions(input_domain={'原始鱼眼图': 'raw', '已去镜头畸变图': 'rectified'}[self.domain.get()],
                               operation={'棋盘平面转正': 'scan', '镜头去畸变': 'lens', '离线旋转与固定裁边': 'roll'}[self.operation.get()],
                               columns=int(self.columns.get()), rows=int(self.rows.get()),
                               cell_pixels=float(self.cell.get()), roll_degrees=float(self.roll.get()),
                               zoom=float(self.zoom.get())).validate()

    def run(self):
        if self._source is None:
            self._notify('请先打开一张完整图片。')
            return
        try:
            options = self._geometry_options()
        except (ValueError, KeyError) as error:
            self._notify(str(error))
            messagebox.showerror('参数无效', str(error), parent=self.winfo_toplevel())
            return
        token = self._invalidate()
        self._cancel = threading.Event()
        self._jobs.put((token, self._source, options, self._cancel))
        if self._worker is None:
            self._worker = threading.Thread(target=self._work, daemon=True, name='OfflineGeometryFiles')
            self._worker.start()
        self._notify('正在计算文件预览与输出角点重检；可以切换其他页面。')

    def _work(self):
        while not self._stop.is_set():
            try:
                token, path, options, cancel = self._jobs.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                value = process_image(path, options, cancelled=cancel.is_set)
                if not cancel.is_set() and not self._stop.is_set():
                    self._results.put((token, value, None))
            except GeometryCancelled:
                continue
            except Exception as error:
                if not cancel.is_set() and not self._stop.is_set():
                    self._results.put((token, None, str(error)))

    def _poll(self):
        if self.guard.closed:
            return
        while True:
            try:
                token, value, error = self._results.get_nowait()
            except queue.Empty:
                break
            if not self.guard.accepts(token):
                continue
            if error:
                self._notify('离线处理未完成：' + error)
                self._set_details(error)
                continue
            self._result = value
            self.view.set('处理后')
            self._show_selected()
            report = value.report
            details = ['文件计算完成；结果未上传 FPGA，原校准未改变。']
            metrics = report.get('after_metrics')
            if metrics:
                details.append(f"重检 {report['redetected_corner_count']} 个角点；横/竖平均格距 "
                               f"{metrics['horizontal_step_mean_px']:.2f} / {metrics['vertical_step_mean_px']:.2f} px；"
                               f"平均直角偏差 {metrics['mean_absolute_right_angle_error_deg']:.2f}°。")
                details.append(f"横/竖格距变异系数 {metrics['horizontal_step_cv_percent']:.2f}% / "
                               f"{metrics['vertical_step_cv_percent']:.2f}%。指标来自输出图重新检测。")
            elif report.get('redetection_warning'):
                details.append('输出角点重检失败，未生成格距指标：' + report['redetection_warning'])
            info = report['output_geometry']
            if 'valid_fraction' in info:
                details.append(f"整画布有效取样比例约 {100*info['valid_fraction']:.2f}%；黑色区域保留。")
            if info.get('map_guard_admitted') is False:
                details.append('当前全画布不满足地图/缓存准入；仅供观察，不能作为上板参数。')
            if report['options']['input_domain'] == 'rectified':
                details.append('已矫正输入没有再次进行镜头去畸变；没有导出原始相机 H。')
            self._set_details('\n'.join(details))
            self._notify('离线预览完成，可切换前后、查看镜像或保存结果。')
        self._after_id = self.after(100, self._poll)

    def _show_selected(self):
        if self._result is None:
            return
        key = {'输入原图': 'input', '镜头矫正后': 'lens', '处理后': 'output', '处理后＋左右镜像': 'mirror'}[self.view.get()]
        self._display = self._result.images[key]
        self._redraw()

    def toggle_compare(self):
        if self._result is None:
            self._notify('请先运行离线预览。')
            return
        self.view.set('处理后' if self.view.get() == '输入原图' else '输入原图')
        self._show_selected()

    def _redraw(self):
        self.canvas.delete('all')
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if width < 2 or height < 2:
            return
        if self._display is None:
            self.canvas.create_text(width/2, height/2, text='打开一张图片，开始离线验证',
                                    fill='#aebed0', font=('Microsoft YaHei UI', 17))
            return
        rgb = cv2.cvtColor(self._display, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        scale = min(width/image.width, height/image.height)
        image = image.resize((max(1, round(image.width*scale)), max(1, round(image.height*scale))), Image.Resampling.LANCZOS)
        self._photo = ImageTk.PhotoImage(image, master=self.canvas)
        self.canvas.create_image(width/2, height/2, image=self._photo, anchor='center')

    def save(self):
        if self._result is None:
            self._notify('没有可保存的当前结果，请先运行预览。')
            return
        directory = filedialog.askdirectory(parent=self.winfo_toplevel(), title='选择结果存放目录')
        if directory:
            try:
                target = save_result(self._result, directory)
                self._notify('图片与报告已保存到：' + str(target))
            except Exception as error:
                self._notify('保存失败：' + str(error))

    def cancel(self):
        if not self.guard.closed:
            self._invalidate()
            self._notify('已取消；正在执行的计算结束后会丢弃旧结果。')

    def close(self):
        if self.guard.closed:
            return
        self._cancel.set()
        self._stop.set()
        self.guard.close()
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
        while True:
            try:
                self._jobs.get_nowait()
            except queue.Empty:
                break
        self._result = None
        self._display = None
        self._photo = None
