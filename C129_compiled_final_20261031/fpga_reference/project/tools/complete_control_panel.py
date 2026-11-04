"""C125 standalone GUI: parameter-only FPGA geometry and enhancement control.

No camera or serial device is opened at startup. USB preview is display-only;
all correction, sampling, interpolation and enhancement happen in the FPGA.
"""
import argparse
import json
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog

from complete_geometry import (DEFAULT_CALIBRATION, FOUR_POINT_OUTPUT, FOUR_POINT_SOURCE,
    IDENTITY_MATRIX, PRESETS, ROOT, check_geometry, configuration, four_point_matrix,
    load_calibration, pose_matrix, preset)
from complete_control import (ComparisonSession, MODE_NAMES, PublishedButNotDisplayed,
    confirm_display, export_configuration, read_status, send_enhancement)


class Panel:
    def __init__(self, app):
        self.app = app
        self.app.title('FPGA 完整版 C125 · 姿态 / 平面透视 / 增强')
        self.events, self.frames = queue.Queue(), queue.Queue(maxsize=1)
        self.busy = False
        self.prepared = None
        self.pending_display = None
        self.compare = ComparisonSession()
        self.capture_stop = threading.Event()
        self.photo = None
        self.port = tk.StringVar()
        self.calibration = tk.StringVar(value=str(DEFAULT_CALIBRATION))
        self.enhancement = tk.StringVar(value='auto')
        self.no_black = tk.BooleanVar(value=False)
        self.preset_name = tk.StringVar(value='safe_pose')
        self.camera_index = tk.StringVar(value='0')
        self.pose = {key: tk.StringVar(value=value) for key, value in
                     [('roll', '3'), ('pitch', '3'), ('yaw', '-3'), ('zoom', '1.3'),
                      ('tx', '.02'), ('ty', '-.01'), ('gain', '1'), ('offset', '0')]}
        outer = ttk.Frame(app, padding=10)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='C97 联合鱼眼＋切向 · 1280×720 · FPGA 处理', font=('', 13, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='虚拟视线 / 逆采样方向；姿态由用户提供。单个平面单应矩阵不能自动恢复任意三维场景。', wraplength=1100).pack(anchor='w')
        line = ttk.Frame(outer); line.pack(fill='x', pady=5)
        ttk.Label(line, text='标定').pack(side='left')
        ttk.Entry(line, textvariable=self.calibration).pack(side='left', fill='x', expand=True, padx=5)
        ttk.Button(line, text='选择', command=self.choose_calibration).pack(side='left')
        line = ttk.Frame(outer); line.pack(fill='x')
        ttk.Label(line, text='板载串口').pack(side='left')
        self.ports = ttk.Combobox(line, textvariable=self.port, width=12, state='readonly'); self.ports.pack(side='left', padx=5)
        ttk.Button(line, text='刷新端口', command=self.refresh_ports).pack(side='left')
        ttk.Button(line, text='读取 C125 状态', command=self.status).pack(side='left', padx=5)
        ttk.Combobox(line, textvariable=self.preset_name, values=tuple(PRESETS), width=22, state='readonly').pack(side='left')
        ttk.Button(line, text='加载预设', command=self.load_preset).pack(side='left', padx=5)
        self.tabs = ttk.Notebook(outer); self.tabs.pack(fill='x', pady=7)
        pose_tab = ttk.Frame(self.tabs, padding=8); matrix_tab = ttk.Frame(self.tabs, padding=8); points_tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(pose_tab, text='姿态 / 平移 / 缩放'); self.tabs.add(matrix_tab, text='3×3 逆投影矩阵'); self.tabs.add(points_tab, text='目标平面四点')
        names = {'roll':'Roll Z(度)', 'pitch':'Pitch X(度)', 'yaw':'Yaw Y(度)', 'zoom':'Zoom', 'tx':'Tx(归一平面)', 'ty':'Ty(归一平面)', 'gain':'亮度增益', 'offset':'整数偏移'}
        for i, (key, value) in enumerate(self.pose.items()):
            cell = ttk.Frame(pose_tab); cell.grid(row=i//4, column=i%4, sticky='w', padx=6, pady=4)
            ttk.Label(cell, text=names[key]).pack(side='left')
            ttk.Entry(cell, textvariable=value, width=9).pack(side='left', padx=4)
        ttk.Label(pose_tab, text='X 向右，Y 向下，Z 向前；H=T·Rz·Ry·Rx。Tx/Ty 是归一平面位移，不是米。旧 cos/sin 默认单位旋转。').grid(row=2, column=0, columnspan=4, sticky='w')
        ttk.Button(pose_tab, text='将姿态矩阵填入矩阵页', command=self.pose_to_matrix).grid(row=3, column=0, columnspan=2, sticky='w', pady=5)
        ttk.Label(matrix_tab, text='行优先 3×3，输出归一平面 → 源去畸变平面；随后 FPGA 执行 X/Z、Y/Z、鱼眼、编译切向和源 K。').pack(anchor='w')
        self.matrix = tk.Text(matrix_tab, height=4, width=95); self.matrix.pack(fill='x')
        self._set_text(self.matrix, [[1,0,0],[0,1,0],[0,0,1]])
        ttk.Label(matrix_tab, text='矩阵页使用上方姿态页的 Zoom、亮度增益和偏移；Roll/Pitch/Yaw/Tx/Ty 不再叠加。').pack(anchor='w')
        ttk.Label(points_tab, text='只能填写同一目标平面的归一化去畸变点，不是原始鱼眼图片像素。顺序按对应关系一致。', foreground='#8a3c00').pack(anchor='w')
        row = ttk.Frame(points_tab); row.pack(fill='x')
        left = ttk.Frame(row); left.pack(side='left', fill='x', expand=True)
        right = ttk.Frame(row); right.pack(side='left', fill='x', expand=True, padx=8)
        ttk.Label(left, text='目标 / 输出平面四点 JSON').pack(anchor='w')
        self.output_points = tk.Text(left, height=4, width=42); self.output_points.pack(fill='x')
        ttk.Label(right, text='源 / 去畸变平面四点 JSON').pack(anchor='w')
        self.source_points = tk.Text(right, height=4, width=42); self.source_points.pack(fill='x')
        self._set_text(self.output_points, FOUR_POINT_OUTPUT); self._set_text(self.source_points, FOUR_POINT_SOURCE)
        ttk.Button(points_tab, text='求逆单应矩阵并填入矩阵页', command=self.points_to_matrix).pack(anchor='w', pady=4)
        actions = ttk.Frame(outer); actions.pack(fill='x')
        ttk.Checkbutton(actions, text='拒绝任何无效黑边（不自动裁剪）', variable=self.no_black).pack(side='left')
        ttk.Button(actions, text='定点预算检查', command=self.prepare).pack(side='left', padx=5)
        ttk.Button(actions, text='导出 27 参数', command=self.export).pack(side='left')
        ttk.Button(actions, text='发送并确认 FPGA 输出', command=self.send).pack(side='left', padx=5)
        ttk.Button(actions, text='仅重查输出（不重发）', command=self.retry_display).pack(side='left')
        line = ttk.Frame(outer); line.pack(fill='x', pady=5)
        ttk.Button(line, text='FPGA 原图对比', command=self.show_raw).pack(side='left')
        ttk.Button(line, text='恢复已保存矫正', command=self.restore).pack(side='left', padx=5)
        ttk.Label(line, text='增强模式').pack(side='left')
        ttk.Combobox(line, textvariable=self.enhancement, values=MODE_NAMES, width=14, state='readonly').pack(side='left', padx=5)
        ttk.Button(line, text='应用增强并确认', command=self.send_enhance).pack(side='left')
        ttk.Label(outer, text='auto：自适应 Gamma；histogram：自适应 Gamma＋低对比度增强；night / highlight：固定提亮 / 压暗；bypass：关闭增强。').pack(anchor='w')
        line = ttk.Frame(outer); line.pack(fill='x', pady=5)
        ttk.Label(line, text='HDMI 采集卡索引').pack(side='left')
        ttk.Entry(line, textvariable=self.camera_index, width=5).pack(side='left', padx=5)
        ttk.Button(line, text='打开仅显示预览', command=self.start_preview).pack(side='left')
        ttk.Button(line, text='关闭预览', command=self.stop_preview).pack(side='left', padx=5)
        self.preview = ttk.Label(outer, text='预览未打开；启动不访问摄像头或串口。', anchor='center'); self.preview.pack(fill='x')
        self.log = tk.Text(outer, height=7, width=120, wrap='word'); self.log.pack(fill='both', expand=True, pady=5)
        self.note('等待操作。定点检查只用于离线验证，不会上传坐标表。完整模式应使用本控制台。')
        app.protocol('WM_DELETE_WINDOW', self.close)
        app.after(50, self.poll)

    @staticmethod
    def _set_text(widget, value):
        widget.delete('1.0', 'end'); widget.insert('1.0', json.dumps(value, ensure_ascii=False))

    def note(self, value):
        self.log.insert('end', str(value)+'\n'); self.log.see('end')

    def run(self, action):
        if self.busy:
            self.note('当前操作尚未结束。'); return
        self.busy = True
        def worker():
            try:
                result = action()
                self.events.put(('log', result if isinstance(result, str) else json.dumps(result, ensure_ascii=False, indent=2)))
                if isinstance(result, dict) and (result.get('display_confirmed') or
                        result.get('geometry', {}).get('display_confirmed')):
                    self.events.put(('pending_display', None))
            except PublishedButNotDisplayed as error:
                self.events.put(('pending_display', (error.port_name, error.publication)))
                self.events.put(('log', str(error)))
            except Exception as error:
                self.events.put(('log', '错误：'+str(error)))
            finally:
                self.events.put(('done', None))
        threading.Thread(target=worker, daemon=True).start()

    def choose_calibration(self):
        value = filedialog.askopenfilename(filetypes=[('Calibration JSON', '*.json')])
        if value: self.calibration.set(value)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports() if p.vid == 0x10c4 and p.pid == 0xea60]
            self.ports['values'] = ports
            if len(ports) == 1: self.port.set(ports[0])
            self.note('CP210x 端口：'+', '.join(ports))
        except Exception as error: self.note(error)

    def collect(self):
        calibration = load_calibration(self.calibration.get())
        values = {key: float(value.get()) for key, value in self.pose.items()}
        matrix = None
        index = self.tabs.index(self.tabs.select())
        if index == 1: matrix = json.loads(self.matrix.get('1.0', 'end'))
        if index == 2: matrix = four_point_matrix(json.loads(self.output_points.get('1.0', 'end')), json.loads(self.source_points.get('1.0', 'end')))
        return configuration(calibration, matrix=matrix, **values)

    def _checked_action(self, callback):
        try:
            words, allow_black = self.collect(), not self.no_black.get()
            port_name = self.port.get()
        except Exception as error:
            self.note('错误：'+str(error)); return
        self.run(lambda: callback(words, allow_black, port_name))

    def prepare(self):
        self._checked_action(lambda words, allow, port: check_geometry(tuple(words), allow_black=allow))

    def export(self):
        folder = filedialog.askdirectory(initialdir=ROOT/'output')
        if folder:
            self._checked_action(lambda words, allow, port: export_configuration(words, folder, allow_black=allow))

    def send(self):
        if self.pending_display:
            self.note('上次映射已经发布。请先点击“仅重查输出（不重发）”，避免重复提交。'); return
        self._checked_action(lambda words, allow, port: self.compare.apply(port, words, allow_black=allow))

    def retry_display(self):
        if not self.pending_display:
            self.note('没有等待输出确认的已发布配置。'); return
        port_name, publication = self.pending_display
        self.run(lambda: confirm_display(port_name, publication))

    def status(self):
        port_name = self.port.get(); self.run(lambda: read_status(port_name))

    def show_raw(self):
        port_name = self.port.get(); self.run(lambda: self.compare.show_raw(port_name))

    def restore(self):
        port_name = self.port.get(); self.run(lambda: self.compare.restore(port_name))

    def send_enhance(self):
        port_name, mode = self.port.get(), self.enhancement.get()
        self.run(lambda: send_enhancement(port_name, mode))

    def pose_to_matrix(self):
        try:
            values = {key: float(self.pose[key].get()) for key in ('roll', 'pitch', 'yaw', 'tx', 'ty')}
            self._set_text(self.matrix, pose_matrix(**values).tolist()); self.tabs.select(1)
        except Exception as error: self.note(error)

    def points_to_matrix(self):
        try:
            matrix = four_point_matrix(json.loads(self.output_points.get('1.0', 'end')), json.loads(self.source_points.get('1.0', 'end')))
            self._set_text(self.matrix, matrix.tolist()); self.tabs.select(1)
        except Exception as error: self.note(error)

    def load_preset(self):
        name = self.preset_name.get()
        if name == 'raw':
            self.note('原图预设请用 FPGA 原图对比按钮；先成功发送矫正参数建立恢复点。'); return
        settings = PRESETS[name]
        for key in self.pose:
            self.pose[key].set(str(settings.get(key, 1 if key in ('zoom', 'gain') else 0)))
        if 'matrix' in settings:
            self._set_text(self.matrix, settings['matrix']); self.tabs.select(1)
        else: self.tabs.select(0)
        self.note('已加载 '+name+'；尚未发送，需定点预算检查。')

    def start_preview(self):
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive(): return
        try: index = int(self.camera_index.get())
        except ValueError: self.note('采集卡索引必须为整数。'); return
        self.capture_stop.clear()
        def worker():
            capture = None
            try:
                import cv2
                capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
                if not capture.isOpened(): raise RuntimeError('无法打开所选 HDMI 采集卡。')
                while not self.capture_stop.is_set():
                    ok, frame = capture.read()
                    if not ok: raise RuntimeError('采集卡未返回画面。')
                    try: self.frames.put_nowait(frame)
                    except queue.Full: pass
            except Exception as error: self.events.put(('log', str(error)))
            finally:
                if capture is not None: capture.release()
        self.capture_thread = threading.Thread(target=worker, daemon=True); self.capture_thread.start()

    def stop_preview(self):
        self.capture_stop.set(); self.preview.configure(image='', text='预览已停止。')

    def poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'done': self.busy = False
                elif kind == 'pending_display': self.pending_display = value
                else: self.note(value)
        except queue.Empty: pass
        try:
            frame = self.frames.get_nowait()
            if not self.capture_stop.is_set():
                import cv2
                from PIL import Image, ImageTk
                image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)); image.thumbnail((850, 320))
                self.photo = ImageTk.PhotoImage(image); self.preview.configure(image=self.photo, text='')
        except queue.Empty: pass
        self.app.after(50, self.poll)

    def close(self):
        self.capture_stop.set(); self.app.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    app = tk.Tk()
    if args.self_check:
        app.withdraw(); Panel(app); app.update_idletasks(); app.destroy()
        print('COMPLETE CONTROL PANEL SELF CHECK PASS (no camera/serial opened)')
    else:
        Panel(app); app.mainloop()


if __name__ == '__main__':
    main()
