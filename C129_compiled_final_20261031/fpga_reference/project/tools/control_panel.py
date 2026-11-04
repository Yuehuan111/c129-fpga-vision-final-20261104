"""Local capture/calibration/configuration GUI for the MES50HP FPGA system.
The preview is HDMI capture-card video; no PC undistortion is applied to it.
Nothing is sent to a serial port until the user clicks a Send button.
"""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk
from serial.tools import list_ports
from host_actions import ROOT, FISHEYE_IDENTITY as IDENTITY, configuration, load_calibration, fitted_configuration
from host_actions import send_configuration, export_boot, brightness_configuration
from advanced_control import send_enhancement, enhancement_status
from project_profile import BUILD_ID

GEOMETRY_PRESETS = {
    '居中 0° / zoom 1.00': (0, 1.0),
    '左旋 -10° / zoom 1.20': (-10, 1.2),
    '右旋 +10° / zoom 1.20': (10, 1.2),
    '居中放大 0° / zoom 1.25': (0, 1.25),
}
GAMMA_LABELS = {'旁路': 'bypass', '自动': 'auto', '夜间': 'night', '强光': 'highlight'}
PREVIEW_SHAPES = ((720, 1280), (1080, 1920))

class Panel:
    def __init__(self, app):
        self.app = app
        app.title('盘古50K · 720p30 C97高级版 A001（FPGA输出采集）')
        app.geometry('1120x950')
        self.events = queue.Queue()
        self.frames = queue.Queue(maxsize=1)
        self.stop_capture = threading.Event()
        self.worker = None
        self.task_busy = False
        self.latest = None
        self.prepared = None
        self.session = None
        from pygrabber.dshow_graph import FilterGraph
        devices = FilterGraph().get_input_devices()
        self.device = tk.StringVar(value=str(devices.index('USB Video')) if 'USB Video' in devices else '')
        self.port = tk.StringVar()
        self.cols = tk.StringVar(value='10')
        self.rows = tk.StringVar(value='5')
        self.square = tk.StringVar(value='20')
        c97=ROOT/'calibration/C97_720p_derived/calibration.json'
        self.calibration = tk.StringVar(value=str(c97) if c97.is_file() else '')
        self.zoom = tk.StringVar(value='1.00')
        self.angle = tk.StringVar(value='0')
        self.gain_q8 = tk.StringVar(value='256')
        self.offset = tk.StringVar(value='0')
        self.gamma_mode = tk.StringVar(value='旁路')
        self.preset = tk.StringVar(value=next(iter(GEOMETRY_PRESETS)))
        self.is_raw = tk.BooleanVar(value=False)
        self.crop = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='先下载本目录高级版A001位流；启动时不发送参数。拍标定图前切原图并将Gamma设为旁路。')
        top = ttk.LabelFrame(app, text='1. 采集卡：这里只显示 FPGA 的 HDMI 输出，不在电脑上矫正图像')
        top.pack(fill='x', padx=8, pady=5)
        ttk.Label(top, text='视频设备编号').grid(row=0, column=0)
        ttk.Spinbox(top, from_=0, to=9, width=5, textvariable=self.device).grid(row=0, column=1)
        ttk.Button(top, text='打开采集卡', command=self.open_capture).grid(row=0, column=2, padx=5)
        ttk.Button(top, text='关闭采集卡', command=self.close_capture).grid(row=0, column=3)
        ttk.Button(top, text='新建标定图组', command=self.new_session).grid(row=0, column=4, padx=5)
        ttk.Button(top, text='保存一张原图', command=self.save_frame).grid(row=0, column=5)
        ttk.Checkbutton(top, text='确认当前是原图模式，未缩放/裁剪/旋转', variable=self.is_raw).grid(row=1, column=0, columnspan=6, sticky='w')
        self.preview = ttk.Label(app, text='预览区（采集卡编号不一定是 0；0 可能是笔记本摄像头）', anchor='center')
        self.preview.pack(fill='both', expand=True, padx=8)
        group = ttk.LabelFrame(app, text='2. 真实镜头标定：拍摄20–30张不同位置、距离和倾斜角度的棋盘格')
        group.pack(fill='x', padx=8, pady=5)
        for col, (label, var) in enumerate((('内角点列',self.cols),('内角点行',self.rows),('格长mm',self.square))):
            ttk.Label(group,text=label).grid(row=0,column=col*2)
            ttk.Entry(group,textvariable=var,width=6).grid(row=0,column=col*2+1)
        ttk.Button(group,text='标定当前图组',command=self.calibrate).grid(row=0,column=6,padx=8)
        ttk.Button(group,text='选择已有标定JSON',command=self.choose_calibration).grid(row=0,column=7)
        ttk.Label(group,textvariable=self.calibration).grid(row=1,column=0,columnspan=8,sticky='w')
        cfg = ttk.LabelFrame(app,text='3. 给FPGA发送参数（USB-UART，115200/8N1）')
        cfg.pack(fill='x',padx=8,pady=5)
        self.ports = ttk.Combobox(cfg,textvariable=self.port,width=10,state='readonly')
        self.ports.grid(row=0,column=0)
        ttk.Button(cfg,text='刷新COM',command=self.refresh_ports).grid(row=0,column=1)
        ttk.Label(cfg,text='zoom').grid(row=0,column=2)
        ttk.Entry(cfg,textvariable=self.zoom,width=6).grid(row=0,column=3)
        ttk.Label(cfg,text='角度°').grid(row=0,column=4)
        ttk.Entry(cfg,textvariable=self.angle,width=6).grid(row=0,column=5)
        ttk.Checkbutton(cfg,text='自动裁剪，消除几何越界黑边',variable=self.crop).grid(row=0,column=6)
        ttk.Button(cfg,text='切到矫正图（发送参数）',command=self.send_corrected).grid(row=1,column=0,columnspan=3,pady=5)
        ttk.Button(cfg,text='切到原图（单位映射）',command=self.send_identity).grid(row=1,column=3,columnspan=2)
        ttk.Button(cfg,text='写入最近参数为上电默认值',command=self.boot).grid(row=1,column=5,columnspan=2)
        ttk.Button(cfg,text='读取板上帧率和状态',command=self.read_status).grid(row=2,column=0,columnspan=3,pady=5)
        ttk.Button(cfg,text='诊断变形（不是镜头标定）',command=self.send_diagnostic).grid(row=2,column=3,columnspan=4)
        ttk.Label(cfg,text='几何预设').grid(row=3,column=0)
        ttk.Combobox(cfg,textvariable=self.preset,values=list(GEOMETRY_PRESETS),state='readonly',width=28).grid(row=3,column=1,columnspan=3)
        ttk.Button(cfg,text='仅填入角度/zoom',command=self.fill_preset).grid(row=3,column=4,columnspan=2)
        ttk.Label(cfg,text='画面内旋转/缩放；新几何由FPGA重建映射，约秒级，最近几何可命中缓存。').grid(row=4,column=0,columnspan=7,sticky='w')
        ttk.Label(cfg,text='gain Q8').grid(row=5,column=0)
        ttk.Spinbox(cfg,textvariable=self.gain_q8,from_=0,to=2048,width=7).grid(row=5,column=1)
        ttk.Label(cfg,text='offset').grid(row=5,column=2)
        ttk.Spinbox(cfg,textvariable=self.offset,from_=-255,to=255,width=7).grid(row=5,column=3)
        ttk.Button(cfg,text='仅发送亮度（保留最近已发送几何）',command=self.send_brightness).grid(row=5,column=4,columnspan=3)
        ttk.Label(cfg,text='Gamma').grid(row=6,column=0)
        ttk.Combobox(cfg,textvariable=self.gamma_mode,values=list(GAMMA_LABELS),state='readonly',width=9).grid(row=6,column=1)
        ttk.Button(cfg,text='发送Gamma模式并确认',command=self.send_gamma).grid(row=6,column=2,columnspan=3)
        ttk.Label(cfg,text='gain 256 = 1倍；亮度使用现有18字提交，Gamma独立按帧边界生效。').grid(row=7,column=0,columnspan=7,sticky='w')
        ttk.Label(app,textvariable=self.status,wraplength=720).pack(fill='x',padx=8)
        self.log = tk.Text(app,height=7,wrap='word')
        self.log.pack(fill='x',padx=8,pady=5)
        self.refresh_ports()
        self.new_session()
        app.after(40,self.poll)
        app.protocol('WM_DELETE_WINDOW',self.close)

    def note(self, text):
        self.status.set(str(text));self.log.insert('end',str(text)+'\n');self.log.see('end')

    def run_task(self, fn):
        if self.task_busy:
            self.note('上一项计算尚未完成，请等待。');return
        self.task_busy=True
        def work():
            try: self.events.put(('log',fn()))
            except Exception as exc:
                self.events.put(('error',str(exc)))
                with (ROOT/'output/host_errors.log').open('a',encoding='utf-8') as f: traceback.print_exc(file=f)
            finally:self.events.put(('done',None))
        threading.Thread(target=work,daemon=True).start()

    def refresh_ports(self):
        values=[p.device for p in list_ports.comports() if p.vid==0x10c4 and p.pid==0xea60]
        self.ports['values']=values
        if self.port.get() not in values:self.port.set(values[0] if values else '')

    def open_capture(self):
        if self.worker and self.worker.is_alive():
            self.note('请先关闭当前采集卡，待释放后再切换编号。');return
        try:index=int(self.device.get())
        except ValueError:self.note('视频设备编号必须是整数。');return
        self.stop_capture.clear()
        def capture():
            cap=cv2.VideoCapture(index,cv2.CAP_DSHOW)
            try:
                if not cap.isOpened():raise RuntimeError('无法打开设备；更换编号，并关闭占用采集卡的软件。')
                # The board outputs 1280x720. This capture card may return a
                # 1920x1080 USB image despite the requested 720p transport.
                mjpg=cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
                cap.set(cv2.CAP_PROP_FPS,30)
                self.events.put(('log',f'采集卡MJPG请求返回={mjpg}；图像尺寸将逐帧核对，算法帧率请读取板端状态。'))
                failed=0;last_usb_shape=None
                while not self.stop_capture.is_set():
                    ok, frame=cap.read()
                    if not ok:
                        failed+=1
                        if failed>=30:raise RuntimeError('采集卡没有返回图像；检查HDMI输入和设备模式。')
                        time.sleep(.05);continue
                    failed=0
                    if frame.shape[:2] not in PREVIEW_SHAPES:
                        raise RuntimeError(f'采集卡实际返回{frame.shape[1]}×{frame.shape[0]}；预览仅接受1280×720或1920×1080，请检查采集卡USB模式。')
                    if frame.shape[:2]!=last_usb_shape:
                        last_usb_shape=frame.shape[:2]
                        self.events.put(('log',f'采集卡实际USB图像={frame.shape[1]}×{frame.shape[0]}；FPGA真实输出仍为1280×720。预览保留USB图像像素，不在PC重采样或矫正；窗口缩略图仅用于显示。'))
                    if self.frames.full():
                        try:self.frames.get_nowait()
                        except queue.Empty:pass
                    self.frames.put(frame)
            except Exception as exc:self.events.put(('error',str(exc)))
            finally:cap.release();self.events.put(('log','采集设备已关闭。'))
        self.worker=threading.Thread(target=capture,daemon=True);self.worker.start()
        self.note(f'正在打开视频设备 {index}，请确认预览来自板上的摄像头。')

    def close_capture(self):
        self.stop_capture.set();self.latest=None;self.is_raw.set(False)

    def new_session(self):
        self.session=ROOT/'data/captures'/time.strftime('%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True,exist_ok=True)
        (self.session/'capture_session.json').write_text(json.dumps({
            'sensor_mode':'ov5640_dvp_rgb565_wide_1280x720_30_candidate_v1',
            'width':1280,'height':720,'source':'FPGA HDMI captured without PC remap',
            'raw_mode_confirmation':'User must confirm identity mode before saving each image',
            'hardware_mode_verified':False},ensure_ascii=False,indent=2),encoding='utf-8')
        self.note('当前图组：'+str(self.session))

    def save_frame(self):
        if self.task_busy:self.note('参数切换或计算尚未完成，请等待画面稳定后再保存原图。');return
        if self.stop_capture.is_set():self.note('采集卡已关闭，请重新打开后保存原图。');return
        if not self.is_raw.get():self.note('标定前先恢复原图，并勾选“确认当前是原图模式”。');return
        if self.latest is None:self.note('先打开采集卡。');return
        frame=self.latest.copy()
        if frame.shape[:2]!=(720,1280):
            self.note(f'当前USB图像为{frame.shape[1]}×{frame.shape[0]}，仅供预览，未保存标定原图。标定需要与FPGA 1280×720坐标逐像素对应的原图；不能用PC降采样代替。');return
        target=self.session/f'view_{time.time_ns()}.png'
        cv2.imencode('.png',frame)[1].tofile(str(target))
        self.note(f'已保存 {len(list(self.session.glob("*.png")))} 张：{target.name}。改变棋盘格姿态后再拍下一张。')

    def choose_calibration(self):
        p=filedialog.askopenfilename(title='选择真实镜头camera.json',filetypes=[('JSON','*.json')])
        if p:self.calibration.set(p)

    def calibrate(self):
        try:cols=int(self.cols.get());rows=int(self.rows.get());square=float(self.square.get())
        except ValueError:self.note('请填写正确的内角点数和格长。');return
        folder=self.session;out=ROOT/'output/calibration'/folder.name
        def work():
            cmd=[sys.executable,'-X','utf8',str(ROOT/'tools/calibrate_camera.py'),str(folder),
                 '--cols',str(cols),'--rows',str(rows),'--square-mm',str(square),'--model','fisheye','--out',str(out)]
            result=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',creationflags=0x08000000)
            self.events.put(('log',result.stdout+result.stderr))
            if result.returncode:raise RuntimeError('标定失败；请查看上述原因以及input_views.json。')
            self.events.put(('calibration',str(out/'camera.json')))
            return '标定文件已生成。训练误差不能代替独立图卡检验；先用基础角度0计算并发送。'
        self.note('正在检测棋盘格和标定，可能需要几分钟。');self.run_task(work)

    def send_corrected(self):
        if self.task_busy:self.note('上一项操作尚未完成，请等待。');return
        try:
            path=self.calibration.get();zoom=float(self.zoom.get());angle=float(self.angle.get())
            gain=int(self.gain_q8.get());offset=int(self.offset.get())
        except ValueError:self.note('zoom/角度需要填写数字，gain Q8/offset需要整数。');return
        port=self.port.get();crop=self.crop.get();self.is_raw.set(False)
        def work():
            d=load_calibration(path)
            words,meta=fitted_configuration(d,zoom,angle,crop)
            words=brightness_configuration(words,gain,offset)
            meta.update(source=str(path),synthetic=False,identity=False,gain_q8=gain,offset=offset)
            result=send_configuration(port,words,meta)
            self.events.put(('prepared',(words,meta)))
            return f'实际zoom={meta["zoom"]:.2f}，越界像素={meta["invalid_pixels"]}。'+result
        self.note('正在检查定点映射和块缓存边界；新几何发送后由FPGA重建映射，约秒级。自动裁剪会缩小视场。');self.run_task(work)

    def fill_preset(self):
        angle,zoom=GEOMETRY_PRESETS[self.preset.get()]
        self.angle.set(str(angle));self.zoom.set(str(zoom));self.crop.set(False)
        self.note('预设仅填入参数，尚未发送。请点击“切到矫正图”发送并确认。')

    def send_brightness(self):
        if self.task_busy:self.note('上一项操作尚未完成，请等待。');return
        if self.prepared is None:
            self.note('请先在本控制台成功发送原图或矫正参数，再保留该几何调整亮度。');return
        try:
            gain=int(self.gain_q8.get());offset=int(self.offset.get())
            words=brightness_configuration(self.prepared[0],gain,offset)
        except ValueError as exc:self.note(str(exc));return
        meta={**self.prepared[1],'gain_q8':gain,'offset':offset}
        port=self.port.get();self.is_raw.set(False)
        def work():
            result=send_configuration(port,words,meta)
            self.events.put(('prepared',(words,meta)))
            return '仅更新gain/offset，保留最近已发送几何；FPGA可复用该几何缓存。'+result
        self.run_task(work)

    def send_gamma(self):
        if self.task_busy:self.note('上一项操作尚未完成，请等待。');return
        port=self.port.get();mode=GAMMA_LABELS[self.gamma_mode.get()]
        self.is_raw.set(False)
        def work():
            result=send_enhancement(port,mode)
            target=ROOT/'output/last_enhancement_confirmation.json'
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            after=result['after']
            lead='板端原已处于所选Gamma模式，未重复发送。' if result['already_applied'] else 'Gamma模式已由后续板端V5确认。'
            return lead+f'模式={after["enhancement_mode_name"]}，当前曲线={after["gamma_name"]}。'
        self.note('先检查高级版A001身份，发送后等待板端V5确认实际模式。');self.run_task(work)

    def send_identity(self):
        if self.task_busy:self.note('上一项操作尚未完成，请等待。');return
        self.is_raw.set(False)
        port=self.port.get()
        words=configuration(IDENTITY)
        meta={'source':'identity','synthetic':True,'identity':True,'width':1280,'height':720,'zoom':1,'angle':0}
        def work():
            result=send_configuration(port,words,meta)
            self.events.put(('prepared',(words,meta)))
            return '恢复原图参数。'+result
        self.run_task(work)

    def send_diagnostic(self):
        if self.task_busy:self.note('上一项操作尚未完成，请等待。');return
        self.is_raw.set(False)
        port=self.port.get()
        def work():
            camera={**IDENTITY,'identity_mode':False,'distortion':[-.05,.005,.002,-.001]}
            words,meta=fitted_configuration(camera,zoom=1,angle=0,auto_crop=False)
            meta.update(source='synthetic_diagnostic',synthetic=True,identity=False,
                        purpose='Visible pipeline test, not measured lens correction')
            result=send_configuration(port,words,meta)
            self.events.put(('prepared',(words,meta)))
            return '诊断参数已提交：用于检查FPGA是否改变画面，不代表镜头已经矫正。'+result
        self.run_task(work)

    def read_status(self):
        port_name=self.port.get()
        def work():
            import serial
            from read_board_status import extract_packets,add_frame_rates
            matches=[p for p in list_ports.comports() if p.device.upper()==port_name.upper()
                     and p.vid==0x10c4 and p.pid==0xea60]
            if len(matches)!=1:raise ValueError('请选择板载CP210x USB-UART对应的COM口。')
            previous=None;buffer=bytearray();readings=[]
            with serial.Serial(port_name,115200,timeout=.2) as serial_port:
                until=time.monotonic()+5
                while time.monotonic()<until:
                    buffer.extend(serial_port.read(256))
                    for data in extract_packets(buffer):
                        add_frame_rates(data,previous);previous=data.copy();readings.append(data)
            if not readings:raise RuntimeError('没有收到有效板端状态；请检查下载的版本和UART连接。')
            target=ROOT/'output/last_board_status.json'
            target.write_text(json.dumps(readings,ensure_ascii=False,indent=2),encoding='utf-8')
            last=readings[-1]
            if last['build_id']!=BUILD_ID:raise RuntimeError('板端尚未运行本目录高级版A001位流。')
            enhancement=enhancement_status(last)
            return (f"输入有效帧率={last.get('observed_raw_fps','等待下一采样')}，"
                    f"新结果帧率={last.get('observed_new_fps','等待下一采样')}；"
                    f"映射版本={last['config_version']}，生成映射中={last['map_building']}；"
                    f"系统错误={last['error_sticky']}，相机帧错误={last.get('camera_error')}，"
                    f"显示欠读={last['underflow']}，采集溢出={last['overflow']}；"
                    f"Gamma模式={enhancement['enhancement_mode_name']}，当前曲线={enhancement['gamma_name']}。记录：{target}")
        self.run_task(work)

    def boot(self):
        if self.prepared is None:self.note('先成功发送一组参数，再写入为上电默认值。');return
        words,meta=self.prepared
        try:self.note(export_boot(words,meta))
        except Exception as exc:self.note(str(exc))

    def poll(self):
        try:
            while True:
                kind,value=self.events.get_nowait()
                if kind in ('log','error'):self.note(('错误：' if kind=='error' else '')+value)
                elif kind=='calibration':self.calibration.set(value)
                elif kind=='prepared':
                    self.prepared=value;self.zoom.set(str(value[1]['zoom']));self.is_raw.set(False)
                    self.gain_q8.set(str(value[0][14]));self.offset.set(str(value[0][15]))
                elif kind=='done':self.task_busy=False
        except queue.Empty:pass
        try:
            frame=self.frames.get_nowait();self.latest=frame
            if self.stop_capture.is_set():self.latest=None;return self.app.after(40,self.poll)
            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            image=Image.fromarray(rgb);image.thumbnail((900,400))
            self.photo=ImageTk.PhotoImage(image);self.preview.configure(image=self.photo,text='')
        except queue.Empty:pass
        self.app.after(40,self.poll)

    def close(self):
        self.stop_capture.set();self.app.destroy()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--self-check',action='store_true');a=ap.parse_args()
    if a.self_check:
        # Verify widgets can be constructed without opening camera or UART.
        app=tk.Tk();app.withdraw();Panel(app);app.update_idletasks();app.destroy()
        print('CONTROL PANEL SELF CHECK PASS (no camera/serial opened)');return
    app=tk.Tk();Panel(app);app.mainloop()

if __name__=='__main__':main()
