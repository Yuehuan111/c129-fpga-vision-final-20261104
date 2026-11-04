"""Unified C127 desktop workbench. Device connections are always explicit."""
from __future__ import annotations
import argparse
from copy import deepcopy
from datetime import datetime
import json
import multiprocessing
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

ROOT=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
from imu_app import ControlPanel, STATE_NAMES, read_mounting
from unified_actions import ActionSequencer


def format_live_status(data):
    if data is None:
        return ('设备未连接','等待 IMU','等待板端状态','尚无已确认输出')
    connected=data.get('connected') or {}
    device='FPGA '+('已连接' if connected.get('fpga') else '未连接')+' · IMU '+('已连接' if connected.get('imu') else '未连接')
    imu=data.get('imu') or {}
    rate=imu.get('rate_hz')
    age=imu.get('age_s')
    imu_text=(f'{rate:.1f} Hz' if isinstance(rate,(int,float)) else '频率待测')+' / '+(f'{age:.3f} s' if isinstance(age,(int,float)) else '无样本')
    if imu.get('stale'):imu_text+=' · 已过期'
    status=data.get('last_status') or {}
    state=STATE_NAMES.get(data.get('state'),data.get('state','待确认'))
    board=f"{state} · 配置 {status.get('config_version','—')}"
    scan=data.get('scan') or {}
    displayed=scan.get('displayed')
    output=(f"已确认版本 {displayed.get('version','—')} · 源帧 {displayed.get('source_frame_id','—')}"
            if displayed else '等待同版本输出确认')
    return device,imu_text,board,output


class UnifiedControlPanel(ControlPanel):
    def __init__(self, window, *, offline=False):
        self.offline=offline
        self.panels=[]
        self._saved_ab_mode='roll_only'
        self._saved_ab_crop=None
        self.settings_path=ROOT/'settings/unified_preferences.json'
        self.last_mounting_path=None
        super().__init__(window)
        self.actions=ActionSequencer(lambda:self.session,self.notify)
        window.title('VISION LAB · 统一视觉演示与调试工作台 · C127')
        width=min(1500,max(1100,window.winfo_screenwidth()-80))
        height=min(960,max(760,window.winfo_screenheight()-100))
        window.geometry(f'{width}x{height}')
        window.minsize(1080,760)
        self.setup_style()
        header=ttk.Frame(window,padding=(22,14),style='Header.TFrame')
        header.pack(side='top',fill='x',before=self.tabs)
        ttk.Label(header,text='VISION LAB',style='Brand.TLabel').pack(side='left')
        ttk.Label(header,text='统一视觉工作台  /  C127',style='Header.TLabel').pack(side='left',padx=20)
        self.connection_badge=tk.StringVar(value='离线文件模式' if offline else '设备未连接')
        ttk.Label(header,textvariable=self.connection_badge,style='Header.TLabel').pack(side='right')
        footer=ttk.Frame(window,padding=(18,8))
        footer.pack(side='bottom',fill='x',before=self.tabs)
        ttk.Label(footer,textvariable=self.footer,wraplength=1300).pack(side='left',fill='x',expand=True)
        self.tabs.tab(0,text='设备与 IMU')
        self.tabs.tab(1,text='模式、扫描与镜像')
        self.tabs.tab(2,text='板端状态与证据')
        self.connection_page=self.tabs.tabs()[0]
        self.mode_page=self.tabs.tabs()[1]
        self.demo_page=ttk.Frame(self.tabs,padding=18)
        self.tabs.insert(0,self.demo_page,text='演示预览')
        self.build_demo()
        from unified_geometry_panel import GeometryPanel
        from unified_optical_tools import OpticalToolsPanel
        from unified_diagnostics import DiagnosticsPanel
        self.geometry_panel=GeometryPanel(self.tabs,on_status=self.notify)
        self.tabs.insert(3,self.geometry_panel,text='离线几何验证')
        self.optical_panel=OpticalToolsPanel(self.tabs,on_status=self.notify)
        self.tabs.insert(4,self.optical_panel,text='标定与测试图卡')
        self.diagnostics_panel=DiagnosticsPanel(self.tabs,snapshot_provider=self.snapshot_or_none,
                                               log_provider=lambda:self.last_log,on_status=self.notify,
                                               allow_devices=not offline)
        self.tabs.add(self.diagnostics_panel,text='日志与安装标定')
        self.panels += [self.geometry_panel,self.optical_panel,self.diagnostics_panel]
        self.tabs.select(self.demo_page)
        window.bind('<F1>',lambda event:self.tabs.select(self.demo_page))
        window.bind('<F2>',lambda event:self.tabs.select(self.connection_page))
        window.bind('<F3>',lambda event:self.tabs.select(self.geometry_panel))
        self.load_preferences()
        self.notify('就绪：选择本地文件演示，或明确连接串口与视频设备。'+(' 当前为离线模式。' if offline else ''))

    def setup_style(self):
        style=ttk.Style(self.window)
        style.theme_use('clam')
        self.window.option_add('*Font',('Microsoft YaHei UI',10))
        style.configure('.',font=('Microsoft YaHei UI',10),background='#eef2f6',foreground='#24354a')
        style.configure('TFrame',background='#eef2f6')
        style.configure('TLabel',background='#eef2f6')
        style.configure('TButton',padding=(10,7))
        style.configure('TNotebook',background='#eef2f6',borderwidth=0)
        style.configure('TNotebook.Tab',padding=(14,10),font=('Microsoft YaHei UI',10))
        style.map('TNotebook.Tab',background=[('selected','#ffffff')],foreground=[('selected','#087f8c')])
        style.configure('TLabelframe',background='#eef2f6',bordercolor='#cbd5e1')
        style.configure('TLabelframe.Label',font=('Microsoft YaHei UI',10,'bold'))
        style.configure('Header.TFrame',background='#13263d')
        style.configure('Header.TLabel',background='#13263d',foreground='#dbe7f4',font=('Microsoft YaHei UI',11))
        style.configure('Brand.TLabel',background='#13263d',foreground='#65d9c4',font=('Segoe UI',19,'bold'))
        style.configure('Primary.TButton',background='#087f8c',foreground='white')
        style.map('Primary.TButton',background=[('active','#066875')])
        style.configure('Metric.TLabel',font=('Microsoft YaHei UI',10,'bold'))
        style.configure('Heading.TLabel',font=('Microsoft YaHei UI',18,'bold'))

    def build_demo(self):
        from unified_video import VideoPanel
        page=self.demo_page
        page.columnconfigure(0,weight=1)
        page.columnconfigure(1,weight=0)
        page.rowconfigure(1,weight=1)
        title=ttk.Frame(page)
        title.grid(row=0,column=0,columnspan=2,sticky='ew',pady=(0,12))
        ttk.Label(title,text='一个画面，集中演示与调试',font=('Microsoft YaHei UI',20,'bold')).pack(side='left')
        ttk.Button(title,text='载入示例照片',command=lambda:self.act(self.load_sample)).pack(side='right')
        ttk.Button(title,text='取当前画面做离线验证',command=lambda:self.act(self.analyze_preview)).pack(side='right',padx=8)
        self.video_panel=VideoPanel(page,on_status=self.notify,allow_devices=not self.offline)
        self.video_panel.grid(row=1,column=0,sticky='nsew',padx=(0,16))
        self.panels.append(self.video_panel)
        sidebar=ttk.Frame(page)
        sidebar.grid(row=1,column=1,sticky='ns')
        side_canvas=tk.Canvas(sidebar,width=330,height=400,highlightthickness=0,background='#eef2f6')
        side_scroll=ttk.Scrollbar(sidebar,orient='vertical',command=side_canvas.yview)
        side_canvas.configure(yscrollcommand=side_scroll.set)
        side_scroll.pack(side='right',fill='y')
        side_canvas.pack(side='left',fill='both',expand=True)
        side=ttk.Frame(side_canvas)
        side_window=side_canvas.create_window(0,0,anchor='nw',window=side)
        side.bind('<Configure>',lambda event:side_canvas.configure(scrollregion=side_canvas.bbox('all')))
        side_canvas.bind('<Configure>',lambda event:side_canvas.itemconfigure(side_window,width=event.width))
        side.columnconfigure(0,weight=1)
        connections=ttk.LabelFrame(side,text='设备与输出',padding=12)
        connections.grid(row=0,column=0,sticky='ew')
        self.device_text=tk.StringVar(value='设备未连接')
        self.imu_text=tk.StringVar(value='等待 IMU')
        self.board_text=tk.StringVar(value='等待板端状态')
        self.output_text=tk.StringVar(value='尚无已确认输出')
        for var in [self.device_text,self.imu_text,self.board_text,self.output_text]:
            ttk.Label(connections,textvariable=var,wraplength=300).pack(anchor='w',pady=3)
        ttk.Button(connections,text='连接与参考姿态设置',command=lambda:self.tabs.select(self.connection_page)).pack(fill='x',pady=(8,0))
        compare=ttk.LabelFrame(side,text='模式对比 · 单路实时切换',padding=12)
        compare.grid(row=1,column=0,sticky='ew',pady=12)
        ttk.Button(compare,text='A  普通去畸变',style='Primary.TButton',command=lambda:self.act(self.compare_normal)).pack(fill='x',pady=3)
        ttk.Button(compare,text='B  姿态回正（单次）',command=lambda:self.act(self.compare_pose)).pack(fill='x',pady=3)
        ttk.Label(compare,text='切换会停止连续提交并等待在途结果。\nA 使用普通视野，B 恢复已选回正/裁边设置。\n镜头原图/矫正对比仍使用板上 KEY1。',wraplength=304).pack(anchor='w',pady=(8,0))
        ttk.Button(compare,text='开始连续补偿',command=lambda:self.act(self.start_continuous)).pack(fill='x',pady=(10,3))
        ttk.Button(compare,text='停止提交 / 取消等待',command=self.stop_all_requests).pack(fill='x',pady=3)
        enhance=ttk.LabelFrame(side,text='自适应亮度增强',padding=12)
        enhance.grid(row=2,column=0,sticky='ew')
        ttk.Combobox(enhance,textvariable=self.enhancement_mode,state='readonly',width=27,
                     values=('0 旁路','1 自动','2 阴影','3 高光','4 直方图')).pack(fill='x')
        ttk.Button(enhance,text='应用增强',command=lambda:self.act(self.apply_enhancement)).pack(fill='x',pady=7)
        ttk.Label(enhance,textvariable=self.enhancement_status,wraplength=300).pack(anchor='w')
        views=ttk.LabelFrame(side,text='平面扫描与镜像',padding=12)
        views.grid(row=3,column=0,sticky='ew',pady=(12,0))
        ttk.Checkbutton(views,text='平面转正（需匹配参数）',variable=self.scan_enabled).pack(anchor='w')
        ttk.Checkbutton(views,text='水平镜像',variable=self.mirror_enabled).pack(anchor='w')
        ttk.Button(views,text='应用扫描 / 镜像',command=lambda:self.act(self.queue_view)).pack(fill='x',pady=7)
        ttk.Button(views,text='拟合、初始化与裁边设置',command=lambda:self.tabs.select(self.mode_page)).pack(fill='x')
        def bind_wheel(widget):
            widget.bind('<MouseWheel>',lambda event:side_canvas.yview_scroll(-int(event.delta/120),'units'))
            for child in widget.winfo_children():bind_wheel(child)
        bind_wheel(side)
        side_canvas.bind('<MouseWheel>',lambda event:side_canvas.yview_scroll(-int(event.delta/120),'units'))
        ttk.Label(page,text='视频来自 HDMI 采集或本地文件；串口只控制与读状态。预览采集帧率不代表 FPGA 新矫正帧率。',wraplength=1200).grid(row=2,column=0,columnspan=2,sticky='w',pady=(12,0))

    def notify(self,text):
        if not self.closing:self.footer.set(str(text))

    def snapshot_or_none(self):
        return self.session.snapshot() if self.session is not None else None

    def refresh_ports(self):
        if self.offline:
            self.imu_combo['values']=[''];self.fpga_combo['values']=['']
            self.imu_port.set('');self.fpga_port.set('')
            self.port_note.set('离线模式：不枚举或打开串口。')
        else:super().refresh_ports()

    def connect(self):
        if self.offline:raise RuntimeError('当前为离线文件模式；关闭后用正常入口启动即可连接设备。')
        if hasattr(self,'diagnostics_panel') and self.diagnostics_panel.capture_busy:
            raise RuntimeError('安装标定正在独立采集，请等待完成或取消后再连接主会话。')
        super().connect()

    def disconnect(self):
        if hasattr(self,'actions'):self.actions.cancel('连接已断开，待执行操作已取消。')
        super().disconnect()

    def compare_normal(self):
        s=self.require_session()
        snap=s.snapshot()
        if snap.get('pose_mode') in ('roll_only','full'):
            self._saved_ab_mode=snap['pose_mode']
            self._saved_ab_crop=deepcopy(snap.get('crop_profile'))
        def send(session):
            session.request_normal()
            self.pose_mode.set('normal')
            self.crop_enabled.set(False)
            self._applied_pose_settings=('normal',None)
        self.actions.begin('A 普通去畸变',[send])

    def compare_pose(self):
        mode=self.pose_mode.get()
        if mode in ('roll_only','full'):
            crop=self.crop_profile()
        else:
            mode=self._saved_ab_mode
            crop=deepcopy(self._saved_ab_crop)
        def settings(session):
            session.set_pose_mode(mode,crop_profile=crop)
            self.pose_mode.set(mode)
            self.crop_enabled.set(crop is not None)
            if crop:
                self.crop_zoom.set(str(crop['zoom']));self.crop_range.set(str(crop['max_abs_roll_deg']))
            self._applied_pose_settings=(mode,deepcopy(crop))
        self.actions.begin('B 姿态回正',[settings,lambda session:session.send_once()])

    def queue_view(self):
        scan,mirror=bool(self.scan_enabled.get()),bool(self.mirror_enabled.get())
        self.actions.begin('扫描 / 镜像切换',[lambda session:session.request_view_mode(scan=scan,mirror=mirror)])

    def stop_all_requests(self):
        if hasattr(self,'actions'):self.actions.cancel()
        if self.session is not None:self.session.stop()
        self.notify('已停止新提交；已在途事务按原协议收尾，画面保留上一有效结果。')

    def load_sample(self):
        p=ROOT/'assets/demo_photo.jpg'
        if not p.exists():raise FileNotFoundError('未找到示例照片，可在画面区打开自己的本地文件。')
        self.video_panel.load_image(p)

    def analyze_preview(self):
        import cv2
        frame=self.video_panel.get_frame_copy()
        if frame is None:raise ValueError('请先打开本地图片、视频或视频预览。')
        if frame.shape[:2] not in ((720,1280),(1080,1920)):
            raise ValueError('当前几何模型只接受完整 1280×720 或同视野 1920×1080；请调整采集分辨率。')
        out=ROOT/'results'/'preview_frames'
        out.mkdir(parents=True,exist_ok=True)
        p=out/(datetime.now().strftime('preview_%Y%m%d_%H%M%S_%f')+'.png')
        ok,data=cv2.imencode('.png',frame)
        if not ok:raise ValueError('不能保存当前画面。')
        p.write_bytes(data.tobytes())
        self.geometry_panel.load_image(p)
        self.tabs.select(self.geometry_panel)
        self.notify('已保存当前预览帧。请先选择“原始鱼眼图”或“已去镜头畸变图”，再运行离线验证。')

    def load_mounting(self):
        p=filedialog.askopenfilename(parent=self.window,initialdir=ROOT,title='载入安装旋转矩阵',filetypes=[('JSON','*.json')])
        if not p:return
        mounting=read_mounting(p)
        if self.session is not None:
            self.session.stop()
            self.session.set_mounting(mounting['R_IC'],convention=mounting['convention'],confirmed=False)
        self.mounting=mounting
        self.last_mounting_path=str(Path(p).resolve())
        self.mount_confirmed.set(False)
        self.mount_note.set(Path(p).name+' | '+mounting['convention']+' | 请核对后勾选确认')
        self.save_preferences()

    def load_preferences(self):
        try:
            data=json.loads(self.settings_path.read_text(encoding='utf-8'))
            p=data.get('last_mounting_path')
            if p and Path(p).is_file():
                self.mounting=read_mounting(p)
                self.last_mounting_path=p
                self.mount_confirmed.set(False)
                self.mount_note.set(Path(p).name+' | 已恢复文件；本次连接仍需核对确认。')
        except (OSError,ValueError,KeyError,TypeError):pass

    def save_preferences(self):
        self.settings_path.parent.mkdir(parents=True,exist_ok=True)
        data={'schema':'unified_host_preferences_v1','last_mounting_path':self.last_mounting_path,
              'auto_connect':False,'auto_apply_mounting':False}
        temp=self.settings_path.with_suffix('.tmp')
        temp.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
        temp.replace(self.settings_path)

    def update(self):
        if self.closing:return
        super().update()
        if hasattr(self,'actions'):self.actions.tick()
        if hasattr(self,'device_text'):
            values=format_live_status(self.snapshot_or_none())
            for var,value in zip([self.device_text,self.imu_text,self.board_text,self.output_text],values):var.set(value)
            self.connection_badge.set('离线文件模式' if self.offline else values[0])

    def close(self):
        if self.closing:return
        self.closing=True
        if hasattr(self,'actions'):self.actions.cancel()
        for panel in self.panels:
            try:panel.close()
            except Exception:pass
        try:self.save_preferences()
        except OSError:pass
        try:self.disconnect()
        finally:
            # This is the application's own Tk interpreter. Cancel its scheduled
            # callbacks before destroying widgets, including the inherited poll.
            for callback in self.window.tk.splitlist(self.window.tk.call('after','info')):
                try:self.window.after_cancel(callback)
                except tk.TclError:pass
            self.window.destroy()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline',action='store_true',help='只使用文件，不枚举/打开串口或视频设备')
    parser.add_argument('--demo-image',type=Path,help='启动后显示指定离线图片')
    args=parser.parse_args(argv)
    window=tk.Tk()
    app=UnifiedControlPanel(window,offline=args.offline)
    if args.demo_image:window.after(150,lambda:app.act(lambda:app.video_panel.load_image(args.demo_image)))
    window.mainloop()


if __name__=='__main__':
    multiprocessing.freeze_support()
    main()
