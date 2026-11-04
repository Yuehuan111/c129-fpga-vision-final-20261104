"""Log analysis, candidate mounting fit and explicitly started IMU endpoint capture."""
from pathlib import Path
import json
import math
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk, filedialog


ROOT = Path(__file__).resolve().parent
ENDPOINTS = tuple(f'{axis}_{end}' for axis in 'xyz' for end in ('start','end'))


def session_connected(snapshot):
    if snapshot is None:return False
    if not isinstance(snapshot,dict):raise ValueError('无法确认当前会话状态，请先断开设备。')
    connected=snapshot.get('connected',{})
    if isinstance(connected,dict):
        if any(connected.values()):return True
    elif connected:return True
    return any(isinstance(snapshot.get(role),dict) and snapshot[role].get('connected') for role in ('imu','fpga'))


def build_capture_command(port,session,label,profile,seconds):
    """Validate without enumerating/opening devices; the original CLI owns capture."""
    port=str(port).strip()
    if not port:raise ValueError('请输入 IMU 串口，例如 COM5。')
    if not str(session).strip():raise ValueError('请选择或填写安装标定会话目录。')
    folder=Path(session).expanduser().resolve()
    if folder.exists() and not folder.is_dir():raise ValueError('会话路径必须是文件夹。')
    if label not in ENDPOINTS:raise ValueError('请选择 x/y/z 的 start 或 end 端点。')
    if profile not in ('static','handheld'):raise ValueError('请选择 static 或 handheld 采集方式。')
    try:duration=float(seconds)
    except (TypeError,ValueError) as exc:raise ValueError('持续时间必须为 0.8～10 秒。') from exc
    if not math.isfinite(duration) or not .8<=duration<=10:raise ValueError('持续时间必须为 0.8～10 秒。')
    if (folder/label).exists():raise ValueError('此端点目录已经存在，不会覆盖；请选择尚未记录的端点或新会话目录。')
    command=[sys.executable,'-B',str(ROOT/'mounting_calibration.py'),'capture','--port',port,
             '--session',str(folder),'--label',label,'--seconds',str(duration)]
    if profile=='handheld':command.append('--handheld')
    return command,folder/label


class MountingCaptureRunner:
    """Owns one explicit CLI child. No serial access in the UI process."""
    def __init__(self):
        self.events=queue.Queue();self._lock=threading.Lock();self._stop=threading.Event()
        self._process=None;self._thread=None;self._closed=False

    @property
    def busy(self):
        with self._lock:return bool(self._thread and self._thread.is_alive())

    def start(self,command):
        with self._lock:
            if self._closed:raise RuntimeError('采集面板已经关闭。')
            if self._thread and self._thread.is_alive():raise RuntimeError('已有端点正在采集。')
            self._stop.clear()
            self._thread=threading.Thread(target=self._work,args=(list(command),),daemon=True,name='mounting-endpoint-capture')
            self._thread.start()

    def _work(self,command):
        output='';code=-1
        try:
            env=os.environ.copy();env.update(PYTHONUTF8='1',PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1')
            kwargs=dict(stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',cwd=str(ROOT),env=env)
            if os.name=='nt':kwargs['creationflags']=subprocess.CREATE_NO_WINDOW
            with self._lock:
                if self._stop.is_set():raise RuntimeError('采集在开始前已取消。')
                process=subprocess.Popen(command,**kwargs);self._process=process
            for line in process.stdout:
                output=(output+line)[-200000:];self.events.put(('output',line))
            code=process.wait();process.stdout.close()
        except Exception as exc:
            output+='\n'+str(exc);self.events.put(('output',str(exc)))
        finally:
            with self._lock:self._process=None
            self.events.put(('finished',dict(returncode=code,cancelled=self._stop.is_set(),output=output)))

    def cancel(self):
        self._stop.set()
        with self._lock:process=self._process
        if process is not None and process.poll() is None:
            try:process.terminate()
            except OSError:pass

    def close(self):
        with self._lock:self._closed=True
        self.cancel()
        with self._lock:thread=self._thread
        if thread and thread is not threading.current_thread():thread.join(timeout=1)
        with self._lock:process=self._process
        if process is not None and process.poll() is None:
            try:process.kill()
            except OSError:pass


class DiagnosticsPanel(ttk.Frame):
    def __init__(self, master, *, snapshot_provider=None, log_provider=None, on_status=None, allow_devices=True):
        super().__init__(master, padding=18)
        self.snapshot_provider = snapshot_provider or (lambda: None)
        self.log_provider = log_provider or (lambda: None)
        self.notify = on_status or (lambda text: None)
        self.allow_devices=bool(allow_devices)
        self.capture_runner=MountingCaptureRunner()
        self.capture_endpoint=None
        self.closed = False
        self.busy = False
        self.results = queue.Queue()
        self.report = None
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)
        ttk.Label(self, text='诊断与离线记录', font=('Microsoft YaHei UI', 18, 'bold')).grid(row=0, column=0, sticky='w')
        bar = ttk.Frame(self)
        bar.grid(row=1, column=0, sticky='ew', pady=12)
        ttk.Button(bar, text='分析本次会话日志', command=self.current_log).pack(side='left', padx=(0,8))
        ttk.Button(bar, text='打开历史 JSONL…', command=self.open_log).pack(side='left', padx=8)
        ttk.Button(bar, text='读取当前状态快照', command=self.snapshot).pack(side='left', padx=8)
        ttk.Button(bar, text='保存当前报告…', command=self.save).pack(side='left', padx=8)
        self.note = tk.StringVar(value='日志和拟合仅处理文件；端点采集需要先断开主会话，再明确点击开始。')
        ttk.Label(self, textvariable=self.note, wraplength=1100).grid(row=2, column=0, sticky='w', pady=(0,12))
        box = ttk.Frame(self)
        box.grid(row=3, column=0, sticky='nsew')
        self.text = tk.Text(box, wrap='word', font=('Consolas',10), state='disabled', padx=12, pady=12)
        scroll = ttk.Scrollbar(box, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        self.text.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        fit = ttk.LabelFrame(self, text='IMU 安装关系：使用已有六个静止端点记录', padding=12)
        fit.grid(row=4, column=0, sticky='ew', pady=(14,0))
        ttk.Button(fit, text='选择记录目录并拟合…', command=self.fit_mounting).pack(side='left')
        ttk.Label(fit, text='输出候选安装矩阵，需人工核对后再载入；不自动采集或应用。').pack(side='left', padx=14)
        self._build_capture()
        self._after=self.after(120, self.poll)

    @property
    def capture_busy(self):return self.capture_runner.busy

    def _build_capture(self):
        capture=ttk.LabelFrame(self,text='安装标定六端点采集（只读 IMU，不写 FPGA）',padding=10)
        capture.grid(row=5,column=0,sticky='ew',pady=(10,0));capture.columnconfigure(1,weight=1)
        self.capture_port=tk.StringVar();self.capture_session=tk.StringVar()
        self.capture_label=tk.StringVar();self.capture_profile=tk.StringVar();self.capture_seconds=tk.StringVar(value='1.2')
        ttk.Label(capture,text='IMU 串口').grid(row=0,column=0,sticky='w',padx=(0,8))
        ttk.Entry(capture,textvariable=self.capture_port,width=14).grid(row=0,column=1,sticky='w')
        ttk.Label(capture,text='例如 COM5；原工具会检查 CH340 IMU 端口。').grid(row=0,column=2,columnspan=2,sticky='w',padx=8)
        ttk.Label(capture,text='会话目录').grid(row=1,column=0,sticky='w',pady=5)
        ttk.Entry(capture,textvariable=self.capture_session).grid(row=1,column=1,columnspan=2,sticky='ew',pady=5)
        ttk.Button(capture,text='选择…',command=self.choose_capture_session).grid(row=1,column=3,padx=8)
        row=ttk.Frame(capture);row.grid(row=2,column=0,columnspan=4,sticky='ew',pady=4)
        for label,var,values in [('端点',self.capture_label,ENDPOINTS),('方式',self.capture_profile,('static','handheld'))]:
            ttk.Label(row,text=label).pack(side='left',padx=(0,5))
            ttk.Combobox(row,textvariable=var,values=values,state='readonly',width=11).pack(side='left',padx=(0,15))
        ttk.Label(row,text='持续秒数').pack(side='left',padx=(0,5));ttk.Entry(row,textvariable=self.capture_seconds,width=7).pack(side='left',padx=(0,15))
        self.capture_start_button=ttk.Button(row,text='开始采集此端点',command=self.start_capture,state='normal' if self.allow_devices else 'disabled')
        self.capture_start_button.pack(side='left')
        self.capture_cancel_button=ttk.Button(row,text='取消本次采集',command=self.capture_runner.cancel,state='disabled')
        self.capture_cancel_button.pack(side='left',padx=8)
        message='每个端点保持静止；每个轴分别记录 start → 正向转动 → end。相机轴：向右 / 向下 / 镜头向前。\nstatic 为固定支撑，handheld 为宽松手持门限且各轴转动至少 10°。已有端点不会覆盖，取消后请用新端点会话。'
        if not self.allow_devices:message='当前为离线模式，禁止采集设备；已有记录仍可离线拟合。\n'+message
        ttk.Label(capture,text=message,wraplength=1100).grid(row=3,column=0,columnspan=4,sticky='w',pady=(6,0))

    def choose_capture_session(self):
        path=filedialog.askdirectory(parent=self,title='选择六端点共用会话目录；新目录可直接在输入框填写')
        if path:self.capture_session.set(path)

    def start_capture(self):
        try:
            if not self.allow_devices:raise RuntimeError('离线模式禁止端点设备采集。')
            if self.busy or self.capture_busy:raise RuntimeError('已有任务运行中，请先完成或取消。')
            if session_connected(self.snapshot_provider()):raise RuntimeError('当前主会话仍有设备连接，请先在连接页断开，再采集安装标定端点。')
            command,endpoint=build_capture_command(self.capture_port.get(),self.capture_session.get(),self.capture_label.get(),self.capture_profile.get(),self.capture_seconds.get())
            self.capture_runner.start(command);self.capture_endpoint=endpoint;self.busy=True
            self.capture_start_button.configure(state='disabled');self.capture_cancel_button.configure(state='normal')
            self.note.set('正在采集 '+endpoint.name+'，请保持静止。')
            self._show({'operation':'receive_only_endpoint_capture','endpoint':str(endpoint),'command':command})
            self.notify(self.note.get());return True
        except Exception as exc:
            self.note.set(str(exc));self.notify(str(exc));return False

    def _show(self, report):
        self.report = report
        self.text.configure(state='normal')
        self.text.delete('1.0','end')
        self.text.insert('1.0',json.dumps(report,ensure_ascii=False,indent=2,default=str))
        self.text.configure(state='disabled')

    def _job(self, label, function):
        if self.busy:
            self.note.set('已有任务正在运行，请等待完成。')
            return
        self.busy=True
        self.note.set(label+'…')
        def worker():
            try:self.results.put((True,label,function()))
            except Exception as exc:self.results.put((False,label,str(exc)))
        threading.Thread(target=worker,daemon=True,name='offline-diagnostics').start()

    def poll(self):
        if self.closed:return
        try:
            ok,label,value=self.results.get_nowait()
            self.busy=False
            self.note.set(label+('完成' if ok else '失败：'+value))
            self._show(value if ok else {'error':value})
            self.notify(self.note.get())
        except queue.Empty:pass
        try:
            while True:
                kind,value=self.capture_runner.events.get_nowait()
                if kind=='output':
                    self.text.configure(state='normal');self.text.insert('end','\n'+value);self.text.see('end');self.text.configure(state='disabled')
                else:
                    self.busy=False
                    self.capture_start_button.configure(state='normal' if self.allow_devices else 'disabled')
                    self.capture_cancel_button.configure(state='disabled')
                    report=dict(value,endpoint=str(self.capture_endpoint))
                    summary=self.capture_endpoint/'summary.json' if self.capture_endpoint else None
                    if summary is not None and summary.is_file():
                        try:report['summary']=json.loads(summary.read_text(encoding='utf-8'))
                        except (OSError,ValueError) as exc:report['summary_read_error']=str(exc)
                    self._show(report)
                    status='已取消；可能留下未完成端点，请使用新会话目录。' if value['cancelled'] else '端点采集及静止质量检查通过。' if value['returncode']==0 else '端点采集或质量检查失败，请查看输出。'
                    self.note.set(status);self.notify(status)
        except queue.Empty:pass
        self._after=self.after(120,self.poll)

    def analyze_path(self, path):
        path=Path(path)
        def analyze():
            from analyze_realtime_log import analyze
            records=[json.loads(line) for line in path.read_text(encoding='utf-8-sig').splitlines() if line.strip()]
            return {'source':str(path),'analysis':analyze(records)}
        self._job('分析日志',analyze)

    def open_log(self):
        p=filedialog.askopenfilename(parent=self,title='选择会话日志',filetypes=[('JSONL日志','*.jsonl'),('所有文件','*.*')])
        if p:self.analyze_path(p)

    def current_log(self):
        p=self.log_provider()
        if not p:self.note.set('本次尚无串口会话日志。');return
        self.analyze_path(p)

    def snapshot(self):
        data=self.snapshot_provider()
        if data is None:self.note.set('设备未连接，没有板端状态。');return
        self._show(data)
        self.note.set('已读取会话内存中的状态，没有发送命令。')

    def fit_mounting(self):
        p=filedialog.askdirectory(parent=self,title='选择已有 x/y/z 起止端点的记录目录')
        if p:
            def fit():
                from mounting_calibration import fit_session
                return fit_session(Path(p))
            self._job('离线拟合安装关系',fit)

    def save(self):
        if self.report is None:self.note.set('尚无可保存的报告。');return
        p=filedialog.asksaveasfilename(parent=self,defaultextension='.json',initialfile='diagnostic_report.json',filetypes=[('JSON','*.json')])
        if p:
            Path(p).write_text(json.dumps(self.report,ensure_ascii=False,indent=2,default=str),encoding='utf-8')
            self.note.set('已保存：'+p)

    def close(self):
        self.closed=True
        try:self.after_cancel(self._after)
        except tk.TclError:pass
        self.capture_runner.close()
