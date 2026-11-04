"""One-window, file-only optical calibration and quality tools.

No board/session imports: candidate calibration cannot be uploaded by this panel.
All long jobs run in an owned child process and are cancellable.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import uuid

ROOT=Path(__file__).resolve().parent
TOOLS=ROOT/'optical_tools'
IMAGE_TYPES=[('图像文件','*.png *.jpg *.jpeg *.bmp *.tif *.tiff'),('所有文件','*.*')]


def _positive_number(value, label):
    try: result=float(value)
    except (TypeError,ValueError) as exc:raise ValueError(label+'必须为数字') from exc
    if not math.isfinite(result) or result<=0:raise ValueError(label+'必须为有限正数')
    return result


def board_parameters(cols,rows,square_mm=20):
    try:c,r=int(cols),int(rows)
    except (TypeError,ValueError) as exc:raise ValueError('内角点列数、行数必须是整数') from exc
    if isinstance(cols,bool) or isinstance(rows,bool) or c!=float(cols) or r!=float(rows):
        raise ValueError('内角点列数、行数必须是整数')
    if min(c,r)<3 or max(c,r)>100:raise ValueError('内角点列数、行数需在 3..100 之间')
    return c,r,_positive_number(square_mm,'方格边长')


def input_file(value):
    p=Path(str(value).strip()).expanduser()
    if not str(value).strip() or not p.is_file():raise ValueError('请选择存在的输入文件')
    return p.resolve()


def output_run(base,kind):
    if not str(base).strip():raise ValueError('请选择结果输出目录')
    p=Path(base).expanduser().resolve()
    # Never write into the bundled production parameters or code tree.
    if (p==ROOT or ROOT in p.parents) and not (p==ROOT/'results' or ROOT/'results' in p.parents):
        raise ValueError('程序目录内仅允许写入 results；不要选择 calibration 或 vendor')
    if p.exists() and not p.is_dir():raise ValueError('输出路径不是文件夹')
    return p/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+kind+'_'+uuid.uuid4().hex[:6])


def build_calibration_command(images,out,cols,rows,square_mm,model='fisheye',synthetic=False):
    folder=Path(str(images).strip()).expanduser()
    if not str(images).strip() or not folder.is_dir():raise ValueError('请选择标定照片文件夹')
    c,r,s=board_parameters(cols,rows,square_mm)
    if model not in ('fisheye','brown'):raise ValueError('未知标定模型')
    command=[sys.executable,'-B',str(TOOLS/'calibrate_camera.py'),str(folder.resolve()),
             '--cols',str(c),'--rows',str(r),'--square-mm',str(s),'--model',model,'--out',str(out)]
    if synthetic:command.append('--synthetic')
    return command


def build_checker_command(image,out,cols,rows):
    c,r,_=board_parameters(cols,rows)
    return [sys.executable,'-B',str(TOOLS/'camera_toolbox.py'),'checker','--image',str(input_file(image)),
            '--cols',str(c),'--rows',str(r),'--out',str(Path(out)/'checker')]


def build_edge_command(image,out,roi,axis,transfer,gamma,source,oversample=4):
    path=input_file(image)
    try:values=[int(v) for v in roi]
    except (TypeError,ValueError) as exc:raise ValueError('ROI 的 X、Y、宽、高必须是整数') from exc
    if len(values)!=4 or min(values[:2])<0 or min(values[2:])<=0:raise ValueError('ROI 需要非负 X/Y 与正的宽高')
    from PIL import Image
    with Image.open(path) as im:
        if values[0]+values[2]>im.width or values[1]+values[3]>im.height:raise ValueError('ROI 超出图像边界')
    if axis not in ('vertical','horizontal') or transfer not in ('srgb','linear','gamma'):
        raise ValueError('请选择斜边方向与传递函数')
    if source not in ('real-chart','synthetic-test'):raise ValueError('请选择真实图卡或合成测试来源')
    if oversample not in (4,8):raise ValueError('过采样仅支持 4 或 8')
    g=_positive_number(gamma,'Gamma')
    return [sys.executable,'-B',str(TOOLS/'measure_quality.py'),'edge','--image',str(path),
            '--roi',*map(str,values),'--axis',axis,'--transfer',transfer,'--gamma',str(g),
            '--source',source,'--oversample',str(oversample),'--out',str(Path(out)/'edge')]


def build_line_command(points,out):
    return [sys.executable,'-B',str(TOOLS/'measure_quality.py'),'line','--points',str(input_file(points)),
            '--units','pixels','--out',str(Path(out)/'line')]


def build_chart_command(out,cols,rows,square_mm,dpi=300,format='both'):
    c,r,s=board_parameters(cols,rows,square_mm)
    if (c+1)*s>240 or (r+1)*s>140:raise ValueError('棋盘尺寸超过 A4 图卡的 240×140 mm 有效区域')
    if dpi not in (150,300,600) or format not in ('svg','png','both'):raise ValueError('图卡输出格式或 DPI 无效')
    return [sys.executable,'-B',str(TOOLS/'generate_charts.py'),'--out',str(out),'--cols',str(c),
            '--rows',str(r),'--square-mm',str(s),'--dpi',str(dpi),'--format',format]


@dataclass
class JobResult:
    returncode:int
    cancelled:bool
    output:str
    directory:str


class OfflineJobRunner:
    """Only launches on explicit start; never touches Tk from its worker."""
    def __init__(self):
        self.events=queue.Queue()
        self._lock=threading.Lock()
        self._stop=threading.Event()
        self._thread=None
        self._process=None
        self._closed=False

    @property
    def busy(self):
        with self._lock:return bool(self._thread and self._thread.is_alive())

    def start(self,command,directory):
        with self._lock:
            if self._closed:raise RuntimeError('任务管理器已经关闭')
            if self._thread and self._thread.is_alive():raise RuntimeError('已有离线任务正在运行，请先完成或取消')
            self._stop.clear()
            self._thread=threading.Thread(target=self._work,args=(list(command),str(directory)),daemon=True)
            self._thread.start()

    def _work(self,command,directory):
        output='';returncode=-1;owned_directory=False
        try:
            Path(directory).mkdir(parents=True,exist_ok=False)
            owned_directory=True
            (Path(directory)/'command.json').write_text(json.dumps({'argv':command,'hardware_accessed':False},ensure_ascii=False,indent=2),encoding='utf-8')
            env=os.environ.copy();env.update(PYTHONUTF8='1',PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',MPLBACKEND='Agg')
            kwargs={'stdout':subprocess.PIPE,'stderr':subprocess.STDOUT,'text':True,'encoding':'utf-8',
                    'errors':'replace','env':env,'cwd':str(TOOLS)}
            if os.name=='nt':kwargs['creationflags']=subprocess.CREATE_NO_WINDOW
            with self._lock:
                if self._stop.is_set():raise RuntimeError('任务在启动前已取消')
                proc=subprocess.Popen(command,**kwargs)
                self._process=proc
            for line in proc.stdout:
                output=(output+line)[-500000:]
                self.events.put(('output',line))
            returncode=proc.wait()
            proc.stdout.close()
        except Exception as exc:
            output+='\n'+str(exc)
            self.events.put(('output',str(exc)+'\n'))
        finally:
            with self._lock:self._process=None
            cancelled=self._stop.is_set()
            result=JobResult(returncode,cancelled,output,directory)
            if owned_directory:
                try:
                    (Path(directory)/'console.txt').write_text(output,encoding='utf-8')
                    (Path(directory)/'job_result.json').write_text(json.dumps(result.__dict__,ensure_ascii=False,indent=2),encoding='utf-8')
                except OSError as exc:
                    result.returncode=-1
                    result.output+='\n无法保存结果日志：'+str(exc)
                    self.events.put(('output','无法保存结果日志：'+str(exc)+'\n'))
            self.events.put(('finished',result))

    def cancel(self):
        self._stop.set()
        with self._lock:proc=self._process
        if proc is not None and proc.poll() is None:
            try:proc.terminate()
            except OSError:pass

    def close(self):
        with self._lock:self._closed=True
        self.cancel()
        with self._lock:thread=self._thread
        if thread and thread is not threading.current_thread():thread.join(timeout=1.5)
        with self._lock:proc=self._process
        if proc is not None and proc.poll() is None:
            try:proc.kill()
            except OSError:pass


class ImageSelection(tk.Toplevel):
    """In-app rectangle / point selector; callbacks use original pixel coordinates."""
    def __init__(self,parent,path,mode,callback):
        super().__init__(parent)
        from PIL import Image,ImageTk
        self.title('框选一条斜边 ROI' if mode=='rect' else '沿同一条线选点')
        self.transient(parent.winfo_toplevel())
        self.mode,self.callback=mode,callback
        with Image.open(path) as source:self.original=source.convert('RGB')
        self.width,self.height=self.original.size
        ratio=min(1,980/self.width,590/self.height)
        dw,dh=max(1,round(self.width*ratio)),max(1,round(self.height*ratio))
        self.sx,self.sy=self.width/dw,self.height/dh
        self.photo=ImageTk.PhotoImage(self.original.resize((dw,dh),Image.Resampling.LANCZOS),master=self)
        self.canvas=tk.Canvas(self,width=dw,height=dh,highlightthickness=0,background='#15202b')
        self.canvas.pack(padx=10,pady=8);self.canvas.create_image(0,0,image=self.photo,anchor='nw')
        self.start=None;self.rect=None;self.points=[];self.overlay=[]
        self.note=tk.StringVar(value='拖动选择；框内只包含单条边，避开角点、文字。' if mode=='rect' else '沿同一物理线依次点击，至少 3 点；Backspace 撤销。')
        ttk.Label(self,textvariable=self.note,wraplength=900).pack(fill='x',padx=10)
        buttons=ttk.Frame(self);buttons.pack(fill='x',padx=10,pady=10)
        ttk.Button(buttons,text='使用选择',command=self.accept).pack(side='left')
        ttk.Button(buttons,text='撤销 / 清空',command=self.undo).pack(side='left',padx=8)
        ttk.Button(buttons,text='取消',command=self.destroy).pack(side='right')
        self.canvas.bind('<ButtonPress-1>',self.press);self.canvas.bind('<B1-Motion>',self.drag)
        self.canvas.bind('<ButtonRelease-1>',self.release)
        self.bind('<BackSpace>',lambda e:self.undo());self.bind('<Escape>',lambda e:self.destroy())
        self.bind('<Return>',lambda e:self.accept())

    def point(self,event):
        return (min(self.width,max(0,event.x*self.sx)),min(self.height,max(0,event.y*self.sy)))

    def press(self,event):
        self.start=self.point(event)
        if self.mode=='points':
            self.points.append([min(self.width-1,self.start[0]),min(self.height-1,self.start[1])]);self.draw_selection()

    def drag(self,event):
        if self.mode=='rect' and self.start is not None:
            end=self.point(event)
            self.rect=[min(self.start[0],end[0]),min(self.start[1],end[1]),max(self.start[0],end[0]),max(self.start[1],end[1])]
            self.draw_selection()

    def release(self,event):
        self.drag(event);self.start=None

    def draw_selection(self):
        for item in self.overlay:self.canvas.delete(item)
        self.overlay=[]
        if self.mode=='rect' and self.rect:
            x0,y0,x1,y1=self.rect
            self.overlay.append(self.canvas.create_rectangle(x0/self.sx,y0/self.sy,x1/self.sx,y1/self.sy,outline='#31e3b1',width=2))
            self.note.set(f'原图范围：({math.floor(x0)}, {math.floor(y0)}) → ({math.ceil(x1)}, {math.ceil(y1)})')
        else:
            for x,y in self.points:
                x,y=x/self.sx,y/self.sy
                self.overlay.append(self.canvas.create_oval(x-3,y-3,x+3,y+3,fill='#ff5b65',outline='white'))
            self.note.set(f'已选 {len(self.points)} 点；请沿同一条线均匀取点。')

    def undo(self):
        if self.points:self.points.pop()
        self.rect=None;self.draw_selection()

    def accept(self):
        if self.mode=='rect':
            if not self.rect:return
            x0,y0,x1,y1=self.rect;x0,y0=math.floor(x0),math.floor(y0);x1,y1=math.ceil(x1),math.ceil(y1)
            if x1<=x0 or y1<=y0:return
            value=[x0,y0,x1-x0,y1-y0]
        else:
            if len(self.points)<3:self.note.set('至少选择 3 个点');return
            value=list(self.points)
        self.callback(value);self.destroy()


class OpticalToolsPanel(ttk.Frame):
    def __init__(self,master,on_status=None):
        super().__init__(master,padding=12)
        self.on_status=on_status
        self.runner=OfflineJobRunner();self._closed=False;self._after=None;self._result_dir=None
        self._selected_points=None;self._selected_points_image=None
        self.output=tk.StringVar(value=str(ROOT/'results'/'optical'))
        self.cols=tk.StringVar(value='10');self.rows=tk.StringVar(value='5');self.square=tk.StringVar(value='20')
        head=ttk.Frame(self);head.pack(fill='x')
        ttk.Label(head,text='标定与光学测量',font=('Microsoft YaHei UI',16,'bold')).pack(side='left')
        ttk.Label(self,text='只处理文件。候选标定不会覆盖或上传当前 C97 联合参数。',wraplength=1000).pack(anchor='w',pady=(3,9))
        row=ttk.Frame(self);row.pack(fill='x',pady=(0,8))
        ttk.Label(row,text='结果目录').pack(side='left')
        ttk.Entry(row,textvariable=self.output).pack(side='left',fill='x',expand=True,padx=8)
        ttk.Button(row,text='选择…',command=self.choose_output).pack(side='left')
        ttk.Button(row,text='打开最近结果',command=self.open_results).pack(side='left',padx=5)
        self.tabs=ttk.Notebook(self);self.tabs.pack(fill='both',expand=True)
        self._build_calibration();self._build_geometry();self._build_edge();self._build_charts()
        status=ttk.Frame(self);status.pack(fill='x',pady=8)
        self.status=tk.StringVar(value='选择输入，明确点击运行。每次任务单独保存结果。')
        ttk.Label(status,textvariable=self.status,wraplength=900).pack(side='left',fill='x',expand=True)
        self.cancel_button=ttk.Button(status,text='取消任务',command=self.runner.cancel,state='disabled');self.cancel_button.pack(side='right')
        self.progress=ttk.Progressbar(self,mode='indeterminate');self.progress.pack(fill='x')
        self.log=tk.Text(self,height=7,wrap='word',state='disabled',font=('Consolas',10))
        self.log.pack(fill='both',expand=False,pady=(7,0))
        self._after=self.after(100,self._poll)

    def _page(self,title):
        p=ttk.Frame(self.tabs,padding=12);p.columnconfigure(1,weight=1);self.tabs.add(p,text=title);return p

    def _path_row(self,page,row,label,var,folder=False):
        ttk.Label(page,text=label).grid(row=row,column=0,sticky='w',padx=(0,8),pady=5)
        ttk.Entry(page,textvariable=var).grid(row=row,column=1,sticky='ew',pady=5)
        def choose():
            value=filedialog.askdirectory(parent=self) if folder else filedialog.askopenfilename(parent=self,filetypes=IMAGE_TYPES if label!='点坐标 CSV' else [('CSV','*.csv')])
            if value:var.set(value)
        ttk.Button(page,text='选择…',command=choose).grid(row=row,column=2,padx=(8,0),pady=5)

    def _board_row(self,page,row):
        group=ttk.Frame(page);group.grid(row=row,column=0,columnspan=3,sticky='w',pady=8)
        for label,var in [('内角点列',self.cols),('内角点行',self.rows),('格长 mm',self.square)]:
            ttk.Label(group,text=label).pack(side='left',padx=(0,5));ttk.Entry(group,textvariable=var,width=7).pack(side='left',padx=(0,16))

    def _build_calibration(self):
        page=self._page('相机标定');self.cal_images=tk.StringVar();self.cal_model=tk.StringVar(value='fisheye');self.synthetic=tk.BooleanVar(value=False)
        self._path_row(page,0,'照片文件夹',self.cal_images,True);self._board_row(page,1)
        model=ttk.Frame(page);model.grid(row=2,column=0,columnspan=3,sticky='w',pady=6)
        ttk.Label(model,text='候选模型').pack(side='left',padx=(0,8))
        ttk.Radiobutton(model,text='纯鱼眼四系数',variable=self.cal_model,value='fisheye').pack(side='left',padx=8)
        ttk.Radiobutton(model,text='Brown 径向＋切向',variable=self.cal_model,value='brown').pack(side='left',padx=8)
        ttk.Checkbutton(page,text='输入是合成图（仅软件测试）',variable=self.synthetic).grid(row=3,column=0,columnspan=3,sticky='w',pady=6)
        ttk.Button(page,text='计算候选标定',command=lambda:self._act(self.start_calibration)).grid(row=4,column=0,sticky='w',pady=8)
        ttk.Label(page,text='至少 10 张同一采集模式的照片，建议 20～30 张，覆盖不同位置、角度和画面边缘。\n纯 fisheye 与 Brown 均不是当前 fisheye4＋tangential2 联合模型，禁止把新 K/D 与旧 p1/p2 直接拼接。',wraplength=900).grid(row=5,column=0,columnspan=3,sticky='w',pady=6)

    def _build_geometry(self):
        page=self._page('棋盘与直线度');self.geometry_image=tk.StringVar();self.points_file=tk.StringVar()
        self._path_row(page,0,'实拍图像',self.geometry_image);self._board_row(page,1)
        buttons=ttk.Frame(page);buttons.grid(row=2,column=0,columnspan=3,sticky='w',pady=6)
        ttk.Button(buttons,text='检测棋盘格距 / 直线度',command=lambda:self._act(self.start_checker)).pack(side='left')
        ttk.Button(buttons,text='在图中沿直线选点…',command=lambda:self._act(self.pick_line)).pack(side='left',padx=8)
        self.points_note=tk.StringVar(value='手动选点或选择已有 x,y CSV，再运行直线度测量。')
        self._path_row(page,3,'点坐标 CSV',self.points_file)
        ttk.Button(page,text='测量所选直线',command=lambda:self._act(self.start_line)).grid(row=4,column=0,sticky='w',pady=6)
        ttk.Label(page,textvariable=self.points_note,wraplength=900).grid(row=5,column=0,columnspan=3,sticky='w',pady=6)
        ttk.Label(page,text='斜拍产生格距变化是正常透视；直线度与格距均匀性应分别判断。手选点结果包含点击误差。',wraplength=900).grid(row=6,column=0,columnspan=3,sticky='w',pady=6)

    def _build_edge(self):
        page=self._page('斜边清晰度');self.edge_image=tk.StringVar();self.roi=[tk.StringVar(value=s) for s in ['0','0','128','192']]
        self.axis=tk.StringVar(value='vertical');self.transfer=tk.StringVar(value='srgb');self.gamma=tk.StringVar(value='2.2');self.source=tk.StringVar(value='real-chart')
        self._path_row(page,0,'斜边实拍图',self.edge_image)
        roirow=ttk.Frame(page);roirow.grid(row=1,column=0,columnspan=3,sticky='w',pady=6)
        for label,var in zip(['X','Y','宽','高'],self.roi):
            ttk.Label(roirow,text=label).pack(side='left',padx=(0,5));ttk.Entry(roirow,textvariable=var,width=7).pack(side='left',padx=(0,12))
        ttk.Button(roirow,text='在图中框选…',command=lambda:self._act(self.pick_roi)).pack(side='left')
        settings=ttk.Frame(page);settings.grid(row=2,column=0,columnspan=3,sticky='w',pady=6)
        for label,var,values in [('边方向',self.axis,['vertical','horizontal']),('传递函数',self.transfer,['srgb','linear','gamma']),('来源',self.source,['real-chart','synthetic-test'])]:
            ttk.Label(settings,text=label).pack(side='left',padx=(0,5));ttk.Combobox(settings,textvariable=var,values=values,state='readonly',width=14).pack(side='left',padx=(0,12))
        ttk.Label(page,text='幂律 Gamma').grid(row=3,column=0,sticky='w',pady=6);ttk.Entry(page,textvariable=self.gamma,width=9).grid(row=3,column=1,sticky='w')
        ttk.Button(page,text='计算近似 SFR / MTF50',command=lambda:self._act(self.start_edge)).grid(row=4,column=0,sticky='w',pady=8)
        ttk.Label(page,text='框内只含一条斜边：沿边至少 48 px、跨边至少 32 px，倾斜约 3～7°。vertical=近竖直边，horizontal=近水平边。\n传递函数是需要记录的输入假设，不会自动识别相机 Gamma。结果用于工程对比，不是完整 ISO 12233 认证。',wraplength=900).grid(row=5,column=0,columnspan=3,sticky='w',pady=6)

    def _build_charts(self):
        page=self._page('测试图卡');self.chart_format=tk.StringVar(value='both');self.dpi=tk.StringVar(value='300')
        self._board_row(page,0)
        group=ttk.Frame(page);group.grid(row=1,column=0,columnspan=3,sticky='w',pady=8)
        for label,var,choices in [('格式',self.chart_format,['both','svg','png']),('PNG DPI',self.dpi,['150','300','600'])]:
            ttk.Label(group,text=label).pack(side='left',padx=(0,8));ttk.Combobox(group,textvariable=var,values=choices,state='readonly',width=8).pack(side='left',padx=(0,16))
        ttk.Button(page,text='生成三类 SVG / PNG 图卡',command=lambda:self._act(self.start_charts)).grid(row=2,column=0,columnspan=2,sticky='w',pady=8)
        ttk.Button(page,text='导出已有 A4 可打印 PDF',command=lambda:self._act(self.export_pdf)).grid(row=3,column=0,columnspan=2,sticky='w',pady=8)
        ttk.Label(page,text='三类图卡：标定棋盘、10 mm 网格、5° 斜边。SVG/PNG 使用上方棋盘参数；已有 PDF 固定为 10×5 内角点、20 mm 格长。\nA4 横向、实际大小 100% 打印，关闭“适合页面”。打印后测量两根 100 mm 标尺，贴平整哑光硬板。\n这是工程图卡，不是尺寸精度认证的标定板；PDF 是之前已检查的原文件副本。',wraplength=900).grid(row=4,column=0,columnspan=3,sticky='w',pady=6)

    def _act(self,action):
        try:action()
        except Exception as exc:
            self._status(str(exc));messagebox.showerror('离线工具未执行',str(exc),parent=self.winfo_toplevel())

    def _status(self,text):
        self.status.set(text)
        if self.on_status:self.on_status(text)

    def choose_output(self):
        path=filedialog.askdirectory(parent=self)
        if path:self.output.set(path)

    def _start(self,command,out,label):
        if self.runner.busy:raise RuntimeError('已有任务运行中，请先完成或取消')
        self.runner.start(command,out);self._result_dir=out
        self._status(label+'运行中…');self.cancel_button.configure(state='normal');self.progress.start(15)
        self.log.configure(state='normal');self.log.delete('1.0','end');self.log.configure(state='disabled')

    def start_calibration(self):
        out=output_run(self.output.get(),'calibration')
        command=build_calibration_command(self.cal_images.get(),out,self.cols.get(),self.rows.get(),self.square.get(),self.cal_model.get(),self.synthetic.get())
        self._start(command,out,'候选标定')

    def start_checker(self):
        out=output_run(self.output.get(),'checker')
        self._start(build_checker_command(self.geometry_image.get(),out,self.cols.get(),self.rows.get()),out,'棋盘测量')

    def start_edge(self):
        out=output_run(self.output.get(),'edge')
        self._start(build_edge_command(self.edge_image.get(),out,[v.get() for v in self.roi],self.axis.get(),self.transfer.get(),self.gamma.get(),self.source.get()),out,'斜边测量')

    def pick_roi(self):
        path=input_file(self.edge_image.get())
        def accept(roi):
            for var,value in zip(self.roi,roi):var.set(str(value))
        ImageSelection(self,path,'rect',accept)

    def pick_line(self):
        path=input_file(self.geometry_image.get())
        def accept(points):
            self._selected_points=points;self._selected_points_image=str(path);self.points_file.set('')
            self.points_note.set(f'已在 {path.name} 选择 {len(points)} 个点；点击测量所选直线。')
        ImageSelection(self,path,'points',accept)

    def start_line(self):
        if self.runner.busy:raise RuntimeError('已有任务运行中')
        out=output_run(self.output.get(),'line')
        if self.points_file.get().strip():points=input_file(self.points_file.get())
        elif self._selected_points:
            if str(input_file(self.geometry_image.get()))!=self._selected_points_image:
                raise ValueError('图像已改变，请重新选点')
            # Store selections next to this unique result directory; command creates result dir itself.
            out.parent.mkdir(parents=True,exist_ok=True)
            points=out.parent/(out.name+'_input_points.csv')
            with points.open('w',encoding='utf-8',newline='') as handle:
                import csv
                writer=csv.writer(handle);writer.writerow(['x','y']);writer.writerows(self._selected_points)
        else:raise ValueError('请先选择 CSV 或在图中至少选 3 点')
        self._start(build_line_command(points,out),out,'直线度测量')

    def start_charts(self):
        out=output_run(self.output.get(),'charts')
        self._start(build_chart_command(out,self.cols.get(),self.rows.get(),self.square.get(),int(self.dpi.get()),self.chart_format.get()),out,'图卡生成')

    def export_pdf(self):
        if self.runner.busy:raise RuntimeError('请先完成当前任务')
        out=output_run(self.output.get(),'printable_pdf');out.mkdir(parents=True,exist_ok=False)
        for name in ['A4_calibration_and_quality_targets.pdf','target_specifications.json']:
            shutil.copyfile(TOOLS/'assets'/name,out/name)
        self._result_dir=out;self._status('已复制原 A4 图卡；实际大小 100% 打印，20 mm 格长。')

    def open_results(self):
        path=self._result_dir or Path(self.output.get()).expanduser()
        if not path.is_dir():self._status('尚无结果目录，请先运行一次工具。');return
        if os.name=='nt':os.startfile(str(path))
        else:self._status('结果目录：'+str(path))

    def _poll(self):
        if self._closed:return
        try:
            while True:
                kind,value=self.runner.events.get_nowait()
                if kind=='output':
                    self.log.configure(state='normal');self.log.insert('end',value)
                    if int(self.log.index('end-1c').split('.')[0])>1500:self.log.delete('1.0','500.0')
                    self.log.see('end');self.log.configure(state='disabled')
                else:
                    self.progress.stop();self.cancel_button.configure(state='disabled')
                    label='已取消' if value.cancelled else '完成' if value.returncode==0 else '失败，请查看输出原因'
                    self._status(label+'：'+value.directory)
        except queue.Empty:pass
        self._after=self.after(100,self._poll)

    def close(self):
        if self._closed:return
        self._closed=True
        if self._after:
            try:self.after_cancel(self._after)
            except tk.TclError:pass
        self.runner.close()
