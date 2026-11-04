"""C127 IMU and view controls; connection remains receive-only by default."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
STATE_NAMES = {
    'disconnected': '未连接', 'monitor': '只读观察', 'geometry': '检查补偿矩阵',
    'ready': '等待发送', 'writing': '正在发送', 'await_accepted': '等待 FPGA 接受',
    'await_published': '等待映射发布', 'await_output': '等待对应版本输出',
    'output_confirmed': '对应版本已输出', 'published_output_timeout': '映射已发布，继续等待输出',
    'frozen': '已停止提交，请查看原因', 'candidate_discarded': '候选姿态已弃用',
    'geometry_failed': '矩阵检查未通过', 'quantized_unchanged': '参数未变化，已跳过',
    'board_rejected': 'FPGA 已拒绝新映射，保留原配置',
    'superseded_or_output_unobserved': '已被更新或输出未观察到（不计输出成功）',
    'published_output_unverifiable': '板复位或身份变化，旧版本输出无法继续确认',
    'stopped': '已停止提交', 'closed': '已关闭',
}


def scan_status_text(data):
    scan = data.get('scan') or {}
    names = {0: '无附加模式', 1: '水平镜像', 2: '扫描', 3: '扫描＋镜像'}
    def name(value):
        return names.get(value, '待确认')
    displayed = scan.get('displayed')
    display_text = '待同版本输出证据'
    if displayed:
        display_text = ('几何原图（附加模式暂停）' if displayed.get('raw') else name(displayed.get('modes')))
        display_text += f"，版本 {displayed.get('version', '—')}，源帧 {displayed.get('source_frame_id', '—')}"
    lines = [f"请求：{name(scan.get('requested_modes'))}　|　已提交：{name(scan.get('committed_modes'))}",
             '已显示：' + display_text,
             f"矩阵上下文：{'有效' if scan.get('matrix_valid') else '未确认'}；平面：{'有效' if scan.get('plane_valid') else '未确认'}"]
    if scan.get('pending'):
        lines.append('正在处理：' + str(scan['pending']))
    if scan.get('message'):
        lines.append(str(scan['message']))
    return '\n'.join(lines)


def telemetry_text(data):
    import math
    imu = data.get('imu') or {}
    board = data.get('last_status') or {}
    health = data.get('health_status') or board
    enhancement = data.get('enhancement') or board
    reference = data.get('reference')
    request = data.get('request') or {}
    q = imu.get('quaternion')
    euler = imu.get('euler_rad')
    lines = [
        '姿态数据',
        '  四元数 [w, x, y, z]：' + ('  '.join(f'{v:.6f}' for v in q) if q else '等待数据'),
        '  欧拉角字段 [1, 2, 3]（度）：' + ('  '.join(f'{math.degrees(v):.2f}' for v in euler) if euler else '等待数据'),
        '  欧拉角用于方向核对；补偿计算使用四元数。',
        '  参考姿态：' + ('已记录' if reference else '未记录'),
        '  安装轴向：' + ('已确认' if data.get('mounting_confirmed') else '尚未确认'),
        '', 'FPGA 状态',
        '  固件：' + (f"0x{board['build_id']:08X}" if board.get('build_id') is not None else '等待数据'),
        '  初始化：' + ' / '.join(f"{name} {'完成' if health.get(key) else '待确认'}" for name, key in
                                 [('DDR', 'ddr_initialized'), ('相机', 'camera_config_done'), ('HDMI', 'hdmi_config_done')]),
        f"  提交计数：{board.get('commits', '—')}  |  拒绝计数：{board.get('rejected', '—')}",
        '  允许新提交：' + ('是' if data.get('can_send') else '否'),
        f"  板端增强模式：{enhancement.get('enhancement_mode', '待确认')}  |  Gamma 档：{enhancement.get('gamma_id', '待确认')}",
    ]
    if data.get('send_blocked_reason'):
        lines.append('  待满足条件：' + str(data['send_blocked_reason']))
    pose = data.get('latest_pose_status') or {}
    if pose:
        lines += [f"  C125 发布保护：{'支持' if pose.get('hardware_admission_capable') else '未确认'}",
                  f"  最近建表无效像素：{pose.get('map_invalid_pixels', '—')}（其中运算域黑区 {pose.get('map_domain_pixels', '—')}）",
                  f"  建表拒绝标志：{pose.get('reject_flags', '—')}；控制失败计数：{pose.get('control_failures', '—')}",
                  f"  等待同版本输出：{len(data.get('pending_outputs', []))}/{data.get('max_pending_outputs', '—')}"]
    if data.get('realtime'):
        def rate(value):
            return f'{value:.2f} Hz' if value is not None else '待测'
        lines += [f"  请求目标：{rate(data.get('target_pose_hz'))}",
                  f"  电脑观察到的发布/输出确认频率：{rate(data.get('publication_observed_hz'))} / {rate(data.get('output_confirmation_observed_hz'))}",
                  '  上述是消息到达频率；IMU 采样、HDMI 刷新和板上每秒新矫正帧必须分别测量。']
        evidence = data.get('version_evidence') or {}
        lines.append(f"  版本证据：已发布 {evidence.get('published', 0)} / 输出确认 {evidence.get('output_confirmed', 0)} / 已被更新或未观察到 {evidence.get('superseded_or_output_unobserved', 0)} / 身份失效无法确认 {evidence.get('published_output_unverifiable', 0)} / 待确认 {evidence.get('pending', 0)}")
    lines += ['', '最近一次补偿']
    if request:
        lines += [f"  请求：{request.get('request_id', '—')}",
                  '  阶段：' + STATE_NAMES.get(request.get('phase'), str(request.get('phase'))),
                  f"  对应源帧：{request.get('source_frame_id', '待确认')}"]
        for key, label in [('geometry_started_at', '开始计算'), ('geometry_done_at', '计算完成'),
                           ('write_started_at', '开始发送'), ('write_done_at', '发送完成'),
                           ('accepted_at', '接受确认'), ('published_at', '发布确认'), ('output_at', '输出证据接收'),
                           ('output_confirmed_at', '全部确认条件齐备')]:
            if key in request:
                lines.append(f"  {label}：{request[key]:.6f} 秒（电脑单调时钟）")
        if request.get('reason'):
            lines.append('  原因：' + str(request['reason']))
        if request.get('admission'):
            lines.append('  准入：' + ('FPGA 全图发布检查（域失效像素为黑）' if request['admission']=='fpga_full_map_guard' else '电脑精确整数检查'))
        if request.get('pose_age_at_write_s') is not None:
            lines.append(f"  开始发送时姿态年龄：{request['pose_age_at_write_s']*1000:.1f} ms")
    else:
        lines.append('  尚未发送；连接操作本身不会写入配置。')
    lines += ['', '通信检查', '  IMU：' + json_text(imu.get('decoder_counters', {})),
              '  FPGA：' + json_text(data.get('framing_counters', {}))]
    if 'scan' in data:
        mode = {'normal': '普通去畸变', 'roll_only': '仅倾斜回正', 'full': '完整三轴补偿'}.get(data.get('pose_mode'), '待确认')
        lines += ['', '当前会话回正设置：' + mode,
                  '固定视野裁边：' + (json_text(data['crop_profile']) if data.get('crop_profile') else '关闭'),
                  '', '扫描／镜像（请求、提交、显示分别确认）', scan_status_text(data)]
    return '\n'.join(lines)


def new_log_path():
    return ROOT / 'logs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jsonl')


def json_text(value):
    return json.dumps(value, ensure_ascii=False, indent=2, default=str)


def read_mounting(path):
    from imu_pose import validate_mounting_rotation
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    validate_mounting_rotation(data['R_IC'])
    if data.get('convention') not in ('imu_to_world', 'world_to_imu'):
        raise ValueError('convention 必须是 imu_to_world 或 world_to_imu')
    return data


def save_mounting(path, mounting):
    """Persist the measured rotation/provenance, never a reusable confirmation."""
    from imu_pose import validate_mounting_rotation
    matrix = validate_mounting_rotation(mounting['R_IC'])
    convention = mounting.get('convention')
    if convention not in ('imu_to_world', 'world_to_imu'):
        raise ValueError('convention 必须是 imu_to_world 或 world_to_imu')
    data = dict(mounting, R_IC=matrix.tolist(), convention=convention)
    data.pop('confirmed', None)
    data['saved_at'] = datetime.now().isoformat(timespec='seconds')
    data['confirmation_policy'] = 'Reconfirm physical mounting in every new session.'
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return data


def read_capture_metadata(path, expected_context_hash):
    """A pose-bound photo must carry its own actual H, not the latest pose."""
    import numpy as np
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    matrix = np.asarray(data.get('capture_h'), dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-10:
        raise ValueError('照片元数据缺少有效的 capture_h 3×3 矩阵。')
    if data.get('capture_context_hash') != expected_context_hash:
        raise ValueError('照片元数据的 capture_context_hash 与当前参考／视野不匹配。')
    if not isinstance(data.get('capture_id'), str) or not data['capture_id'].strip():
        raise ValueError('照片元数据必须包含该实际帧的 capture_id。')
    return {key: data[key] for key in ('capture_h', 'capture_context_hash', 'capture_id')}


class ControlPanel:
    def __init__(self, window):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.window = tk, ttk, window
        self.session = None
        self.mounting = None
        self.scan_profile = None
        self._applied_pose_settings = ('roll_only', None)
        import queue
        self.validation_results = queue.Queue()
        self.validation_busy = False
        self.scan_validation = None
        self.last_log = None
        self.closing = False
        window.title('C127 · 图像回正、平面扫描与镜像')
        window.geometry('1020x820')
        window.minsize(850, 720)
        window.protocol('WM_DELETE_WINDOW', self.close)
        self.tabs = ttk.Notebook(window)
        self.tabs.pack(fill='both', expand=True)
        frame = ttk.Frame(self.tabs, padding=18)
        self.tabs.add(frame, text='连接与回正')
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text='IMU 与 FPGA 联调', font=('Microsoft YaHei UI', 19, 'bold')).grid(row=0, column=0, sticky='w')
        ttk.Label(frame, text='安装：IMU 固定主板，相机与主板共同转动。先观察姿态，再核对轴向并启用低速补偿。').grid(row=1, column=0, sticky='w', pady=(5, 12))
        connections = ttk.LabelFrame(frame, text='1  连接设备（连接后默认只接收）', padding=10)
        connections.grid(row=2, column=0, sticky='ew')
        self.imu_port = tk.StringVar()
        self.fpga_port = tk.StringVar()
        ttk.Label(connections, text='IMU · CH340').grid(row=0, column=0, sticky='w')
        self.imu_combo = ttk.Combobox(connections, textvariable=self.imu_port, state='readonly', width=28)
        self.imu_combo.grid(row=0, column=1, padx=8, sticky='w')
        ttk.Label(connections, text='FPGA · CP210x').grid(row=1, column=0, sticky='w', pady=8)
        self.fpga_combo = ttk.Combobox(connections, textvariable=self.fpga_port, state='readonly', width=28)
        self.fpga_combo.grid(row=1, column=1, padx=8, sticky='w')
        ttk.Button(connections, text='刷新设备', command=lambda: self.act(self.refresh_ports)).grid(row=0, column=2, padx=4)
        ttk.Button(connections, text='连接', command=lambda: self.act(self.connect)).grid(row=0, column=3, padx=4)
        ttk.Button(connections, text='断开', command=lambda: self.act(self.disconnect)).grid(row=1, column=3, padx=4)
        ttk.Button(connections, text='IMU 设置 100 Hz', command=lambda: self.act(lambda: self.require_session().set_imu_100hz())).grid(row=1, column=2, padx=4)
        self.port_note = tk.StringVar()
        ttk.Label(connections, textvariable=self.port_note, wraplength=900).grid(row=2, column=0, columnspan=4, sticky='w')
        reference = ttk.LabelFrame(frame, text='2  参考方向与安装关系', padding=10)
        reference.grid(row=3, column=0, sticky='ew', pady=10)
        ttk.Button(reference, text='记录当前参考姿态', command=lambda: self.act(lambda: self.require_session().capture_reference())).grid(row=0, column=0, sticky='w')
        ttk.Button(reference, text='载入安装矩阵…', command=lambda: self.act(self.load_mounting)).grid(row=0, column=1, padx=10)
        ttk.Button(reference, text='另存安装矩阵…', command=lambda: self.act(self.save_mounting)).grid(row=0, column=2, padx=6)
        self.mount_note = tk.StringVar(value='尚未确认轴向；示例单位矩阵不代表真实安装关系。')
        ttk.Label(reference, textvariable=self.mount_note, wraplength=890).grid(row=1, column=0, columnspan=3, sticky='w', pady=6)
        self.mount_confirmed = tk.BooleanVar(value=False)
        ttk.Checkbutton(reference, text='我已用三个轴小角度转动核对 R_IC 和四元数方向', variable=self.mount_confirmed,
                        command=lambda: self.act(self.apply_mounting)).grid(row=2, column=0, columnspan=3, sticky='w')
        control = ttk.LabelFrame(frame, text='3  发送补偿', padding=10)
        control.grid(row=4, column=0, sticky='ew')
        ttk.Button(control, text='单次补偿', command=lambda: self.act(lambda: self.require_session().send_once())).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(control, text='开始低速连续', command=lambda: self.act(self.start_continuous)).grid(row=0, column=1, padx=8)
        self.interval = tk.StringVar(value='2.0')
        ttk.Label(control, text='最小间隔（秒）').grid(row=0, column=2)
        ttk.Entry(control, textvariable=self.interval, width=6).grid(row=0, column=3, padx=8)
        ttk.Button(control, text='停止提交', command=lambda: self.act(lambda: self.require_session().stop())).grid(row=0, column=4, padx=8)
        ttk.Button(control, text='快速连续（目标 30 Hz）',
                   command=lambda: self.act(lambda: self.require_session().start_realtime())).grid(row=1, column=0, columnspan=2, sticky='w', pady=(8, 0))
        ttk.Label(control, text='需 C125 发布保护；首次全量检查。停止保留已发布配置。').grid(row=1, column=2, columnspan=3, sticky='w', pady=(8, 0))
        ttk.Button(control, text='确认 ROI／全黑拒绝',
                   command=lambda: self.act(lambda: self.require_session().acknowledge_geometry_rejection())).grid(row=2, column=0, columnspan=2, sticky='w', pady=(8, 0))
        ttk.Label(control, text='仅解除已证明的几何拒绝；不自动重试。').grid(row=2, column=2, columnspan=3, sticky='w', pady=(8, 0))
        ttk.Label(control, text='接受 → 发布 → 同版本输出分别记录；目标频率不是已测实板帧率，尚不绑定曝光。').grid(row=3, column=0, columnspan=5, sticky='w', pady=(8, 0))
        self.enhancement_mode = tk.StringVar(value='0 旁路')
        ttk.Label(control, text='图像增强').grid(row=4, column=0, sticky='w', pady=(8, 0))
        ttk.Combobox(control, textvariable=self.enhancement_mode, state='readonly', width=16,
                     values=('0 旁路', '1 自动', '2 阴影', '3 高光', '4 直方图')).grid(row=4, column=1, sticky='w', pady=(8, 0))
        ttk.Button(control, text='应用增强', command=lambda: self.act(self.apply_enhancement)).grid(row=4, column=2, pady=(8, 0))
        self.enhancement_status = tk.StringVar(value='板端模式／Gamma：待确认')
        ttk.Label(control, textvariable=self.enhancement_status).grid(row=4, column=3, columnspan=2, sticky='w', padx=8)
        self.summary = tk.StringVar(value='未连接')
        ttk.Label(frame, textvariable=self.summary, font=('Microsoft YaHei UI', 11), wraplength=970).grid(row=5, column=0, sticky='ew', pady=12)
        details = ttk.Frame(self.tabs, padding=12)
        self.tabs.add(details, text='状态与验证证据')
        self.details = tk.Text(details, font=('Consolas', 10), height=16, wrap='word', state='disabled')
        scroll = ttk.Scrollbar(details, command=self.details.yview)
        self.details.configure(yscrollcommand=scroll.set)
        self.details.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        self.footer = tk.StringVar(value='日志保存在本目录 logs；断开或姿态过期会停止新提交。')
        ttk.Label(frame, textvariable=self.footer, wraplength=970).grid(row=7, column=0, sticky='w', pady=(10, 0))
        self.build_view_controls()
        self.refresh_ports()
        window.after(250, self.update)

    def build_view_controls(self):
        tk, ttk = self.tk, self.ttk
        page = ttk.Frame(self.tabs, padding=18)
        self.tabs.insert(1, page, text='模式、裁边与平面扫描')
        page.columnconfigure(0, weight=1)
        pose = ttk.LabelFrame(page, text='回正模式', padding=12)
        pose.grid(row=0, column=0, sticky='ew')
        self.pose_mode = tk.StringVar(value='roll_only')
        for row, (value, label) in enumerate([
                ('normal', '普通去畸变（不跟随姿态）'),
                ('roll_only', '仅回正画面倾斜（默认；忽略该相机分解中的俯仰/偏航）'),
                ('full', '完整三轴补偿（保留原功能）')]):
            ttk.Radiobutton(pose, text=label, value=value, variable=self.pose_mode).grid(row=row, column=0, sticky='w')
        ttk.Button(pose, text='应用模式设置（停止连续提交）', command=lambda: self.act(self.apply_pose_mode)).grid(row=3, column=0, sticky='w', pady=8)
        ttk.Button(pose, text='发送普通去畸变', command=lambda: self.act(self.send_normal)).grid(row=3, column=1, padx=10)
        ttk.Label(pose, text='模式设置不会自动发送图像配置；普通模式不需要 IMU。回正以记录的参考方向为准。', wraplength=900).grid(row=4, column=0, columnspan=2, sticky='w')
        crop = ttk.LabelFrame(page, text='固定视野裁边（减少黑边；仅用于倾斜回正）', padding=12)
        crop.grid(row=1, column=0, sticky='ew', pady=12)
        self.crop_enabled = tk.BooleanVar(value=False)
        self.crop_zoom = tk.StringVar(value='1.3')
        self.crop_range = tk.StringVar(value='15')
        ttk.Checkbutton(crop, text='启用固定缩放', variable=self.crop_enabled).grid(row=0, column=0, sticky='w')
        ttk.Label(crop, text='缩放倍数').grid(row=0, column=1, padx=8)
        ttk.Entry(crop, textvariable=self.crop_zoom, width=7).grid(row=0, column=2)
        ttk.Label(crop, text='圆周范围 ± 度（180 为整圈）').grid(row=0, column=3, padx=8)
        ttk.Entry(crop, textvariable=self.crop_range, width=7).grid(row=0, column=4)
        ttk.Label(crop, text='缩放固定，不随角度变化；按“应用模式设置”生效。范围外保留上一有效结果。\n固定缩放用于减少黑边；保留 FPGA 映射准入保护，不保证任意角度或全画幅无黑边。\n更改裁边会使平面参数过期，请重新拟合或加载匹配参数并初始化。', wraplength=900).grid(row=1, column=0, columnspan=5, sticky='w', pady=8)
        scan = ttk.LabelFrame(page, text='平面扫描与最终水平镜像', padding=12)
        scan.grid(row=2, column=0, sticky='ew')
        ttk.Button(scan, text='加载平面参数…', command=lambda: self.act(self.load_scan_profile)).grid(row=0, column=0, sticky='w')
        ttk.Button(scan, text='从点对应拟合…', command=lambda: self.act(self.fit_scan_profile)).grid(row=0, column=1, padx=8)
        ttk.Button(scan, text='初始化／上传参数', command=lambda: self.act(self.initialize_scan_profile)).grid(row=0, column=2, padx=8)
        ttk.Button(scan, text='仅初始化镜像', command=lambda: self.act(self.initialize_mirror)).grid(row=0, column=3, padx=8)
        ttk.Button(scan, text='离线检查四种模式', command=lambda: self.act(self.validate_scan_profile)).grid(row=1, column=3, padx=8)
        self.scan_enabled = tk.BooleanVar(value=False)
        self.mirror_enabled = tk.BooleanVar(value=False)
        ttk.Checkbutton(scan, text='扫描', variable=self.scan_enabled).grid(row=1, column=0, sticky='w', pady=10)
        ttk.Checkbutton(scan, text='水平镜像', variable=self.mirror_enabled).grid(row=1, column=1, sticky='w')
        ttk.Button(scan, text='发送模式请求', command=lambda: self.act(self.send_view_mode)).grid(row=1, column=2)
        self.scan_note = tk.StringVar(value='未加载平面参数；加载后仍需明确初始化。镜像也需初始化矩阵上下文。')
        ttk.Label(scan, textvariable=self.scan_note, wraplength=900).grid(row=2, column=0, columnspan=3, sticky='w')
        self.view_status = tk.StringVar(value='请求：待确认　|　已提交：待确认　|　已显示：待确认')
        ttk.Label(scan, textvariable=self.view_status, wraplength=900).grid(row=3, column=0, columnspan=3, sticky='w', pady=8)
        ttk.Label(page, text='板上：KEY1 几何原图／恢复；KEY2 扫描；KEY3 水平镜像。\nKEY1 原图期间暂停附加模式，退出后恢复；请求、提交、实际显示状态分别确认。\n平面扫描需固定目标和平面参数；没有在线检测时，不会自动跟踪移动目标。', wraplength=900).grid(row=3, column=0, sticky='w', pady=14)

    def crop_profile(self):
        from roll_crop import FixedRollCrop
        if not self.crop_enabled.get():
            return None
        if self.pose_mode.get() != 'roll_only':
            raise ValueError('固定倾斜裁边仅适用于“仅回正画面倾斜”；请关闭裁边或选择该模式。')
        return FixedRollCrop(float(self.crop_zoom.get()), float(self.crop_range.get())).to_dict()

    def apply_pose_mode(self):
        profile = self.crop_profile()
        session = self.require_session()
        session.set_pose_mode(self.pose_mode.get(), crop_profile=profile)
        # FOV/crop context is part of a plane profile, never silently reuse it.
        settings = (self.pose_mode.get(), profile)
        if settings != self._applied_pose_settings:
            self.scan_profile = None
            self.scan_validation = None
            self.scan_note.set('模式／裁边设置已应用；平面参数已过期，请重新加载或拟合并初始化。')
        self._applied_pose_settings = settings
        return '已停止连续提交并应用设置；已在途事务由会话完成，未自动发送新配置。'

    def send_normal(self):
        session = self.require_session()
        self.pose_mode.set('normal')
        self.crop_enabled.set(False)
        self.apply_pose_mode()
        return session.request_normal()

    def save_mounting(self):
        from tkinter import filedialog
        if self.mounting is None:
            raise ValueError('请先载入经过测量的安装矩阵。')
        path = filedialog.asksaveasfilename(parent=self.window, initialdir=ROOT, title='另存安装矩阵', defaultextension='.json', filetypes=[('JSON', '*.json')])
        if path:
            save_mounting(path, self.mounting)
            return '已保存安装矩阵；新会话仍需确认实际安装未变化。'

    def load_scan_profile(self):
        from tkinter import filedialog
        from scan_plane import load_profile
        session = self.require_session()
        path = filedialog.askopenfilename(parent=self.window, initialdir=ROOT, title='加载匹配当前视野和参考姿态的平面参数', filetypes=[('JSON', '*.json')])
        if path:
            profile = load_profile(path, expected_context=session.scan_context())
            self.scan_profile = profile
            self.scan_note.set(Path(path).name + '：已加载，尚未上传；请明确初始化后再开启扫描。')

    def initialize_scan_profile(self):
        from scan_plane import validate_profile
        session = self.require_session()
        if self.scan_profile is None:
            raise ValueError('请先加载或拟合平面参数；仅需镜像时选择“仅初始化镜像”。')
        validate_profile(self.scan_profile, expected_context=session.scan_context())
        result = session.set_scan_profile(self.scan_profile)
        self.scan_note.set('参数初始化请求已提交；等待板端确认，尚不代表已显示扫描结果。')
        return result

    def initialize_mirror(self):
        from scan_plane import make_mirror_profile
        session = self.require_session()
        self.scan_profile = make_mirror_profile(session.scan_context())
        result = session.set_scan_profile(self.scan_profile)
        self.scan_enabled.set(False)
        self.scan_note.set('仅镜像上下文初始化请求已提交；没有有效平面，扫描不能开启。')
        return result

    def send_view_mode(self):
        return self.require_session().request_view_mode(scan=self.scan_enabled.get(), mirror=self.mirror_enabled.get())

    def validate_scan_profile(self):
        from copy import deepcopy
        import threading
        from scan_plane import validate_modes
        if self.scan_profile is None:
            raise ValueError('请先加载或拟合平面参数。')
        if self.validation_busy:
            raise ValueError('离线检查仍在运行，请等待结果。')
        source_profile = self.scan_profile
        profile = deepcopy(source_profile)
        self.validation_busy = True
        self.scan_note.set('正在按该照片的 H 做四种模式整幅整数／ROI 检查；不会连接硬件或发送坐标。')
        def work():
            try:
                capture_h = profile.get('fit', {}).get('capture_h')
                result = validate_modes(profile, base_h=capture_h)
                self.validation_results.put((source_profile, result, None))
            except Exception as exc:
                self.validation_results.put((source_profile, None, str(exc)))
        threading.Thread(target=work, daemon=True, name='c127-offline-plane-check').start()

    def consume_validation_result(self):
        import queue
        try:
            profile, result, error = self.validation_results.get_nowait()
        except queue.Empty:
            return
        self.validation_busy = False
        if profile is not self.scan_profile:
            self.scan_note.set('检查期间平面上下文已变化，旧检查结果未用于当前参数。')
            return
        if error:
            self.scan_note.set('离线检查失败：' + error)
            return
        self.scan_validation = result
        labels = {'00': '普通', '01': '镜像', '10': '扫描', '11': '扫描＋镜像'}
        lines = []
        for name, mode in result['modes'].items():
            reasons = mode.get('reasons') or [mode.get('reason', '')]
            lines.append(labels[name] + '：' + ('准入通过' if mode['admitted'] else '拒绝 ' + ','.join(reasons))
                         + f"，无效像素 {mode.get('invalid_pixels', '—')}")
        self.scan_note.set('\n'.join(lines) + '\n仅该照片 H 的离线检查；不代表其他姿态、实板或时序验收。')

    def fit_scan_profile(self):
        """Collect real raw-frame points; never synthesize a sample target."""
        from tkinter import filedialog
        from PIL import Image, ImageTk
        session = self.require_session()
        context = session.scan_context()
        path = filedialog.askopenfilename(parent=self.window, title='选择同视野原始照片（1280×720 或 1920×1080，未经裁剪或矫正）',
                                          filetypes=[('图像', '*.png *.jpg *.jpeg *.bmp')])
        if not path:
            return
        source = Image.open(path)
        if source.size not in ((1280, 720), (1920, 1080)):
            source.close()
            raise ValueError('仅支持同视野原始 1280×720 或 1920×1080 照片；请勿裁剪或用已矫正照片代替。')
        source.load()
        capture = {}
        if context.get('reference') is not None:
            metadata_path = filedialog.askopenfilename(parent=self.window,
                title='当前是姿态参考域：加载该照片实际拍摄帧的 H／上下文元数据', filetypes=[('JSON', '*.json')])
            if not metadata_path:
                source.close()
                raise ValueError('姿态模式拟合需要该照片的 capture_h、capture_context_hash、capture_id。普通模式的原始照片可在普通上下文中拟合；不能用最新姿态代替拍摄姿态。')
            try:
                capture = read_capture_metadata(metadata_path, context['context_hash'])
            except Exception:
                source.close()
                raise
        dialog = self.tk.Toplevel(self.window)
        dialog.title('选取同一平面的四角')
        dialog.transient(self.window)
        body = self.ttk.Frame(dialog, padding=12)
        body.pack(fill='both', expand=True)
        self.ttk.Label(body, text='按目标的左上 → 右上 → 右下 → 左下依次点击。四点必须来自同一平面。\n图像仅用于选点，上传的是少量参数；FPGA 生成逐像素映射。').grid(row=0, column=0, columnspan=4, sticky='w')
        display = source.copy()
        display.thumbnail((768, 432))
        photo = ImageTk.PhotoImage(display, master=dialog)
        canvas = self.tk.Canvas(body, width=display.width, height=display.height, highlightthickness=0)
        canvas.grid(row=1, column=0, columnspan=4, pady=10)
        canvas.create_image(0, 0, image=photo, anchor='nw')
        canvas.image = photo
        points = []
        point_text = self.tk.StringVar(value='已选 0/4 点')
        self.ttk.Label(body, textvariable=point_text).grid(row=2, column=0, columnspan=4, sticky='w')
        def pick(event):
            if len(points) >= 4 or not 0 <= event.x < display.width or not 0 <= event.y < display.height:
                return
            points.append([event.x * source.width / display.width, event.y * source.height / display.height])
            canvas.create_oval(event.x-4, event.y-4, event.x+4, event.y+4, outline='#ff4e33', width=2, tags='points')
            canvas.create_text(event.x+12, event.y-10, text=str(len(points)), fill='#ff4e33', tags='points')
            point_text.set(f'已选 {len(points)}/4 点：' + '；'.join(f'{x:.1f},{y:.1f}' for x, y in points))
        canvas.bind('<Button-1>', pick)
        def clear():
            points.clear()
            canvas.delete('points')
            point_text.set('已选 0/4 点')
        self.ttk.Button(body, text='重新选点', command=clear).grid(row=3, column=0, sticky='w')
        self.ttk.Label(body, text='目标实际宽／高').grid(row=3, column=1)
        ratio = self.tk.StringVar(value='')
        self.ttk.Entry(body, textvariable=ratio, width=10).grid(row=3, column=2)
        self.ttk.Label(body, text='比例必须实测；透视照片上的像素宽高不是实际比例。').grid(row=4, column=0, columnspan=4, sticky='w', pady=8)
        def fit():
            from scan_plane import fit_four_points, save_profile
            if len(points) != 4:
                raise ValueError('请选满四个点。')
            # Recheck context after interactive selection so a reference/crop
            # change cannot bless old point geometry as current.
            if session.scan_context()['context_hash'] != context['context_hash']:
                raise ValueError('选点期间参考／视野已变化，请重新打开拟合。')
            profile = fit_four_points(points, float(ratio.get()), context, input_domain='raw', input_size=source.size, **capture)
            save_path = filedialog.asksaveasfilename(parent=dialog, title='保存拟合平面参数', defaultextension='.json', filetypes=[('JSON', '*.json')])
            if not save_path:
                return
            save_profile(profile, save_path)
            self.scan_profile = profile
            self.scan_note.set(Path(save_path).name + '：已拟合并保存，尚未上传；初始化后再开启扫描。')
            source.close()
            dialog.destroy()
        self.ttk.Button(body, text='拟合并另存参数（不发送）', command=lambda: self.act(fit)).grid(row=5, column=0, columnspan=3, sticky='w')
        def cancel():
            source.close()
            dialog.destroy()
        dialog.protocol('WM_DELETE_WINDOW', cancel)

    def act(self, function):
        from tkinter import messagebox
        try:
            result = function()
            if result is not None:
                self.footer.set(str(result))
        except Exception as error:
            self.footer.set(str(error))
            messagebox.showerror('操作未完成', str(error), parent=self.window)

    def require_session(self):
        if self.session is None:
            raise RuntimeError('请先连接设备。')
        return self.session

    def refresh_ports(self):
        from imu_session import list_serial_ports
        if self.session is not None:
            raise RuntimeError('请先断开后再更换端口。')
        ports = list_serial_ports()
        imu = [''] + [p['device'] for p in ports if p.get('role') == 'imu']
        fpga = [''] + [p['device'] for p in ports if p.get('role') == 'fpga']
        self.imu_combo['values'], self.fpga_combo['values'] = imu, fpga
        self.imu_port.set(imu[1] if len(imu) == 2 else '')
        self.fpga_port.set(fpga[1] if len(fpga) == 2 else '')
        self.port_note.set('；'.join(f"{p['device']}  {p['description']}" for p in ports) or '未发现串口')

    def connect(self):
        from c127_session import C127HostSession as HostSession
        if self.session is not None:
            raise RuntimeError('设备已经连接，请先断开。')
        self.last_log = new_log_path()
        session = HostSession(log_path=self.last_log)
        try:
            session.start(self.imu_port.get() or None, self.fpga_port.get() or None)
        except BaseException:
            session.close()
            raise
        self.session = session
        # Every connection is a new session; never carry an old reference.
        self.mount_confirmed.set(False)
        if self.mounting is not None:
            self.mount_note.set('已载入安装矩阵；本次连接尚未应用，请核对实际安装后勾选确认。')
        self.scan_profile = None
        self.pose_mode.set('roll_only')
        self.crop_enabled.set(False)
        self._applied_pose_settings = ('roll_only', None)
        self.scan_note.set('新连接未初始化平面／镜像上下文；请明确加载并初始化。')
        self.view_status.set('请求：待确认　|　已提交：待确认　|　已显示：待确认')
        self.imu_combo.configure(state='disabled')
        self.fpga_combo.configure(state='disabled')
        self.footer.set('已连接，仅接收。日志：' + str(self.last_log))

    def disconnect(self):
        if self.session is not None:
            self.session.close()
            self.session = None
        self.imu_combo.configure(state='readonly')
        self.fpga_combo.configure(state='readonly')
        self.mount_confirmed.set(False)
        self.summary.set('已断开；设备保留上次已生效的配置。')

    def load_mounting(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(parent=self.window, initialdir=ROOT, title='载入已核对的安装旋转', filetypes=[('JSON', '*.json')])
        if not path:
            return
        mounting = read_mounting(path)
        if self.session is not None:
            self.session.stop()
            try:
                self.session.set_mounting(mounting['R_IC'], convention=mounting['convention'], confirmed=False)
            except Exception:
                self.mount_confirmed.set(self.session.snapshot().get('mounting_confirmed', False))
                raise
        self.mounting = mounting
        self.mount_confirmed.set(False)
        self.mount_note.set(Path(path).name + '  |  ' + self.mounting['convention'] + '  |  请核对后勾选确认')

    def apply_mounting(self):
        if self.mounting is None:
            self.mount_confirmed.set(False)
            raise RuntimeError('请先载入安装矩阵；示例不能当作已完成的轴向校准。')
        try:
            session = self.require_session()
            if not self.mount_confirmed.get():
                session.stop()
            session.set_mounting(self.mounting['R_IC'], convention=self.mounting['convention'],
                                 confirmed=self.mount_confirmed.get())
        except Exception:
            actual = self.session.snapshot().get('mounting_confirmed', False) if self.session else False
            self.mount_confirmed.set(actual)
            raise

    def start_continuous(self):
        self.require_session().start_continuous(interval_s=float(self.interval.get()))

    def apply_enhancement(self):
        mode = int(self.enhancement_mode.get().split()[0])
        if mode not in range(5):
            raise ValueError('增强模式必须为 0..4。')
        return self.require_session().request_enhancement(mode)

    def update(self):
        if self.closing:
            return
        self.consume_validation_result()
        if self.session is not None:
            data = self.session.snapshot()
            self.view_status.set(scan_status_text(data))
            imu = data.get('imu', {})
            status = data.get('last_status') or {}
            enhancement = data.get('enhancement') or status
            self.enhancement_status.set(f"板端模式 {enhancement.get('enhancement_mode', '待确认')}／Gamma {enhancement.get('gamma_id', '待确认')}")
            age = imu.get('age_s')
            age_text = f'{age:.3f}s' if isinstance(age, (float, int)) else '无数据'
            rate = imu.get('rate_hz')
            rate_text = f'{rate:.1f}Hz' if isinstance(rate, (float, int)) else '待测'
            version = status.get('config_version', '—')
            state = STATE_NAMES.get(data.get('state'), data.get('state'))
            self.summary.set(f"状态：{state}  |  IMU：{rate_text} / 样本年龄 {age_text}  |  配置版本：{version}\n{data.get('error') or ''}")
            self.details.configure(state='normal')
            self.details.delete('1.0', 'end')
            self.details.insert('1.0', telemetry_text(data))
            self.details.configure(state='disabled')
        self.window.after(250, self.update)

    def close(self):
        self.closing = True
        try:
            self.disconnect()
        finally:
            self.window.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description='C127 桌面上位机；probe 只接收，不发送任何命令。')
    commands = parser.add_subparsers(dest='command')
    commands.add_parser('gui', help='打开桌面控制界面')
    commands.add_parser('ports', help='仅列出串口身份')
    probe = commands.add_parser('probe', help='限时只读双串口诊断')
    probe.add_argument('--imu-port')
    probe.add_argument('--fpga-port')
    probe.add_argument('--seconds', type=float, default=10)
    probe.add_argument('--out', type=Path)
    probe.add_argument('--verbose', action='store_true', help='同时打印完整诊断记录')
    args = parser.parse_args(argv)
    if args.command == 'ports':
        from imu_session import list_serial_ports
        print(json_text(list_serial_ports()))
        return 0
    if args.command == 'probe':
        import math
        if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
            parser.error('--seconds 必须在 (0,60] 内')
        if not (args.imu_port or args.fpga_port):
            parser.error('至少指定一个串口；不会按旧 COM 号自动连接')
        from c127_session import C127HostSession as HostSession
        output = args.out or ROOT / 'logs' / (datetime.now().strftime('probe_%Y%m%d_%H%M%S_%f') + '.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        session = HostSession(log_path=output.with_suffix('.jsonl'))
        try:
            session.start(args.imu_port, args.fpga_port)
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                time.sleep(min(.1, max(0, deadline - time.monotonic())))
            result = session.snapshot()
            result['probe_receive_only'] = True
            result['requested_duration_s'] = args.seconds
            failures = []
            if args.imu_port and (not result['imu'].get('latest') or result['imu'].get('stale')):
                failures.append('没有收到新鲜的 IMU 四元数')
            if args.fpga_port and not result.get('last_status'):
                failures.append('没有收到已验证的 FPGA 状态')
            if args.fpga_port and result['fpga'].get('status_age_s') is not None and result['fpga']['status_age_s'] > session.status_stale_s:
                failures.append('FPGA 状态已过期')
            if result.get('error'):
                failures.append(result['error'])
            result['probe_failures'] = failures
            result['probe_passed'] = not failures
            output.write_text(json_text(result), encoding='utf-8')
            summary = {key: result[key] for key in ('probe_passed', 'probe_failures', 'connected', 'state', 'imu', 'fpga')}
            summary['build_id'] = hex((result.get('last_status') or {}).get('build_id', 0))
            print(json_text(result if args.verbose else summary))
            print('结果：' + str(output))
            return 0 if result['probe_passed'] else 1
        finally:
            session.close()
    import tkinter as tk
    window = tk.Tk()
    ControlPanel(window)
    window.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
