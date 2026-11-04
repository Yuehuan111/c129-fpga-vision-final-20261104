"""Five explicit demo targets plus optional C129 FPGA detail processing."""
import time
from copy import deepcopy

from unified_actions import ActionSequencer, WaitUntil
from roll_crop import checked_demo_crop

MODE_LABELS = {'raw': '原图', 'normal': '普通矫正', 'imu': 'IMU 动态矫正',
               'scan': '平面扫描', 'enhance': '自适应增强'}


def board_status(data):
    return (data or {}).get('latest_pose_status') or (data or {}).get('last_status') or {}


def output_token(data):
    shown = ((data or {}).get('scan') or {}).get('displayed') or {}
    return (shown.get('version'), shown.get('source_frame_id'), shown.get('output_first_us'))


def display_status(data, *, target=None, waiting=False):
    if not data or not (data.get('connected') or {}).get('fpga'):
        return '未连接 FPGA', '在“设备与参数”中连接设备。'
    board = board_status(data)
    if data.get('frozen'):
        return '已停止更新', data.get('error') or '请到设备与参数查看原因。'
    if data.get('geometry_recovery'):
        if data['geometry_recovery'].get('source') == 'visual':
            return ('扫描几何保护 · 等待新观测', '保留上一组有效映射；等待新的棋盘观测通过准入并确认输出')
        return ('越界保护 · 转回后自动恢复', '保留上一组有效矫正参数；等待新姿态通过准入并确认输出')
    visual = data.get('visual_scan') or {}
    if board.get('raw_active'):
        name = '原图'
    elif visual.get('enabled'):
        name = '平面扫描'
        if (visual.get('tracking') or {}).get('state') in ('lost', 'stale', 'acquiring', 'rejected'):
            name += ' · 跟踪丢失／等待重新识别'
    elif board.get('committed_modes', 0) & 2:
        name = '静态平面参数（高级设置）'
    elif data.get('continuous'):
        name = 'IMU 动态矫正'
    elif (data.get('request') or {}).get('request_kind') == 'imu_static_comparison' and (data.get('request') or {}).get('phase') == 'output_confirmed':
        name = '同倍率静态对照（非同帧）'
    elif target in ('normal', 'enhance'):
        name = MODE_LABELS[target]
    elif target == 'scan':
        name = '平面扫描 · 跟踪已停止，保留上一映射'
    elif target == 'imu':
        name = 'IMU 动态矫正 · 保留上一姿态'
    else:
        request = data.get('request') or {}
        if request.get('request_kind') == 'normal_correction' and request.get('phase') == 'output_confirmed':
            name = '普通矫正'
        elif request.get('phase') == 'output_confirmed' and request.get('pose_mode') in ('full', 'roll_only'):
            name = 'IMU 动态矫正 · 保留上一姿态'
        else:
            name = '矫正画面 · 模式待确认'
    enhanced = (data.get('enhancement') or {}).get('enhancement_mode') == 1
    if enhanced and name != MODE_LABELS['enhance']:
        name += '＋自适应'
    if (data.get('detail') or {}).get('active'):
        name += '＋细节增强'
    shown = (data.get('scan') or {}).get('displayed') or {}
    version = board.get('config_version')
    confirmed = shown.get('version') == version and bool(shown)
    detail = '对应版本输出已确认' if confirmed else '等待对应版本输出确认'
    if waiting:
        detail = '切换中；等待完整目标配置与对应输出'
    elif data.get('error') and not data.get('continuous'):
        detail = '更新暂停：' + str(data['error'])
    return name, f'板端版本 {version if version is not None else "—"} · {detail}'


def independent_status(data, video):
    """Keep video loss, serial loss, stale IMU and geometry holds distinct."""
    data, video = data or {}, video or {}
    connected = data.get('connected') or {}
    fpga = data.get('fpga') or {}
    imu = data.get('imu') or {}
    video_text = ('实时取帧' if video.get('live') and not video.get('stale') else
                  '取帧暂停／保留末帧' if video.get('has_frame') else '未打开')
    if video.get('source_kind') in ('image', 'video'):
        video_text = '本地文件（不参与实时扫描）'
    fpga_text = '串口未连接' if not connected.get('fpga') else (
        '状态过期' if fpga.get('status_age_s') is None or fpga.get('status_age_s', 0) > 2.5 else '串口已连接')
    imu_text = '未连接' if not connected.get('imu') else '数据过期' if imu.get('stale', True) else '数据新鲜'
    guard = '几何越界等待恢复' if data.get('geometry_recovery') else ('保护停止' if data.get('frozen') else '就绪')
    return f'视频：{video_text}   |   FPGA：{fpga_text}   |   IMU：{imu_text}   |   {guard}'


class DemoController:
    def __init__(self, session_provider, notify, *, profile_provider=lambda: None,
                 imu_settings=None, start_scan=None, stop_scan=None, clock=time.monotonic):
        self.session_provider, self.notify = session_provider, notify
        self.profile_provider = profile_provider
        self.imu_settings = imu_settings or self._default_imu_settings
        self.start_scan_callback = start_scan or (lambda session: None)
        self.stop_scan_callback = stop_scan or (lambda: None)
        self.clock = clock
        self.target = None
        self.confirmed_target = None
        self.imu_static_comparison = False
        self._pre_raw_target = None
        self._confirmed_pose_settings = None
        self.adaptive_preferences = dict(raw=False, normal=False, imu=False, scan=False, enhance=True)
        self.detail_preferences = dict.fromkeys(MODE_LABELS, False)
        self._awaiting = None
        self.sequence = ActionSequencer(session_provider, self._sequence_note, clock=clock, timeout=90.)

    @staticmethod
    def _default_imu_settings(session):
        return 'roll_only', checked_demo_crop(session.calibration, geometry_options=session.geometry_options)

    def _sequence_note(self, message):
        if '未完成' in message or '已取消' in message or '超时' in message:
            self._awaiting = None
        if '连续提交仍停止' in message:
            message = message.split('：', 1)[0] + '：配置请求已提交，等待对应输出。'
        self.notify(message)

    @property
    def pending(self):
        return self.sequence.pending is not None or self._awaiting is not None

    @property
    def adaptive_allowed(self):
        return self.target in MODE_LABELS and self.target != 'normal'

    @property
    def detail_allowed(self):
        session = self.session_provider()
        return bool(self.target in MODE_LABELS and self.target != 'raw' and session is not None
                    and (session.snapshot().get('detail') or {}).get('capable'))

    def session(self):
        session = self.session_provider()
        if session is None:
            raise RuntimeError('请先在“设备与参数”中连接 FPGA。')
        return session

    @staticmethod
    def _clear_view(session):
        board = board_status(session.snapshot())
        if board.get('committed_modes') or board.get('requested_modes'):
            session.request_view_mode(scan=False, mirror=False)

    @staticmethod
    def _ensure_owned_baseline(session):
        if not session.snapshot().get('static_configuration_owned'):
            session.request_normal()

    @staticmethod
    def _route_displayed(session, *, raw=False, observation=False):
        data = session.snapshot()
        board = board_status(data)
        if bool(board.get('raw_active')) != raw or bool(board.get('observation_active')) != observation:
            return False
        # Initial, already-corrected no-op route may precede our first ordinary
        # baseline. A changed route always requires actual matching output.
        if not data.get('static_configuration_owned') and not raw and not observation:
            return True
        shown = (data.get('scan') or {}).get('displayed') or {}
        return bool(shown and shown.get('version') == board.get('config_version')
                    and bool(shown.get('raw')) == raw and bool(shown.get('observation')) == observation)

    def _arm(self, session, *, adaptive_only=False, detail_only=False):
        data = session.snapshot()
        self._awaiting = {'session': session, 'started_at': self.clock(),
                          'deadline': self.clock() + 90., 'output_before': output_token(data),
                          'output_count_before': (data.get('version_evidence') or {}).get('output_confirmed', 0),
                          'adaptive_only': adaptive_only, 'detail_only': detail_only}

    def _begin(self, mode, steps):
        if self.pending:
            raise RuntimeError('正在等待当前模式的实际输出；请等待或停止当前请求。')
        self.session()
        self.stop_scan_callback()
        self.target = mode
        self.confirmed_target = None
        self.sequence.begin(MODE_LABELS[mode], steps)

    def select(self, mode):
        if mode not in MODE_LABELS:
            raise ValueError('Unknown demo mode')
        session = self.session()
        if self.pending:
            raise RuntimeError('正在等待当前模式的实际输出；请等待或停止当前请求。')
        settings = None
        if mode == 'imu':
            data = session.snapshot()
            if not data.get('mounting_confirmed'):
                raise RuntimeError('请导入 IMU 标定，并确认相机与 IMU 的安装未改变。')
            if (data.get('imu') or {}).get('stale', True):
                raise RuntimeError('尚未收到新鲜 IMU 数据，请检查连接。')
            settings = self.imu_settings(session)
        desired = 0 if mode == 'normal' else int(self.adaptive_preferences[mode])
        desired_detail = False if mode == 'raw' else self.detail_preferences[mode]
        self.imu_static_comparison = False
        # Every target resets its full route/geometry/enhancement context.
        # Baseline restoration removes an earlier crop, never edits calibration.
        steps = [self._ensure_owned_baseline,
                 lambda s: s.request_output_route(raw=False, observation=False),
                 WaitUntil(self._route_displayed), self._clear_view,
                 lambda s: s.request_enhancement(desired),
                 lambda s: s.request_detail_enhancement(desired_detail)]
        if mode in ('normal', 'enhance'):
            def normal(s):
                self._arm(s)
                s.request_normal()
            steps.append(normal)
        else:
            steps.append(lambda s: s.request_normal())
            if mode == 'raw':
                def raw(s):
                    self._arm(s)
                    s.request_output_route(raw=True, observation=False)
                steps.append(raw)
            elif mode == 'imu':
                steps.extend([lambda s: s.set_pose_mode(settings[0], crop_profile=settings[1]),
                              self._reference_if_needed, self._start_imu])
            else:
                steps.extend([lambda s: s.request_output_route(raw=False, observation=True),
                    WaitUntil(lambda s: self._route_displayed(s, observation=True)), self._start_scan])
        self._pre_raw_target = None
        self._begin(mode, steps)

    def raw(self): self.select('raw')
    def normal(self): self.select('normal')
    def imu(self): self.select('imu')
    def scan(self): self.select('scan')
    def enhance(self): self.select('enhance')

    @staticmethod
    def _reference_if_needed(session):
        if session.snapshot().get('reference') is None:
            session.capture_reference()

    def _start_imu(self, session):
        self.imu_static_comparison = False
        self._arm(session)
        session.start_realtime(auto_recover_geometry=True)

    def _start_scan(self, session):
        self._arm(session)
        session.start_visual_scan()
        self.start_scan_callback(session)

    def reference(self):
        if self.target != 'imu':
            raise RuntimeError('请先选择 IMU 动态矫正模式。')
        self._begin('imu', [lambda s: s.capture_reference(), self._start_imu])

    def adaptive(self, enabled):
        if self.pending:
            raise RuntimeError('正在等待当前模式的实际输出；请等待或停止当前请求。')
        if not self.adaptive_allowed:
            self.adaptive_preferences['normal'] = False
            raise RuntimeError('普通矫正始终关闭自适应。请使用自适应增强模式进行对比。')
        mode = self.target
        self.adaptive_preferences[mode] = bool(enabled)
        def apply(s):
            self._arm(s, adaptive_only=True)
            s.request_enhancement(int(bool(enabled)))
        # E2 changes only brightness. Keep H, FOV, crop and reference intact.
        steps = [apply]
        if mode == 'imu' and not self.imu_static_comparison:
            steps.append(lambda s: s.start_realtime(auto_recover_geometry=True))
        elif mode == 'scan':
            steps.append(lambda s: (s.start_visual_scan(), self.start_scan_callback(s)))
        self._begin(mode, steps)

    def compare_imu_static(self):
        if self.target != 'imu':
            raise RuntimeError('请先选择IMU模式，再建立相同倍率的静态对照。')
        def compare(session):
            self._arm(session)
            session.request_static_comparison()
            self.imu_static_comparison = True
        self._begin('imu', [compare])

    def detail(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('Detail enhancement must be a boolean')
        if self.pending:
            raise RuntimeError('正在等待当前模式的实际输出；请等待或停止当前请求。')
        if not self.detail_allowed:
            raise RuntimeError('细节增强需要 C129 固件；原图模式保持关闭。')
        mode = self.target
        session = self.session()
        if bool((session.snapshot().get('detail') or {}).get('active')) == enabled:
            self.detail_preferences[mode] = enabled
            return
        self.detail_preferences[mode] = enabled
        def apply(s):
            self._arm(s, detail_only=True)
            s.request_detail_enhancement(enabled)
        # Wait for this output-frame state before allowing continuous geometry
        # to publish another version. Only E4 changes; H/crop/gamma stay intact.
        steps = [apply, WaitUntil(lambda s: bool((s.snapshot().get('detail_control') or {}).get('output_confirmed')))]
        if mode == 'imu' and not self.imu_static_comparison:
            steps.append(lambda s: s.start_realtime(auto_recover_geometry=True))
        elif mode == 'scan':
            steps.append(lambda s: (s.start_visual_scan(), self.start_scan_callback(s)))
        self._begin(mode, steps)

    def observe(self):
        wait = self._awaiting
        if self.sequence.pending:
            return
        if wait is None:
            self.synchronize_actual_route()
            return
        session = self.session_provider()
        if session is not wait['session'] or session is None:
            self._awaiting = None
            self.notify('连接已改变，旧模式输出等待已取消。')
            return
        data = session.snapshot()
        if data.get('frozen') or self.clock() > wait['deadline']:
            self._awaiting = None
            self.notify('模式输出尚未确认：' + (data.get('error') or '等待超时；未标记为切换成功。'))
            return
        board = board_status(data)
        shown = (data.get('scan') or {}).get('displayed') or {}
        desired_adaptive = 0 if self.target == 'normal' else int(self.adaptive_preferences[self.target])
        desired_detail = False if self.target == 'raw' else self.detail_preferences[self.target]
        if ((data.get('enhancement') or {}).get('enhancement_mode') != desired_adaptive
                or bool((data.get('detail') or {}).get('active')) != desired_detail
                or bool(shown.get('detail_active')) != desired_detail
                or bool(board.get('raw_active')) != (self.target == 'raw')
                or bool(board.get('observation_active')) != (self.target == 'scan')
                or board.get('committed_modes', 0) != 0
                or not shown or shown.get('version') != board.get('config_version')
                or output_token(data) == wait['output_before']):
            return
        if wait['detail_only']:
            detail = data.get('detail_control') or {}
            if not detail.get('output_confirmed') or detail.get('enabled') != desired_detail:
                return
        elif wait['adaptive_only']:
            enhancement = data.get('enhancement_control') or {}
            if not enhancement.get('output_confirmed') or enhancement.get('mode') != desired_adaptive:
                return
        elif self.target == 'scan' and not (data.get('visual_scan') or {}).get('output_confirmed'):
            return
        elif (self.target in ('normal', 'enhance', 'imu', 'scan') and
              (data.get('version_evidence') or {}).get('output_confirmed', 0) <= wait['output_count_before']):
            return
        self.confirmed_target = self.target
        self._confirmed_pose_settings = (data.get('pose_mode'), deepcopy(data.get('crop_profile')))
        self._awaiting = None
        self.notify(MODE_LABELS[self.target] + '：目标配置与对应版本输出已确认。')

    def synchronize_actual_route(self):
        """Reflect external KEY1/KEY2 changes without submitting a command."""
        session = self.session_provider()
        if session is None or self.confirmed_target is None:
            return
        data = session.snapshot()
        board = board_status(data)
        shown = (data.get('scan') or {}).get('displayed') or {}
        if not shown or shown.get('version') != board.get('config_version'):
            return
        if (self._confirmed_pose_settings is not None and
                (data.get('pose_mode'), data.get('crop_profile')) != self._confirmed_pose_settings):
            self.target = self.confirmed_target = None
            self.notify('高级设置已改变几何上下文；请选择完整演示模式以重新建立目标配置。')
            return
        if board.get('committed_modes', 0):
            self.target = self.confirmed_target = None
            self.notify('板上按键已切换静态扫描／镜像；当前实际状态以板端为准。')
            return
        if board.get('raw_active') and self.target != 'raw':
            self._pre_raw_target = self.target
            self.target = self.confirmed_target = 'raw'
        elif not board.get('raw_active') and self.target == 'raw':
            self.target = self.confirmed_target = (self._pre_raw_target or
                ('imu' if data.get('pose_mode') in ('roll_only', 'full') else 'normal'))
            self._pre_raw_target = None
        adaptive = (data.get('enhancement') or {}).get('enhancement_mode')
        if self.target == 'normal' and adaptive == 1:
            self.target = self.confirmed_target = 'enhance'
        if self.target in self.adaptive_preferences and adaptive in (0, 1):
            self.adaptive_preferences[self.target] = bool(adaptive)

    def tick(self):
        self.sequence.tick()
        self.observe()

    def stop(self):
        self.stop_scan_callback()
        self._awaiting = None
        self.sequence.cancel()
        session = self.session_provider()
        if session is not None:
            session.stop()
