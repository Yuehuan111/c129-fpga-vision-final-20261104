"""C127 view controls over the single persistent FPGA serial owner.

Only 3x3 matrices/configurations cross the link. The board generates maps,
composes view factors, admits every tile and atomically publishes a map bank.
Connection is receive-only. Every mutation below is an explicit UI action.
"""
from copy import deepcopy
import hashlib
import math
import multiprocessing as mp
import queue
import numpy as np

from imu_session import HostSession, _forward, _jsonable
from imu_pose import mode_compensation_matrix, camera_relative_roll
from roll_crop import FixedRollCrop
from scan_plane import make_context, validate_profile, compose_words, validate_modes
from camera_config import packet
from complete_geometry import configuration, validate_words
from complete_control import enhancement_status
from pose_protocol import VIEW_BUILD_ID, VIEW_BUILD_IDS, SCAN_BUILD_IDS, DETAIL_BUILD_ID
from read_latency import LatencyAnalysis
from visual_scan_session import VisualScanSessionMixin

INVALIDATE_VIEW_PAYLOAD = packet(0xEA, 0x504C494E)


def profile_packets(profile, wire_context=None):
    profile = validate_profile(profile)
    context = profile['context_id'] if wire_context is None else wire_context
    if isinstance(context, bool) or not isinstance(context, int) or not 0 < context < 2**32:
        raise ValueError('Nonzero 32-bit context required')
    return (packet(0xE8, context) + packet(0xE7, int(profile['scan_valid'])) +
            b''.join(packet(0x40+i, word) for i, word in enumerate(profile['mode_words'])) +
            packet(0xE9, context))


def _c127_geometry_process(words, allow_black, output):
    """C127 policy: domain-invalid pixels are black, ROI/all-black reject."""
    try:
        from complete_geometry_reference import vector_fixed_map
        from tile_budget import tile_budget
        words = validate_words(list(words))
        entries, domain = vector_fixed_map(1280, 720, words[:18], words[18:])
        invalid = int(np.count_nonzero((entries >> 38) == 0))
        budget = tile_budget(entries, 1280, 720, 64, 16, 256, 64)
        if invalid == 1280*720:
            raise ValueError('All source samples are invalid')
        if budget['unsupported_tiles']:
            raise ValueError('FPGA source-cache envelope exceeded: '+str(budget['unsupported_tiles'][:3]))
        if invalid and not allow_black:
            raise ValueError(f'{invalid} black pixels remain')
        output.put((True, dict(fixed_point_checked=True, width=1280, height=720,
                              invalid_pixels=invalid, invalid_fraction=invalid/(1280*720),
                              domain_invalid_pixels=int(domain), arithmetic_error_pixels=0,
                              domain_policy='invalid samples become black', tile_budget=budget,
                              meaning='Offline integer geometry/ROI check; not board verification')))
    except Exception as exc:
        output.put((False, str(exc)))


class C127HostSession(VisualScanSessionMixin, HostSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pose_mode = 'roll_only'
        self._crop_profile = None
        self._view_epoch = 0
        self._scan_profile = None
        self._scan_wire_context = None
        self._view_upload_sequence = 0
        self._used_wire_contexts = set()
        self._view_command = None
        self._view_versions = {}
        self._view_displayed = None
        self._view_message = '尚未初始化扫描／镜像参数'
        self._view_dirty = False
        self._pending_normal = None
        self._enhancement_control = None
        self._detail_control = None

    def stop(self):
        with self._lock:
            self._pending_normal = None
            return super().stop()

    stop_continuous = stop

    def _fail(self, reason, *, state='frozen', invalidate=True):
        with self._lock:
            self._pending_normal = None
            return super()._fail(reason, state=state, invalidate=invalidate)

    def _gate(self, *, require_idle=True, require_pose=True, require_stream=True):
        if self._view_command is not None or self._view_dirty:
            return 'View context update pending; wait for board acknowledgement'
        if (self._status or {}).get('raw_active'):
            return 'KEY1 raw comparison is active; press KEY1 again to restore correction'
        status = self._pose_status or {}
        if status.get('committed_modes') and not self._owns_view(status):
            return 'Active view factors are not owned by this session; turn scan/mirror off first'
        normal = self._pose_mode == 'normal'
        return super()._gate(require_idle=require_idle, require_pose=require_pose and not normal,
                             require_stream=require_stream and not normal)

    def _identity_gate(self):
        # Unlike ordinary correction, the preserved identity transport
        # diagnostic records a fresh IMU observation and uses its age checks.
        reason = self._stream_gate(self._clock())
        return reason if reason else super()._identity_gate()

    def _owns_view(self, status):
        return bool(self._scan_profile is not None and self._scan_wire_context is not None and
                    status.get('view_context') == self._scan_wire_context and status.get('matrix_valid'))

    def _view_context_evidence(self):
        context = self.scan_context()
        active = self._active or {}
        words = active.get('words') if 'published_at' in active else self._last_words
        owned = bool(words and tuple(words[:18]) == tuple(context['words18']))
        return dict(context_hash=context['context_hash'] if owned else None, lens_configuration_owned=owned)

    def _configuration_options(self):
        options = dict(self.geometry_options)
        if self._crop_profile is not None:
            options['zoom'] = self._crop_profile['zoom']
        return options

    def scan_context(self):
        with self._lock:
            words = configuration(self.calibration, matrix=np.eye(3), legacy_angle=0,
                                  raw=False, **self._configuration_options())
            return make_context(self.calibration, words[:18], reference=None if self._pose_mode == 'normal' else self._reference,
                                mounting=self._mounting, convention=self._convention, epoch=self._view_epoch)

    def _invalidate_view(self):
        self._view_epoch += 1
        self._scan_profile = None
        self._scan_wire_context = None
        self._view_message = '参考／视野已改变，旧平面参数失效'
        status = self._pose_status or {}
        if status.get('build_id') in VIEW_BUILD_IDS and (status.get('matrix_valid') or status.get('view_context') or status.get('committed_modes') or status.get('requested_modes')):
            if self._view_command is not None:
                raise RuntimeError('Wait for the current view command before changing context')
            self._view_dirty = True
            self._view_command = dict(kind='invalidate', phase='queued', payload=packet(0xEA, 0x504C494E), queued_at=self._clock())
            self._wake.set()

    def _context_change_gate(self):
        if self._view_command is not None:
            raise RuntimeError('Wait for the pending scan/mirror command')
        if self._active or self._pending_outputs:
            self.stop()
            raise RuntimeError('已停止新提交；请等待在途配置和输出确认，再更改设置。')

    def set_pose_mode(self, mode, crop_profile=None):
        if mode not in ('normal', 'roll_only', 'full'):
            raise ValueError('Unknown pose mode')
        crop = None if crop_profile is None else FixedRollCrop(**crop_profile).to_dict()
        if crop is not None and mode != 'roll_only':
            raise ValueError('Fixed roll crop requires roll_only mode')
        with self._lock:
            self._context_change_gate()
            self.stop()
            if mode != self._pose_mode or crop != self._crop_profile:
                self._pose_mode, self._crop_profile = mode, crop
                self._invalidate_view()
            self._log('pose_mode_set', mode=mode, crop=crop,
                      policy='Fixed zoom reduces black borders; no arbitrary-angle zero-black guarantee')

    def capture_reference(self):
        with self._lock:
            self._context_change_gate()
            result = super().capture_reference()
            self._invalidate_view()
            return result

    def restore_reference(self, quaternion, provenance):
        with self._lock:
            self._context_change_gate()
            result = super().restore_reference(quaternion, provenance)
            self._invalidate_view()
            return result

    def set_mounting(self, matrix, *, convention='imu_to_world', confirmed=False):
        with self._lock:
            self._context_change_gate()
            result = super().set_mounting(matrix, convention=convention, confirmed=confirmed)
            self._invalidate_view()
            return result

    def _queue_request(self, **kwargs):
        if self._visual_enabled:
            # Offers are driven only by new source frames. The inherited
            # scheduler must never synthesize or repeat an IMU-like request.
            return None
        force_full = kwargs.pop('force_full_configuration', False)
        result = super()._queue_request(**kwargs)
        self._candidate.update(pose_mode=self._pose_mode, crop_profile=deepcopy(self._crop_profile))
        if force_full:
            self._candidate['force_full_configuration'] = True
        if self._pose_mode == 'normal' and not kwargs.get('identity_test'):
            self._candidate['request_kind'] = 'normal_correction'
        return result

    def request_normal(self):
        with self._lock:
            if self._pose_mode != 'normal':
                self.set_pose_mode('normal')
            if self._view_command:
                if self._view_command['kind'] != 'invalidate' or self._frozen:
                    raise RuntimeError('Wait for the pending view command before sending normal correction')
                # This one explicit user action authorizes exactly one full
                # ordinary configuration after the necessary invalidation ACK.
                self._pending_normal = dict(context_hash=self.scan_context()['context_hash'],
                                            requested_at=self._clock())
                self._log('normal_deferred_until_invalidation', pending=self._pending_normal)
                return '普通去畸变已排队；撤销旧参数获板端确认后自动提交一次。'
            reason = self._ordinary_full_gate()
            if reason:
                raise RuntimeError(reason)
            return self._queue_request(realtime=False, force_full_configuration=True)

    def _release_pending_normal(self):
        pending, self._pending_normal = self._pending_normal, None
        if pending is None:
            return
        if (self._pose_mode != 'normal' or self.scan_context()['context_hash'] != pending['context_hash']):
            self._log('deferred_normal_cancelled', reason='Context changed')
            return
        reason = self._ordinary_full_gate()
        if reason:
            self._view_message = '普通配置未发送：'+reason
            self._log('deferred_normal_cancelled', reason=reason)
            return
        self._queue_request(realtime=False, force_full_configuration=True)
        self._view_message = '旧参数已撤销；已按本次操作自动排队普通去畸变'

    def _ordinary_full_gate(self):
        # An explicit full ordinary configuration establishes known lens/H
        # ownership even when a previous session left the board in raw mode.
        # It does not trust or replay the unknown saved board calibration.
        if self._view_command is not None or self._view_dirty:
            return 'View context update pending; wait for board acknowledgement'
        status = self._pose_status or {}
        if status.get('raw_active') and status.get('build_id') not in SCAN_BUILD_IDS:
            return 'KEY1 raw comparison is active; press KEY1 again to restore correction'
        if status.get('committed_modes') or status.get('requested_modes'):
            return 'Disable static scan/mirror factors before establishing ordinary geometry'
        return HostSession._gate(self, require_pose=False, require_stream=False)

    def _request_gate(self, request):
        if request.get('request_kind') == 'imu_static_comparison':
            return self._gate(require_pose=False, require_stream=False)
        if request.get('request_kind') == 'normal_correction' and request.get('force_full_configuration'):
            return self._ordinary_full_gate()
        return super()._request_gate(request)

    def _scheduled_stream_gate(self, now):
        if any(request and request.get('request_kind') == 'imu_static_comparison'
               for request in (self._active, self._candidate)):
            return None
        return super()._scheduled_stream_gate(now)

    def request_static_comparison(self):
        """Identity H at current IMU lens/centre/crop, without an IMU pose.

        This asynchronous same-scale baseline does not replace ordinary U.
        """
        with self._lock:
            self.stop()
            if self._pose_mode not in ('roll_only', 'full'):
                raise RuntimeError('Select IMU mode before requesting its same-scale static comparison')
            if self._active or self._candidate or self._pending_outputs:
                raise RuntimeError('Wait for current configuration/output evidence before static comparison')
            reason = self._gate(require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            status = self._pose_status or {}
            if status.get('committed_modes') or status.get('requested_modes') or status.get('observation_active'):
                raise RuntimeError('Static IMU comparison requires scan, mirror and observation route off')
            context = self.scan_context()
            if self._last_words is None or tuple(self._last_words[:18]) != tuple(context['words18']):
                raise RuntimeError('Confirm the current IMU lens/crop configuration before comparing it')
            self._request_sequence += 1
            self._candidate = dict(request_id=self._request_sequence, request_kind='imu_static_comparison',
                request_source='static_reference', queued_at=self._clock(), epoch=self._epoch,
                phase='queued', realtime=False, force_full_configuration=True,
                pose_mode=self._pose_mode, crop_profile=deepcopy(self._crop_profile),
                comparison=dict(context_hash=context['context_hash'], lens_words18=list(context['words18']),
                    reference_id=(self._reference or {}).get('reference_id'),
                    meaning='Identity H, same lens/centre/fixed magnification; asynchronous, not a same-frame pair'))
            self._log('static_comparison_queued', request=self._candidate)
            self._wake.set()
            return self._candidate['request_id']

    def _candidate_age(self, request):
        if request.get('request_kind') in ('normal_correction', 'imu_static_comparison'):
            return 0.
        return super()._candidate_age(request)

    def _prepare_matrix_words(self, request):
        if request.get('request_source') == 'visual':
            return super()._prepare_matrix_words(request)
        mode = request.get('pose_mode', self._pose_mode)
        if mode == 'normal' or request.get('request_kind') in ('identity_transport_test', 'imu_static_comparison'):
            h = np.eye(3)
        else:
            args = (request['sample']['quaternion'], request['reference']['sample']['quaternion'], request['mounting'])
            h = mode_compensation_matrix(*args, mode=mode, convention=request['convention'])
            if request.get('crop_profile'):
                crop = FixedRollCrop(**request['crop_profile'])
                crop.check_angle(camera_relative_roll(*args, convention=request['convention']))
        options = dict(self.geometry_options)
        if request.get('crop_profile'):
            options['zoom'] = request['crop_profile']['zoom']
        words = validate_words(configuration(self.calibration, matrix=h, legacy_angle=0, raw=False, **options))
        return h, words

    def _check_request_geometry(self, words, request):
        with self._lock:
            profile = deepcopy(self._scan_profile)
            status = deepcopy(self._pose_status or {})
        modes = status.get('committed_modes', 0)
        if modes:
            if profile is None or not self._owns_view(status):
                raise ValueError('Current view factors are not owned by this session')
            if tuple(profile['context']['words18']) != tuple(words[:18]):
                raise ValueError('View factors do not match candidate optical settings')
            final = compose_words(profile, np.asarray(words[18:]).reshape(3, 3)/65536,
                                  scan=bool(modes & 2), mirror=bool(modes & 1))
            # This is an offline check only. The transmitted words remain H_base.
            return super()._check_request_geometry(final, request)
        return super()._check_request_geometry(words, request)

    def _hardware_admission_ready(self, words):
        with self._lock:
            status = self._pose_status or {}
            return bool(super()._hardware_admission_ready(words) and
                        (not status.get('committed_modes') or self._owns_view(status)))

    def _check_geometry_cancellable(self, words):
        if self._geometry_checker is not None:
            return self._geometry_checker(tuple(words))
        context = mp.get_context('spawn')
        output = context.Queue(maxsize=1)
        process = context.Process(target=_c127_geometry_process, args=(tuple(words), self.allow_black, output))
        process.start()
        try:
            while not self._shutdown.is_set():
                try:
                    ok, result = output.get(timeout=.05)
                    if not ok:
                        raise ValueError(result)
                    return result
                except queue.Empty:
                    if not process.is_alive():
                        raise RuntimeError('Geometry process exited without a result')
            raise InterruptedError('Session closing')
        finally:
            if process.is_alive():
                process.terminate()
            process.join(timeout=1.)
            if process.is_alive():
                process.kill()
                process.join(timeout=1.)
            output.close()
            output.join_thread()

    def _view_gate(self):
        if self._frozen:
            raise RuntimeError(self._error or 'Session frozen')
        if self._view_command is not None:
            raise RuntimeError('Wait for the previous view command')
        if self._active or self._candidate or self._pending_outputs:
            raise RuntimeError('Stop submissions and wait for configuration/output evidence before changing view')
        reason = super()._gate(require_pose=False, require_stream=False)
        if reason:
            raise RuntimeError(reason)
        status = self._pose_status or {}
        if status.get('build_id') not in VIEW_BUILD_IDS or not status.get('view_capable'):
            raise RuntimeError('C127 V8 FPGA capabilities are required')
        if status.get('raw_active'):
            raise RuntimeError('Leave KEY1 raw mode first')
        return status

    def set_scan_profile(self, profile):
        with self._lock:
            self.stop()
            status = self._view_gate()
            profile = validate_profile(profile, expected_context=self.scan_context())
            if status.get('requested_modes') or status.get('committed_modes'):
                raise RuntimeError('Turn scan and mirror off before replacing their parameters')
            if self._last_words is None or tuple(self._last_words[:18]) != tuple(profile['context']['words18']):
                raise RuntimeError('先发送并确认当前普通／姿态配置，再初始化镜像或扫描。')
            base_words = tuple(self._last_words)
            base = np.asarray(base_words[18:]).reshape(3, 3)/65536
        evidence = validate_modes(profile, base)
        required = ('00', '01', '10', '11') if profile['scan_valid'] else ('00', '01')
        if any(not evidence['modes'][m]['admitted'] for m in required):
            raise ValueError('平面参数超出整幅图像／缓存范围；请重新选点或增大目标布局。')
        with self._lock:
            status = self._view_gate()
            validate_profile(profile, expected_context=self.scan_context())
            if status.get('requested_modes') or status.get('committed_modes') or tuple(self._last_words or ()) != base_words:
                raise RuntimeError('Base/view changed during validation; retry with the current frame context')
            # The board echoes this upload nonce, not merely the mathematical
            # context hash. Old valid tables cannot acknowledge a replacement.
            while True:
                self._view_upload_sequence += 1
                token = int.from_bytes(hashlib.sha256(
                    f'{self.session_id}:{self._view_upload_sequence}:{profile["profile_hash"]}'.encode()).digest()[:4], 'big')
                if token and token != status.get('view_context') and token not in self._used_wire_contexts:
                    self._used_wire_contexts.add(token)
                    break
            self._view_command = dict(kind='profile', phase='queued', payload=profile_packets(profile, token),
                                      wire_context=token, profile=profile, evidence=evidence, queued_at=self._clock())
            self._wake.set()
            return '参数已排队；等待 FPGA 回显上下文后才能使用。'

    def request_view_mode(self, scan=False, mirror=False):
        if not isinstance(scan, bool) or not isinstance(mirror, bool):
            raise ValueError('View choices must be booleans')
        with self._lock:
            self.stop()
            status = self._view_gate()
            mode = int(scan)*2+int(mirror)
            profile = self._scan_profile
            if mode and not self._owns_view(status):
                raise RuntimeError('先明确初始化镜像／平面参数并等待板端确认。')
            if scan and (not profile['scan_valid'] or not status.get('plane_valid')):
                raise RuntimeError('没有已确认的平面参数，不能开启扫描。')
            self._view_command = dict(kind='mode', phase='queued', payload=packet(0xEB, mode), mode=mode, queued_at=self._clock())
            self._wake.set()
            return '切换已排队；提交和实际显示将分别确认。'

    def invalidate_scan_profile(self):
        with self._lock:
            self._context_change_gate()
            self.stop()
            self._invalidate_view()

    def request_output_route(self, *, raw=False, observation=False):
        """Explicit C128 software route, atomically published by the board."""
        if type(raw) is not bool or type(observation) is not bool or (raw and observation):
            raise ValueError('Raw/observation choices must be booleans and are mutually exclusive')
        with self._lock:
            self.stop()
            if self._view_command or self._active or self._candidate or self._pending_outputs:
                raise RuntimeError('Wait for current configuration/output evidence before changing the route')
            reason = super()._gate(require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            status = self._pose_status or {}
            if status.get('build_id') not in SCAN_BUILD_IDS or not status.get('software_route_capable'):
                raise RuntimeError('C128 software route capability is required')
            if observation and not status.get('observation_capable'):
                raise RuntimeError('C128 raw observation capability is required')
            if bool(status.get('raw_active')) == raw and bool(status.get('observation_active')) == observation:
                return '目标输出通路已生效；实际显示仍以同版本LAT为准。'
            if self._last_words is None:
                if not raw and not observation:
                    return self.request_normal()
                raise RuntimeError('Confirm an owned ordinary configuration before changing output route')
            self._view_command = dict(kind='route', phase='queued', raw=raw, observation=observation,
                payload=packet(0xEC, int(raw)|(int(observation)<<1)), queued_at=self._clock())
            self._wake.set()
            return '输出通路已排队；等待发布及同版本LAT。'

    def request_raw(self, enabled):
        return self.request_output_route(raw=enabled, observation=False)

    def request_observation(self, enabled):
        return self.request_output_route(raw=False, observation=enabled)

    def request_enhancement(self, mode):
        """Explicit existing E2 mode request through the same FPGA writer."""
        if isinstance(mode, bool) or not isinstance(mode, int) or not 0 <= mode <= 4:
            raise ValueError('Enhancement mode must be an integer in [0,4]')
        with self._lock:
            if self._view_command is not None:
                raise RuntimeError('Wait for the pending control command')
            reason = super()._gate(require_idle=False, require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            current = enhancement_status(self._health_status())['enhancement_mode']
            if current == mode:
                return '该增强模式已生效，无需重复发送。'
            self.stop()
            if self._active or self._pending_outputs:
                raise RuntimeError('Wait for current configuration/output evidence before changing enhancement')
            reason = super()._gate(require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            self._view_command = dict(kind='enhancement', phase='queued', payload=packet(0xE2, mode),
                                      mode=mode, previous_mode=current, queued_at=self._clock())
            self._wake.set()
            return '增强模式已排队；等待新状态回显。'

    def request_detail_enhancement(self, enabled):
        """C129 E4, through the existing single writer; default is bypass."""
        if type(enabled) is not bool:
            raise ValueError('Detail enhancement must be a boolean')
        with self._lock:
            if self._view_command is not None:
                raise RuntimeError('Wait for the pending control command')
            reason = super()._gate(require_idle=False, require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            status = self._pose_status or {}
            capable = status.get('build_id') == DETAIL_BUILD_ID and status.get('detail_capable')
            if not capable:
                if enabled or status.get('detail_active'):
                    raise RuntimeError('C129 detail enhancement capability is required')
                return '此固件没有细节增强；保持旁路。'
            if enabled and status.get('raw_active'):
                raise RuntimeError('Raw mode keeps detail enhancement bypassed')
            if bool(status.get('detail_active')) == enabled:
                return '目标细节增强状态已生效；没有重复发送。'
            self.stop()
            if self._active or self._candidate or self._pending_outputs:
                raise RuntimeError('Wait for current configuration/output evidence before changing detail')
            reason = super()._gate(require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            self._detail_control = None
            self._view_command = dict(kind='detail', phase='queued', payload=packet(0xE4, int(enabled)),
                                      enabled=enabled, queued_at=self._clock())
            self._wake.set()
            return '细节增强请求已排队；等待实际输出帧边界及同帧状态确认。'

    def _ack_detail(self, packet, now):
        command = self._view_command
        if (self._frozen or not command or command.get('kind') != 'detail'
                or not command.get('before') or packet.get('build_id') != DETAIL_BUILD_ID
                or packet.get('protocol_version') != 8 or not packet.get('detail_capable')):
            return False
        before = command['before']
        board_us = packet.get('board_time_us')
        if board_us is None or not _forward(board_us, before.get('board_time_us', 0)):
            return False
        if (packet.get('error_sticky') or packet.get('camera_error') or
                any(packet.get(key) != before.get(key) for key in ('config_version', 'commits', 'rejected', 'control_failures'))):
            self._fail('Board fault or changed geometry during detail acknowledgement')
            return False
        if bool(packet.get('detail_active')) != command['enabled']:
            return False
        if 'write_done_at' not in command:
            command['deferred_detail_ack'] = (deepcopy(packet), now)
            return False
        command['phase'] = 'acknowledged'
        self._detail_control = dict(enabled=command['enabled'], acknowledged_at=now,
            acknowledged_board_us=board_us, version=packet['config_version'], output_confirmed=False,
            raw=bool(packet.get('raw_active')), observation=bool(packet.get('observation_active')),
            output_pipeline_delay_pixclk=1656,
            meaning='POS1 actual output-frame state plus matching LAT detail tag; not optical quality verification')
        self._view_command = None
        self._view_message = '细节增强的输出帧边界状态已确认；等待对应 LAT 帧标志'
        self._log('detail_acknowledged', command={k:v for k,v in command.items() if k != 'payload'}, status=packet)
        return True

    def _observe_status(self, packet, now):
        super()._observe_status(packet, now)
        # E2 is an existing register. It may be reported only by periodic V5,
        # so do not require a new projective version or rebase the POS1 ledger.
        with self._lock:
            self._ack_enhancement(packet, now)

    def _ack_enhancement(self, packet, now):
        command = self._view_command
        if self._frozen or not command or command['kind'] != 'enhancement' or not command.get('before'):
            return False
        before = command['before']
        board_us = packet.get('board_time_us')
        if board_us is None and 'sample_ticks' in packet:
            board_us = (packet['sample_ticks']*1_000_000) & 0xffffffff
        if board_us is None or not _forward(board_us, before.get('board_time_us', 0)):
            return False
        if (packet.get('build_id') != before.get('build_id') or packet.get('error_sticky') or packet.get('camera_error') or
                any(packet[key] < before[key] for key in ('captured', 'processed', 'commits') if key in packet and key in before)):
            self._fail('Board fault/reset during enhancement acknowledgement')
            return False
        if packet.get('rejected') != before.get('rejected'):
            self._fail('FPGA rejection during enhancement request')
            return False
        if enhancement_status(packet)['enhancement_mode'] != command['mode']:
            return False
        command['phase'] = 'acknowledged'
        self._enhancement_control = dict(mode=command['mode'], requested_mode=command['mode'], acknowledged_at=now,
            acknowledged_board_us=board_us, version=packet.get('config_version'), output_confirmed=False,
            meaning='Register acknowledgement; matching later LAT is output timing evidence, not optical verification')
        self._view_command = None
        self._view_message = '增强模式已由新状态确认'
        self._log('enhancement_acknowledged', command={k:v for k,v in command.items() if k != 'payload'}, status=packet)
        return True

    def _service_control(self):
        with self._lock:
            command = self._view_command
            if command is None:
                return False
            if self._frozen:
                return True
            # An already written geometry transaction must finish first.
            if self._active or self._pending_outputs:
                return False
            if command['phase'] == 'queued':
                reason = super()._gate(require_pose=False, require_stream=False)
                if reason:
                    if self._clock()-command['queued_at'] > self.publish_timeout_s:
                        self._fail('View command could not start: '+reason)
                    return True
                command.update(phase='writing', before=deepcopy(self._pose_status), write_started_at=self._clock())
                payload = command['payload']
            else:
                if self._clock()-command['write_started_at'] > self.publish_timeout_s:
                    self._fail('View acknowledgement timed out; outcome unknown; no retransmission')
                return True
        port = self._ports['fpga']
        if port.write(payload) != len(payload):
            raise IOError('Incomplete view transaction; outcome unknown')
        self._drain(port)
        with self._lock:
            command['write_done_at'] = self._clock()
            if command['phase'] == 'writing':
                command['phase'] = 'await_ack'
            self._log('view_write_complete', command={k:v for k,v in command.items() if k != 'payload'})
            deferred = command.pop('deferred_bootstrap_publication', None)
            if deferred is not None and not self._frozen:
                self._observe_control_status(*deferred)
            detail_ack = command.pop('deferred_detail_ack', None)
            if detail_ack is not None and not self._frozen:
                self._ack_detail(*detail_ack)
        return True

    def _unowned_bootstrap_invalidation(self, packet, previous, command, *, require_write_done=True):
        """Attribute only our explicit EA before a queued full baseline.

        The previous saved lens remains unowned. A completed EA replay is
        merely a prerequisite for FE; no arbitrary external writer is trusted.
        """
        before = (command or {}).get('before') or {}
        pending = self._pending_normal
        if (self._frozen or self._last_words is not None or self._active or self._pending_outputs
                or not self._view_dirty or self._pose_mode != 'normal' or not pending
                or not command or command.get('kind') != 'invalidate'
                or command.get('payload') != INVALIDATE_VIEW_PAYLOAD
                or command.get('phase') not in ('writing', 'await_ack')
                or 'write_started_at' not in command
                or (require_write_done and 'write_done_at' not in command)
                or packet.get('build_id') not in SCAN_BUILD_IDS
                or before.get('build_id') != packet.get('build_id') or previous.get('build_id') != packet.get('build_id')
                or pending['context_hash'] != self.scan_context()['context_hash']
                or before.get('raw_active') or not any(before.get(key) for key in
                    ('committed_modes', 'requested_modes', 'matrix_valid', 'plane_valid', 'view_context'))):
            return False
        if (packet.get('config_version') != ((before['config_version']+1) & 65535)
                or previous.get('config_version') != before['config_version']
                or not packet.get('last_publish_from_key')
                or not _forward(packet.get('board_time_us', 0), before.get('board_time_us', 0))
                or not _forward(packet.get('board_time_us', 0), previous.get('board_time_us', 0))):
            return False
        if any(packet.get(key) != before.get(key) or previous.get(key) != before.get(key)
               for key in ('commits', 'rejected', 'control_failures')):
            return False
        if (packet.get('view_context') != 0 or packet.get('requested_modes') != 0 or packet.get('committed_modes') != 0
                or any(packet.get(key) for key in ('matrix_valid', 'plane_valid', 'raw_active', 'key_busy', 'map_building',
                                                  'mode_reject', 'reject_flags', 'error_sticky', 'camera_error'))
                or packet.get('observation_active') != before.get('observation_active')
                or not all(packet.get(key) for key in ('saved_valid', 'view_capable', 'ddr_initialized',
                                                      'camera_config_done', 'hdmi_config_done'))):
            return False
        return all(packet[key] >= previous[key] and packet[key] >= before[key]
                   for key in ('captured', 'processed', 'underflow', 'overflow', 'dropped')
                   if key in packet and key in previous and key in before)

    def _observe_control_status(self, packet, now):
        with self._lock:
            previous = self._status or {}
            previous_at = self._status_at
            command = self._view_command
            if (command and 'write_done_at' not in command and
                    self._unowned_bootstrap_invalidation(packet, previous, command, require_write_done=False)):
                command['deferred_bootstrap_publication'] = (deepcopy(packet), now)
                self._log('bootstrap_invalidation_output_waits_for_write', status=packet)
                return
            if packet.get('build_id') in VIEW_BUILD_IDS and previous.get('build_id') in VIEW_BUILD_IDS:
                steps = (packet['config_version']-previous['config_version']) & 65535
                active = self._active
                written = bool(active and 'before' in active and 'published_at' not in active)
                unchanged_view = all(packet.get(field) == previous.get(field) for field in (
                    'config_version', 'commits', 'rejected', 'view_context', 'matrix_valid',
                    'plane_valid', 'requested_modes', 'committed_modes', 'raw_active', 'observation_active'))
                uninitialized_key_rejection = (
                    not self._frozen and command is None and not self._view_dirty and
                    (active is None or 'before' not in active) and unchanged_view and
                    packet.get('mode_reject') and not packet.get('plane_valid') and packet.get('saved_valid') and
                    packet.get('control_failures', -1) == previous.get('control_failures', -2)+1 and
                    not packet.get('key_busy') and not packet.get('map_building') and not packet.get('reject_flags') and
                    not packet.get('error_sticky') and not packet.get('camera_error') and
                    all(packet.get(field) for field in ('ddr_initialized', 'camera_config_done', 'hdmi_config_done')) and
                    all(packet[field] >= previous[field] for field in ('captured', 'processed') if field in packet and field in previous) and
                    _forward(packet.get('board_time_us', 0), previous.get('board_time_us', 0)))
                if uninitialized_key_rejection:
                    # A rejected KEY2/KEY3 with no usable plane/table changes
                    # neither configuration nor ownership. Consume only this
                    # one proven failure count; the base ledger still checks
                    # every other counter/fault. UART/unknown writes never
                    # enter this path, and no success evidence is produced.
                    self.stop()
                    self._status = dict(previous, control_failures=packet['control_failures'])
                    self._view_message = '按键请求被拒绝；请先初始化平面／镜像参数，当前画面已保留。'
                    self._log('view_rejected', reason='missing_plane_or_matrix', status=packet)
                key_step = bool(steps and packet.get('last_publish_from_key'))
                # A pose acceptance and a key publication may coexist in one
                # status snapshot. Never label that key's version a pose result.
                if key_step and written:
                    self._fail('Key/view publication collided with an active geometry transaction; outcome is not attributed to the pose')
                owned_base = bool(self._last_words is not None or (active and 'published_at' in active))
                bootstrap_invalidation = self._unowned_bootstrap_invalidation(packet, previous, command)
                pending_profile_owned = bool(command and command.get('kind') == 'profile' and
                    packet.get('view_context') == command.get('wire_context'))
                view_owned = bool(not packet.get('committed_modes') or self._owns_view(packet) or pending_profile_owned)
                # KEY1 is also valid before C tables are initialized, provided
                # this session owns the saved base calibration/H.
                key_publication = (steps == 1 and key_step and (owned_base or bootstrap_invalidation) and view_owned and not written and
                    packet['commits'] == previous['commits'] and packet['rejected'] == previous['rejected'] and
                    _forward(packet.get('board_time_us', 0), previous.get('board_time_us', 0)) and
                    not packet.get('error_sticky') and not packet.get('camera_error'))
                if key_publication:
                    self._status = dict(previous, config_version=packet['config_version'])
                    self._view_versions[packet['config_version']] = dict(build_id=packet.get('build_id'), modes=packet.get('committed_modes', 0),
                        raw=packet.get('raw_active', False), observation=packet.get('observation_active', False), lower_us=previous.get('board_time_us', 0),
                        observed_at=now, not_before_received_at=previous_at if previous_at is not None else now,
                        **self._view_context_evidence())
                    self._log('view_published', status=packet)
                    if bootstrap_invalidation:
                        self._log('bootstrap_invalidation_published', status=packet,
                                  meaning='Explicit EA prerequisite acknowledged; old saved lens is still unowned; full ordinary FE must follow')
                    # Retire pending old output evidence without calling it success.
                    superseded = list(self._pending_outputs)
                    if active and 'published_at' in active:
                        superseded.append(active)
                        self._last_words = tuple(active['words'])
                        self._active = None
                    for older in superseded:
                        older.update(phase='superseded_or_output_unobserved', reason='View replay superseded or output unobserved')
                        self._output_unobserved_count += 1
                        self._last_request = deepcopy(older)
                        self._log('superseded_or_output_unobserved', request=older)
                    self._pending_outputs.clear()
                    if packet.get('raw_active'):
                        self.stop()
            # The base ledger checks hardware faults, counter resets and
            # external writers before an auxiliary acknowledgement is trusted.
            super()._observe_control_status(packet, now)
            if packet.get('build_id') in VIEW_BUILD_IDS and not self._frozen:
                self._ack_enhancement(packet, now)
                self._ack_detail(packet, now)
                if (command is self._view_command and command and command.get('before') and
                        _forward(packet.get('board_time_us', 0), command['before'].get('board_time_us', 0))):
                    before = command['before']
                    fault = (packet['rejected'] != before['rejected'] or
                             packet.get('control_failures', 0) != before.get('control_failures', 0) or
                             (packet.get('mode_reject') and not before.get('mode_reject')))
                    if fault:
                        self._view_message = '切换被拒绝，保留上一幅有效输出；请检查参数'
                        self._fail(self._view_message)
                    else:
                        idle = not packet.get('key_busy') and not packet.get('map_building')
                        kind = command['kind']
                        replayed = (packet['config_version'] == ((before['config_version']+1) & 65535) and
                                    packet.get('last_publish_from_key') and packet['commits'] == before['commits'])
                        invalidated = (packet.get('view_context') == 0 and not packet.get('matrix_valid') and
                                       not packet.get('plane_valid') and packet.get('requested_modes') == 0)
                        acknowledged = ((kind == 'profile' and packet.get('view_context') == command['wire_context'] and
                                         packet.get('matrix_valid') and bool(packet.get('plane_valid')) == command['profile']['scan_valid']) or
                            (kind == 'mode' and replayed and packet.get('requested_modes') == command['mode'] and
                             packet.get('committed_modes') == command['mode']) or
                            (kind == 'route' and replayed and bool(packet.get('raw_active')) == command['raw'] and
                             bool(packet.get('observation_active')) == command['observation']) or
                            (kind == 'invalidate' and invalidated and
                             ((replayed and packet.get('committed_modes') == 0) or packet.get('raw_active'))))
                        if idle and acknowledged:
                            if kind == 'profile':
                                self._scan_profile = command['profile']
                                self._scan_wire_context = command['wire_context']
                            self._view_dirty = False
                            command['phase'] = 'acknowledged'
                            self._view_command = None
                            self._view_message = '板端已确认；实际显示仍以同版本 LAT1 为准'
                            self._log('view_acknowledged', command={k:v for k,v in command.items() if k != 'payload'}, status=packet)
                            if kind == 'invalidate':
                                self._release_pending_normal()
            if (packet.get('build_id') in VIEW_BUILD_IDS and not self._frozen and not packet.get('map_building')):
                self._view_versions.setdefault(packet['config_version'], dict(build_id=packet.get('build_id'), modes=packet.get('committed_modes', 0),
                    raw=packet.get('raw_active', False), observation=packet.get('observation_active', False), lower_us=previous.get('board_time_us', packet.get('board_time_us', 0)),
                    observed_at=now, not_before_received_at=previous_at if previous_at is not None else now,
                    **self._view_context_evidence()))
                while len(self._view_versions) > 128:
                    self._view_versions.pop(next(iter(self._view_versions)))
                self._confirm_view_output()

    def _invalidate_published_evidence(self, reason):
        super()._invalidate_published_evidence(reason)
        self._view_versions.clear()
        self._view_displayed = None
        self._scan_profile = None
        self._scan_wire_context = None

    def _confirm_output(self):
        super()._confirm_output()
        self._confirm_view_output()

    def _accept_output_for_request(self, request, item):
        if request.get('published_status', {}).get('build_id') == DETAIL_BUILD_ID:
            words = request.get('words') or ()
            if len(words) != 27 or bool(item['flags'] & 64) != (words[16] == 0):
                return False
        return super()._accept_output_for_request(request, item)

    def _confirm_view_output(self):
        if not hasattr(self, '_view_versions'):
            return
        for received, item in reversed(self._latencies):
            detail = self._detail_control
            if (detail and not detail['output_confirmed'] and received >= detail['acknowledged_at']
                    and item['config_version'] == detail['version']
                    and bool(item['flags'] & 32) == detail['enabled']
                    and bool(item['flags'] & 64) == detail['raw']
                    and bool(item['flags'] & 16) == detail['observation']
                    and _forward(item['warp_start_us'], detail['acknowledged_board_us'])
                    and LatencyAnalysis().accept(item)['accepted']):
                detail.update(output_confirmed=True, source_frame_id=item['source_frame_id'],
                    output_first_us=item['output_first_us'], confirmed_at=self._clock())
                self._log('detail_output_confirmed', evidence=detail)
            enhancement = self._enhancement_control
            if (enhancement and not enhancement['output_confirmed'] and received >= enhancement['acknowledged_at']
                    and item['config_version'] == enhancement['version']
                    and _forward(item['warp_start_us'], enhancement['acknowledged_board_us'])
                    and LatencyAnalysis().accept(item)['accepted']):
                enhancement.update(output_confirmed=True, source_frame_id=item['source_frame_id'],
                    output_first_us=item['output_first_us'], confirmed_at=self._clock())
            evidence = self._view_versions.get(item['config_version'])
            if (evidence and received >= evidence.get('not_before_received_at', evidence['observed_at']) and
                    (evidence.get('build_id') not in SCAN_BUILD_IDS or bool(item['flags'] & 16) == bool(evidence.get('observation'))) and
                    (evidence.get('build_id') != DETAIL_BUILD_ID or bool(item['flags'] & 64) == bool(evidence.get('raw'))) and
                    _forward(item['warp_start_us'], evidence['lower_us'])):
                if LatencyAnalysis().accept(item)['accepted']:
                    prior = self._view_displayed
                    # Host arrival order can differ from board output order.
                    if prior is None or _forward(item['output_first_us'], prior['output_first_us']):
                        self._view_displayed = dict(evidence, version=item['config_version'], source_frame_id=item['source_frame_id'],
                                                    output_first_us=item['output_first_us'],
                                                    detail_active=bool(item['flags'] & 32) if evidence.get('build_id') == DETAIL_BUILD_ID else False)

    def snapshot(self):
        with self._lock:
            result = super().snapshot()
            status = self._pose_status or {}
            health = self._health_status()
            result['enhancement'] = enhancement_status(health) if health else None
            result['enhancement_control'] = deepcopy(self._enhancement_control)
            result['detail'] = dict(capable=status.get('build_id') == DETAIL_BUILD_ID and status.get('detail_capable', False),
                                    active=bool(status.get('detail_active')),
                                    output_pipeline_delay_pixclk=1656 if status.get('build_id') == DETAIL_BUILD_ID else 0)
            result['detail_control'] = deepcopy(self._detail_control)
            result.update(pose_mode=self._pose_mode, crop_profile=deepcopy(self._crop_profile), scan=dict(
                requested_modes=status.get('requested_modes'), committed_modes=status.get('committed_modes'),
                displayed=deepcopy(self._view_displayed), matrix_valid=status.get('matrix_valid', False),
                plane_valid=status.get('plane_valid', False), context=status.get('view_context'),
                owned=self._owns_view(status), profile_context_hash=(self._scan_profile or {}).get('context_hash'),
                normal_pending=self._pending_normal is not None,
                pending=self._view_command['kind'] if self._view_command else None, message=self._view_message))
            return result
