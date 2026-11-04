"""C128 visual matrix source for the existing single serial writer.

Only a small H and source provenance enter the shared geometry/version/LAT
ledger. Visual observations never acquire quaternion or IMU sample fields.
"""
from collections import deque
from copy import deepcopy
import math

import numpy as np

from complete_geometry import configuration, validate_words
from pose_protocol import SCAN_BUILD_IDS, visual_packets


def _forward32(new, old):
    return 0 < ((int(new)-int(old)) & 0xffffffff) < 0x80000000


class VisualScanSessionMixin:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._visual_enabled = False
        self._visual_age_s = .35
        self._visual_source_age_s = 1.5
        self._visual_latest = None
        self._visual_tracking = dict(state='stopped', reason=None)
        self._visual_last_output = None
        self._visual_accepted_observations = 0
        self._visual_offer_times = deque(maxlen=120)

    def start_visual_scan(self, *, max_observation_age_s=.35, max_source_age_s=1.5):
        """Start demand-driven visual offers after an owned ordinary baseline.

        The raw inset must already be published and observed. No static scan
        table is uploaded, and no periodically repeated matrix is queued.
        """
        for value in (max_observation_age_s, max_source_age_s):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Visual age limits must be finite and positive')
        with self._lock:
            if getattr(self, '_pose_mode', None) != 'normal':
                raise RuntimeError('Visual scan requires the ordinary lens context, without IMU crop')
            reason = self._gate(require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            if self._active or self._candidate or self._pending_outputs or self._continuous:
                raise RuntimeError('Finish previous requests before starting visual scan')
            status = self._pose_status or {}
            if (status.get('build_id') not in SCAN_BUILD_IDS or not status.get('visual_matrix_capable')
                    or not status.get('observation_capable') or not status.get('observation_active')
                    or status.get('raw_active') or status.get('committed_modes')
                    or status.get('requested_modes')):
                raise RuntimeError('C128 raw observation route and visual matrix capabilities are required; static scan/mirror must be off')
            displayed = getattr(self, '_view_displayed', None) or {}
            if not displayed.get('observation') or displayed.get('raw'):
                raise RuntimeError('Wait for matching LAT output from the raw observation route')
            self._realtime = True
            if not self._hardware_admission_ready(self._last_words or []):
                raise RuntimeError('Confirm an owned ordinary configuration with full-map FPGA admission first')
            self._visual_enabled = self._continuous = True
            self._auto_recover_geometry = True
            self._geometry_recovery = None
            self._visual_age_s = float(max_observation_age_s)
            self._visual_source_age_s = float(max_source_age_s)
            # Preserve the last source token across an enhancement pause so
            # a repeated capture-card buffer cannot restart submissions.
            self._visual_tracking = dict(state='acquiring', reason='Waiting for a fresh source inset')
            self._interval_s = .05
            self._next_due = self._clock()
            self._log('visual_scan_started', context_hash=self.scan_context()['context_hash'],
                      observation_age_limit_s=self._visual_age_s, source_age_limit_s=self._visual_source_age_s,
                      transport='C128 FC/VI general H; full-map FPGA admission; no static scan toggles')
            self._wake.set()
            return '视觉扫描已启动，等待可追溯的新源帧和50角点。'

    def stop_visual_scan(self):
        return self.stop()

    def stop(self):
        self._visual_enabled = False
        if hasattr(self, '_visual_tracking'):
            self._visual_tracking.update(state='stopped')
        return super().stop()

    def _fail(self, reason, **kwargs):
        self._visual_enabled = False
        if hasattr(self, '_visual_tracking'):
            self._visual_tracking.update(state='fault', reason=str(reason))
        return super()._fail(reason, **kwargs)

    def report_visual_tracking(self, state):
        with self._lock:
            if not isinstance(state, dict):
                raise ValueError('Tracking state must be a dictionary')
            if self._visual_enabled:
                self._visual_tracking = deepcopy(state)
                self._visual_tracking['reported_at'] = self._clock()
                if state.get('state') in ('lost', 'stale', 'acquiring'):
                    # A no-longer-observed candidate must not leak out later.
                    self._candidate = None
                    if self._active and self._active.get('request_source') == 'visual' and 'write_started_at' not in self._active:
                        self._active['observation_lost'] = True

    def offer_visual_scan(self, candidate):
        """Replace one unsent candidate; accept only a new, source-bound frame."""
        with self._lock:
            if not self._visual_enabled:
                raise RuntimeError('Visual scan is not running')
            reason = self._gate(require_idle=False, require_pose=False, require_stream=False)
            if reason:
                raise RuntimeError(reason)
            if not isinstance(candidate, dict) or candidate.get('schema') != 'c128_visual_candidate_v1':
                raise ValueError('An attributed visual candidate is required')
            candidate = deepcopy(candidate)
            context = self.scan_context()
            if candidate.get('context_hash') != context['context_hash']:
                raise ValueError('Visual candidate belongs to an obsolete lens/output context')
            observation = candidate.get('observation', {})
            if (observation.get('domain') != 'raw' or observation.get('source') != 'c128_raw_inset'
                    or observation.get('live') is not True or observation.get('metadata_crc_valid') is not True
                    or observation.get('source_size') != [1280, 720]
                    or observation.get('observation_size') != [320, 180]
                    or observation.get('source_transform') != [4, 0, 1.5, 0, 4, 1.5]):
                raise ValueError('Only the CRC-bound, full-FOV C128 raw inset is a live visual source')
            for key, limit in (('source_frame_id', 2**32), ('source_capture_us', 2**32), ('output_version', 2**16)):
                value = observation.get(key)
                if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < limit:
                    raise ValueError('Invalid observation '+key)
            output_route = getattr(self, '_view_versions', {}).get(observation['output_version'], {})
            if (not output_route.get('observation') or output_route.get('raw')
                    or not output_route.get('lens_configuration_owned')
                    or output_route.get('context_hash') != context['context_hash']):
                raise ValueError('Source footer version is not a known observation-route publication')
            received = observation.get('received_at')
            if not isinstance(received, (float, int)) or not math.isfinite(received):
                raise ValueError('Observation receipt time is required')
            age = self._clock()-received
            if not 0 <= age <= self._visual_age_s:
                raise ValueError('Visual observation is stale or future-dated')
            status = self._status or {}
            if status.get('board_time_us') is None or self._status_at is None:
                raise ValueError('Source time requires the current board clock')
            estimated_now = (status['board_time_us'] + round((self._clock()-self._status_at)*1e6)) & 0xffffffff
            board_age = ((estimated_now-observation['source_capture_us']) & 0xffffffff)/1e6
            # Capture arrival and a status packet can race by a few ms. A
            # genuinely future frame is still rejected; no host clock epoch
            # is compared directly to the board clock.
            if board_age > self._visual_source_age_s:
                raise ValueError('Source capture timestamp is stale or not bound to the current board clock')
            if self._visual_latest is not None and (
                    not _forward32(observation['source_frame_id'], self._visual_latest['source_frame_id'])
                    or not _forward32(observation['source_capture_us'], self._visual_latest['source_capture_us'])):
                raise ValueError('Duplicate, reordered or stale source frame')
            quality = candidate.get('quality', {})
            if (quality.get('corners') != 50 or quality.get('accepted') is not True
                    or not math.isfinite(quality.get('rms_target_pixels', math.inf))
                    or quality['rms_target_pixels'] > 3.):
                raise ValueError('All 50 qualified corners and a bounded target residual are required')
            h = np.asarray(candidate.get('H'), float)
            if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-10:
                raise ValueError('Visual H must be a finite nonsingular 3x3 matrix')
            words = validate_words(configuration(self.calibration, matrix=h, legacy_angle=0, raw=False,
                                                  **self._configuration_options()))
            if not self._hardware_admission_ready(words):
                raise RuntimeError('Visual matrix requires unchanged owned lens words and FPGA full-map admission')
            self._request_sequence += 1
            request = dict(request_id=self._request_sequence, request_kind='visual_scan', request_source='visual',
                           observation=observation, H=h.tolist(), visual_quality=quality,
                           target_grid=deepcopy(candidate.get('target_grid')), context_hash=context['context_hash'],
                           queued_at=self._clock(), epoch=self._epoch, phase='queued', realtime=True)
            recovery = self._geometry_recovery
            replacing_probe = bool(recovery and self._candidate and recovery.get('probe_request_id') == self._candidate['request_id'])
            if recovery and recovery.get('source') == 'visual' and (recovery.get('phase') == 'waiting_observation' or replacing_probe):
                request['geometry_recovery_probe'] = True
                recovery.update(phase='probing', probe_request_id=request['request_id'])
            if self._candidate:
                self._log('candidate_replaced', request_id=self._candidate['request_id'], by=request['request_id'])
            self._candidate = request
            self._visual_latest = observation
            self._visual_accepted_observations += 1
            self._visual_offer_times.append(self._clock())
            self._log('visual_candidate_queued', request=request)
            self._wake.set()
            return request['request_id']

    def _candidate_age(self, request):
        if request.get('request_source') == 'visual':
            if request.get('observation_lost'):
                return math.inf
            status = self._status or {}
            if status.get('board_time_us') is None or self._status_at is None:
                return math.inf
            estimated_now = (status['board_time_us']+round((self._clock()-self._status_at)*1e6)) & 0xffffffff
            if ((estimated_now-request['observation']['source_capture_us']) & 0xffffffff)/1e6 > self._visual_source_age_s:
                return math.inf
            return self._clock()-request['observation']['received_at']
        return super()._candidate_age(request)

    def _pose_age_limit(self, request):
        if request.get('request_source') == 'visual':
            return self._visual_age_s
        return super()._pose_age_limit(request)

    def _prepare_matrix_words(self, request):
        if request.get('request_source') != 'visual':
            return super()._prepare_matrix_words(request)
        if request['context_hash'] != self.scan_context()['context_hash']:
            raise ValueError('Visual geometry context changed')
        h = np.asarray(request['H'], float)
        words = validate_words(configuration(self.calibration, matrix=h, legacy_angle=0, raw=False,
                                              **self._configuration_options()))
        if not self._hardware_admission_ready(words):
            raise ValueError('Visual full-map admission unavailable')
        return h, words

    def _request_packets(self, request, compact):
        if request.get('request_source') == 'visual':
            status = self._pose_status or {}
            if (not compact or not self._visual_enabled or status.get('build_id') not in SCAN_BUILD_IDS
                    or not status.get('visual_matrix_capable') or not status.get('observation_active')
                    or status.get('committed_modes') or status.get('requested_modes')):
                raise RuntimeError('Visual route/capability changed; no fallback IMU or full-config write')
            request['transaction_source'] = 'visual_FC_VI'
            request['observation_age_at_write_s'] = self._candidate_age(request)
            return visual_packets(request['words'], self._status['config_version'])
        return super()._request_packets(request, compact)

    def _finish_unsent(self, request, phase, reason=None):
        if request.get('request_source') != 'visual' or not self._visual_enabled or self._frozen:
            return super()._finish_unsent(request, phase, reason)
        if request['epoch'] != self._epoch:
            return super()._finish_unsent(request, phase, reason)
        # Discard only this old candidate. New frames may reacquire or pass the
        # unchanged guard; no rejected matrix is blindly retransmitted.
        request.update(phase=phase, reason=reason, finished_at=self._clock())
        self._last_request = deepcopy(request)
        if self._active is request:
            self._active = None
        self._state = phase
        self._error = None
        self._log(phase, request=request)
        if phase == 'quantized_unchanged':
            self._complete_geometry_recovery(request, 'Fresh observation returns to the retained mapping; no retransmission')
        elif self._geometry_recovery and self._geometry_recovery.get('source') == 'visual' and self._geometry_recovery.get('probe_request_id') == request['request_id']:
            self._geometry_recovery.update(phase='waiting_observation')
            self._geometry_recovery.pop('probe_request_id', None)

    def _hold_geometry_rejection(self, active, status, now):
        if active.get('request_source') != 'visual':
            return super()._hold_geometry_rejection(active, status, now)
        if (not self._visual_enabled or not self._auto_recover_geometry or not self._continuous
                or self._frozen or active.get('admission') != 'fpga_full_map_guard'
                or active.get('epoch') != self._epoch or not self._last_words
                or tuple(active['words'][:18]) != self._last_words[:18]
                or not self._connected['fpga'] or not status.get('observation_active')
                or status.get('raw_active') or status.get('committed_modes') or status.get('requested_modes')):
            return False
        active['automatic_geometry_hold'] = True
        self._geometry_recovery = dict(source='visual', phase='waiting_observation', epoch=self._epoch,
            rejected_request_id=active['request_id'], rejected_at=now,
            rejected_source_frame_id=active['observation']['source_frame_id'], retained_version=status['config_version'],
            board_context={key: status.get(key) for key in ('build_id', 'config_version', 'commits', 'rejected',
                'control_failures', 'view_context', 'committed_modes', 'requested_modes', 'raw_active', 'observation_active')})
        self._visual_tracking.update(state='geometry_hold', reason='FPGA rejected this geometry; waiting for a newer observation')
        self._state = 'geometry_hold'
        self._error = None
        self._log('visual_geometry_recovery_waiting', request=active, recovery=self._geometry_recovery)
        return True

    def _service_geometry_recovery(self, now):
        recovery = self._geometry_recovery
        if not recovery or recovery.get('source') != 'visual':
            return super()._service_geometry_recovery(now)
        if not self._visual_enabled or recovery['epoch'] != self._epoch or self._frozen:
            self._cancel_geometry_recovery('Visual source stopped or context changed')
            return True
        if any((self._status or {}).get(key) != value for key, value in recovery['board_context'].items()):
            if recovery['phase'] == 'waiting_observation':
                self._fail('Configuration changed while waiting for a new visual observation')
                return True
        active = self._active
        if active and active.get('phase') == 'board_rejected' and active.get('automatic_geometry_hold'):
            if 'write_done_at' not in active:
                return True
            self._last_request = deepcopy(active)
            self._active = None
            self._log('visual_geometry_rejection_retired', request=active)
        if recovery['phase'] == 'waiting_observation' and self._candidate:
            candidate = self._candidate
            if (candidate.get('request_source') != 'visual' or not _forward32(
                    candidate['observation']['source_frame_id'], recovery['rejected_source_frame_id'])):
                self._candidate = None
            else:
                candidate['geometry_recovery_probe'] = True
                recovery.update(phase='probing', probe_request_id=candidate['request_id'])
        return True

    def _complete_geometry_recovery(self, request, evidence):
        recovery = self._geometry_recovery
        if request.get('request_source') != 'visual':
            return super()._complete_geometry_recovery(request, evidence)
        if recovery and recovery.get('probe_request_id') == request['request_id']:
            self._geometry_recovery = None
            self._error = None
            self._log('visual_geometry_recovery_resumed', request_id=request['request_id'], evidence=evidence)

    def _confirm_one_output(self, active):
        before = active.get('phase')
        super()._confirm_one_output(active)
        if (active.get('request_source') == 'visual' and active.get('phase') == 'output_confirmed'
                and before != 'output_confirmed'):
            self._visual_last_output = dict(request_id=active['request_id'],
                version=active.get('published_status', {}).get('config_version'),
                observed_source_frame_id=active['observation']['source_frame_id'],
                rendered_source_frame_id=active.get('source_frame_id'),
                confirmed_at=active['output_confirmed_at'],
                meaning='LAT confirms output of this matrix version; it is not exposure-synchronized to the observation')

    def _accept_output_for_request(self, request, item):
        if request.get('request_source') == 'visual':
            return bool(item['flags'] & 16)
        return super()._accept_output_for_request(request, item)

    def snapshot(self):
        result = super().snapshot()
        now = self._clock()
        recent = [t for t in self._visual_offer_times if now-t <= 5.]
        rate = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else None
        result['visual_scan'] = deepcopy(dict(enabled=self._visual_enabled,
            tracking=self._visual_tracking, latest_observation=self._visual_latest,
            accepted_observations=self._visual_accepted_observations, offer_observed_hz=rate,
            output_confirmed=self._visual_last_output, observation_age_limit_s=self._visual_age_s,
            source_age_limit_s=self._visual_source_age_s,
            output_note='Observation, matrix publication, LAT output and HDMI refresh are separate rates'))
        if self._visual_enabled:
            result['target_pose_hz'] = None
            result['continuous_source'] = 'visual'
        return result
