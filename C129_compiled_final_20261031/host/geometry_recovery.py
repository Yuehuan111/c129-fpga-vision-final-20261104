"""Opt-in recovery from an acknowledged geometric rejection, under session lock.

Only fresh poses are probed, through the existing serial owner and FPGA guard.
Transport ambiguity and hardware faults never grant permission to recover.
"""
from copy import deepcopy
import math


class GeometryRecoveryMixin:
    _recovery_interval_s = .5
    _recovery_motion_deg = .1

    def _init_geometry_recovery(self):
        self._auto_recover_geometry = False
        self._geometry_recovery = None

    def _cancel_geometry_recovery(self, reason):
        recovery = self._geometry_recovery
        self._geometry_recovery = None
        self._auto_recover_geometry = False
        if recovery is not None:
            self._log('geometry_recovery_cancelled', reason=reason, recovery=recovery)

    def _replace_expired_unsent_pose(self, request, phase, reason):
        """Drop an old CPU candidate, never resend it or loosen the 100 ms gate."""
        if not (phase == 'candidate_discarded' and reason in (
                'Candidate pose expired during geometry validation',
                'Candidate pose expired before write')
                and self._auto_recover_geometry and self._continuous and self._realtime
                and not self._frozen and self._active is request
                and request.get('phase') in ('geometry', 'ready')
                and 'write_started_at' not in request
                and request.get('realtime') and request.get('admission') == 'fpga_full_map_guard'
                and request.get('epoch') == self._epoch
                and request.get('reference_id') == (self._reference or {}).get('reference_id')
                and not self._gate(require_idle=False)
                and self._hardware_admission_ready(request['words'])):
            return False
        sample = self._history.latest()
        if (sample is None or sample.id == request['sample']['id']
                or self._clock()-sample.received_at > self.realtime_pose_age_s):
            return False
        recovery = self._geometry_recovery
        if recovery is not None:
            if (not request.get('geometry_recovery_probe')
                    or recovery.get('probe_request_id') != request['request_id']):
                return False
            recovery.update(phase='waiting_pose', next_probe_at=self._clock()+self._recovery_interval_s)
            recovery.pop('probe_request_id', None)
        self._candidate = None
        self._next_due = self._clock()
        self._log('expired_pose_replaced', request_id=request['request_id'],
                  fresh_sample_id=sample.id, reason=reason,
                  policy='Old candidate discarded without a write; next request uses a fresh pose')
        self._wake.set()
        return True

    def _hold_geometry_rejection(self, active, status, now):
        # Called only after the ordinary ledger proves the rejection and checks
        # all counters. No new bootstrap, one-shot, stale context or scan replay.
        if not (self._auto_recover_geometry and self._continuous and self._realtime
                and not self._frozen and active.get('realtime')
                and active.get('admission') == 'fpga_full_map_guard'
                and active.get('epoch') == self._epoch and self._last_words
                and tuple(active['words'][:18]) == self._last_words[:18]
                and all(self._connected.values()) and not self._shutdown.is_set()
                and not self._gate(require_idle=False)
                and not status.get('raw_active')
                and not status.get('committed_modes') and not status.get('requested_modes')):
            return False
        previous = self._geometry_recovery or {}
        self._candidate = None
        self._geometry_recovery = dict(
            phase='waiting_pose', rejected_request_id=active['request_id'],
            rejected_at=now, next_probe_at=now+self._recovery_interval_s,
            rejected_quaternion=list(active['sample']['quaternion']),
            epoch=self._epoch, view_epoch=getattr(self, '_view_epoch', None),
            reference_id=active.get('reference_id'),
            rejection_count=previous.get('rejection_count', 0)+1,
            retained_version=status['config_version'],
            board_context={key: status.get(key) for key in (
                'build_id', 'config_version', 'commits', 'rejected', 'control_failures',
                'view_context', 'matrix_valid', 'plane_valid', 'committed_modes', 'requested_modes', 'raw_active')})
        active['automatic_geometry_hold'] = True
        self._state = 'geometry_hold'
        self._error = None
        self._log('geometry_recovery_waiting', request=active, recovery=self._geometry_recovery)
        self._wake.set()
        return True

    def _service_geometry_recovery(self, now):
        recovery = self._geometry_recovery
        if recovery is None:
            return False
        if not self._continuous or self._frozen:
            self._cancel_geometry_recovery('Continuous intent ended')
            return True
        if (recovery['epoch'] != self._epoch
                or recovery['view_epoch'] != getattr(self, '_view_epoch', None)
                or recovery['reference_id'] != (self._reference or {}).get('reference_id')):
            self.stop()
            return True
        active = self._active
        if active and active.get('phase') == 'board_rejected' and active.get('automatic_geometry_hold'):
            # A status can beat completion of the host-side serial drain.
            # Do not release this owner until the full write also completed.
            if 'write_done_at' not in active:
                return True
            active['rejection_closed_at'] = now
            self._last_request = deepcopy(active)
            self._active = None
            self._log('geometry_rejection_retired', request=active)
        if recovery['phase'] != 'waiting_pose':
            if not (self._active or self._candidate or self._pending_outputs):
                self.stop()
                self._error = 'Recovery probe ended without matching output evidence'
            return True
        if self._active or self._candidate or self._pending_outputs:
            return True
        reason = self._gate()
        if reason:
            if (self._status or {}).get('key_busy') or (self._status or {}).get('map_building'):
                return True
            self.stop()
            self._error = reason
            return True
        if any((self._status or {}).get(key) != value for key, value in recovery['board_context'].items()):
            self._fail('Configuration changed while waiting for geometry recovery')
            return True
        if now < recovery['next_probe_at']:
            return True
        sample = self._history.latest()
        if now-sample.received_at > self.realtime_pose_age_s:
            return True
        a, b = recovery['rejected_quaternion'], sample.quaternion
        dot = min(1., abs(sum(x*y for x, y in zip(a, b))))
        angle = math.degrees(2.*math.acos(dot))
        if angle < self._recovery_motion_deg:
            return True
        request_id = self._queue_request(realtime=True)
        self._candidate['geometry_recovery_probe'] = True
        recovery.update(phase='probing', probe_request_id=request_id)
        self._state = 'geometry_probe'
        self._log('geometry_recovery_probe', request_id=request_id, pose_change_deg=angle)
        return True

    def _complete_geometry_recovery(self, request, evidence):
        recovery = self._geometry_recovery
        if (recovery is None or not request.get('geometry_recovery_probe')
                or recovery.get('probe_request_id') != request['request_id']
                or not self._continuous or self._frozen or request['epoch'] != self._epoch):
            return
        self._geometry_recovery = None
        self._error = None
        self._next_due = self._clock()
        self._log('geometry_recovery_resumed', request_id=request['request_id'], evidence=evidence,
                  retained_reference_id=request.get('reference_id'))
        self._wake.set()
