"""Sequence explicit UI commands through the existing C127 serial owner.

No new wire protocol or serial writer. Never reconnects or resumes streaming.
"""
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class WaitUntil:
    """A read-only evidence barrier; never retries a mutating command."""
    predicate: object


class ActionSequencer:
    def __init__(self, session_provider, notify=None, *, clock=time.monotonic, timeout=25.):
        self.session_provider = session_provider
        self.notify = notify or (lambda message: None)
        self.clock, self.timeout = clock, timeout
        self.pending = None

    @staticmethod
    def _epoch(session):
        return (getattr(session, '_epoch', None), getattr(session, '_view_epoch', None))

    def begin(self, label, steps):
        if self.pending:
            raise RuntimeError('上一项操作仍在等待，请先取消或等待完成。')
        session = self.session_provider()
        if session is None:
            raise RuntimeError('请先在“设备与 IMU”页面连接 FPGA。')
        with session._lock:
            snapshot = session.snapshot()
            if snapshot.get('frozen'):
                raise RuntimeError(snapshot.get('error') or '会话已停止，请先排查设备状态。')
            session.stop()
            self.pending = {'session': session, 'label': label, 'steps': list(steps),
                            'deadline': self.clock()+self.timeout, 'epoch': self._epoch(session)}
        self.notify(label+'：已停止连续提交，等待在途事务完成。')

    def cancel(self, reason='操作已取消；未自动恢复连续提交。'):
        self.pending = None
        self.notify(reason)

    def tick(self):
        task = self.pending
        if task is None:
            return
        session = self.session_provider()
        if session is not task['session'] or session is None:
            self.cancel('连接已改变，已取消待执行操作。')
            return
        if self.clock() >= task['deadline']:
            self.cancel('等待事务结束超时，未重复发送；请查看状态与日志。')
            return
        # C127 exposes no separate is_idle API. Read its existing ownership
        # ledger under its lock; never mutate that ledger from the GUI.
        with session._lock:
            if self._epoch(session) != task['epoch']:
                self.cancel('参考或视野上下文已改变，已取消旧操作。')
                return
            shutdown = getattr(session, '_shutdown', None)
            if (getattr(session, '_closed', False) or getattr(session, '_frozen', False)
                    or (shutdown is not None and shutdown.is_set())):
                self.cancel('会话关闭或保护停止，待执行操作已取消。')
                return
            if any((getattr(session, '_active', None), getattr(session, '_candidate', None),
                    getattr(session, '_pending_outputs', None),
                    getattr(session, '_view_command', None), getattr(session, '_view_dirty', False))):
                return
            if not task['steps']:
                self.pending = None
                self.notify(task['label']+'：请求已交给原控制链路；以“已显示”状态为准。')
                return
            step = task['steps'][0]
            try:
                if isinstance(step, WaitUntil):
                    if not step.predicate(session):
                        return
                else:
                    step(session)
                task['steps'].pop(0)
            except Exception as exc:
                self.cancel(task['label']+'未完成：'+str(exc))
                return
            task['epoch'] = self._epoch(session)
            if not task['steps']:
                self.pending = None
                self.notify(task['label']+'：请求已提交，等待板端确认；连续提交仍停止。')
