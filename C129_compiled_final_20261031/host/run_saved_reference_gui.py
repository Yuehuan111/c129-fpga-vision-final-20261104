"""Open the existing controls with a validated, saved software reference.

The operator's previous physical direction checks authorize the temporary
mounting for this session only. Saved capture and mounting files are unchanged.
No compensation is submitted unless --start-continuous is explicitly supplied
or the operator subsequently uses the normal GUI controls.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from imu_app import ControlPanel
from run_pose_test import load_pose_inputs


class SavedReferencePanel(ControlPanel):
    def __init__(self, window, inputs, *, imu_port, fpga_port,
                 start_continuous=False, interval_s=2., realtime=False):
        self._inputs = inputs
        self._startup_pending = False
        self._startup_session = None
        self._auto_start = start_continuous
        self._interval_s = interval_s
        self._start_realtime = realtime
        self._reported_error = None
        super().__init__(window)
        self.imu_port.set(imu_port)
        self.fpga_port.set(fpga_port)
        self.interval.set(str(interval_s))
        self.mounting = inputs["mounting"]
        label = inputs["reference_provenance"]["label"]
        self.mount_note.set(f"{Path(inputs['mounting_path']).name} | "
                            f"{self.mounting['convention']} | 待恢复保存参考 {label}")
        try:
            # connect() applies an unconfirmed matrix without sending anything.
            self.connect()
            self._startup_session = self.session
            self._startup_deadline = time.monotonic() + 8.
            self._startup_pending = True
            self.footer.set(f"等待新鲜 IMU 和健康 FPGA，随后恢复保存参考 {label}；最多 8 秒。")
            self.window.after(0, self._prepare_saved_reference)
        except Exception as error:
            self._startup_failed(error)

    def _startup_failed(self, error):
        self._startup_pending = False
        if self.session is not None:
            self.session.stop()
        self.footer.set(f"启动未完成，连续补偿已停止：{type(error).__name__}: {error}。"
                        "不会自动重试；设备保留上次已生效的配置。")

    def _prepare_saved_reference(self):
        if not self._startup_pending:
            return
        if self.closing or self.session is not self._startup_session:
            self._startup_pending = False
            return
        try:
            data = self.session.snapshot()
            if data["frozen"]:
                raise RuntimeError(data["error"] or "会话已冻结")
            if time.monotonic() >= self._startup_deadline:
                raise TimeoutError(data["identity_test_blocked_reason"] or "设备就绪等待超过 8 秒")
            # This gate checks live streams and healthy, idle FPGA status without
            # requiring a reference or consuming the identity-test allowance.
            if not data["can_send_identity_test"]:
                self.window.after(50, self._prepare_saved_reference)
                return
            # Consume the startup action before changing session state. Every
            # subsequent GUI action is manual; failures never re-arm startup.
            self._startup_pending = False
            self.session.set_mounting(self.mounting["R_IC"],
                convention=self.mounting["convention"], confirmed=True)
            self.session.restore_reference(self._inputs["reference_quaternion"],
                                           self._inputs["reference_provenance"])
            self.mount_confirmed.set(True)
            label = self._inputs["reference_provenance"]["label"]
            self.mount_note.set(f"{Path(self._inputs['mounting_path']).name} | "
                f"{self.mounting['convention']} | 已按既有方向核对记录确认；保存参考 {label}")
            self.session._log("saved_reference_gui_prepared", inputs=self._inputs,
                confirmation="Previously reviewed physical direction checks; session only",
                automatic_continuous_requested=self._auto_start,
                source_file_confirmed=False)
            if self._auto_start:
                if self._start_realtime:
                    self.session.start_realtime()
                else:
                    self.session.start_continuous(interval_s=self._interval_s)
            mode = (('C125 连续补偿已开启，目标 30 Hz（实际频率待测）' if self._start_realtime else
                     f'连续补偿已开启，间隔 {self._interval_s:g} 秒') if self._auto_start else
                    '连续补偿关闭，可手动开始')
            self.footer.set(f"已恢复保存参考 {label}；{mode}。日志：{self.last_log}")
        except Exception as error:
            self._startup_failed(error)

    def act(self, function):
        if self._startup_pending:
            self._startup_pending = False
            self.footer.set("手动操作已取消自动启动；请使用界面继续。")
        super().act(function)

    def disconnect(self):
        self._startup_pending = False
        super().disconnect()

    def update(self):
        if self.closing:
            return
        super().update()
        if self.session is None:
            self.summary.set("连续补偿：关闭 | 未连接")
            return
        data = self.session.snapshot()
        mode = "开启" if data["continuous"] else "关闭"
        if self._startup_pending:
            mode += "（等待恢复参考）"
        reference = data.get("reference") or {}
        source = reference.get("provenance", {}).get("label")
        reference_text = f"保存参考 {source}" if source else ("当前采样" if reference else "未记录")
        self.summary.set(f"连续补偿：{mode} | 参考：{reference_text}\n{self.summary.get()}")
        error = data.get("error")
        if error and not data["continuous"] and error != self._reported_error:
            self._reported_error = error
            self.footer.set(f"连续补偿已停止：{error}。不会自动重试；设备保留上次已生效的配置。")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mounting", required=True, type=Path)
    parser.add_argument("--reference-endpoint", required=True, type=Path)
    parser.add_argument("--imu-port", required=True)
    parser.add_argument("--fpga-port", required=True)
    parser.add_argument("--start-continuous", action="store_true")
    parser.add_argument('--realtime', action='store_true', help='With --start-continuous, enable C125 guarded latest-pose stream, target 30 Hz')
    parser.add_argument("--interval", type=float, default=2.)
    args = parser.parse_args(argv)
    if args.realtime and not args.start_continuous:
        parser.error('--realtime requires explicit --start-continuous')
    if not math.isfinite(args.interval) or args.interval < .5:
        parser.error("--interval 必须为至少 0.5 秒的有限数值")
    if args.imu_port == args.fpga_port:
        parser.error("IMU 与 FPGA 必须使用不同端口")
    try:
        inputs = load_pose_inputs(args.mounting, args.reference_endpoint,
                                  confirm_temporary_mount=True)
        if inputs["reference_provenance"]["port"] != args.imu_port:
            raise ValueError("Selected IMU port differs from the saved installation capture")
    except Exception as error:
        parser.error(f"保存输入验证失败：{type(error).__name__}: {error}")
    import tkinter as tk
    window = tk.Tk()
    SavedReferencePanel(window, inputs, imu_port=args.imu_port, fpga_port=args.fpga_port,
                        start_continuous=args.start_continuous, interval_s=args.interval,
                        realtime=args.realtime)
    window.mainloop()
    return 0


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    raise SystemExit(main())
