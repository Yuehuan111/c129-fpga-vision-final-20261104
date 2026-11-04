"""Explicit advanced Gamma control; FPGA executes all pixel operations.

Importing this module opens no device. Only send_enhancement/main transmit an
8-byte E2 command, after verifying the advanced firmware's CRC-valid V5 status.
"""
import argparse
import json
import math
from numbers import Integral
import time

from camera_config import packet
BUILD_ID = 0x0720A001  # Historical four-mode protocol; refuse C021.

MODE_NAMES = ('bypass', 'auto', 'night', 'highlight')
GAMMA_NAMES = ('neutral', 'night', 'highlight', 'reserved')


def mode_value(mode):
    if isinstance(mode, str) and mode in MODE_NAMES:
        return MODE_NAMES.index(mode)
    if isinstance(mode, bool) or not isinstance(mode, Integral) or not 0 <= mode <= 3:
        raise ValueError('Gamma mode must be bypass/auto/night/highlight or integer 0..3.')
    return int(mode)


def mode_packet(mode):
    return packet(0xe2, mode_value(mode))


def enhancement_status(status):
    """Decode the advanced-only V5 extension; old builds cannot verify E2."""
    if status.get('build_id') != BUILD_ID:
        raise ValueError('板端版本不匹配，需要高级版 0x0720A001；指令未确认。')
    if status.get('protocol_version') != 5:
        raise ValueError('需要高级版 V5 状态确认 Gamma，旧状态格式不适用。')
    bits = (status['flags'] >> 7) & 15
    mode, gamma = bits >> 2, bits & 3
    return dict(enhancement_mode=mode, enhancement_mode_name=MODE_NAMES[mode],
                gamma_id=gamma, gamma_name=GAMMA_NAMES[gamma])


def send_enhancement(port_name, mode, timeout=6.0):
    """Verify identity, send E2 once, then require matching V5 applied mode.

    The reported mode is sampled from the FPGA video domain after a frame
    boundary. UART write completion alone never establishes application.
    """
    mode = mode_value(mode)
    if not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive.')
    if not port_name:
        raise ValueError('请先选择板载 USB-UART 对应的 COM 口。')
    import serial
    from serial.tools import list_ports
    from read_board_status import extract_packets
    if not any(p.device == port_name and p.vid == 0x10c4 and p.pid == 0xea60
               for p in list_ports.comports()):
        raise ValueError('所选端口不是已确认的CP210x板载串口。')
    with serial.Serial(port_name, 115200, timeout=.2, write_timeout=2) as port:
        buffer = bytearray()

        def await_status(predicate):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                buffer.extend(port.read(256))
                for status in extract_packets(buffer):
                    decoded = enhancement_status(status)
                    observed = {**status, **decoded}
                    if predicate(observed):
                        return observed
            raise TimeoutError('未收到匹配的板端 Gamma 状态，不能认定模式已生效。')

        before = await_status(lambda decoded: True)
        if before['enhancement_mode'] == mode:
            return dict(requested_mode=mode, requested_mode_name=MODE_NAMES[mode],
                        confirmed=True, already_applied=True, transmitted=False,
                        before=before, after=before,
                        confirmation='CRC-valid advanced V5 mode already active; no E2 sent')
        payload = mode_packet(mode)
        if port.write(payload) != len(payload):
            raise IOError('串口未发送完整 Gamma 数据包，模式未确认。')
        port.flush()
        after = await_status(lambda observed: observed['enhancement_mode'] == mode and
            0 < ((observed['sample_ticks'] - before['sample_ticks']) & 0xffffffff) < 0x80000000)
    return dict(requested_mode=mode, requested_mode_name=MODE_NAMES[mode],
                confirmed=True, already_applied=False, transmitted=True,
                before=before, after=after,
                confirmation='CRC-valid advanced V5 applied-mode telemetry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--mode', required=True, choices=MODE_NAMES)
    args = parser.parse_args()
    print(json.dumps(send_enhancement(args.port, args.mode), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
