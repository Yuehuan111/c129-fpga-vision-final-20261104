"""Complete C125 parameter transport. Importing this file opens no device.

Only explicit --port / GUI Send actions transmit. Geometry admission requires
an offline exact-integer cache check. Success requires CRC-valid C125 V5
telemetry with a newer publication version and map_building=false, followed by
a valid CRC LAT1 sample carrying that version at the FPGA video output.
"""
import argparse
from collections import deque
import json
import math
from numbers import Integral
from pathlib import Path
import time

from camera_config import packet
from complete_geometry import (ADDRESSES, BUILD_ID, DEFAULT_CALIBRATION, PRESETS,
    PRODUCTION_BUDGET, ROOT, check_geometry, configuration, four_point_matrix,
    load_calibration, preset, validate_words)
from read_board_status import extract_packets
from project_profile import JOINT_MODEL, TANGENTIAL_Q16

MODE_NAMES = ('bypass', 'auto', 'night', 'highlight', 'histogram')
GAMMA_NAMES = ('neutral', 'night', 'highlight', 'reserved')
GAMMA_VALUES = (0.65, 0.75, 0.85, 1.0, 1.15, 1.30, 1.45)


def mode_value(mode):
    if isinstance(mode, str) and mode in MODE_NAMES:
        return MODE_NAMES.index(mode)
    if isinstance(mode, bool) or not isinstance(mode, Integral) or not 0 <= mode <= 4:
        raise ValueError('Enhancement mode must be bypass/auto/night/highlight/histogram or integer 0..4.')
    return int(mode)


def mode_packet(mode):
    return packet(0xe2, mode_value(mode))


def verify_firmware(status):
    if status.get('build_id') != BUILD_ID:
        raise ValueError('板端版本不匹配：必须为完整版 0x0720C125，参数未确认。')
    if status.get('protocol_version') != 5:
        raise ValueError('完整版控制要求 CRC 有效的 V5 状态。')
    return status


def enhancement_status(status):
    verify_firmware(status)
    mode, gamma, tone = (status['flags'] >> 12) & 7, (status['flags'] >> 10) & 3, (status['flags'] >> 7) & 7
    if mode > 4:
        raise ValueError('Board reported an undefined complete enhancement mode.')
    if tone >= len(GAMMA_VALUES):
        raise ValueError('Board reported an undefined C125 Gamma index.')
    return dict(enhancement_mode=mode, enhancement_mode_name=MODE_NAMES[mode],
                gamma_id=gamma, gamma_name=GAMMA_NAMES[gamma],
                gamma_index=tone, gamma_value=GAMMA_VALUES[tone],
                # Compatibility field: C125's low three bits identify Gamma,
                # not the previous C121 histogram-curve numbering.
                hist_curve=tone)


def configuration_packets(words):
    words = validate_words(words)
    return (packet(0xfe, 0x434f4d50) +
            b''.join(packet(address, word) for address, word in zip(ADDRESSES, words)) +
            packet(0xff, 0x434f4d54))


def export_configuration(words, directory, *, profile=PRODUCTION_BUDGET, allow_black=True, extra=None):
    words = validate_words(words)
    checked = check_geometry(tuple(words), profile, allow_black)
    metadata = dict(build_id=f'0x{BUILD_ID:08X}', protocol_version=5, register_count=27,
                    addresses=list(ADDRESSES), words=words, geometry=checked,
                    transaction='BEGIN FE/434F4D50, all 27 words, COMMIT FF/434F4D54; 232 UART bytes',
                    projection_order='legacy cos/sin, row-major inverse H, X/Z Y/Z, C97 fisheye k1..k4, compiled tangential p1/p2, source K',
                    model=JOINT_MODEL, compiled_tangential_q16=list(TANGENTIAL_Q16),
                    hardware_verified=False, calibration_adaptation_measured=False)
    if extra:
        metadata['description'] = extra
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'parameters.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    (directory / 'parameters.hex').write_text(''.join(f'{w & 0xffffffff:08x}\n' for w in words), encoding='ascii')
    (directory / 'uart_packets.bin').write_bytes(configuration_packets(words))
    return metadata


def _timeout(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('Timeout must be finite and positive.')
    return float(value)


def _open_serial(port_name):
    if not port_name:
        raise ValueError('请先选择板载 CP210x USB-UART 对应的 COM 口。')
    import serial
    from serial.tools import list_ports
    if not any(p.device == port_name and p.vid == 0x10c4 and p.pid == 0xea60 for p in list_ports.comports()):
        raise ValueError('所选端口不是已确认的 CP210x 板载串口。')
    port = serial.Serial(port_name, 115200, timeout=.2, write_timeout=2)
    # Discard queued bytes from an earlier connection; the next complete V5
    # frame establishes this connection's baseline before any command is sent.
    if hasattr(port, 'reset_input_buffer'):
        try:
            port.reset_input_buffer()
        except Exception:
            port.close()
            raise
    return port


def _forward(new, old, bits=32):
    return 0 < ((new - old) & ((1 << bits) - 1)) < (1 << (bits - 1))


class StatusReader:
    def __init__(self, port):
        # Deferred import avoids the read_latency -> enhancement_status import
        # cycle while retaining its single CRC-checked V5/V6 framing contract.
        from read_latency import MixedFramer
        self.port, self.framer = port, MixedFramer()
        self.latencies = deque(maxlen=128)
        self.last_status = None

    def read_packets(self):
        statuses = []
        for item in self.framer.feed(self.port.read(256)):
            if item['packet_type'] == 'status':
                verify_firmware(item)
                observed = {**item, **enhancement_status(item)}
                self.last_status = observed
                statuses.append(observed)
            elif self.last_status is not None:
                self.latencies.append(item)
        return statuses

    def await_status(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for observed in self.read_packets():
                if predicate(observed):
                    return observed
        raise TimeoutError('未收到满足条件的 CRC 有效 C125 V5 状态，不能认定指令已生效。')

    def await_display(self, version, before, timeout):
        from read_latency import LatencyAnalysis
        analysis = LatencyAnalysis()
        analysis.accept({**before, 'packet_type': 'status'})
        deadline = time.monotonic() + timeout
        before_us = (before['sample_ticks'] * 1_000_000) & 0xffffffff
        while time.monotonic() < deadline:
            while self.latencies:
                item = self.latencies.popleft()
                if item['config_version'] != version or not _forward(item['warp_start_us'], before_us):
                    continue
                checked = analysis.accept(item)
                if checked['accepted']:
                    return checked
            self.read_packets()
        raise TimeoutError('没有收到同一新配置版本的有效 CRC LAT1 输出样本。')


class PublishedButNotDisplayed(TimeoutError):
    def __init__(self, port_name, publication):
        self.port_name = port_name
        self.publication = publication
        super().__init__('映射已发布，但未确认该版本完整输出；不要重复发送，请仅重新读取输出确认。')


def confirm_display(port_name, publication, timeout=12.):
    """Retry output confirmation only; never retransmit a configuration."""
    timeout = _timeout(timeout)
    if not publication.get('published') or not publication.get('after'):
        raise ValueError('需要已有的映射发布记录才能重查输出。')
    with _open_serial(port_name) as port:
        reader = StatusReader(port)
        current = reader.await_status(lambda s: True, timeout)
        expected = publication['after']
        if current['config_version'] != expected['config_version'] or current['commits'] != expected['commits']:
            raise ValueError('板端配置已变化，不能把其他版本输出算作本次确认。')
        try:
            sample = reader.await_display(expected['config_version'], publication['before'], timeout)
        except TimeoutError as error:
            raise PublishedButNotDisplayed(port_name, publication) from error
    return {**publication, 'confirmed': True, 'display_confirmed': True,
            'output_sample': sample, 'read_only_retry': True, 'retransmitted': False,
            'confirmation': 'CRC-valid C125 V5 publication and matching valid LAT1 FPGA output'}


def read_status(port_name, timeout=6.):
    timeout = _timeout(timeout)
    with _open_serial(port_name) as port:
        return StatusReader(port).await_status(lambda s: True, timeout)


def send_configuration(port_name, words, *, timeout=45., profile=PRODUCTION_BUDGET, allow_black=True):
    """Send BEGIN + 27 words + FF, confirm publication and matching FPGA output.

    LAT1 establishes a complete FPGA output frame, excluding the panel/USB path.
    Actual new-corrected FPS comes from board processed counters, never USB FPS.
    """
    timeout = _timeout(timeout)
    words = validate_words(words)
    checked = check_geometry(tuple(words), profile, allow_black)
    if profile != PRODUCTION_BUDGET:
        raise ValueError('Transmission must use the production tile64x16 / source-cache256x64 profile.')
    payload = configuration_packets(words)
    with _open_serial(port_name) as port:
        reader = StatusReader(port)
        before = reader.await_status(lambda s: not s['map_building'], timeout)
        # UART 8N1 byte spacing gives the receiver ample CRC/handshake cycles.
        # Flush the entire BEGIN/27-word/COMMIT transaction before telemetry
        # confirmation; host sleeps do not protect atomicity.
        if port.write(payload) != len(payload):
            raise IOError('串口未发送完整参数事务；不能认定参数生效。')
        port.flush()
        def published(status):
            if not _forward(status['sample_ticks'], before['sample_ticks']):
                return False
            if status['rejected'] != before['rejected']:
                raise RuntimeError('板端拒绝计数增加；本次参数未确认。')
            return (status['commits'] == ((before['commits'] + 1) & 0xffffffff) and
                    status['config_version'] == ((before['config_version'] + 1) & 0xffff) and
                    not status['map_building'])
        after = reader.await_status(published, timeout)
        result = dict(confirmed=False, published=True, transmitted=True,
                      confirmation='new CRC-valid C125 V5 map publication; output not yet confirmed',
                      display_confirmed=False, before=before, after=after, geometry=checked,
                      warning='LAT1 confirms FPGA video output; not HDMI panel response or measured 30 new frames/s.')
        try:
            sample = reader.await_display(after['config_version'], before, timeout)
        except TimeoutError as error:
            raise PublishedButNotDisplayed(port_name, result) from error
    return {**result, 'confirmed': True, 'display_confirmed': True, 'output_sample': sample,
            'confirmation': 'CRC-valid C125 V5 publication and matching valid LAT1 FPGA output'}


def send_enhancement(port_name, mode, timeout=6.):
    mode, timeout = mode_value(mode), _timeout(timeout)
    with _open_serial(port_name) as port:
        reader = StatusReader(port)
        before = reader.await_status(lambda s: True, timeout)
        if before['enhancement_mode'] == mode:
            return dict(confirmed=True, transmitted=False, already_applied=True, before=before, after=before)
        payload = mode_packet(mode)
        if port.write(payload) != len(payload):
            raise IOError('串口未发送完整增强包，模式未确认。')
        port.flush()
        after = reader.await_status(lambda s: s['enhancement_mode'] == mode and
            _forward(s['sample_ticks'], before['sample_ticks']), timeout)
    return dict(confirmed=True, transmitted=True, already_applied=False, before=before, after=after)


class ComparisonSession:
    """Keep an explicitly confirmed correction for reversible FPGA raw viewing."""
    def __init__(self):
        self.corrected_words = None
        self.saved_mode = None
        self.raw_active = False

    def apply(self, port_name, words, **kwargs):
        words = validate_words(words)
        if words[16] != 1:
            raise ValueError('Apply requires corrected mode; use show_raw for raw comparison.')
        try:
            result = send_configuration(port_name, words, **kwargs)
        except PublishedButNotDisplayed:
            self.corrected_words = words.copy()
            self.raw_active = False
            raise
        self.corrected_words = words.copy()
        self.raw_active = False
        return result

    def show_raw(self, port_name):
        if self.corrected_words is None:
            raise ValueError('先成功发送一次矫正配置，才能保存并恢复当前参数。')
        if self.raw_active:
            raise ValueError('已经处于原图对比模式；请恢复矫正。')
        from complete_geometry import IDENTITY_MATRIX
        self.saved_mode = read_status(port_name)['enhancement_mode']
        raw_words = self.corrected_words.copy()
        raw_words[14:16] = [256, 0]
        raw_words[16] = 0
        raw_words[18:] = IDENTITY_MATRIX
        enhancement = send_enhancement(port_name, 'bypass')
        # Keep an explicit restoration path even if the subsequent geometry
        # transport fails: enhancement may already have changed successfully.
        self.raw_active = True
        try:
            geometry = send_configuration(port_name, raw_words)
        except PublishedButNotDisplayed:
            self.raw_active = True
            raise
        self.raw_active = True
        return dict(geometry=geometry, enhancement=enhancement, raw_active=True)

    def restore(self, port_name):
        if not self.raw_active or self.corrected_words is None or self.saved_mode is None:
            raise ValueError('没有本会话保存的原图对比恢复点。')
        enhancement = send_enhancement(port_name, self.saved_mode)
        try:
            geometry = send_configuration(port_name, self.corrected_words)
        except PublishedButNotDisplayed:
            self.raw_active = False
            raise
        self.raw_active = False
        return dict(geometry=geometry, enhancement=enhancement, raw_active=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Explicit physical send/read authorization; omitted means offline only.')
    parser.add_argument('--status', action='store_true')
    parser.add_argument('--confirm-output', type=Path, help='Read-only retry from saved status_confirmation.json; no parameters resent.')
    parser.add_argument('--enhancement', choices=MODE_NAMES)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--preset', choices=tuple(PRESETS), default='c97_joint')
    parser.add_argument('--roll', type=float, default=0.)
    parser.add_argument('--pitch', type=float, default=0.)
    parser.add_argument('--yaw', type=float, default=0.)
    parser.add_argument('--zoom', type=float, default=1.)
    parser.add_argument('--tx', type=float, default=0.)
    parser.add_argument('--ty', type=float, default=0.)
    parser.add_argument('--gain', type=float, default=1.)
    parser.add_argument('--offset', type=int, default=0)
    parser.add_argument('--pose', action='store_true', help='Use explicit pose/zoom/translation instead of preset.')
    parser.add_argument('--matrix-json', type=Path, help='JSON 3x3 output-to-source inverse H.')
    parser.add_argument('--four-points-json', type=Path, help='JSON {output_points:[[x,y]...],source_points:[[x,y]...]}; normalized undistorted plane.')
    parser.add_argument('--no-black', action='store_true', help='Reject any invalid pixels; never auto-crop.')
    parser.add_argument('--out', type=Path, default=ROOT/'output/complete_parameters')
    args = parser.parse_args(argv)
    if args.confirm_output:
        if not args.port:
            parser.error('--confirm-output requires --port')
        publication = json.loads(args.confirm_output.read_text(encoding='utf-8-sig'))
        result = confirm_display(args.port, publication)
        args.confirm_output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    if args.status:
        if not args.port:
            parser.error('--status requires --port')
        print(json.dumps(read_status(args.port), ensure_ascii=False, indent=2))
        return
    if args.enhancement:
        if not args.port:
            parser.error('--enhancement requires --port')
        print(json.dumps(send_enhancement(args.port, args.enhancement), ensure_ascii=False, indent=2))
        return
    if args.matrix_json and args.four_points_json:
        parser.error('Choose either --matrix-json or --four-points-json.')
    calibration = load_calibration(args.calibration)
    if args.matrix_json or args.four_points_json or args.pose:
        matrix = None
        if args.matrix_json:
            matrix = json.loads(args.matrix_json.read_text(encoding='utf-8-sig'))
        if args.four_points_json:
            points = json.loads(args.four_points_json.read_text(encoding='utf-8-sig'))
            matrix = four_point_matrix(points['output_points'], points['source_points'])
        words = configuration(calibration, matrix=matrix, roll=args.roll, pitch=args.pitch, yaw=args.yaw,
                              zoom=args.zoom, tx=args.tx, ty=args.ty, gain=args.gain, offset=args.offset)
    else:
        words = preset(args.preset, calibration)
    result = export_configuration(words, args.out, allow_black=not args.no_black,
                                  extra='Parameters only; no map or video data uploaded.')
    if args.port:
        try:
            result['application'] = send_configuration(args.port, words, allow_black=not args.no_black)
        except PublishedButNotDisplayed as error:
            (args.out/'status_confirmation.json').write_text(json.dumps(error.publication, ensure_ascii=False, indent=2), encoding='utf-8')
            parser.exit(2, str(error) + '\n用 --confirm-output ' + str(args.out/'status_confirmation.json') + ' --port 重查。\n')
        (args.out/'status_confirmation.json').write_text(json.dumps(result['application'], ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
