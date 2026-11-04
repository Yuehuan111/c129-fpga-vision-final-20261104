"""Read OV5640 registers or write ONE register through verified UART v4 or later.

Requires exclusive UART access. Each command is sent once, without retries.
Writes are independently read back unless --skip-verify is explicitly selected
for a self-clearing/write-only register. No serial device is opened on import.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

from read_board_status import extract_packets
from sccb_packets import read_packet, write_packet

COUNTERS = ('sccb_rejected_count', 'sccb_crc_error_count',
            'sccb_framing_error_count', 'sccb_packet_error_count')
ERROR_NAMES = {1: 'sensor_nack', 2: 'sensor_timeout', 3: 'ownership_revoked'}
SCCB_REQUIRED_FIELDS = frozenset(COUNTERS + (
    'sample_ticks', 'build_id', 'sccb_accepted_seq', 'sccb_completed_seq',
    'sccb_runtime_owner', 'sccb_boot_config_done', 'sccb_busy', 'sccb_scl_high',
    'sccb_sda_high', 'sccb_result_valid', 'sccb_register', 'sccb_write',
    'sccb_error', 'sccb_nack_phase', 'sccb_value'))


class SensorError(RuntimeError):
    def __init__(self, code, message, status=None):
        super().__init__(message)
        self.code, self.status = code, status


def verify_cp210x(port_name, ports):
    """Validate enumerated identity before constructing/opening Serial."""
    matches = [p for p in ports if p.device.upper() == port_name.upper()
               and p.vid == 0x10c4 and p.pid == 0xea60]
    if len(matches) != 1:
        raise SensorError('port_not_verified', 'Selected port is not one verified CP210x (10C4:EA60).')
    return matches[0]


class JsonlLog:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('a', encoding='utf-8')

    def __call__(self, event):
        record = {'utc': datetime.now(timezone.utc).isoformat(), **event}
        self.stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + '\n')
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def close(self):
        self.stream.close()


class SensorSession:
    def __init__(self, port, *, timeout=5.0, emit=None, monotonic=time.monotonic):
        if not math.isfinite(timeout) or not 0 < timeout <= 5:
            raise ValueError('Each step timeout must be greater than zero and at most 5 seconds.')
        self.port, self.timeout = port, timeout
        self.emit = emit or (lambda event: None)
        self.monotonic = monotonic
        self.buffer = bytearray()
        self.pending = deque()
        self.operation_id = 0
        self.last_status = None

    def _next_status(self, deadline):
        while self.monotonic() < deadline:
            if self.pending:
                status, received = self.pending.popleft()
                if status['protocol_version'] < 4:
                    raise SensorError('protocol_not_v4', 'Runtime SCCB requires UART v4 or later.', status)
                missing = SCCB_REQUIRED_FIELDS.difference(status)
                if missing:
                    raise SensorError('sccb_status_incomplete',
                                      'Status lacks required SCCB fields: ' + ', '.join(sorted(missing)), status)
                return status, received
            self.port.timeout = min(0.1, max(0, deadline - self.monotonic()))
            self.buffer.extend(self.port.read(4096))
            received = self.monotonic()
            self.pending.extend((s, received) for s in extract_packets(self.buffer))
        raise SensorError('status_timeout', 'No matching fresh UART v4+ status before the step deadline.')

    @staticmethod
    def _tick_delta(current, previous):
        delta = (current['sample_ticks'] - previous['sample_ticks']) & 0xffffffff
        if current['build_id'] != previous['build_id'] or delta >= 0x80000000:
            raise SensorError('board_reset_or_out_of_order', 'Board reset or status moved backwards.', current)
        return delta

    def _fresh_baseline(self):
        # Discard both our parser queue and the OS receive queue. Then require
        # advancing v4+ board ticks whose arrival times are >=0.5 s apart: two
        # buffered old packets in one USB read cannot form a fresh baseline.
        # This is a freshness guard, not a host-clock rate measurement.
        self.buffer.clear()
        self.pending.clear()
        self.port.reset_input_buffer()
        deadline = self.monotonic() + self.timeout
        previous, received_before = self._next_status(deadline)
        if self.last_status is not None:
            self._tick_delta(previous, self.last_status)
            if previous['sccb_accepted_seq'] != self.last_status['sccb_accepted_seq']:
                raise SensorError('concurrent_transaction', 'SCCB sequence changed between this session\'s operations.', previous)
        while True:
            current, received = self._next_status(deadline)
            ticks = self._tick_delta(current, previous)
            if not ticks:
                continue
            if current['sccb_accepted_seq'] != previous['sccb_accepted_seq']:
                raise SensorError('concurrent_transaction', 'SCCB sequence changed before this command was sent.', current)
            if received - received_before < 0.5:
                previous, received_before = current, received
                continue
            if not current['sccb_runtime_owner'] or not current['sccb_boot_config_done']:
                raise SensorError('ownership_unavailable', 'Boot has not handed the sensor bus to runtime control.', current)
            if current['sccb_busy'] or current['sccb_accepted_seq'] != current['sccb_completed_seq']:
                raise SensorError('sensor_busy', 'An earlier sensor transaction is still outstanding; no command sent.', current)
            if not current['sccb_scl_high'] or not current['sccb_sda_high']:
                raise SensorError('sensor_bus_not_idle', 'SCL/SDA is held low; sensor reset/recovery may be required.', current)
            self.last_status = current
            return current

    def transaction(self, register, value=None, *, purpose='requested'):
        # Validate before touching even the receive queue.
        payload = read_packet(register) if value is None else write_packet(register, value)
        operation = 'read' if value is None else 'write'
        self.operation_id += 1
        context = {'operation_id': self.operation_id, 'operation': operation,
                   'register': register, 'register_hex': f'0x{register:04X}',
                   'requested_value': value, 'purpose': purpose}
        sent = False
        self.emit({'event': 'operation_started', **context})
        try:
            baseline = self._fresh_baseline()
            expected = (baseline['sccb_accepted_seq'] + 1) & 0xffffffff
            self.emit({'event': 'fresh_baseline', **context,
                       'expected_seq': expected, 'status': baseline})
            deadline = self.monotonic() + self.timeout
            # Log intent before writing so interrupted/partial writes leave an
            # audit trail. A serial write return is never treated as acceptance.
            self.emit({'event': 'command_sending', **context,
                       'expected_seq': expected, 'packet_hex': payload.hex(' ')})
            remaining = deadline - self.monotonic()
            if remaining <= 0:
                raise SensorError('status_timeout', 'Step deadline expired before transmission; no command sent.')
            self.port.write_timeout = remaining
            sent = True
            count = self.port.write(payload)
            if count != len(payload):
                raise SensorError('serial_short_write', f'Only {count} of {len(payload)} command bytes were queued.')
            self.emit({'event': 'command_sent', **context, 'expected_seq': expected})
            while True:
                status, _ = self._next_status(deadline)
                if not self._tick_delta(status, baseline):
                    continue
                self.last_status = status
                changed = [key for key in COUNTERS if status[key] != baseline[key]]
                if changed:
                    raise SensorError('command_rejected', 'SCCB receive/reject counters changed: ' + ', '.join(changed), status)
                seq = status['sccb_accepted_seq']
                if seq == baseline['sccb_accepted_seq']:
                    if not status['sccb_runtime_owner']:
                        raise SensorError('ownership_revoked', 'Runtime ownership disappeared before command acceptance.', status)
                    continue
                if seq != expected:
                    raise SensorError('unexpected_sequence', 'Another command/reset changed the SCCB sequence.', status)
                if status['sccb_busy'] or not status['sccb_result_valid']:
                    continue
                if status['sccb_completed_seq'] != expected:
                    raise SensorError('completion_sequence_mismatch', 'Completion sequence does not match this command.', status)
                if status['sccb_register'] != register or status['sccb_write'] != (value is not None):
                    raise SensorError('result_identity_mismatch', 'Completed register/read-write type does not match this command.', status)
                error = status['sccb_error']
                if error:
                    raise SensorError(ERROR_NAMES.get(error, 'unknown_sensor_error'),
                                      f'SCCB error {error}; NACK phase {status["sccb_nack_phase"]}.', status)
                if not status['sccb_runtime_owner']:
                    raise SensorError('ownership_revoked', 'Runtime ownership disappeared at completion.', status)
                result = {**context, 'sequence': expected, 'value': status['sccb_value'],
                          'value_hex': f'0x{status["sccb_value"]:02X}', 'status': status}
                self.emit({'event': 'operation_complete', **result})
                return result
        except Exception as error:
            self.emit({'event': 'operation_failed', **context,
                       'error_code': getattr(error, 'code', 'host_io_error'),
                       'message': str(error), 'status': getattr(error, 'status', None),
                       'command_may_have_been_sent': sent,
                       'write_may_have_applied': sent and value is not None})
            raise

    def write_register(self, register, value, *, verify=True):
        result = self.transaction(register, value)
        if not verify:
            self.emit({'event': 'verification_skipped', 'register': register,
                       'sequence': result['sequence'], 'reason': 'explicit_skip_verify'})
            return {'write': result, 'verified': False}
        readback = self.transaction(register, purpose='verify_write')
        if readback['value'] != value:
            self.emit({'event': 'verification_failed', 'register': register,
                       'expected_value': value, 'read_value': readback['value'],
                       'write_sequence': result['sequence'], 'read_sequence': readback['sequence']})
            raise SensorError('readback_mismatch',
                              f'Readback 0x{readback["value"]:02X} differs from requested 0x{value:02X}.',
                              readback['status'])
        self.emit({'event': 'verification_complete', 'register': register, 'value': value,
                   'write_sequence': result['sequence'], 'read_sequence': readback['sequence']})
        return {'write': result, 'readback': readback, 'verified': True}


def unsigned(bits):
    def parse(text):
        try:
            value = int(text, 0)
        except ValueError as error:
            raise argparse.ArgumentTypeError('Use a decimal or 0x-prefixed integer.') from error
        if not 0 <= value < 1 << bits:
            raise argparse.ArgumentTypeError(f'Value must fit unsigned {bits} bits.')
        return value
    return parse


def step_timeout(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError('Timeout must be a number.') from error
    if not math.isfinite(value) or not 0 < value <= 5:
        raise argparse.ArgumentTypeError('Timeout must be greater than zero and at most 5 seconds.')
    return value


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='Explicit CP210x UART port, e.g. COM15.')
    parser.add_argument('--timeout', type=step_timeout, default=5.0, help='Per-step deadline, at most 5 seconds.')
    parser.add_argument('--log', type=Path, help='Append JSONL audit log; default output/sensor_register/<UTC>.jsonl.')
    commands = parser.add_subparsers(dest='operation', required=True)
    read = commands.add_parser('read', help='Read one or more registers, one transaction at a time.')
    read.add_argument('registers', nargs='+', type=unsigned(16))
    write = commands.add_parser('write', help='Write exactly one register and independently read it back.')
    write.add_argument('register', type=unsigned(16))
    write.add_argument('value', type=unsigned(8))
    write.add_argument('--skip-verify', action='store_true', help='Explicitly skip readback for a self-clearing/write-only register.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    log_path = args.log or (Path(__file__).resolve().parents[1] / 'output' / 'sensor_register' /
                            (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '.jsonl'))
    log = JsonlLog(log_path)

    def emit(event):
        log({'port': args.port, **event})
        if event['event'] in ('operation_complete', 'operation_failed', 'verification_complete',
                               'verification_failed', 'verification_skipped'):
            print(json.dumps(event, ensure_ascii=False), flush=True)

    try:
        import serial
        from serial.tools import list_ports
        device = verify_cp210x(args.port, list_ports.comports())
        emit({'event': 'session_opening', 'vid': device.vid, 'pid': device.pid,
              'operation': args.operation, 'step_timeout_seconds': args.timeout,
              'log_path': str(log.path)})
        # No other device/port is opened. Serial writes have an explicit bound.
        with serial.Serial(device.device, 115200, timeout=0.1, write_timeout=args.timeout) as port:
            session = SensorSession(port, timeout=args.timeout, emit=emit)
            if args.operation == 'read':
                for register in args.registers:
                    session.transaction(register)
            else:
                session.write_register(args.register, args.value, verify=not args.skip_verify)
        emit({'event': 'session_complete'})
        print(json.dumps({'success': True, 'log_path': str(log.path)}), flush=True)
        return 0
    except KeyboardInterrupt:
        emit({'event': 'session_interrupted', 'message': 'Stopped without retry; inspect prior command log before writing again.'})
        return 130
    except (SensorError, OSError, ImportError) as error:
        event = {'event': 'session_failed', 'error_code': getattr(error, 'code', 'host_io_error'),
                 'message': str(error), 'status': getattr(error, 'status', None), 'log_path': str(log.path)}
        emit(event)
        print(json.dumps(event, ensure_ascii=False), file=sys.stderr, flush=True)
        return 2
    finally:
        log.close()


if __name__ == '__main__':
    raise SystemExit(main())
