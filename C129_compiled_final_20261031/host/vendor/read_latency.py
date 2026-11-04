"""Read-only FPGA telemetry framing (V5 + V6/LAT1 + C125 V7/POS1).

Boundary: first accepted camera pixel at FPGA input -> corresponding source
frame's first/last active pixel at FPGA video output. Exposure, sensor pipeline,
HDMI capture card, USB, PC preview and panel response are excluded.
No configuration bytes are transmitted. V5 board ticks establish FPS; neither
USB video FPS nor host receive times establish FPGA throughput or latency.
"""
import argparse
import csv
import json
import statistics
import struct
import time
from collections import Counter
from pathlib import Path

from camera_config import crc16
from read_board_status import FORMATS, add_frame_rates, extract_packets
from complete_control import enhancement_status
from pose_protocol import REALTIME_BUILD_ID, REALTIME_BUILD_IDS, decode_pose, verify_pose

MAGIC = 0x4C415431
from project_profile import BUILD_ID as EXPECTED_BUILD_ID
MASK = 0xFFFFFFFF
LATENCY_FIELDS = (
    'magic', 'source_frame_id', 'version_flags', 'unit_ns',
    'input_first_us', 'capture_done_us', 'warp_start_us', 'warp_done_us',
    'output_first_us', 'output_last_us', 'repeated_output_frames',
    'display_underflow', 'missing_capture_tags', 'missing_display_tags',
    'metadata_overrun', 'uart_drops')
METRICS = ('capture_us', 'input_queue_us', 'warp_us', 'display_wait_us',
           'first_pixel_latency_us', 'scanout_us', 'complete_frame_latency_us')
BOUNDARY = ('FPGA camera first-valid-pixel input to the same source frame first/'
            'last active pixel at FPGA video output; excludes exposure, sensor '
            'pipeline, capture card, USB, PC preview and panel response')


class MixedFramer:
    """CRC checked, bounded-frame resynchronization; accept arbitrary read cuts."""
    def __init__(self):
        self.buffer = bytearray()
        self.counters = Counter()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while True:
            pos = self.buffer.find(b'\x55\xaa')
            if pos < 0:
                keep = 1 if self.buffer.endswith(b'\x55') else 0
                self.counters['discarded_bytes'] += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = b'\x55'
                else:
                    self.buffer.clear()
                break
            if pos:
                self.counters['discarded_bytes'] += pos
                del self.buffer[:pos]
            if len(self.buffer) < 4:
                break
            version, length = self.buffer[2:4]
            if (version, length) not in FORMATS and (version, length) not in ((6, 64), (7, 52), (8, 56)):
                self.counters['unsupported_headers'] += 1
                del self.buffer[0]
                continue
            total = length + 6
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            if crc16(frame[:-2]) != int.from_bytes(frame[-2:], 'big'):
                self.counters['crc_failures'] += 1
                del self.buffer[0]
                continue
            del self.buffer[:total]
            if version in (7, 8):
                data = decode_pose(struct.unpack('<13I' if version == 7 else '<14I', frame[4:-2]))
            elif version == 6:
                data = dict(zip(LATENCY_FIELDS, struct.unpack('<16I', frame[4:-2])))
                data['config_version'] = data['version_flags'] & 65535
                data['flags'] = data.pop('version_flags') >> 16
                data['protocol_version'] = 6
                data['packet_type'] = 'latency'
            else:
                data = extract_packets(bytearray(frame))[0]
                data['packet_type'] = 'status'
            self.counters['crc_valid_packets'] += 1
            packets.append(data)
        return packets


def percentile(values, percent):
    ordered = sorted(values)
    if not ordered:
        return None
    point = (len(ordered) - 1) * percent / 100.0
    lower = int(point)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (point - lower)


class LatencyAnalysis:
    def __init__(self, max_seconds=5.0):
        if not 0 < max_seconds < 0x80000000 / 1_000_000:
            raise ValueError('max_seconds must be positive and less than half timestamp range')
        self.max_us = int(max_seconds * 1_000_000)
        self.samples = []
        self.statuses = []
        self.pose_statuses = []
        self.seen_ids = set()
        self.last_id = None
        self.rejections = Counter()
        self.diagnostic_active = False
        self.firmware_verified = False
        self.first_counters = None
        self.latest_counters = {}

    def accept(self, packet):
        """Annotate a copy, retaining rejected records for audit, never statistics."""
        data = dict(packet)
        if data['packet_type'] == 'pose_status':
            verify_pose(data)
            self.pose_statuses.append(data.copy())
            # POS1 is control evidence. It is never a LAT1 frame sample nor a
            # periodic V5 throughput window, so do not contaminate either.
            return data
        if data['packet_type'] == 'status':
            if data['build_id'] not in (EXPECTED_BUILD_ID, *REALTIME_BUILD_IDS):
                raise ValueError(f'Wrong firmware: 0x{data["build_id"]:08X}')
            self.firmware_verified = True
            if data.get('protocol_version') == 5:
                data.update(enhancement_status(data))
            add_frame_rates(data, self.statuses[-1] if self.statuses else None)
            self.statuses.append(data.copy())
            return data
        reason = None
        if data['magic'] != MAGIC:
            reason = 'bad_magic'
        elif data['unit_ns'] != 1000:
            reason = 'unsupported_time_unit'
        else:
            # Only an actual CRC-valid LAT1 record establishes diagnostic mode.
            self.diagnostic_active = True
            self.latest_counters = {key: data[key] for key in LATENCY_FIELDS[10:]}
            if self.first_counters is None:
                self.first_counters = self.latest_counters.copy()
            if not data['flags'] & 1:
                reason = 'invalid_sample_flag'
            elif data['flags'] & 2:
                reason = 'bad_display_frame'
            elif data['flags'] & 4:
                reason = 'missing_frame_tag'
            elif data['flags'] & 8:
                reason = 'metadata_overrun_flag'
            elif data['source_frame_id'] in self.seen_ids:
                reason = 'duplicate_source_frame'
            elif self.last_id is not None and not (
                    0 < ((data['source_frame_id'] - self.last_id) & MASK) < 0x80000000):
                reason = 'out_of_order_source_frame'
            else:
                base = data['input_first_us']
                names = LATENCY_FIELDS[5:10]
                offsets = [((data[name] - base) & MASK) for name in names]
                if any(offset > self.max_us for offset in offsets):
                    reason = 'timestamp_outside_bound'
                elif offsets != sorted(offsets):
                    reason = 'stage_order_invalid'
                else:
                    cap, start, done, first, last = offsets
                    data.update(capture_us=cap, input_queue_us=start-cap,
                                warp_us=done-start, display_wait_us=first-done,
                                first_pixel_latency_us=first, scanout_us=last-first,
                                complete_frame_latency_us=last)
                    if (sum(data[k] for k in METRICS[:4]) != first or
                            first + data['scanout_us'] != last):
                        reason = 'stage_sum_invalid'
        data['accepted'] = reason is None
        if reason:
            self.rejections[reason] += 1
            data['rejection_reason'] = reason
        else:
            self.seen_ids.add(data['source_frame_id'])
            self.last_id = data['source_frame_id']
            self.samples.append(data.copy())
        return data

    @staticmethod
    def distributions(samples):
        distributions = {}
        for key in METRICS:
            values = [sample[key] for sample in samples]
            distributions[key] = (dict(count=len(values), min=min(values),
                max=max(values), mean=statistics.fmean(values),
                p50=percentile(values, 50), p95=percentile(values, 95)) if values else None)
        return distributions

    def summary(self):
        versions = sorted({s['config_version'] for s in self.samples})
        enhancement_modes = sorted({s['enhancement_mode'] for s in self.statuses
                                    if 'enhancement_mode' in s})
        gamma_ids = sorted({s['gamma_id'] for s in self.statuses if 'gamma_id' in s})
        per_config = {str(version): self.distributions([
            s for s in self.samples if s['config_version'] == version]) for version in versions}
        counter_deltas = {key: (self.latest_counters[key]-value) & MASK
                          for key, value in (self.first_counters or {}).items()}
        rates = {}
        status_health = {}
        # add_frame_rates computes from FPGA second ticks, not UART arrival time.
        if len(self.statuses) >= 2:
            whole = dict(self.statuses[-1])
            add_frame_rates(whole, self.statuses[0])
            rates = {key: whole.get(key) for key in
                ('observed_raw_fps', 'observed_new_fps', 'sample_interval_seconds', 'rate_status')}
            for key in ('underflow', 'overflow', 'dropped', 'camera_skipped_frames'):
                if key in self.statuses[0] and key in whole:
                    rates['delta_' + key] = (whole[key] - self.statuses[0][key]) & MASK
        if self.statuses:
            status_health = dict(
                system_error_seen=any(s.get('system_error', s.get('error_sticky', False))
                                      for s in self.statuses),
                camera_error_seen=any(s.get('camera_error', False) for s in self.statuses),
                first_cumulative_counters={key: self.statuses[0].get(key) for key in
                    ('underflow', 'overflow', 'dropped', 'camera_skipped_frames')},
                last_cumulative_counters={key: self.statuses[-1].get(key) for key in
                    ('underflow', 'overflow', 'dropped', 'camera_skipped_frames')},
                first_flags=self.statuses[0].get('flags'),
                last_flags=self.statuses[-1].get('flags'),
                note='system_error_seen uses V5 error_sticky (flags bit2); '
                     'historical nonzero counters alone are not new window errors')
        reasons = []
        if not self.diagnostic_active:
            reasons.append('diagnostic_not_detected')
        if not self.firmware_verified:
            reasons.append('firmware_not_verified')
        if not self.samples:
            reasons.append('no_accepted_latency_samples')
        if len(self.statuses) < 2 or rates.get('rate_status') != 'ok':
            reasons.append('insufficient_or_invalid_V5_rate_window')
        if status_health.get('system_error_seen'):
            reasons.append('V5_system_error_flag_seen')
        if status_health.get('camera_error_seen'):
            reasons.append('V5_camera_error_flag_seen')
        for key in ('underflow', 'overflow', 'dropped', 'camera_skipped_frames'):
            if rates.get('delta_' + key, 0):
                reasons.append('V5_window_' + key + '_increment_or_counter_reset')
        for key in ('display_underflow', 'missing_capture_tags', 'missing_display_tags',
                    'metadata_overrun', 'uart_drops'):
            if counter_deltas.get(key, 0):
                reasons.append('LAT1_window_' + key + '_increment_or_counter_reset')
        if self.rejections:
            reasons.append('latency_records_rejected_in_window')
        if len(versions) > 1:
            reasons.append('mixed_configuration_window')
        if len(enhancement_modes) > 1:
            reasons.append('mixed_enhancement_modes_window')
        return dict(boundary=BOUNDARY, timestamp_unit_ns=1000,
            timing_uncertainty_note='1 us is counter resolution, not certified +/-1 us '
                'accuracy. Cross-clock Gray synchronization and quantization may '
                'introduce several microseconds of uncertainty; absolute accuracy '
                'has not been independently instrument-verified.',
            diagnostic_active=self.diagnostic_active, firmware_verified=self.firmware_verified,
            expected_build_id=f'0x{EXPECTED_BUILD_ID:08X}',
            supported_build_ids=[f'0x{x:08X}' for x in (EXPECTED_BUILD_ID, *REALTIME_BUILD_IDS)],
            pose_event_snapshots=len(self.pose_statuses),
            accepted_samples=len(self.samples), rejected_samples=sum(self.rejections.values()),
            rejection_reasons=dict(self.rejections), max_allowed_latency_us=self.max_us,
            config_versions=versions,
            distributions_us=self.distributions(self.samples) if len(versions) <= 1 and
                len(enhancement_modes) <= 1 else None,
            distributions_us_by_config=per_config,
            first_cumulative_counters=self.first_counters,
            latest_cumulative_counters=self.latest_counters,
            cumulative_counter_deltas_in_received_window=counter_deltas,
            status_packets=len(self.statuses), board_tick_rates=rates,
            V5_window_health=status_health,
            enhancement_observations=dict(selected_modes=enhancement_modes,
                gamma_ids=gamma_ids,
                association='V5 status observations only; LAT1 carries no Gamma mode or curve ID',
                measurement_protocol='Collect a separate stable run for each fixed mode '
                    '(bypass, night, highlight). Record auto mode separately; its curve may '
                    'change between display frames. Do not label LAT1 samples with an '
                    'exact Gamma curve inferred from asynchronous V5 status.'),
            measurement_trust=dict(passed=not reasons, reasons=reasons,
                scope='Measurement-window integrity only; not automatic contest '
                      'acceptance or a full-system performance certification'),
            notes=['Repeated displays are not independent new-frame samples.',
                   'No USB FPS or host packet arrival times are used for FPGA metrics.',
                   'Statistics exclude CRC errors, invalid flags, duplicate IDs, '
                   'out-of-order IDs and implausible/out-of-order timestamps.',
                   'Different config versions are reported separately, never pooled.',
                   'Cumulative counters may include startup events. Window deltas '
                   'cover only the first-to-last received LAT1 record; UART drops '
                   'mean latency statistics are based on a subset of source frames.',
                   'Startup, remap and steady-state intervals must be reported separately.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM15')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--max-latency-seconds', type=float, default=5.0)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 86400:
        parser.error('--seconds must be within (0, 86400]')
    analysis = LatencyAnalysis(args.max_latency_seconds)
    import serial
    from serial.tools import list_ports
    from sensor_register import verify_cp210x
    verify_cp210x(args.port, list_ports.comports())
    args.out.mkdir(parents=True, exist_ok=True)
    framer = MixedFramer()
    failure = None
    begin = time.monotonic()
    try:
        with (args.out/'raw.jsonl').open('w', encoding='utf-8') as raw, \
             (args.out/'uart_stream.bin').open('wb') as stream, \
             serial.Serial(args.port, 115200, timeout=.2) as port:
            # Read only. No port.write, no map uploads, no parameter changes.
            while time.monotonic() - begin < args.seconds:
                chunk = port.read(1024)
                stream.write(chunk)
                for packet in framer.feed(chunk):
                    record = analysis.accept(packet)
                    record['host_received_monotonic_s'] = time.monotonic() - begin
                    raw.write(json.dumps(record, ensure_ascii=False) + '\n')
                    raw.flush()
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
    summary = analysis.summary()
    summary.update(host_collection_seconds=time.monotonic()-begin,
                   framing_counters=dict(framer.counters), failure=failure)
    if failure or framer.counters.get('crc_failures', 0):
        summary['measurement_trust']['passed'] = False
        summary['measurement_trust']['reasons'].append(
            'host_collection_failure' if failure else 'UART_CRC_errors_in_window')
    with (args.out/'samples.csv').open('w', newline='', encoding='utf-8-sig') as handle:
        columns = ['source_frame_id', 'config_version'] + list(METRICS)
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(analysis.samples)
    (args.out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not summary['measurement_trust']['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
