"""Receive-only quaternion-to-matrix recording. No serial writes are performed."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import serial

from imu_pose import ImuDecoder, PoseHistory, quaternion_to_matrix
from imu_session import list_serial_ports


def collect(port_name, seconds, output):
    if not math.isfinite(seconds) or not 0 < seconds <= 3600:
        raise ValueError('Duration must be finite and in (0, 3600] seconds')
    if not any(p['device'] == port_name and p['role'] == 'imu' for p in list_serial_ports()):
        raise ValueError('Select an enumerated CH340 IMU port')
    output.mkdir(parents=True, exist_ok=False)
    metadata = dict(port=port_name, baud=115200, serial_format='8N1', requested_seconds=seconds,
                    receive_only=True, transmitted_bytes=0, quaternion_order='w,x,y,z',
                    matrix_field='R_from_quaternion', matrix_order='row-major',
                    quaternion_convention_physically_confirmed=False, camera_mounting_confirmed=False,
                    matrix_meaning='Right-handed active rotation from normalized wxyz quaternion. '
                    'Sensor-to-world versus world-to-sensor direction has not been physically verified. '
                    'This is not the camera compensation matrix H.',
                    timing_meaning='Host monotonic receive time; USB batches share timestamps. '
                    'Not IMU acquisition time or camera exposure time.')
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    decoder, history = ImuDecoder(), PoseHistory()
    stream = serial.Serial(port=None, baudrate=115200, bytesize=8, parity='N', stopbits=1,
                           timeout=.05, write_timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
    stream.dtr = stream.rts = False
    stream.port = port_name
    count = euler_count = byte_count = 0
    started = time.monotonic()
    first_time = last_time = None
    first_q = last_q = first_r = last_r = None
    max_gap = max_norm_error = max_orthogonal_error = max_relative_angle_deg = 0.
    determinant_min, determinant_max = float('inf'), float('-inf')
    error = None
    wall_started = datetime.now().astimezone().isoformat()
    try:
        stream.open()
        stream.reset_input_buffer()
        started = time.monotonic()
        deadline, next_report = started + seconds, started + 5
        print(json.dumps({'state': 'recording', 'port': port_name, 'seconds': seconds,
                          'directory': str(output.resolve())}), flush=True)
        with (output/'raw.bin').open('xb') as raw, \
                (output/'matrices.jsonl').open('x', encoding='utf-8') as records, \
                (output/'matrices.csv').open('x', newline='', encoding='utf-8') as table:
            writer = csv.writer(table)
            writer.writerow(['sample_id', 'host_monotonic_s', 'elapsed_s', 'qw', 'qx', 'qy', 'qz']
                            + [f'r{i}{j}' for i in range(3) for j in range(3)])
            while time.monotonic() < deadline:
                chunk = stream.read(min(4096, max(1, stream.in_waiting)))
                received = time.monotonic()
                raw.write(chunk)
                byte_count += len(chunk)
                for packet in decoder.feed(chunk, received_at=received):
                    if packet['kind'] == 'euler':
                        euler_count += 1
                        continue
                    sample = history.add(packet['quaternion'], packet['received_at'])
                    q = np.asarray(sample.quaternion)
                    matrix = quaternion_to_matrix(q)
                    row = dict(sample_id=sample.id, host_monotonic_s=sample.received_at,
                               elapsed_s=sample.received_at-started, quaternion_wxyz=q.tolist(),
                               R_from_quaternion=matrix.tolist())
                    records.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
                    writer.writerow([sample.id, sample.received_at, sample.received_at-started,
                                     *sample.quaternion, *matrix.ravel().tolist()])
                    if first_time is None:
                        first_time, first_q, first_r = sample.received_at, q.tolist(), matrix.tolist()
                    if last_time is not None:
                        max_gap = max(max_gap, sample.received_at-last_time)
                    count += 1
                    last_time, last_q, last_r = sample.received_at, q.tolist(), matrix.tolist()
                    max_norm_error = max(max_norm_error, abs(float(np.linalg.norm(q))-1))
                    max_orthogonal_error = max(max_orthogonal_error, float(np.max(np.abs(matrix.T@matrix-np.eye(3)))))
                    determinant = float(np.linalg.det(matrix))
                    determinant_min, determinant_max = min(determinant_min, determinant), max(determinant_max, determinant)
                    relative = np.asarray(first_r).T @ matrix
                    angle = math.degrees(math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1))))
                    max_relative_angle_deg = max(max_relative_angle_deg, angle)
                if received >= next_report:
                    raw.flush()
                    records.flush()
                    table.flush()
                    print(json.dumps({'state': 'recording', 'samples': count,
                                      'elapsed_s': round(received-started, 2)}), flush=True)
                    next_report = received + 5
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        ended = time.monotonic()
        stream.close()
    duration = ended-started
    counters = {key: getattr(decoder, key) for key in
                ('accepted_packets', 'checksum_errors', 'length_errors', 'unknown_functions',
                 'invalid_values', 'discarded_bytes', 'buffered_bytes')}
    summary = dict(**metadata, started_at=wall_started, actual_duration_s=duration,
                   samples=count, euler_packets=euler_count, bytes_received=byte_count,
                   rate_hz=count/duration if duration else 0,
                   sample_span_rate_hz=(count-1)/(last_time-first_time) if count>1 and last_time>first_time else None,
                   latest_age_s=ended-last_time if last_time is not None else None,
                   max_host_receive_gap_s=max_gap, decoder_counters=counters,
                   quaternion_max_norm_error=max_norm_error, matrix_max_orthogonality_error=max_orthogonal_error,
                   determinant_range=[determinant_min, determinant_max] if count else None,
                   max_rotation_from_first_deg=max_relative_angle_deg,
                   first_quaternion=first_q, last_quaternion=last_q,
                   first_matrix=first_r, last_matrix=last_r, error=error,
                   capture_complete=error is None and count>0 and duration>=seconds,
                   files={p.name:dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                          for p in sorted(output.iterdir()) if p.is_file()})
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({key: summary[key] for key in ('capture_complete', 'samples', 'rate_hz',
                    'actual_duration_s', 'decoder_counters', 'max_rotation_from_first_deg', 'error')}, indent=2), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    output = args.out or Path(__file__).resolve().parent/'captures'/datetime.now().strftime('%Y%m%d_%H%M%S')
    summary = collect(args.port, args.seconds, output)
    return 0 if summary['capture_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
