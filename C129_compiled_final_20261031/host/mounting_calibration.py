"""Receive-only, temporary three-axis camera/IMU mounting calibration.

Camera axes: image right, image down, lens forward. No serial command is sent.
Successful labels are immutable; explicit retries archive failed captures.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np

from imu_pose import (ImuDecoder, matrix_to_quaternion, normalize_quaternion,
                      quaternion_to_matrix, validate_mounting_rotation)

LABELS = tuple(f'{axis}_{end}' for axis in 'xyz' for end in ('start', 'end'))
WINDOW_LIMITS = dict(min_samples=50, min_span_s=0.7, min_rate_hz=70., max_rate_hz=130.,
                     max_gap_s=0.15, max_rms_deg=0.2, max_radius_deg=0.6,
                     max_drift_deg=0.3)
WINDOW_PROFILES = {
    'static': WINDOW_LIMITS,
    'handheld': dict(WINDOW_LIMITS, max_rms_deg=0.5, max_radius_deg=1.2, max_drift_deg=1.0),
}
FIT_LIMITS = dict(max_angle_deg=30., max_axis_residual_deg=5.,
                  max_orthogonality_error_deg=10., min_triple_product=0.9,
                  min_score_separation_deg=20., max_start_separation_deg=20.,
                  min_angle_to_noise_ratio=10.)


def rotation_vector(matrix):
    """Principal SO(3) logarithm, robust also near zero and pi."""
    q = np.asarray(matrix_to_quaternion(matrix))
    length = float(np.linalg.norm(q[1:]))
    if length < 1e-12:
        return 2 * q[1:]
    return q[1:] * (2 * math.atan2(length, q[0]) / length)


def rotation_distance_deg(first, second):
    return math.degrees(float(np.linalg.norm(rotation_vector(first.T @ second))))


def quaternion_mean(quaternions):
    """Sign-invariant Markley mean of normalized wxyz samples."""
    qs = np.asarray([normalize_quaternion(q) for q in quaternions])
    if not len(qs):
        raise ValueError('No quaternion samples')
    _, vectors = np.linalg.eigh(qs.T @ qs)
    q = vectors[:, -1]
    return normalize_quaternion(q if q[0] >= 0 else -q)


def analyze_window(samples, *, profile='static'):
    """Check a whole endpoint window; never cherry-pick a still subsection."""
    if profile not in WINDOW_PROFILES:
        raise ValueError('Unknown endpoint profile')
    limits = WINDOW_PROFILES[profile]
    report = dict(passed=False, reasons=[], sample_count=len(samples), profile=profile, limits=limits.copy())
    reasons = report['reasons']
    if len(samples) < 2:
        reasons.append('fewer_than_two_samples')
        return report
    try:
        times = np.asarray([s['received_at'] for s in samples], dtype=float)
        qs = np.asarray([normalize_quaternion(s['quaternion']) for s in samples])
        if not np.isfinite(times).all() or np.any(np.diff(times) < 0):
            raise ValueError('Receive times must be finite and nondecreasing')
        mean = quaternion_mean(qs)
        distances = np.degrees(2 * np.arccos(np.clip(np.abs(qs @ mean), 0, 1)))
        quarter = max(1, len(qs) // 4)
        drift = rotation_distance_deg(quaternion_to_matrix(quaternion_mean(qs[:quarter])),
                                      quaternion_to_matrix(quaternion_mean(qs[-quarter:])))
        span = float(times[-1] - times[0])
        rate = (len(qs) - 1) / span if span > 0 else 0.
        report.update(mean_quaternion_wxyz=list(mean), span_s=span, receive_rate_hz=rate,
                      max_receive_gap_s=float(np.max(np.diff(times))),
                      rms_deg=float(np.sqrt(np.mean(distances ** 2))),
                      max_radius_deg=float(np.max(distances)), drift_deg=drift,
                      first_received_at=float(times[0]), last_received_at=float(times[-1]),
                      timing='Host receive time, not sensor acquisition or camera exposure time')
        checks = [(len(qs) >= limits['min_samples'], 'too_few_samples'),
                  (span >= limits['min_span_s'], 'short_sample_span'),
                  (limits['min_rate_hz'] <= rate <= limits['max_rate_hz'], 'receive_rate_outside_limits'),
                  (report['max_receive_gap_s'] <= limits['max_gap_s'], 'receive_gap_too_large'),
                  (report['rms_deg'] <= limits['max_rms_deg'], 'endpoint_noise_or_motion'),
                  (report['max_radius_deg'] <= limits['max_radius_deg'], 'endpoint_outlier_or_motion'),
                  (drift <= limits['max_drift_deg'], 'endpoint_drift_or_motion')]
        reasons.extend(reason for passed, reason in checks if not passed)
    except (ValueError, KeyError, TypeError) as exc:
        reasons.append(f'invalid_samples: {exc}')
    report['passed'] = not reasons
    return report


def fit_mounting(endpoints, *, min_angle_deg=5.):
    """Fit both quaternion conventions from all six independent endpoints.

    endpoints maps labels to analyze_window reports. Every movement uses its
    own start. Closely spaced starting attitudes are required to make the
    opposite convention's handedness test meaningful.
    """
    if not math.isfinite(min_angle_deg) or not 1 <= min_angle_deg <= 5:
        raise ValueError('Minimum angle must be in [1, 5] degrees')
    report = dict(passed=False, reasons=[], candidates=[], confirmed=False,
                  limits=dict(FIT_LIMITS, min_angle_deg=min_angle_deg),
                  camera_axes='right, down, forward', image_orientation='standard',
                  purpose='Temporary coarse mounting; physical validation still required',
                  identifiability_warning='Fit assumes correctly labeled positive physical motions. '
                  'A reversed action can masquerade as the opposite quaternion convention. '
                  'Three-axis endpoint data alone cannot detect every labeling error; '
                  'independent physical direction validation is mandatory.')
    missing = [label for label in LABELS if label not in endpoints]
    if missing:
        report['reasons'].append('missing_endpoints: ' + ', '.join(missing))
        return report
    bad = [label for label in LABELS if endpoints[label].get('passed') is not True]
    if bad:
        report['reasons'].append('failed_endpoints: ' + ', '.join(bad))
        return report
    profiles = {label: endpoints[label].get('profile', 'static') for label in LABELS}
    if any(profile not in WINDOW_PROFILES for profile in profiles.values()):
        report['reasons'].append('unknown_endpoint_profile')
        return report
    handheld = 'handheld' in profiles.values()
    min_angle_deg = max(min_angle_deg, 10.) if handheld else min_angle_deg
    min_noise_ratio = 20. if handheld else FIT_LIMITS['min_angle_to_noise_ratio']
    report['endpoint_profiles'] = profiles
    report['limits'].update(min_angle_deg=min_angle_deg, min_angle_to_noise_ratio=min_noise_ratio)
    report['endpoint_uncertainty_meaning'] = ('For handheld fits, endpoint fluctuation is max(RMS, drift/2); '
        'combined by root sum square. This is a conservative motion/noise proxy, not a calibrated '
        'confidence interval or optical accuracy. No division by sqrt(sample count).')
    matrices = {label: quaternion_to_matrix(endpoints[label]['mean_quaternion_wxyz']) for label in LABELS}
    starts = {f'{a}_{b}': rotation_distance_deg(matrices[f'{a}_start'], matrices[f'{b}_start'])
              for a, b in (('x', 'y'), ('x', 'z'), ('y', 'z'))}
    report['start_separation_deg'] = starts
    if max(starts.values()) > FIT_LIMITS['max_start_separation_deg']:
        report['reasons'].append('starting_attitudes_too_far_apart_for_convention_test')
    for convention in ('imu_to_world', 'world_to_imu'):
        r = {key: value if convention == 'imu_to_world' else value.T for key, value in matrices.items()}
        axes, angles, noise_ratios, combined_noise = [], {}, {}, {}
        candidate_reasons = []
        for axis in 'xyz':
            vector = rotation_vector(r[f'{axis}_start'].T @ r[f'{axis}_end'])
            length = float(np.linalg.norm(vector))
            angle = math.degrees(length)
            angles[axis] = angle
            endpoint_noise = [max(endpoints[f'{axis}_{end}']['rms_deg'],
                                  endpoints[f'{axis}_{end}']['drift_deg'] / 2 if handheld else 0.)
                              for end in ('start', 'end')]
            noise = math.hypot(*endpoint_noise)
            combined_noise[axis] = noise
            # A finite floor avoids infinite JSON values for exact synthetic data.
            noise_ratios[axis] = angle / max(noise, 1e-6)
            if not min_angle_deg <= angle <= FIT_LIMITS['max_angle_deg']:
                candidate_reasons.append(f'{axis}: angle_outside_limits')
            if noise_ratios[axis] < min_noise_ratio:
                candidate_reasons.append(f'{axis}: insufficient_angle_to_endpoint_noise')
            axes.append(vector / length if length > 1e-10 else np.zeros(3))
        m = np.column_stack(axes)
        u, singular, vt = np.linalg.svd(m)
        mounting = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
        residuals = {axis: math.degrees(math.acos(float(np.clip(m[:, i] @ mounting[:, i], -1, 1))))
                     for i, axis in enumerate('xyz')}
        orth = {f'{a}_{b}': abs(90 - math.degrees(math.acos(float(np.clip(m[:, i] @ m[:, j], -1, 1)))))
                for i, j, a, b in ((0, 1, 'x', 'y'), (0, 2, 'x', 'z'), (1, 2, 'y', 'z'))}
        triple = float(np.linalg.det(m))
        if max(residuals.values()) > FIT_LIMITS['max_axis_residual_deg']:
            candidate_reasons.append('axis_fit_residual_too_large')
        if max(orth.values()) > FIT_LIMITS['max_orthogonality_error_deg']:
            candidate_reasons.append('axes_not_orthogonal')
        if triple < FIT_LIMITS['min_triple_product']:
            candidate_reasons.append('axis_triple_product_not_positive_and_near_one')
        report['candidates'].append(dict(convention=convention, angles_deg=angles,
            angle_to_noise_ratio=noise_ratios, combined_endpoint_fluctuation_deg=combined_noise,
            measured_axes_columns=m.tolist(),
            R_IC=mounting.tolist(), axis_residual_deg=residuals,
            orthogonality_error_deg=orth, triple_product=triple, singular_values=singular.tolist(),
            score_deg=float(np.sqrt(np.mean(np.square(list(residuals.values()))))),
            passed=not candidate_reasons, reasons=candidate_reasons))
    candidates = sorted(report['candidates'], key=lambda c: c['score_deg'])
    winner, runner = candidates
    separation = runner['score_deg'] - winner['score_deg']
    report['score_separation_deg'] = separation
    if not winner['passed']:
        report['reasons'].append('best_candidate_failed_quality_checks')
    if runner['passed'] or separation < FIT_LIMITS['min_score_separation_deg']:
        report['reasons'].append('quaternion_convention_ambiguous')
    report['passed'] = not report['reasons']
    if report['passed']:
        validate_mounting_rotation(winner['R_IC'])
        report['mounting'] = dict(R_IC=winner['R_IC'], convention=winner['convention'], confirmed=False,
            note='Temporary three-axis coarse fit. Confirm physical directions and image orientation before use.')
    return report


def _write_json(path, value):
    with path.open('x', encoding='utf-8') as output:
        json.dump(value, output, indent=2, allow_nan=False)
        output.write('\n')


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _archive_failed(session, label):
    """Rename only an explicitly failed endpoint inside this resolved session."""
    root = session.resolve(strict=True)
    source = root / label
    if source.resolve(strict=True) != source or source.parent != root:
        raise ValueError('Retry source must be a direct, non-linked session endpoint')
    summary = json.loads((source / 'summary.json').read_text(encoding='utf-8'))
    passed = summary.get('quality', {}).get('passed')
    if summary.get('label') != label or passed is True or not (passed is False or summary.get('capture_complete') is False):
        raise ValueError('Only an explicitly failed/incomplete endpoint can be retried; successful endpoints are immutable')
    archive_root = root / 'failed_attempts'
    if archive_root.resolve() != archive_root or archive_root.parent != root:
        raise ValueError('Retry archive must be inside this session and cannot be linked')
    archive_root.mkdir(exist_ok=True)
    target = archive_root / (label + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8])
    if target.resolve() != target or not target.is_relative_to(root) or source.stat().st_dev != archive_root.stat().st_dev:
        raise ValueError('Retry archive must stay inside the same session filesystem')
    source.rename(target)
    return str(target.relative_to(root))


def capture(port, session, label, seconds=1.2, *, handheld=False, retry_failed=False):
    """Read only one endpoint. Never open FPGA, configure IMU, or send bytes."""
    import serial
    from imu_session import list_serial_ports

    session = Path(session)
    if label not in LABELS:
        raise ValueError('Unknown endpoint label')
    if not math.isfinite(seconds) or not .8 <= seconds <= 10:
        raise ValueError('Duration must be in [0.8, 10] seconds')
    if not any(p['device'] == port and p['role'] == 'imu' for p in list_serial_ports()):
        raise ValueError('Select an enumerated CH340 IMU port')
    session.mkdir(parents=True, exist_ok=True)
    output = session / label
    archived_attempt = _archive_failed(session, label) if output.exists() and retry_failed else None
    output.mkdir(exist_ok=False)
    profile = 'handheld' if handheld else 'static'
    decoder, samples = ImuDecoder(), []
    metadata = dict(label=label, port=port, baud=115200, serial_format='8N1',
                    requested_seconds=seconds, started_at=datetime.now().astimezone().isoformat(),
                    receive_only=True, transmitted_bytes=0, quaternion_order='wxyz',
                    confirmed=False, capture_complete=False, error=None, profile=profile,
                    archived_attempt=archived_attempt)
    _write_json(output / 'metadata.json', metadata)
    stream = None
    try:
        # Files are created even if opening the serial device fails.
        with (output / 'raw.bin').open('xb') as raw, (output / 'samples.jsonl').open('x', encoding='utf-8') as records:
            stream = serial.Serial(port=None, baudrate=115200, bytesize=8, parity='N', stopbits=1,
                                   timeout=.02, xonxoff=False, rtscts=False, dsrdtr=False)
            stream.dtr = stream.rts = False
            stream.port = port
            stream.open()
            stream.reset_input_buffer()
            started = time.monotonic()
            while time.monotonic() < started + seconds:
                chunk = stream.read(min(4096, max(1, stream.in_waiting)))
                received = time.monotonic()
                raw.write(chunk)
                for packet in decoder.feed(chunk, received_at=received):
                    if packet['kind'] == 'quaternion':
                        row = dict(sample_id=len(samples)+1, received_at=received,
                                   elapsed_s=received-started, quaternion=list(packet['quaternion']))
                        samples.append(row)
                        records.write(json.dumps(row, allow_nan=False) + '\n')
            metadata['actual_duration_s'] = time.monotonic() - started
            metadata['latest_sample_age_s'] = time.monotonic() - samples[-1]['received_at'] if samples else None
            metadata['capture_complete'] = True
    except (Exception, KeyboardInterrupt) as exc:
        metadata['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if stream is not None:
            try:
                stream.close()
            except Exception as exc:
                metadata['error'] = metadata['error'] or f'close_failed: {exc}'
    quality = analyze_window(samples, profile=profile)
    counters = {name: getattr(decoder, name) for name in ('accepted_packets', 'checksum_errors',
        'length_errors', 'unknown_functions', 'invalid_values', 'discarded_bytes', 'buffered_bytes')}
    if any(counters[name] for name in ('checksum_errors', 'length_errors', 'invalid_values')):
        quality['reasons'].append('decoder_integrity_errors')
    if metadata['error'] or not metadata['capture_complete']:
        quality['reasons'].append('capture_incomplete')
    if samples and (quality.get('span_s', 0) < .8 * seconds or
                    metadata.get('latest_sample_age_s', 0) > .15):
        quality['reasons'].append('capture_span_or_freshness_failed')
    quality['passed'] = not quality['reasons']
    summary = dict(**metadata, quality=quality, decoder_counters=counters,
                   sha256={p.name: _hash(p) for p in output.iterdir() if p.is_file()})
    _write_json(output / 'summary.json', summary)
    return summary


def fit_session(session, *, min_angle_deg=5.):
    """Recheck saved windows and produce a unique fit directory; no serial access."""
    session = Path(session)
    endpoints, source_errors, sources = {}, [], {}
    session_port, previous_last_time = None, None
    for label in LABELS:
        try:
            directory = session / label
            summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
            metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
            if summary['label'] != label or metadata['label'] != label:
                raise ValueError('directory/summary/metadata label mismatch')
            # All absent profile fields are the original static schema. Partial
            # or contradictory fields must never silently loosen old captures.
            profile_fields = [metadata.get('profile'), summary.get('profile'), summary['quality'].get('profile')]
            if all(value is None for value in profile_fields):
                profile = 'static'
            elif len(set(profile_fields)) != 1 or profile_fields[0] not in WINDOW_PROFILES:
                raise ValueError('unknown or inconsistent endpoint profile')
            else:
                profile = profile_fields[0]
            if not summary['port'] or summary['port'] != metadata['port']:
                raise ValueError('summary/metadata port mismatch')
            if session_port is not None and summary['port'] != session_port:
                raise ValueError('endpoint from a different IMU port')
            session_port = summary['port']
            if summary['quality']['passed'] is not True or not summary['capture_complete'] or summary['error']:
                raise ValueError('capture did not pass')
            for name in ('raw.bin', 'samples.jsonl', 'metadata.json'):
                if _hash(directory / name) != summary['sha256'][name]:
                    raise ValueError(f'hash mismatch: {name}')
            samples = [json.loads(line) for line in (directory / 'samples.jsonl').read_text(encoding='utf-8').splitlines()]
            quality = analyze_window(samples, profile=profile)
            if not quality['passed']:
                raise ValueError('saved sample window no longer passes')
            if previous_last_time is not None and quality['first_received_at'] <= previous_last_time:
                raise ValueError('endpoint receive times are not in x_start/x_end/y_start/y_end/z_start/z_end order')
            previous_last_time = quality['last_received_at']
            endpoints[label] = quality
            sources[label] = dict(summary_sha256=_hash(directory / 'summary.json'),
                                  files_sha256=summary['sha256'], quality=endpoints[label])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            source_errors.append(f'{label}: {exc}')
    report = fit_mounting(endpoints, min_angle_deg=min_angle_deg)
    report['sources'] = sources
    if source_errors:
        report['passed'] = False
        report['reasons'].extend(source_errors)
        report.pop('mounting', None)
    output = session / ('fit_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)
    report['output_directory'] = str(output.resolve())
    _write_json(output / 'quality_report.json', report)
    if report['passed']:
        _write_json(output / 'mounting.temporary.json', report['mounting'])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    record = commands.add_parser('capture', help='Hold still while recording one endpoint')
    record.add_argument('--port', required=True)
    record.add_argument('--session', type=Path, required=True)
    record.add_argument('--label', choices=LABELS, required=True)
    record.add_argument('--seconds', type=float, default=1.2)
    record.add_argument('--handheld', action='store_true', help='Explicit coarse handheld profile; fit requires all motions >=10 degrees')
    record.add_argument('--retry-failed', action='store_true', help='Archive an explicitly failed endpoint before retry; never replace success')
    fit = commands.add_parser('fit', help='Offline three-axis fit; always leaves confirmed=false')
    fit.add_argument('--session', type=Path, required=True)
    fit.add_argument('--min-angle-deg', type=float, default=5.,
                     help='Default 5; allow 1..5 for smaller trial motions, with noise checks still enforced')
    args = parser.parse_args(argv)
    try:
        if args.command == 'capture':
            result = capture(args.port, args.session, args.label, args.seconds,
                             handheld=args.handheld, retry_failed=args.retry_failed)
            print(json.dumps(result, indent=2, allow_nan=False))
            return 0 if result['quality']['passed'] else 1
        result = fit_session(args.session, min_angle_deg=args.min_angle_deg)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['passed'] else 1
    except (OSError, ValueError) as exc:
        parser.exit(1, f'{type(exc).__name__}: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
