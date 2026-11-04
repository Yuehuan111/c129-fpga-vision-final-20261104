import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_pose import matrix_to_quaternion, quaternion_to_matrix
from mounting_calibration import (LABELS, analyze_window, capture, fit_mounting,
                                  fit_session, quaternion_mean, rotation_distance_deg)


def rotation(vector):
    vector = np.asarray(vector, dtype=float)
    angle = np.linalg.norm(vector)
    if angle == 0:
        return np.eye(3)
    return quaternion_to_matrix([math.cos(angle / 2), *(vector / angle * math.sin(angle / 2))])


def window(matrix, noise_deg=0., seed=17):
    rng = np.random.default_rng(seed)
    return [dict(received_at=100 + i / 100,
                 quaternion=matrix_to_quaternion(matrix @ rotation(np.deg2rad(rng.normal(0, noise_deg, 3)))))
            for i in range(120)]


def endpoints(mounting, convention='imu_to_world', angle_deg=15., noise_deg=0., profile='static'):
    result = {}
    # A nonidentity world attitude and distinct start attitudes exercise local
    # relative rotations; replacing all starts by one reference is incorrect.
    base = rotation([.41, -.28, .73])
    for i, axis in enumerate('xyz'):
        start = base @ rotation(np.eye(3)[(i+1) % 3] * .04 * i)
        end = start @ rotation(mounting[:, i] * math.radians(angle_deg))
        for name, matrix in (('start', start), ('end', end)):
            qmatrix = matrix if convention == 'imu_to_world' else matrix.T
            result[f'{axis}_{name}'] = analyze_window(window(qmatrix, noise_deg, 17 + i + (name == 'end')), profile=profile)
    return result


class MountingMathTests(unittest.TestCase):
    def test_recorded_handheld_window_requires_explicit_profile(self):
        fixture = Path(__file__).parent / 'fixtures' / 'handheld_x_end_20261025.jsonl'
        samples = [json.loads(line) for line in fixture.read_text().splitlines()]
        static = analyze_window(samples)
        handheld = analyze_window(samples, profile='handheld')
        self.assertFalse(static['passed'])
        self.assertAlmostEqual(static['rms_deg'], .2105, places=3)
        self.assertAlmostEqual(static['drift_deg'], .5095, places=3)
        self.assertTrue(handheld['passed'], handheld)
        self.assertEqual(static['sample_count'], handheld['sample_count'])
        self.assertEqual(static['mean_quaternion_wxyz'], handheld['mean_quaternion_wxyz'])

    def test_handheld_still_rejects_movement_and_unknown_profile(self):
        moving = [dict(received_at=i / 100, quaternion=matrix_to_quaternion(rotation([0, 0, math.radians(i * .025)])))
                  for i in range(120)]
        self.assertFalse(analyze_window(moving, profile='handheld')['passed'])
        with self.assertRaises(ValueError):
            analyze_window(window(np.eye(3)), profile='loose')

    def test_handheld_fit_requires_large_angle_and_stronger_noise_ratio(self):
        for angle, noise, passed in ((9., 0., False), (11., .25, False), (20., .25, True)):
            data = endpoints(rotation([.4, .7, -.6]), angle_deg=angle, noise_deg=noise, profile='handheld')
            self.assertTrue(all(q['passed'] for q in data.values()))
            report = fit_mounting(data, min_angle_deg=1.)
            self.assertEqual(report['passed'], passed, report)
            self.assertEqual(report['limits']['min_angle_deg'], 10.)
            self.assertEqual(report['limits']['min_angle_to_noise_ratio'], 20.)
        # Even one handheld endpoint selects the stronger fit requirements.
        mixed = endpoints(np.eye(3), angle_deg=9.)
        mixed['x_start']['profile'] = 'handheld'
        self.assertFalse(fit_mounting(mixed)['passed'])
    def test_arbitrary_mountings_both_conventions_with_distinct_starts(self):
        rng = np.random.default_rng(9)
        for convention in ('imu_to_world', 'world_to_imu'):
            for _ in range(6):
                expected = rotation(rng.normal(0, 1.5, 3))
                report = fit_mounting(endpoints(expected, convention))
                self.assertTrue(report['passed'], report)
                self.assertEqual(report['mounting']['convention'], convention)
                self.assertFalse(report['mounting']['confirmed'])
                self.assertLess(rotation_distance_deg(expected, np.asarray(report['mounting']['R_IC'])), 1e-5)
                self.assertGreater(max(report['start_separation_deg'].values()), 1)
                wrong = next(c for c in report['candidates'] if c['convention'] != convention)
                self.assertLess(wrong['triple_product'], -.9)
                self.assertFalse(wrong['passed'])

    def test_noise_tolerated_but_reported(self):
        expected = rotation([.8, -.4, 1.3])
        report = fit_mounting(endpoints(expected, noise_deg=.035))
        self.assertTrue(report['passed'], report)
        self.assertLess(rotation_distance_deg(expected, np.asarray(report['mounting']['R_IC'])), .15)

    def test_only_two_axes_cannot_fit(self):
        data = endpoints(np.eye(3))
        del data['z_end']
        report = fit_mounting(data)
        self.assertFalse(report['passed'])
        self.assertNotIn('mounting', report)

    def test_parallel_axes_rejected(self):
        data = endpoints(np.eye(3))
        start = quaternion_to_matrix(data['z_start']['mean_quaternion_wxyz'])
        data['z_end'] = analyze_window(window(start @ rotation([.2, 0, 0])))
        report = fit_mounting(data)
        self.assertFalse(report['passed'], report)
        self.assertNotIn('mounting', report)

    def test_reversed_action_can_mimic_other_convention_never_confirms(self):
        # Fundamental ambiguity: an odd sign mistake reverses measured
        # handedness. The data cannot prove the user's physical action label.
        data = endpoints(np.eye(3))
        start = quaternion_to_matrix(data['z_start']['mean_quaternion_wxyz'])
        data['z_end'] = analyze_window(window(start @ rotation([0, 0, -.2])))
        report = fit_mounting(data)
        correct_convention = report['candidates'][0]
        self.assertFalse(correct_convention['passed'])
        self.assertLess(correct_convention['triple_product'], 0)
        self.assertIn('reversed action', report['identifiability_warning'])
        if report['passed']:
            self.assertFalse(report['mounting']['confirmed'])

    def test_nonorthogonal_actions_rejected(self):
        data = endpoints(np.eye(3))
        start = quaternion_to_matrix(data['z_start']['mean_quaternion_wxyz'])
        data['z_end'] = analyze_window(window(start @ rotation(np.array([.7, 0, 1]) * .2)))
        self.assertFalse(fit_mounting(data)['passed'])

    def test_small_trial_angle_requires_explicit_lower_limit(self):
        data = endpoints(rotation([.3, .4, .5]), angle_deg=2., noise_deg=.01)
        self.assertFalse(fit_mounting(data)['passed'])
        self.assertTrue(fit_mounting(data, min_angle_deg=1.)['passed'])
        noisy = endpoints(np.eye(3), angle_deg=1.1, noise_deg=.075)
        self.assertTrue(all(r['passed'] for r in noisy.values()))
        self.assertFalse(fit_mounting(noisy, min_angle_deg=1.)['passed'])

    def test_zero_excessive_motion_or_distant_starts_rejected(self):
        for angle in (0., 40.):
            self.assertFalse(fit_mounting(endpoints(np.eye(3), angle_deg=angle))['passed'])
        data = endpoints(np.eye(3))
        start = rotation([1., 0, 0])
        data['z_start'] = analyze_window(window(start))
        data['z_end'] = analyze_window(window(start @ rotation([0, 0, .2])))
        self.assertIn('starting_attitudes_too_far_apart_for_convention_test', fit_mounting(data)['reasons'])

    def test_window_mean_sign_invariant_and_moving_window_rejected(self):
        samples = window(rotation([.2, .5, -.3]))
        for sample in samples[::2]:
            sample['quaternion'] = [-x for x in sample['quaternion']]
        self.assertTrue(analyze_window(samples)['passed'])
        moving = [dict(received_at=i / 100, quaternion=matrix_to_quaternion(rotation([0, 0, i * .0002])))
                  for i in range(120)]
        result = analyze_window(moving)
        self.assertFalse(result['passed'])
        self.assertIn('endpoint_drift_or_motion', result['reasons'])

    def test_rate_gap_invalid_and_short_windows_rejected(self):
        data = window(np.eye(3))
        self.assertFalse(analyze_window(data[:10])['passed'])
        self.assertFalse(analyze_window([])['passed'])
        for row in data:
            row['received_at'] *= 10
        self.assertFalse(analyze_window(data)['passed'])
        data[3]['quaternion'] = [float('nan'), 0, 0, 0]
        self.assertFalse(analyze_window(data)['passed'])
        data = window(np.eye(3))
        for row in data:
            row['received_at'] = 1.
        self.assertFalse(analyze_window(data)['passed'])


class FakeSerial:
    def __init__(self, clock, quaternion, **kwargs):
        self.clock, self.kwargs, self.closed = clock, kwargs, False
        payload = struct.pack('<4f', *quaternion)
        raw = b'\x7e\x23\x15\x16' + payload
        self.frame = raw + bytes([sum(raw) & 255])
        self.in_waiting = len(self.frame)

    def open(self):
        pass

    def reset_input_buffer(self):
        pass

    def read(self, size):
        self.clock[0] += .01
        return self.frame

    def write(self, data):
        raise AssertionError('Calibration must not write serial bytes')

    def close(self):
        self.closed = True


class CaptureSessionTests(unittest.TestCase):
    def fake_capture(self, directory, label, quaternion=(1, 0, 0, 0), serial_error=None, **options):
        clock = [10. + 100 * LABELS.index(label)]
        made = []
        def factory(**kwargs):
            if serial_error:
                raise serial_error
            stream = FakeSerial(clock, quaternion, **kwargs)
            made.append(stream)
            return stream
        with patch('serial.Serial', side_effect=factory), \
             patch('imu_session.list_serial_ports', return_value=[dict(device='COM_FAKE', role='imu')]), \
             patch('mounting_calibration.time.monotonic', side_effect=lambda: clock[0]):
            result = capture('COM_FAKE', directory, label, **options)
        if made:
            self.assertTrue(made[0].closed)
            self.assertFalse(made[0].dtr)
            self.assertFalse(made[0].rts)
        return result

    def test_capture_only_reads_and_preserves_raw_samples_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = self.fake_capture(temp, 'x_start')
            self.assertTrue(summary['quality']['passed'], summary)
            self.assertEqual(summary['transmitted_bytes'], 0)
            path = Path(temp) / 'x_start'
            self.assertGreater((path / 'raw.bin').stat().st_size, 2000)
            rows = [json.loads(line) for line in (path / 'samples.jsonl').read_text().splitlines()]
            self.assertIn('received_at', rows[0])
            previous = (path / 'summary.json').read_bytes()
            with self.assertRaises(FileExistsError):
                self.fake_capture(temp, 'x_start')
            self.assertEqual(previous, (path / 'summary.json').read_bytes())

    def test_open_failure_leaves_failed_summary(self):
        with tempfile.TemporaryDirectory() as temp:
            result = self.fake_capture(temp, 'x_start', serial_error=OSError('simulated busy port'))
            self.assertFalse(result['capture_complete'])
            self.assertFalse(result['quality']['passed'])
            self.assertIn('simulated busy port', result['error'])
            self.assertTrue((Path(temp) / 'x_start' / 'summary.json').exists())

    def test_failed_retry_preserves_attempt_and_success_is_immutable(self):
        with tempfile.TemporaryDirectory() as temp:
            self.fake_capture(temp, 'x_start', serial_error=OSError('busy'))
            directory = Path(temp) / 'x_start'
            before = {p.name: p.read_bytes() for p in directory.iterdir()}
            result = self.fake_capture(temp, 'x_start', handheld=True, retry_failed=True)
            self.assertTrue(result['quality']['passed'])
            archive = Path(temp) / result['archived_attempt']
            self.assertEqual(before, {p.name: p.read_bytes() for p in archive.iterdir()})
            self.assertEqual(result['profile'], 'handheld')
            previous = (directory / 'summary.json').read_bytes()
            with self.assertRaises(ValueError):
                self.fake_capture(temp, 'x_start', retry_failed=True)
            self.assertEqual(previous, (directory / 'summary.json').read_bytes())
            self.assertEqual(len(list((Path(temp) / 'failed_attempts').iterdir())), 1)

    def test_retry_refuses_uncertain_or_external_archive(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / 'x_start'
            directory.mkdir()
            (directory / 'summary.json').write_text(json.dumps(dict(label='x_start', quality={})))
            with self.assertRaises(ValueError):
                self.fake_capture(temp, 'x_start', retry_failed=True)
        with tempfile.TemporaryDirectory() as temp:
            self.fake_capture(temp, 'x_start', serial_error=OSError('busy'))
            archive = Path(temp).resolve() / 'failed_attempts'
            original_resolve = Path.resolve
            def fake_resolve(path, *args, **kwargs):
                return archive.parent.parent / 'outside' if path == archive else original_resolve(path, *args, **kwargs)
            with patch.object(Path, 'resolve', fake_resolve), self.assertRaises(ValueError):
                self.fake_capture(temp, 'x_start', retry_failed=True)
            self.assertTrue((Path(temp) / 'x_start' / 'summary.json').exists())

    def test_handheld_session_profile_is_rechecked_and_cannot_be_downgraded(self):
        with tempfile.TemporaryDirectory() as temp:
            data = endpoints(np.eye(3), angle_deg=20.)
            for label in LABELS:
                self.fake_capture(temp, label, data[label]['mean_quaternion_wxyz'], handheld=True)
            result = fit_session(temp)
            self.assertTrue(result['passed'], result)
            self.assertEqual(result['limits']['min_angle_deg'], 10.)
            self.assertFalse(result['mounting']['confirmed'])
            summary_path = Path(temp) / 'x_start' / 'summary.json'
            summary = json.loads(summary_path.read_text())
            summary['profile'] = 'static'
            summary_path.write_text(json.dumps(summary))
            self.assertFalse(fit_session(temp)['passed'])

    def test_full_session_fit_gui_schema_and_tamper_failure(self):
        from imu_app import read_mounting
        expected = rotation([.6, -.8, 1.])
        data = endpoints(expected, 'world_to_imu')
        with tempfile.TemporaryDirectory() as temp:
            for label in LABELS:
                self.fake_capture(temp, label, data[label]['mean_quaternion_wxyz'])
            report = fit_session(temp)
            self.assertTrue(report['passed'], report)
            mounting = read_mounting(Path(report['output_directory']) / 'mounting.temporary.json')
            self.assertEqual(mounting['convention'], 'world_to_imu')
            self.assertFalse(mounting['confirmed'])
            self.assertLess(rotation_distance_deg(expected, np.asarray(mounting['R_IC'])), 1e-4)
            path = Path(temp) / 'x_end' / 'samples.jsonl'
            path.write_text(path.read_text() + '\n')
            failed = fit_session(temp)
            self.assertFalse(failed['passed'])
            self.assertFalse((Path(failed['output_directory']) / 'mounting.temporary.json').exists())

    def test_missing_or_failed_endpoint_does_not_emit_mounting(self):
        with tempfile.TemporaryDirectory() as temp:
            self.fake_capture(temp, 'x_start', serial_error=OSError('failure'))
            report = fit_session(temp)
            self.assertFalse(report['passed'])
            self.assertNotIn('mounting', report)
            self.assertTrue((Path(report['output_directory']) / 'quality_report.json').exists())

    def test_label_port_or_capture_order_mismatch_rejected(self):
        import hashlib
        for field in ('label', 'port', 'receive_order'):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as temp:
                data = endpoints(np.eye(3))
                for label in LABELS:
                    self.fake_capture(temp, label, data[label]['mean_quaternion_wxyz'])
                directory = Path(temp) / 'y_start'
                summary = json.loads((directory / 'summary.json').read_text())
                if field == 'receive_order':
                    rows = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
                    for row in rows:
                        row['received_at'] -= 200
                    (directory / 'samples.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
                    summary['sha256']['samples.jsonl'] = hashlib.sha256((directory / 'samples.jsonl').read_bytes()).hexdigest()
                else:
                    metadata = json.loads((directory / 'metadata.json').read_text())
                    metadata[field] = summary[field] = 'z_start' if field == 'label' else 'COM_OTHER'
                    (directory / 'metadata.json').write_text(json.dumps(metadata))
                    summary['sha256']['metadata.json'] = hashlib.sha256((directory / 'metadata.json').read_bytes()).hexdigest()
                (directory / 'summary.json').write_text(json.dumps(summary))
                result = fit_session(temp)
                self.assertFalse(result['passed'], result)
                self.assertNotIn('mounting', result)


if __name__ == '__main__':
    unittest.main()
