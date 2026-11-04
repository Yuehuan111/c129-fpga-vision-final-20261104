"""Offline log-analysis timing boundaries, independently specified timestamps."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_realtime_log import analyze


class AnalysisTests(unittest.TestCase):
    def test_distinct_host_board_and_uncalibrated_clock_metrics(self):
        records = []
        for number in (1, 2):
            base = number*1.
            records.append(dict(event='output_confirmed', request=dict(request_id=number,
                phase='output_confirmed', sample={'received_at': base},
                geometry_started_at=base+.001, geometry_done_at=base+.002,
                write_started_at=base+.003, write_done_at=base+.011,
                accepted_at=base+.014, published_at=base+.04, output_at=base+.09,
                published_status=dict(config_version=number, map_cycles=1_000_000,
                                      board_time_us=0xFFFFFF00),
                output_sample=dict(input_first_us=0x00002610))))
            records.append(dict(event='status_health', status=dict(protocol_version=5,
                build_id=0x0720C125, sample_ticks=number, captured=number*30, processed=number*27)))
        result = analyze(records)
        metrics = result['metrics_seconds']
        self.assertAlmostEqual(metrics['pose_age_at_write_s']['p50'], .003)
        self.assertAlmostEqual(metrics['pose_age_at_publish_observed_s']['p50'], .04)
        self.assertAlmostEqual(metrics['pose_age_at_output_observed_s']['p50'], .09)
        self.assertAlmostEqual(metrics['source_input_minus_pose_receive_proxy_s']['p50'], .05)
        self.assertEqual(result['host_observed_rates_hz']['publication'], 1.)
        self.assertEqual(result['board_v5_rates']['processed_fps'], 27.)
        self.assertIn('not exposure alignment', result['timing_proxy_warning'])

    def test_v5_reset_invalidates_board_rate_window(self):
        records = [dict(event='status_health', status=dict(protocol_version=5,
                    build_id=1, sample_ticks=t, captured=t, processed=t)) for t in (1, 3, 2, 4)]
        self.assertIsNone(analyze(records)['board_v5_rates'])

    def test_mixed_session_logs_rejected_before_request_ids_merge(self):
        records = [dict(session_id=name, event='candidate_queued',
                        request=dict(request_id=1)) for name in ('session-a', 'session-b')]
        with self.assertRaisesRegex(ValueError, 'Mixed HostSession'):
            analyze(records)

    def test_unobserved_publication_counted_separately_from_output_success(self):
        result = analyze([dict(event='superseded_or_output_unobserved', request=dict(
            request_id=1, published_at=1., phase='superseded_or_output_unobserved')),
            dict(event='output_confirmed', monotonic_s=2., request=dict(
                request_id=2, published_at=1.5, output_at=1.9, phase='output_confirmed'))])
        self.assertEqual(result['version_evidence'], dict(published=2,
            output_confirmed=1, superseded_or_output_unobserved=1, published_output_unverifiable=0))


if __name__ == '__main__':
    unittest.main()
