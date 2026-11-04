"""Analyze a host JSONL log offline, without opening any serial port."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    def percentile(p):
        at = (len(values)-1)*p
        lower = int(at)
        upper = min(lower+1, len(values)-1)
        return values[lower]+(at-lower)*(values[upper]-values[lower])
    return dict(count=len(values), min=values[0], p50=statistics.median(values),
                p95=percentile(.95), max=values[-1])


def analyze(records):
    requests, statuses, event_counts = {}, [], Counter()
    session_ids = set()
    for item in records:
        if item.get('session_id') is not None:
            session_ids.add(item['session_id'])
            if len(session_ids) > 1:
                raise ValueError('Mixed HostSession IDs in one log; analyze each session separately to avoid request-ID and counter collisions')
        event = item.get('event', '')
        event_counts[event] += 1
        request = item.get('request')
        request_id = item.get('request_id', (request or {}).get('request_id'))
        if request_id is not None:
            target = requests.setdefault(request_id, {'request_id': request_id})
            if request:
                target.update(request)
            if event == 'continuous_pose_refreshed':
                target['sample'] = item['sample']
            for event_name, field in (('accepted', 'accepted_at'), ('published', 'published_at')):
                if event == event_name:
                    target[field] = item.get('received_at', item.get('monotonic_s'))
                    target[event_name+'_status'] = item.get('status')
            if event == 'write_complete':
                target['write_done_at'] = item['write_done_at']
            if event == 'output_confirmed':
                target['output_confirmed_at'] = target.get('output_confirmed_at', item.get('monotonic_s'))
        if event in ('status', 'status_health'):
            status = item.get('status') or {}
            if status.get('protocol_version') == 5:
                statuses.append(status)
    metrics = {name: [] for name in ('geometry_s', 'serial_write_drain_s',
               'write_to_accept_observed_s', 'write_to_publish_observed_s',
               'write_to_output_observed_s', 'pose_age_at_write_s',
               'write_to_all_confirmed_s', 'pose_age_at_all_confirmed_s',
               'pose_age_at_publish_observed_s', 'pose_age_at_output_observed_s',
               'map_rebuild_reported_s', 'source_input_minus_pose_receive_proxy_s')}
    rows = []
    for target in requests.values():
        row = {key: target.get(key) for key in ('request_id', 'request_kind', 'phase',
                  'admission', 'transaction', 'source_frame_id', 'reason')}
        sample_at = (target.get('sample') or {}).get('received_at')
        pairs = [('geometry_s', target.get('geometry_done_at'), target.get('geometry_started_at')),
                 ('serial_write_drain_s', target.get('write_done_at'), target.get('write_started_at')),
                 ('write_to_accept_observed_s', target.get('accepted_at'), target.get('write_started_at')),
                 ('write_to_publish_observed_s', target.get('published_at'), target.get('write_started_at')),
                 ('write_to_output_observed_s', target.get('output_at'), target.get('write_started_at')),
                 ('write_to_all_confirmed_s', target.get('output_confirmed_at'), target.get('write_started_at')),
                 ('pose_age_at_write_s', target.get('write_started_at'), sample_at),
                 ('pose_age_at_publish_observed_s', target.get('published_at'), sample_at),
                 ('pose_age_at_output_observed_s', target.get('output_at'), sample_at)]
        pairs.append(('pose_age_at_all_confirmed_s', target.get('output_confirmed_at'), sample_at))
        for name, end, start in pairs:
            if end is not None and start is not None:
                row[name] = end-start
                metrics[name].append(row[name])
        status = target.get('published_status') or {}
        row['config_version'] = status.get('config_version')
        for key in ('map_invalid_pixels', 'map_domain_pixels', 'reject_flags'):
            if key in status:
                row[key] = status[key]
        if 'map_cycles' in status:
            row['map_rebuild_reported_s'] = status['map_cycles']/100_000_000
            metrics['map_rebuild_reported_s'].append(row['map_rebuild_reported_s'])
        output = target.get('output_sample') or {}
        if (sample_at is not None and target.get('published_at') is not None and
                'board_time_us' in status and 'input_first_us' in output):
            delta = ((output['input_first_us']-status['board_time_us']+(1 << 31)) & 0xFFFFFFFF)-(1 << 31)
            if abs(delta) <= 5_000_000:
                proxy = target['published_at']+delta/1_000_000-sample_at
                row['source_input_minus_pose_receive_proxy_s'] = proxy
                metrics['source_input_minus_pose_receive_proxy_s'].append(proxy)
        rows.append(row)
    def event_rate(field):
        moments = sorted(t[field] for t in requests.values() if t.get(field) is not None)
        return ((len(moments)-1)/(moments[-1]-moments[0]) if len(moments)>1 and moments[-1]>moments[0] else None)
    board_rates = None
    if len(statuses) >= 2:
        first, last = statuses[0], statuses[-1]
        ticks = last.get('sample_ticks', 0)-first.get('sample_ticks', 0)
        resets = any(b.get('sample_ticks', 0) < a.get('sample_ticks', 0) or
                     any(b.get(k, 0) < a.get(k, 0) for k in ('captured', 'processed')) or
                     a.get('build_id') != b.get('build_id') for a, b in zip(statuses, statuses[1:]))
        if ticks > 0 and not resets:
            board_rates = dict(window_seconds=ticks,
                               captured_fps=(last['captured']-first['captured'])/ticks,
                               processed_fps=(last['processed']-first['processed'])/ticks,
                               basis='Periodic V5 FPGA second ticks and counters; independently establish corrected/raw mode and output quality')
    return dict(requests=rows, event_counts=dict(event_counts), request_count=len(rows),
                version_evidence=dict(published=sum(t.get('published_at') is not None for t in requests.values()),
                    output_confirmed=sum(t.get('output_at') is not None for t in requests.values()),
                    superseded_or_output_unobserved=sum(t.get('phase') == 'superseded_or_output_unobserved' for t in requests.values()),
                    published_output_unverifiable=sum(t.get('phase') == 'published_output_unverifiable' for t in requests.values())),
                metrics_seconds={key: distribution(values) for key, values in metrics.items()},
                host_observed_rates_hz=dict(write=event_rate('write_started_at'),
                    publication=event_rate('published_at'), output_evidence_arrival=event_rate('output_at'),
                    output_confirmation=event_rate('output_confirmed_at')),
                board_v5_rates=board_rates,
                timing_proxy_warning='Source-input/pose-receive proxy uses POS1 receive time as an uncalibrated board-to-host clock anchor. It includes unknown UART/USB/OS delay; IMU measurement latency is unknown. Signed proxy is not exposure alignment or end-to-end error.',
                boundaries='Host times are observations. LAT1 starts at FPGA camera valid pixel, excludes exposure, sensor, capture card, USB display and panel. map_cycles includes controller work/waits, not necessarily pure coordinate computation.',
                hardware_opened=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    records = []
    malformed = []
    for number, line in enumerate(args.log.read_text(encoding='utf-8-sig').splitlines(), 1):
        if not line.strip():
            continue
        try:
            item = json.loads(line)
            if not isinstance(item, dict):
                raise ValueError('Log record is not an object')
            records.append(item)
        except (ValueError, TypeError) as exc:
            malformed.append(dict(line=number, reason=str(exc)))
    try:
        result = analyze(records)
    except ValueError as exc:
        parser.error(str(exc))
    result.update(input_log=str(args.log.resolve()), malformed_records=malformed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x', encoding='utf-8') as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
    print(json.dumps({key: value for key, value in result.items() if key != 'requests'}, ensure_ascii=False, indent=2))
    return 2 if malformed else 0


if __name__ == '__main__':
    raise SystemExit(main())
