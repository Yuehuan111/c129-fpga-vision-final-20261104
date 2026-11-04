"""Read archived C129 quality evidence; never opens hardware or processes frames."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import statistics


def load_quality_bundle(directory):
    directory = Path(directory).resolve()
    manifest_raw = (directory / 'manifest.json').read_bytes()
    manifest = json.loads(manifest_raw)
    if manifest.get('schema') != 'c129_imported_quality_v1':
        raise ValueError('未知的画质资料格式')
    files = {'manifest.json': manifest_raw}
    assets = {}
    for key, entry in manifest['assets'].items():
        path = (directory / entry['path']).resolve()
        if not path.is_relative_to(directory):
            raise ValueError('画质资料路径越界')
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != entry['sha256']:
            raise ValueError('画质资料哈希不一致：' + key)
        files[path.relative_to(directory).as_posix()] = raw
        assets[key] = (path, raw)

    sharp = json.loads(assets['sharpness'][1])
    straight = json.loads(assets['straightness'][1])
    inputs = {item['id']: item for item in json.loads(assets['inputs'][1])['inputs']}
    verification = json.loads(assets['verification'][1])
    quality = {'original': {}, 'corrected': {}}
    raw_reports = {}

    def close(actual, expected):
        if (not math.isfinite(actual) or actual < 0 or
                not math.isclose(actual, expected, rel_tol=1e-7, abs_tol=1e-9)):
            raise ValueError('画质汇总与明细不一致')

    def record(role, group, report_key, input_id, measurement, values):
        source_path, source_raw = assets[input_id]
        source_hash = hashlib.sha256(source_raw).hexdigest()
        if source_hash != inputs[input_id]['sha256']:
            raise ValueError('画质原图与输入清单不一致')
        report_path, report_raw = assets[report_key]
        quality[role][group] = {
            'report_path': str(report_path),
            'report_sha256': hashlib.sha256(report_raw).hexdigest(),
            'source_path': str(source_path), 'source_sha256_reported': source_hash,
            'source_sha256_at_import': source_hash, 'source_exists_at_import': True,
            'measurement': measurement, 'values': values,
            'note': '2026-11-02 普通矫正／原图历史实拍；细节增强关闭；非当前实时帧。',
            'report': json.loads(report_raw),
        }
        raw_reports[(role, group)] = report_raw

    common = sharp['sharpness']['1']['common_edges']
    if common != sharp['sharpness']['2']['common_edges'] or not common:
        raise ValueError('原图与矫正图没有相同的匹配斜边')
    for role, image_id, input_id in [('corrected', '1', 'A1'), ('original', '2', 'A2')]:
        summary = sharp['sharpness'][image_id]
        details = {roi['label']: roi for roi in sharp['sharpness_roi_details'][image_id]}
        values = []
        for label in common:
            roi = details[label]
            measurement = roi['measurement']
            mtf = measurement['mtf50_cycles_per_pixel']
            if (roi['status'] != 'accepted' or roi['axis'] != 'horizontal'
                    or measurement['mtf50_status'] != 'measured'
                    or not isinstance(mtf, (int, float)) or not 0 < mtf <= .5):
                raise ValueError('匹配斜边缺少有效 MTF50 交点')
            values.append(mtf)
        median = statistics.median(values)
        close(median, summary['common_edge_median_mtf50_cyc_per_px'])
        close(median, verification['common_edge_median_mtf50_cycles_per_pixel'][image_id])
        if summary['common_edge_count'] != len(common):
            raise ValueError('匹配边数不一致')
        record(role, 'edge', 'sharpness', input_id, 'matched_slanted_edge_sfr_summary',
               {'mtf50_cycles_per_pixel': median, 'mtf50_status': 'measured',
                'common_edge_count': len(common), 'common_edges': common,
                'transfer': 'srgb', 'gamma': None, 'roi': None})

    for role, index, input_id in [('corrected', 0, 'B1'), ('original', 1, 'B2')]:
        image = straight['images'][index]
        if image['sha256'] != inputs[input_id]['sha256']:
            raise ValueError('直线度报告与原图不一致')
        lines = image['lines']
        count = sum(line['point_count'] for line in lines)
        if count <= 0 or any(line['point_count'] < 3 for line in lines):
            raise ValueError('直线度有效点数不足')
        pooled = math.sqrt(sum(line['point_count'] * line['rms_px'] ** 2 for line in lines) / count)
        maximum = max(line['max_abs_px'] for line in lines)
        worst = max(line['rms_px'] for line in lines)
        summary = image['all_lines']
        close(pooled, summary['rms_px'])
        close(maximum, summary['max_abs_px'])
        close(worst, summary['worst_line_rms_px'])
        if count != summary['point_count'] or len(lines) != summary['line_count']:
            raise ValueError('直线度样本数不一致')
        record(role, 'line', 'straightness', input_id, 'grid_line_straightness_summary',
               {'line_rms_pooled_px': pooled, 'line_rms_max_px': worst,
                'line_max_residual_px': maximum, 'line_count': len(lines),
                'point_count': count, 'p95_abs_px': summary['p95_abs_px'],
                'label': '整体RMS / 最大残差'})

    reduction = 100 * (1 - quality['corrected']['line']['values']['line_rms_pooled_px'] /
                       quality['original']['line']['values']['line_rms_pooled_px'])
    close(reduction, straight['rms_reduction_percent'])
    text = assets['retest'][1].decode('utf-8-sig')
    pairs, supplemental = [], []
    for line in text.splitlines():
        parts = [part.strip() for part in line.split('|')]
        if len(parts) == 1:
            parts = [part.strip() for part in line.split('/')]
        if len(parts) == 7 and parts[0].startswith('新样本'):
            pairs.append({'batch': parts[0], 'corrected_file': parts[1],
                          'original_file': parts[2], 'common_edge_count': int(parts[3]),
                          'corrected_mtf50': float(parts[4]), 'original_mtf50': float(parts[5]),
                          'common_edges': [edge.strip() for edge in parts[6].split(',')],
                          'evidence': 'report_summary_only', 'detail_enhancement': None})
        elif len(parts) == 4 and parts[0].isdigit():
            supplemental.append({'batch': int(parts[0]), 'common_edge_count': int(parts[1]),
                                 'corrected_mtf50': None if parts[2] == '—' else float(parts[2]),
                                 'original_mtf50': None if parts[3] == '—' else float(parts[3])})
    if len(pairs) != 4 or len(supplemental) != 5:
        raise ValueError('复测汇总缺少批次')
    for pair in pairs + supplemental:
        if pair['common_edge_count'] < 0:
            raise ValueError('复测边数无效')
        for key in ('corrected_mtf50', 'original_mtf50'):
            value = pair[key]
            if ((pair['common_edge_count'] == 0 and value is not None) or
                    (pair['common_edge_count'] > 0 and (value is None or not 0 < value <= .5))):
                raise ValueError('复测 MTF50 或空值状态无效')
    if any(pair['common_edge_count'] != len(pair['common_edges']) for pair in pairs):
        raise ValueError('复测共同边数与边名不一致')

    rows = [('11/02 整体直线 RMS 降幅', f'{reduction:.2f}% · 两图各40条网格线'),
            ('11/03 清晰度复测（报告值）', '近水平边 MTF50 中位数；单位 cycles/pixel；原图 → 矫正')]
    for index, pair in enumerate(pairs, 1):
        rows.append((f"复测{index} · {pair['common_edge_count']}条共同边",
                     f"{pair['original_mtf50']:.3f} → {pair['corrected_mtf50']:.3f}"))
    rows.append(('近垂直边补充（独立统计）', 'MTF50：原图 → 矫正；不与近水平边混算'))
    for index, pair in enumerate(supplemental):
        label = '11/02旧样本' if index == 0 else f'11/03复测{index}'
        value = (f"{pair['original_mtf50']:.3f} → {pair['corrected_mtf50']:.3f} · {pair['common_edge_count']}条边"
                 if pair['common_edge_count'] else '无共同有效交点（保留空值）')
        rows.append((label + '近垂直边', value))
    rows.append(('画质资料范围', '11/02原图哈希与明细汇总已核对；11/03仅报告汇总，原图及逐边JSON未附，细节增强设置未确认。'))
    summary = {'schema': 'c129_imported_quality_summary_v1', 'archives': manifest['archives'],
               'assets': manifest['assets'], 'straightness_rms_reduction_percent': reduction,
               'sharpness_retests_20261103': pairs, 'near_vertical_edge_supplement': supplemental,
               'baseline_date': '2026-11-02', 'retest_date': '2026-11-03',
               'note': 'Historical offline image measurements, not current-frame telemetry. '
                       'Near-horizontal and near-vertical edges stay separate; no crossing remains null. '
                       'Retests are rounded report values, without original images or per-ROI JSON; '
                       'detail enhancement is unconfirmed. Old pair is not a new capture.'}
    return quality, raw_reports, summary, rows, files
