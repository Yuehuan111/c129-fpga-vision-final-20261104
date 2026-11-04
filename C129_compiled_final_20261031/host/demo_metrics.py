"""Read-only presentation metrics. No serial port, image processing or configuration writes."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from copy import deepcopy

VENDOR = Path(__file__).resolve().parent / 'vendor'
if str(VENDOR) not in sys.path:
    sys.path.insert(0, str(VENDOR))
from read_latency import LatencyAnalysis, BOUNDARY, percentile
from read_board_status import add_frame_rates
from complete_control import enhancement_status

STALE_SECONDS = 2.5
MODES = {'raw': '原图', 'normal': '普通矫正', 'imu': 'IMU 动态矫正',
         'scan': '平面扫描', 'enhance': '自适应增强'}


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _fresh(age):
    return _number(age) and 0 <= age <= STALE_SECONDS


def _fmt(value, unit='', digits=2):
    return f'{value:.{digits}f}{unit}' if _number(value) else '—'


class DemoMetrics:
    def __init__(self, project_root):
        self.root = Path(project_root)
        self._session_id = None
        self._regime = None
        self._previous_v5 = None
        self._last_v5_token = None
        self._last_latency_token = None
        self._rates = {}
        self._latency = None
        self._latency_version = None
        self._latency_validator = LatencyAnalysis()
        self._stats_key = None
        self._stats_count = 0
        self._stats = {}
        self._recording = None
        self._points = []
        self._segments = []
        self._last_point_second = None
        self._regime_started_at = None
        self.last_view = None
        self._resources = self._load_resources()
        self._quality = {'original': {}, 'corrected': {}}
        self._quality_bytes = {}
        self._quality_bundle = None
        self._quality_bundle_rows = []
        self._quality_bundle_files = {}
        bundle = self.root / 'host/results/imported_quality_20261103'
        if (bundle / 'manifest.json').is_file():
            try:
                from demo_quality_bundle import load_quality_bundle
                (self._quality, self._quality_bytes, self._quality_bundle,
                 self._quality_bundle_rows, self._quality_bundle_files) = load_quality_bundle(bundle)
            except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as error:
                self._quality_bundle_rows = [('历史画质资料未载入', str(error))]

    def _load_resources(self):
        results = list((self.root / 'fpga_reference/project/output').glob('c12[89]/result.json'))
        for path in results:
            try:
                result = json.loads(path.read_text(encoding='utf-8'))
                if result.get('status') == 'PASS':
                    return {'build_id': result['build_id'], 'resources': result['after']['resources'],
                            'timing': result['after']['timing']['corners'],
                            'path': str(path.relative_to(self.root)),
                            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                            'kind': 'offline PDS compile result; not a live measurement'}
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return None

    def load_quality_report(self, path, role='corrected'):
        """Import existing optical-tool measurements; never computes or invents quality."""
        if role not in self._quality:
            raise ValueError('请选择 original 或 corrected 结果角色。')
        path = Path(path).resolve()
        raw = path.read_bytes()
        if len(raw) > 8 * 1024 * 1024:
            raise ValueError('画质报告过大，请选择现有光学工具生成的 JSON 结果。')
        report = json.loads(raw.decode('utf-8-sig'), parse_constant=lambda value: (_ for _ in ()).throw(ValueError('报告含非有限数值。')))
        if not isinstance(report, dict):
            raise ValueError('不是光学工具结果对象。')
        def finite_tree(value):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('报告含非有限数值。')
            if isinstance(value, dict):
                for key, item in value.items():
                    if 'synthetic' in key.lower() and item not in (False, None, '', 0):
                        raise ValueError('合成测试图不能作为演示实测画质。')
                    finite_tree(item)
            elif isinstance(value, list):
                for item in value:
                    finite_tree(item)
            elif isinstance(value, str) and value.lower() == 'synthetic-test':
                raise ValueError('合成测试图不能作为演示实测画质。')
        finite_tree(report)
        kind = report.get('measurement')
        if kind not in ('checkerboard_geometry', 'manual_point_line_straightness', 'approximate_slanted_edge_sfr'):
            raise ValueError('不支持此 JSON；请选择棋盘几何、人工直线或斜边 SFR 工具结果。')
        def nonnegative(value):
            if not _number(value) or value < 0:
                raise ValueError('画质结果缺少有效、有限且非负的数值。')
            return float(value)
        values, evidence_note = {}, '离线图像分析；不自动认证为本机实拍或当前画面。'
        if kind == 'checkerboard_geometry':
            row_lines, column_lines = report.get('row_lines'), report.get('column_lines')
            if not isinstance(row_lines, list) or not isinstance(column_lines, list):
                raise ValueError('棋盘报告缺少逐行、逐列直线结果。')
            lines = row_lines + column_lines
            if not all(isinstance(line, dict) for line in lines):
                raise ValueError('棋盘逐线结果格式无效。')
            if not lines or not isinstance(report.get('source_sha256'), str) or len(report['source_sha256']) != 64:
                raise ValueError('棋盘结果缺少直线统计或原图哈希。')
            rms = [nonnegative(line.get('rms_residual')) for line in lines]
            maximum = [nonnegative(line.get('max_absolute_residual')) for line in lines]
            if any(line.get('units') != 'pixels' or line.get('point_count', 0) < 3 for line in lines):
                raise ValueError('棋盘直线统计单位或样本数无效。')
            aggregate = nonnegative(report.get('line_rms_max_px'))
            if not math.isclose(aggregate, max(rms), rel_tol=1e-7, abs_tol=1e-7):
                raise ValueError('棋盘汇总与逐线结果不一致。')
            values = {'line_rms_max_px': aggregate, 'line_max_residual_px': max(maximum),
                      'line_count': len(lines), 'label': '最差直线RMS / 最大残差'}
            source = report.get('source')
            source_hash = report.get('source_sha256')
            group = 'line'
        elif kind == 'manual_point_line_straightness':
            if report.get('units') != 'pixels' or report.get('fit') != 'orthogonal_total_least_squares' or report.get('point_count', 0) < 3:
                raise ValueError('需要至少3个点、像素单位的现有直线拟合报告。')
            residuals = report.get('residuals')
            if not isinstance(residuals, list) or len(residuals) != report['point_count'] or not all(_number(v) for v in residuals):
                raise ValueError('直线逐点残差缺失或无效。')
            rms, maximum = nonnegative(report.get('rms_residual')), nonnegative(report.get('max_absolute_residual'))
            if not math.isclose(rms, math.sqrt(sum(v*v for v in residuals)/len(residuals)), rel_tol=1e-7, abs_tol=1e-7) or not math.isclose(maximum, max(abs(v) for v in residuals), rel_tol=1e-7, abs_tol=1e-7):
                raise ValueError('直线汇总与逐点残差不一致。')
            values = {'line_rms_max_px': rms, 'line_max_residual_px': maximum, 'line_count': 1, 'label': '人工直线RMS / 最大残差'}
            source = report.get('image') or report.get('points_file')
            source_hash = report.get('image_sha256')
            group = 'line'
            evidence_note += ' 人工取点误差包含在结果内。'
        else:
            if report.get('source') != 'real-chart' or report.get('iso_certified') is not False or report.get('frequency_units') != 'cycles_per_pixel_along_edge_normal':
                raise ValueError('需要声明 real-chart、具有正确单位的近似斜边 SFR 报告。')
            roi = report.get('roi')
            if not isinstance(roi, list) or len(roi) != 4 or any(type(v) is not int for v in roi) or min(roi[:2]) < 0 or min(roi[2:]) <= 0:
                raise ValueError('斜边报告必须保留有效 ROI。')
            transfer = report.get('transfer')
            if transfer not in ('srgb', 'linear', 'gamma') or (transfer == 'gamma' and (not _number(report.get('gamma')) or report['gamma'] <= 0)):
                raise ValueError('斜边传递函数记录缺失或无效。')
            mtf = report.get('mtf50_cycles_per_pixel')
            if report.get('mtf50_status') == 'measured':
                if not _number(mtf) or not 0 < mtf <= .5:
                    raise ValueError('MTF50 超出报告允许的0至0.5 cycles/pixel频带。')
            elif report.get('mtf50_status') != 'not_crossed_below_nyquist' or mtf is not None:
                raise ValueError('MTF50 状态或数值无效。')
            if report.get('rows_used', 0) < 3 or not _number(report.get('linear_contrast')) or report['linear_contrast'] <= 0:
                raise ValueError('斜边报告没有有效拟合数据。')
            values = {'mtf50_cycles_per_pixel': mtf, 'mtf50_status': report['mtf50_status'], 'roi': roi,
                      'transfer': transfer, 'gamma': report.get('gamma')}
            source, source_hash = report.get('image'), report.get('image_sha256')
            group = 'edge'
            evidence_note += ' 来源类型由生成报告时声明；近似SFR，非ISO认证。'
        if not isinstance(source, str) or not source:
            raise ValueError('缺少原图或原始点文件路径，无法保留来源。')
        source_path = Path(source)
        if not source_path.is_absolute():
            source_path = path.parent/source_path
        source_exists = source_path.is_file()
        current_hash = hashlib.sha256(source_path.read_bytes()).hexdigest() if source_exists else None
        if source_hash is not None and (not isinstance(source_hash, str) or len(source_hash) != 64
                                       or any(char not in '0123456789abcdefABCDEF' for char in source_hash)):
            raise ValueError('原始来源哈希格式无效。')
        if source_hash and current_hash and source_hash.lower() != current_hash:
            raise ValueError('报告记录的原图哈希与当前文件不同，不能载入。')
        if not source_exists:
            evidence_note += ' 原始来源文件未随报告找到；仅能展示报告值。'
        elif not source_hash:
            evidence_note += ' 原报告未绑定来源哈希；当前文件哈希仅用于留档。'
        self._quality[role][group] = {'path': str(path), 'report_sha256': hashlib.sha256(raw).hexdigest(),
            'source_path': str(source_path), 'source_sha256_reported': source_hash,
            'source_sha256_at_import': current_hash, 'source_exists_at_import': source_exists,
            'measurement': kind, 'values': values, 'note': evidence_note, 'report': report}
        self._quality_bytes[(role, group)] = raw
        return f"已载入{'原图' if role == 'original' else '矫正后'}{('直线度' if group == 'line' else '近似MTF50')}报告；显示为离线结果。"

    def _clear_live(self):
        self._previous_v5 = None
        self._last_v5_token = None
        self._rates = {}
        self._latency = None
        self._latency_version = None
        self._last_latency_token = None
        self._latency_validator = LatencyAnalysis()
        self._stats_key, self._stats_count, self._stats = None, 0, {}

    def observe(self, data, video, mode=None, waiting=False, now=None):
        now = time.monotonic() if now is None else now
        data, video = data or {}, video or {}
        connected = bool((data.get('connected') or {}).get('fpga'))
        session_id = data.get('session_id')
        board = data.get('latest_pose_status') or data.get('last_status') or {}
        v5 = data.get('latest_v5_status') or {}
        v5_fresh = connected and _fresh(data.get('latest_v5_age_s'))
        board_fresh = connected and _fresh((data.get('fpga') or {}).get('status_age_s'))
        enhance = data.get('enhancement') or {}
        try:
            if v5_fresh:
                enhance = enhancement_status(v5)
        except (ValueError, KeyError, TypeError):
            enhance = {}
        regime = (mode, bool(board.get('raw_active')), board.get('committed_modes'),
                  enhance.get('enhancement_mode'), bool(data.get('continuous')),
                  bool((data.get('visual_scan') or {}).get('enabled')), bool(waiting),
                  board.get('observation_active'), board.get('detail_active'))
        changed = session_id != self._session_id or regime != self._regime
        if changed:
            self._clear_live()
            self._session_id, self._regime = session_id, regime
            self._regime_started_at = now
        if not board_fresh or waiting:
            self._clear_live()
        if self._recording and session_id == self._recording['session_id']:
            if not self._segments or changed:
                self._segments.append({'start_monotonic_s': now, 'mode': mode,
                    'regime': list(regime), 'waiting': bool(waiting)})
        v5_received = data.get('latest_v5_received_at')
        v5_in_regime = (_number(v5_received) and self._regime_started_at is not None
                        and v5_received >= self._regime_started_at)
        if board_fresh and v5_fresh and v5_in_regime and not waiting:
            token = (v5.get('build_id'), v5.get('sample_ticks'))
            if token != self._last_v5_token:
                current = dict(v5)
                try:
                    add_frame_rates(current, self._previous_v5)
                except (KeyError, ValueError, TypeError, ZeroDivisionError):
                    current = {}
                self._rates = current if current.get('rate_status') == 'ok' else {}
                self._last_v5_token = token
                self._previous_v5 = dict(v5)
        else:
            self._rates = {}
            self._previous_v5 = None
            self._last_v5_token = None
        packet = data.get('latest_latency_packet')
        latency_fresh = connected and _fresh(data.get('latest_latency_age_s'))
        version = board.get('config_version')
        # Dynamic pose versions may change faster than V5. Throughput remains a
        # mode-window counter rate; latency values always belong to one labelled
        # source/config version and never become a fabricated current sample.
        if self._latency_version != version:
            self._latency = None
        received_at = data.get('latest_latency_received_at')
        received_in_regime = (_number(received_at) and
                              self._regime_started_at is not None and received_at >= self._regime_started_at)
        if board_fresh and latency_fresh and received_in_regime and not waiting and packet:
            token = (packet.get('source_frame_id'), packet.get('config_version'),
                     packet.get('input_first_us'), packet.get('output_first_us'))
            if token != self._last_latency_token:
                self._last_latency_token = token
                try:
                    checked = self._latency_validator.accept(packet)
                    route_matches = ('observation_active' not in board or
                        bool(checked.get('flags', 0) & 16) == bool(board['observation_active']))
                    if board.get('build_id') == 0x0720C129:
                        route_matches = route_matches and bool(checked.get('flags', 0) & 64) == bool(board.get('raw_active'))
                        if 'detail_active' in board:
                            route_matches = route_matches and bool(checked.get('flags', 0) & 32) == bool(board['detail_active'])
                    if checked.get('accepted') and checked.get('config_version') == version and route_matches:
                        self._latency = checked
                        self._latency_version = version
                    else:
                        self._latency = None
                except (KeyError, TypeError, ValueError):
                    self._latency = None
        if not latency_fresh or not board_fresh:
            self._latency = None
        latency = self._latency or {}
        stats_key = ((latency['config_version'], latency['flags'] & 0x70, len(self._latency_validator.samples)) if latency else None)
        if stats_key != self._stats_key:
            sample_group = [sample for sample in self._latency_validator.samples
                            if latency and sample['config_version'] == latency['config_version']
                            and (sample['flags'] & 0x70) == (latency['flags'] & 0x70)]
            self._stats_key, self._stats_count = stats_key, len(sample_group)
            self._stats = {name: {'p50': percentile([sample[name] for sample in sample_group], 50),
                                  'p95': percentile([sample[name] for sample in sample_group], 95)}
                           for name in ('first_pixel_latency_us', 'complete_frame_latency_us')} if len(sample_group) >= 5 else {}
        count = self._stats_count if latency else 0
        distribution = self._stats.get('first_pixel_latency_us') if latency else None
        complete_distribution = self._stats.get('complete_frame_latency_us') if latency else None
        imu = data.get('imu') or {}
        dynamic = mode in ('imu', 'scan')
        rates = self._rates
        rows = [
            ('采集卡帧尺寸', f"{video['width']} × {video['height']}" if video.get('live') and not video.get('stale') and video.get('width') and video.get('height') else '—'),
            ('相机有效输入', _fmt(rates.get('observed_raw_fps'), ' fps')),
            ('FPGA 新处理结果', _fmt(rates.get('observed_new_fps'), ' fps')),
            ('板内首像素延迟', _fmt(latency.get('first_pixel_latency_us', math.nan)/1000, ' ms')),
            ('板内整帧延迟', _fmt(latency.get('complete_frame_latency_us', math.nan)/1000, ' ms')),
            ('本段同版本有效样本', str(count) if latency else '待测'),
            ('首延迟 P50 / P95', f"{distribution['p50']/1000:.2f} / {distribution['p95']/1000:.2f} ms"
             if distribution else '待测（至少5个有效样本）'),
            ('整帧延迟 P50 / P95', f"{complete_distribution['p50']/1000:.2f} / {complete_distribution['p95']/1000:.2f} ms"
             if complete_distribution else '待测（至少5个有效样本）'),
            ('IMU 接收速率', _fmt(imu.get('rate_hz'), ' Hz', 1) if connected and not imu.get('stale', True) else '—'),
            ('近5秒主机更新确认', _fmt(data.get('output_confirmation_observed_hz'), ' 次/秒')
             if dynamic and board_fresh and not waiting and now-self._regime_started_at >= 5
             and (data.get('continuous') or (data.get('visual_scan') or {}).get('enabled')) else '—'),
            ('增强 / Gamma', (f"{enhance.get('enhancement_mode_name', '—')} / 档 {enhance.get('gamma_id', '—')}"
                             if v5_fresh else '—')),
            ('输出证据', f"版本 {latency['config_version']} / 源帧 {latency['source_frame_id']}" if latency else '等待有效同版本样本'),
        ]
        if self._resources:
            resources = self._resources['resources']
            timing = self._resources['timing']
            rows += [('PDS 资源（离线）', f"LUT {resources['LUT']['percent']:.1f}% · DSP {resources['APM']['percent']:.1f}%"),
                     ('PDS 布局（离线）', f"CLM {resources['CLM']['percent']:.1f}% · RAM {resources['DRM']['percent']:.1f}%"),
                     ('PDS 寄存器（离线）', f"FF {resources['FF']['percent']:.1f}%"),
                     ('PDS 慢角 setup（离线）', f"{timing['slow']['setup']['worst_slack_ns']:+.3f} ns"),
                     ('PDS 快角 hold（离线）', f"{timing['fast']['hold']['worst_slack_ns']:+.3f} ns")]
        else:
            rows += [('PDS 资源 / 时序（离线）', '未找到已通过编译的结果报告')]
        for role, label in [('original', '原图'), ('corrected', '矫正后')]:
            line = self._quality[role].get('line')
            edge = self._quality[role].get('edge')
            rows.append((label+'直线度（离线）',
                (f"整体 RMS {line['values']['line_rms_pooled_px']:.3f} px"
                 if 'line_rms_pooled_px' in line['values'] else
                 f"RMS {line['values']['line_rms_max_px']:.3f} px")
                if line else '待测 / 待载入报告'))
            rows.append((label+'近似 MTF50（离线）',
                _fmt(edge['values']['mtf50_cycles_per_pixel'], ' cycles/pixel', 4)
                if edge and edge['values']['mtf50_status'] == 'measured' else
                '未测得50%交点' if edge else '待测 / 待载入报告'))
        if self._quality_bundle is None:
            rows.extend(self._quality_bundle_rows)
        title = '实时指标' if board_fresh and not waiting else '切换中 · 等待测量' if waiting else '实时指标 · 未连接或已过期'
        note = ('FPS 为板端计数差；动态模式可跨姿态版本。延迟仅计 FPGA 输入到输出，不含电脑预览。'
                '更新确认是近5秒主机接收证据频率；Gamma 为异步状态。'
                '面板分位数使用界面观察到的有效样本；导出按完整日志窗口统计。')
        if self._resources:
            note += f" 离线PDS报告：{self._resources['build_id']}。"
            if board.get('build_id') is not None and board['build_id'] != int(self._resources['build_id'], 16):
                note += ' 当前板端固件与此PDS报告不匹配，资源/时序不得归给当前板。'
        if any(self._quality.values()):
            note += ' 画质为载入文件的离线结果，非当前实时帧；原图与矫正后须同场景/尺度才能比较。'
        if self._quality_bundle:
            note += ' 清晰度为近似斜边SFR，非ISO认证；批次间图卡尺度与条件不同，不合并均值，不将差值全部归因于算法。'
        if not video.get('live') or video.get('stale'):
            note += ' 当前画面不是新鲜实时采集，指标不能认定对应当前画面。'
        if self._recording:
            note += ' 正在记录；导出后结束本次记录。'
            if session_id != self._recording['session_id']:
                note += ' 会话已更换，本次只导出原会话。'
        values = {'camera_input_fps': rates.get('observed_raw_fps'),
                  'new_processed_fps': rates.get('observed_new_fps'),
                  'board_rate_interval_s': rates.get('sample_interval_seconds'),
                  'first_pixel_latency_ms': latency.get('first_pixel_latency_us', math.nan)/1000,
                  'complete_frame_latency_ms': latency.get('complete_frame_latency_us', math.nan)/1000,
                  'latency_source_id': latency.get('source_frame_id'), 'latency_config_version': latency.get('config_version'),
                  'latency_same_version_route_samples': count,
                  'first_pixel_latency_p50_ms': distribution['p50']/1000 if distribution else None,
                  'first_pixel_latency_p95_ms': distribution['p95']/1000 if distribution else None,
                  'complete_frame_latency_p50_ms': complete_distribution['p50']/1000 if complete_distribution else None,
                  'complete_frame_latency_p95_ms': complete_distribution['p95']/1000 if complete_distribution else None,
                  'gamma_id_async': enhance.get('gamma_id') if v5_fresh else None,
                  'mode': mode, 'waiting': bool(waiting), 'live_video': bool(video.get('live') and not video.get('stale'))}
        values = {key: None if isinstance(value, float) and not math.isfinite(value) else value for key, value in values.items()}
        if self._recording and session_id == self._recording['session_id'] and int(now) != self._last_point_second:
            self._points.append(dict(monotonic_s=now, **values))
            self._last_point_second = int(now)
        self.last_view = {'title': title, 'rows': rows, 'note': note, 'values': values}
        return self.last_view

    def start_recording(self, data, mode=None):
        data = data or {}
        if not (data.get('connected') or {}).get('fpga') or not data.get('session_id') or not data.get('log_path'):
            raise ValueError('请先连接 FPGA；离线照片不能生成实测记录。')
        if not _fresh((data.get('fpga') or {}).get('status_age_s')):
            raise ValueError('板端状态已过期，请恢复连接后再记录。')
        if self._recording:
            raise ValueError('已有记录正在进行，请先导出，避免覆盖。')
        self._recording = {'session_id': data['session_id'], 'log_path': data['log_path'],
                           'start_monotonic_s': time.monotonic(),
                           'started_local': datetime.now().astimezone().isoformat(), 'initial_mode': mode,
                           'start_framing_counters': deepcopy(data.get('framing_counters'))}
        self._points = []
        self._segments = []
        self._last_point_second = None
        self._clear_live()
        self._regime_started_at = self._recording['start_monotonic_s']
        return '已开始本次指标记录；按模式分别展示，完成后点击导出。'

    def export(self, directory, data=None):
        if not self._recording:
            raise ValueError('请先开始记录；不能把历史或离线数据当作本次实测。')
        end = time.monotonic()
        recording = deepcopy(self._recording)
        start_framing = recording.get('start_framing_counters')
        end_framing = (data or {}).get('framing_counters') if (data or {}).get('session_id') == recording['session_id'] else None
        framing_known = isinstance(start_framing, dict) and isinstance(end_framing, dict)
        framing_deltas = ({key: end_framing.get(key, 0)-start_framing.get(key, 0)
                           for key in set(start_framing) | set(end_framing)} if framing_known else None)
        framing_bad = (not framing_known or any(value < 0 or (key != 'crc_valid_packets' and value > 0)
                                                for key, value in (framing_deltas or {}).items()))
        framing = {'start': start_framing, 'end': end_framing, 'deltas': framing_deltas,
                   'known': framing_known, 'passed': not framing_bad,
                   'scope': 'Host parser counter changes across button-recording window; raw JSONL contains CRC-valid packets only.'}
        records, malformed = [], []
        path = Path(recording['log_path'])
        with path.open(encoding='utf-8-sig') as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    item = json.loads(line)
                    if (item.get('session_id') == recording['session_id'] and
                        recording['start_monotonic_s'] <= item.get('monotonic_s', -1) <= end):
                        records.append(item)
                except (ValueError, TypeError, AttributeError):
                    malformed.append(line_no)
        destination = Path(directory) / ('demo_metrics_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        destination.mkdir(parents=True, exist_ok=False)
        (destination/'session_window.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False, allow_nan=False)+'\n' for r in records), encoding='utf-8')
        segments = self._segments or [{'start_monotonic_s': recording['start_monotonic_s'], 'mode': recording['initial_mode'], 'waiting': False}]
        segments[0]['start_monotonic_s'] = recording['start_monotonic_s']
        summaries, samples = [], []
        for index, segment in enumerate(segments):
            finish = segments[index+1]['start_monotonic_s'] if index+1 < len(segments) else end
            selected = [r for r in records if segment['start_monotonic_s'] <= r['monotonic_s'] < finish]
            analysis = LatencyAnalysis()
            parse_errors = 0
            for record in selected:
                event = record.get('event')
                packet = record.get('packet') if event == 'latency' else record.get('status') if event in ('status', 'status_health') else None
                if not packet:
                    continue
                try:
                    packet = dict(packet)
                    if event == 'latency':
                        packet['packet_type'] = 'latency'
                        if segment.get('waiting'):
                            analysis.rejections['host_mode_switching'] += 1
                            samples.append(dict(segment=index, host_observed_mode=segment.get('mode'),
                                **packet, accepted=False, rejection_reason='host_mode_switching'))
                            continue
                    elif packet.get('protocol_version') == 5:
                        packet['packet_type'] = 'status'
                    else:
                        continue
                    checked = analysis.accept(packet)
                    if checked.get('packet_type') == 'latency':
                        samples.append(dict(segment=index, host_observed_mode=segment.get('mode'), **checked))
                except (ValueError, KeyError, TypeError):
                    parse_errors += 1
            summary = analysis.summary()
            routes = {}
            for sample in analysis.samples:
                route = f"version_{sample['config_version']}_LAT_flags_{sample['flags'] & 0x70:02x}"
                routes.setdefault(route, []).append(sample)
            summary['distributions_us_by_config_and_actual_LAT_route'] = {
                key: analysis.distributions(group) for key, group in routes.items()}
            if len({sample['flags'] & 0x70 for sample in analysis.samples}) > 1:
                # An unchanged config number does not prove an unchanged raw,
                # observation or detail path. Never leave a pooled fallback.
                summary['distributions_us'] = None
                summary['distributions_us_by_config'] = {}
                summary['measurement_trust']['passed'] = False
                summary['measurement_trust']['reasons'].append('mixed_actual_LAT_route_window')
            summary.update(segment=index, host_observed_mode=segment.get('mode'), waiting=segment.get('waiting'),
                           start_monotonic_s=segment['start_monotonic_s'], end_monotonic_s=finish,
                           malformed_packets=parse_errors)
            if segment.get('waiting') or parse_errors or malformed:
                summary['measurement_trust']['passed'] = False
                summary['measurement_trust']['reasons'].append('mode_switching_or_packet_parse_error_or_malformed_log')
            if framing_bad:
                summary['measurement_trust']['passed'] = False
                summary['measurement_trust']['reasons'].append('host_framing_errors_or_counter_window_unavailable')
            summaries.append(summary)
        from analyze_realtime_log import analyze
        begun = {r.get('request_id', (r.get('request') or {}).get('request_id')) for r in records
                 if r.get('event') == 'write_started'}
        bounded = [r for r in records if r.get('request_id', (r.get('request') or {}).get('request_id')) in begun
                   or r.get('request_id', (r.get('request') or {}).get('request_id')) is None]
        dynamic = analyze(bounded)
        dynamic['window_filter'] = 'Transaction metrics include only requests with write_started recorded inside this window.'
        report = {'schema': 'demo_metrics_v1', 'recording': dict(recording, end_monotonic_s=end),
                  'boundary': BOUNDARY, 'segments': summaries, 'dynamic_log_analysis': dynamic,
                  'offline_pds': self._resources, 'malformed_log_lines': malformed,
                  'host_framing_window': framing,
                  'offline_quality_reports': self._quality,
                  'imported_quality_bundle': self._quality_bundle,
                  'notes': ['No new hardware access. Source is the existing single-owner session log.',
                            'Throughput may span pose configuration versions within a mode; latency distributions are separated by config version.',
                            'Mode is host-observed context, not an exact same-frame assertion. Actual LAT route bits and config version are separate grouping keys.',
                            'Unknown/absent metrics remain null. This report is not automatic contest acceptance.',
                            'Gamma status is asynchronous, not exact per-frame Gamma attribution.',
                            'Geometric error, MTF and physical viewing latency are not measured by this telemetry.']}
        if self._quality_bytes:
            (destination/'quality_reports').mkdir()
            for (role, group), raw in self._quality_bytes.items():
                (destination/'quality_reports'/f'{role}_{group}.json').write_bytes(raw)
        for relative_path, raw in self._quality_bundle_files.items():
            evidence_path = destination / 'quality_reports/imported_20261103' / relative_path
            evidence_path.parent.mkdir(parents=True, exist_ok=True)
            evidence_path.write_bytes(raw)
        (destination/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        for filename, rows in [('display_metrics.csv', self._points), ('latency_samples.csv', samples)]:
            columns = list(dict.fromkeys(k for row in rows for k in row)) or ['no_samples']
            with (destination/filename).open('w', encoding='utf-8-sig', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                writer.writerows(rows)
        self._recording = None
        return destination
