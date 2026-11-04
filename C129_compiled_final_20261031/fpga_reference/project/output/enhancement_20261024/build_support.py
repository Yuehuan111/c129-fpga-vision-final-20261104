"""Source guards for adaptive enhancement and one exact approved equivalent gain implementation."""
from pathlib import Path
import ast, hashlib, json, re, xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
BUILD = ROOT / 'board/build_cmos1'
BASE = OUT / 'baseline'
ARTIFACTS = ['compile/camera_board50_top_comp.adf', 'synthesize/camera_board50_top_syn.adf',
             'synthesize/camera_board50_top.snr',
             'device_map/camera_board50_top_map.adf', 'place_route/camera_board50_top_pnr.adf',
             'place_route/camera_board50_top.prr', 'report_timing/camera_board50_top.rtr',
             'report_timing/rtr.db', 'generate_bitstream/camera_board50_top.sbit']
SYNTHESIS_ARTIFACTS = ARTIFACTS[:4]
SOURCE_GUARD_KEYS = ('source_hashes', 'rtl_hashes', 'board_hashes', 'host_hashes', 'fdc_sha256', 'math_sha256')
APPROVED_GAIN_SPLIT_SHA256 = '850e4ccf19363112d3f719edca5f5422cd256082c90201886c31725b2e3691a8'
GAIN_SPLIT_RTL = 'rtl/bilinear_rgb565_pipeline.v'

def verify_reuse_origin(state):
    """Verify a self-contained full-compile origin, without recursive provenance."""
    from datetime import datetime
    reused = state.get('reused_artifacts', {})
    if not reused:
        assert state['build_kind'] == 'full_compile', 'Missing synthesis provenance'
        return None
    assert state['build_kind'] == 'reused_synthesis_pnr'
    assert set(reused) == set(SYNTHESIS_ARTIFACTS), 'Unexpected reused artifacts'
    record = state['reuse_provenance']
    path = (ROOT / record['path']).resolve()
    assert path.is_relative_to(OUT.resolve()), 'Provenance path leaves enhancement output'
    assert sha(path) == record['sha256'], 'Synthesis provenance changed'
    origin = json.loads(read(path))
    assert origin['build_kind'] == 'full_compile' and not origin.get('reused_artifacts'), 'Origin must be a direct full compile'
    assert origin['status'] == 'COMPLETED_PENDING_AUDIT' and origin['returncode'] == 0 and origin['sources_unchanged']
    assert set(origin['artifacts']) == set(ARTIFACTS), 'Incomplete original full build'
    assert all(origin[k] == state[k] for k in SOURCE_GUARD_KEYS), 'Origin source identity differs'
    for rel, artifact in reused.items():
        assert artifact == origin['artifacts'][rel] == state['artifacts'][rel], 'Reused record differs: ' + rel
        assert sha(BUILD / rel) == artifact['sha256'], 'Reused file differs: ' + rel
        assert datetime.fromisoformat(artifact['mtime']) >= datetime.fromisoformat(origin['started_at']), 'Origin artifact was not fresh: ' + rel
    return origin

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def read(path):
    return path.read_text(encoding='utf-8-sig', errors='replace')

def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

def tree_hashes(folder, relative_to):
    return {p.relative_to(relative_to).as_posix(): sha(p)
            for p in sorted(folder.rglob('*')) if p.is_file()}

def runtime_host_hashes():
    """Freeze runtime host entry points and their local imports, not offline generators/tests."""
    names = ['project_profile', 'complete_control', 'complete_control_panel', 'read_latency',
             'host_actions', 'complete_geometry', 'complete_geometry_reference', 'live_720_check']
    seen = {}
    while names:
        name = names.pop()
        path = ROOT / 'tools' / (name + '.py')
        if name in seen or not path.is_file():
            continue
        seen[name] = path
        for node in ast.walk(ast.parse(read(path), filename=str(path))):
            if isinstance(node, ast.ImportFrom) and node.module:
                names.append(node.module.split('.')[0])
            elif isinstance(node, ast.Import):
                names.extend(alias.name.split('.')[0] for alias in node.names)
    return {path.relative_to(ROOT).as_posix(): sha(path) for name, path in sorted(seen.items())}

def source_guard(require_identity=True):
    baseline = json.loads(read(OUT / 'baseline_hashes.json'))
    for rel, digest in baseline.items():
        assert sha(BASE / rel) == digest, 'Baseline modified: ' + rel
    snapshot = tree_hashes(BUILD / 'source', BUILD)
    baseline_source = {k.removeprefix('board/build_cmos1/'): v for k, v in baseline.items()
                       if k.startswith('board/build_cmos1/source/')}
    assert snapshot.keys() == baseline_source.keys(), 'Prepared source file list changed'
    allowed = {'source/rtl/complete_image_enhance.v', 'source/rtl/camera_board50_top.v'}
    if snapshot['source/' + GAIN_SPLIT_RTL] == APPROVED_GAIN_SPLIT_SHA256:
        allowed.add('source/' + GAIN_SPLIT_RTL)
    changes = [rel for rel, digest in snapshot.items() if baseline_source[rel] != digest]
    assert set(changes) <= allowed, 'Out-of-scope prepared changes: ' + repr(changes)
    root_rtl = tree_hashes(ROOT / 'rtl', ROOT)
    baseline_rtl = {k: v for k, v in baseline.items() if k.startswith('rtl/')}
    assert root_rtl.keys() == baseline_rtl.keys(), 'RTL file list changed'
    assert all(digest == baseline_rtl[rel] or rel == 'rtl/complete_image_enhance.v'
               or (rel == GAIN_SPLIT_RTL and digest == APPROVED_GAIN_SPLIT_SHA256)
               for rel, digest in root_rtl.items()), 'Out-of-scope root RTL modification'
    for rel, digest in root_rtl.items():
        prepared = BUILD / 'source' / rel
        if prepared.exists():
            assert sha(prepared) == digest, 'Root/prepared mismatch: ' + rel
    top = ROOT / 'board/camera_board50_top.v'
    assert sha(top) == sha(BUILD / 'source/rtl/camera_board50_top.v'), 'Top mirrors differ'
    top_before = (BASE / 'board/camera_board50_top.v').read_bytes()
    strip_id = lambda b: re.sub(rb"\.BUILD_ID\(32'h[0-9a-fA-F]{8}\)", b'.BUILD_ID(IDENTITY)', b)
    expected_top = top_before.replace(b"enhance_status_core<=8'b00100000", b"enhance_status_core<=8'b00100011")
    assert top_before.count(b"enhance_status_core<=8'b00100000") == 1, 'Unexpected baseline status reset'
    assert strip_id(top.read_bytes()) in (strip_id(top_before), strip_id(expected_top)), 'Top changed beyond BUILD_ID/exact enhancement-status reset'
    if require_identity:
        assert len(re.findall(rb"\.BUILD_ID\(32'h0720c124\)", top.read_bytes(), re.I)) == 1, 'Expected C124 identity'
        assert strip_id(top.read_bytes()) == strip_id(expected_top), 'Expected unity-gamma boot status 0x23'
        assert re.search(r'BUILD_ID\s*=\s*0x0720C124\b', read(ROOT / 'tools/project_profile.py'), re.I), 'Host identity mismatch'
    fdc = BUILD / 'camera_board50_top.fdc'
    assert sha(fdc) == baseline['board/build_cmos1/camera_board50_top.fdc'], 'Constraints changed'
    return {'source_hashes': snapshot, 'rtl_hashes': root_rtl,
            'board_hashes': {'board/camera_board50_top.v': sha(top)},
            'host_hashes': runtime_host_hashes(),
            'changed_prepared_files': changes, 'fdc_sha256': sha(fdc),
            'math_sha256': sha(ROOT / 'rtl/fisheye_math.v')}

def resources(path):
    result, parent = {}, None
    for line in read(path).splitlines():
        cells = [x.strip() for x in line.split('|')[1:]]
        if len(cells) < 3:
            continue
        name = cells[0]
        if name.startswith('Use of '):
            parent = name[7:]
            try:
                result[parent] = {'used': float(cells[1]), 'available': float(cells[2])}
            except ValueError:
                continue
        elif name in ('FF', 'LUT') and parent in ('CLMA', 'CLMS'):
            result[parent + '_' + name] = {'used': float(cells[1]), 'available': float(cells[2])}
        if name in ('+ enhancement', '+ original_gamma'):
            result[name[2:]] = {'LUT': int(cells[1]), 'FF': int(cells[2]),
                               'APM': float(cells[4]), 'DRM': float(cells[5])}
    for name in ('FF', 'LUT'):
        result[name] = {k: sum(result[x + '_' + name][k] for x in ('CLMA', 'CLMS'))
                        for k in ('used', 'available')}
    result['CLM'] = {k: sum(result[x][k] for x in ('CLMA', 'CLMS')) for k in ('used', 'available')}
    for values in result.values():
        if 'available' in values:
            values['percent'] = 100 * values['used'] / values['available'] if values['available'] else 0
    return result

def enhancement_resources(path):
    for line in read(path).splitlines():
        cells = [x.strip() for x in line.split('|')[1:]]
        if len(cells) >= 6 and cells[0] == '+ enhancement':
            return {'LUT': int(cells[1]), 'FF': int(cells[2]), 'APM': float(cells[4]), 'DRM': float(cells[5])}
    raise AssertionError('Missing enhancement hierarchy: ' + str(path))

def timing(path):
    xml = ET.parse(path).getroot()
    result = {'corners': {}, 'cdc': {}, 'coverage': {}}
    table = xml.find("table[@id='check_timing_summary']")
    assert table is not None, 'Missing check_timing coverage'
    for row in table.findall('row'):
        cells = [d.text for d in row.findall('data')]
        result['coverage'][cells[0]] = int(cells[1])
    for corner in ('slow', 'fast'):
        checks = {}
        for kind in ('setup', 'hold', 'recovery', 'removal', 'mpw'):
            table = xml.find(f"table[@id='report_timing_{corner}_{kind}']")
            assert table is not None, f'Missing {corner}/{kind}'
            headers = table.find('column_headers').findall('data')
            rows = []
            for row in table.findall('row'):
                cells = [d.text or '' for d in row.findall('data')]
                if len(cells) != len(headers):
                    continue
                ix = 1 if kind == 'mpw' else 2
                rows.append({'clocks': cells[:ix], 'slack_ns': float(cells[ix]),
                             'total_negative_slack_ns': float(cells[ix + 1]),
                             'failing_endpoints': int(cells[ix + 2]), 'total_endpoints': int(cells[ix + 3])})
            checks[kind] = {'worst_slack_ns': min((r['slack_ns'] for r in rows), default=None),
                            'failing_endpoints': sum(r['failing_endpoints'] for r in rows), 'rows': rows}
        result['corners'][corner] = checks
        table = xml.find(f"table[@id='report_timing_{corner}_path_max_skew']")
        assert table is not None, f'Missing {corner} skew assertions'
        assertions = []
        for row in table.findall('row'):
            cells = [d.text or '' for d in row.findall('data')]
            assertions.append({'assertion': cells[0], 'slack_ns': float(cells[1]),
                               'limit_ns': float(cells[10]), 'actual_skew_ns': float(cells[11])})
        result['cdc'][corner] = {'groups': len(assertions), 'assertions': assertions,
                                 'worst_slack_ns': min((r['slack_ns'] for r in assertions), default=None)}
    return result
