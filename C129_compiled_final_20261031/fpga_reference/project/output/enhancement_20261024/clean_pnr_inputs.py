"""Remove only temporary database-input options from tsk_pnr/cmd_pnr, byte-exact elsewhere."""
from pathlib import Path
import hashlib
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'timing_closure'))
from clear_resume_option import _nodes

def remove_pnr_input_options(data):
    nodes = _nodes(data)
    commands = [n for n in nodes if n['atoms'][:2] == [b'_command', b'cmd_pnr']]
    assert len(commands) == 1, 'Expected exactly one cmd_pnr'
    command = commands[0]
    assert command['parent'] and command['parent']['atoms'][:2] == [b'_task', b'tsk_pnr'], 'cmd_pnr is outside tsk_pnr'
    names = (b'input_db_file', b'input_place_db_file')
    removed = []
    for name in names:
        matches = [n for n in nodes if n['parent'] is command and n['atoms'][:2] == [b'_option', name]]
        assert len(matches) <= 1, 'Duplicate PNR database-input option: ' + name.decode()
        if not matches:
            continue
        node = matches[0]
        start, end = node['start'], node['end']
        line_start = data.rfind(b'\n', 0, start) + 1
        next_newline = data.find(b'\n', end)
        line_end = next_newline + 1 if next_newline >= 0 else len(data)
        if not data[line_start:start].strip() and not data[end:line_end].strip():
            start, end = line_start, line_end
        removed.append({'option': name.decode(), 'removed_byte_span': [start, end],
                        'removed_lines': [data.count(b'\n', 0, start) + 1,
                                          data.count(b'\n', 0, node['end']) + 1],
                        'removed_block': data[start:end].decode('utf-8'), 'removed_bytes': end - start})
    cleaned = data
    for record in sorted(removed, key=lambda r: r['removed_byte_span'][0], reverse=True):
        start, end = record['removed_byte_span']
        cleaned = cleaned[:start] + cleaned[end:]
    after = _nodes(cleaned)
    for node in after:
        if node['atoms'][:1] == [b'_option'] and len(node['atoms']) >= 2 and node['atoms'][1] in names:
            assert not (node['parent'] and node['parent']['atoms'][:2] == [b'_command', b'cmd_pnr']), 'Temporary PNR input option remains'
    digest = lambda value: hashlib.sha256(value).hexdigest()
    return cleaned, {'before_sha256': digest(data), 'after_sha256': digest(cleaned),
                     'removed_count': len(removed), 'removed_options': removed,
                     'other_bytes_unchanged': True, 'balanced_s_expressions': True,
                     'scope': 'Only cmd_pnr input_db_file and input_place_db_file; all source/constraint/other option bytes preserved.'}
