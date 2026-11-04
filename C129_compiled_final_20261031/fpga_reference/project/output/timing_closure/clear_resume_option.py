"""Remove only PDS cmd_pnr.input_place_db_file, preserving other bytes.

CLI writes a NEW COPY only; it never overwrites its input.  After PDS has exited,
the build/delivery driver may import remove_resume_option() and explicitly apply
the returned bytes, retaining the original project as audit evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


TOKEN = re.compile(rb'"(?:\\.|[^"\\])*"|[()]|[^()\s"]+', re.DOTALL)


def _nodes(data: bytes) -> list[dict]:
    """Read balanced PDS S-expressions without rewriting or decoding strings."""
    nodes, stack = [], []
    cursor = 3 if data.startswith(b'\xef\xbb\xbf') else 0
    roots = 0
    for match in TOKEN.finditer(data, cursor):
        if data[cursor:match.start()].strip():
            raise ValueError('Invalid or unterminated PDS string')
        value = match.group()
        if value == b'(':
            parent = stack[-1] if stack else None
            roots += parent is None
            node = {'start': match.start(), 'parent': parent, 'atoms': []}
            nodes.append(node)
            stack.append(node)
        elif value == b')':
            if not stack:
                raise ValueError('Unmatched PDS closing parenthesis')
            stack.pop()['end'] = match.end()
        elif stack:
            stack[-1]['atoms'].append(value)
        else:
            raise ValueError('PDS atom outside its root expression')
        cursor = match.end()
    if stack or data[cursor:].strip() or roots != 1:
        raise ValueError('Incomplete or multiple PDS root expressions')
    if not nodes or nodes[0]['atoms'][:1] != [b'_flow']:
        raise ValueError('Expected a PDS _flow root')
    return nodes


def remove_resume_option(data: bytes) -> tuple[bytes, dict]:
    """Return cleaned bytes and evidence; do not read or write any files."""
    matches = [n for n in _nodes(data)
               if n['atoms'][:2] == [b'_option', b'input_place_db_file']]
    if len(matches) > 1:
        raise ValueError('Multiple placement resume options; manual review required')
    digest = lambda b: hashlib.sha256(b).hexdigest()
    evidence = {'removed_count': len(matches), 'before_sha256': digest(data)}
    if not matches:
        evidence['after_sha256'] = evidence['before_sha256']
        return data, evidence
    node = matches[0]
    parent = node['parent']
    task = parent['parent'] if parent else None
    if (not parent or parent['atoms'][:2] != [b'_command', b'cmd_pnr']
            or not task or task['atoms'][:2] != [b'_task', b'tsk_pnr']):
        raise ValueError('Placement resume option is outside tsk_pnr/cmd_pnr')
    start, end = node['start'], node['end']
    line_start = data.rfind(b'\n', 0, start) + 1
    next_newline = data.find(b'\n', end)
    line_end = next_newline + 1 if next_newline >= 0 else len(data)
    if not data[line_start:start].strip() and not data[end:line_end].strip():
        start, end = line_start, line_end
    cleaned = data[:start] + data[end:]
    remaining = _nodes(cleaned)
    if any(n['atoms'][:2] == [b'_option', b'input_place_db_file'] for n in remaining):
        raise ValueError('Placement resume option remains after removal')
    evidence.update(
        after_sha256=digest(cleaned), removed_byte_span=[start, end],
        removed_lines=[data.count(b'\n', 0, start) + 1,
                       data.count(b'\n', 0, node['end']) + 1],
        removed_bytes=end-start, removed_block=data[start:end].decode('utf-8'),
        other_bytes_unchanged=True, balanced_s_expressions=True,
    )
    return cleaned, evidence


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path, help='New copy; cannot already exist')
    parser.add_argument('--metadata', type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error('Input must remain unchanged; choose a different output path')
    if args.metadata and (args.metadata.exists()
                          or args.metadata.resolve() in (args.source.resolve(), args.output.resolve())):
        parser.error('Metadata must be a different new file')
    before = args.source.read_bytes()
    cleaned, evidence = remove_resume_option(before)
    with args.output.open('xb') as stream:
        stream.write(cleaned)
    if args.source.read_bytes() != before:
        raise RuntimeError('Input changed concurrently; discard this copy and repeat after PDS exits')
    if args.metadata:
        with args.metadata.open('x', encoding='utf-8', newline='\n') as stream:
            json.dump(evidence, stream, indent=2)
            stream.write('\n')
    print(json.dumps({k: v for k, v in evidence.items() if k != 'removed_block'}, indent=2))


if __name__ == '__main__':
    main()
