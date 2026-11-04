"""Hash PDS design/flow definitions while excluding observed run metadata."""
import hashlib
import json
import re

# PDS rewrites these after a run. Do NOT ignore options, attributes, inputs,
# source/IP files, target device, constraints, or command configuration.
RUNTIME_NODES = {'_comment', '_status', '_gci_state', '_output', '_db_output', '_timespec'}


def pds_semantic_sha256(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text.lstrip('\ufeff'))
    index = 0

    def parse():
        nonlocal index
        if index >= len(tokens) or tokens[index] != '(':
            raise ValueError('Invalid PDS expression')
        index += 1
        result = []
        while index < len(tokens) and tokens[index] != ')':
            if tokens[index] == '(':
                result.append(parse())
            else:
                result.append(tokens[index]); index += 1
        if index >= len(tokens):
            raise ValueError('Truncated PDS expression')
        index += 1
        return result

    tree = parse()
    if index != len(tokens):
        raise ValueError('Trailing PDS content')

    def strip_runtime(node):
        if not isinstance(node, list):
            return node
        if node and node[0] in RUNTIME_NODES:
            return None
        return [value for item in node if (value := strip_runtime(item)) is not None]

    canonical = json.dumps(strip_runtime(tree), ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(canonical.encode('utf-8')).hexdigest()
