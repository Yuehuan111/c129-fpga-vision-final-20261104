"""PDS 2022.2 native CDC constraints, isolated by bus and CDC direction.

PDS User Guide 6.4.1 (printed p198) explicitly keeps set_max_skew active under
set_clock_groups / set_false_path. ADS supports syn_preserve and syn_srlstyle;
it does not honor the Xilinx-style ASYNC_REG attribute.
"""
import re

# These three statistics buses have consumers in camera_board50_top. The
# unconnected camera_skipped_frames output is trimmed; do not constrain an
# absent skipped_stats endpoint or group independent Gray counters together.
BASELINE_GRAY_BUSES = (
    ('raw_frames', 'raw_frame_stats', 'source_gray', 'gray_sync1', 32),
    ('display_underflow', 'system_inst/display_stats', 'source_gray', 'gray_sync1', 32),
    ('capture_overflow', 'system_inst/overflow_stats', 'source_gray', 'gray_sync1', 32),
    ('capture_fifo_write', 'system_inst/capture/fifo', 'wg', 'wg1', 12),
    ('capture_fifo_read', 'system_inst/capture/fifo', 'rg', 'rg1', 12),
)

GRAY_BUSES = BASELINE_GRAY_BUSES + (
    ('latency_capture_overrun', 'system_inst/capture/metadata_overrun_counter', 'source_gray', 'gray_sync1', 32),
    ('latency_missing_tags', 'system_inst/display/diagnostics/missing_counter', 'source_gray', 'gray_sync1', 32),
    ('latency_capture_overrun_pix', 'system_inst/display/diagnostics/cam_overrun_counter', 'source_gray', 'gray_sync1', 32),
    ('latency_select_overrun', 'system_inst/display/diagnostics/select_overrun_counter', 'source_gray', 'gray_sync1', 32),
)

# Cross-instance time buses share a registered Gray source, but are independent
# directions: never merge camera and display receiver paths into one group.
CROSS_GRAY_BUSES = (
    ('latency_time_camera', 'system_inst/diagnostic_timebase', 'time_gray',
     'system_inst/capture/camera_clock_time', 'gray_sync1', 32),
    ('latency_time_display', 'system_inst/diagnostic_timebase', 'time_gray',
     'system_inst/display/diagnostics/pixel_clock_time', 'gray_sync1', 32),
)

BUNDLED_BUSES = (
    ('latency_capture_metadata', 'system_inst/capture/capture_metadata_mailbox', 65),
    ('latency_selected_metadata', 'system_inst/display/diagnostics/selected_mailbox', 288),
    ('latency_result_metadata', 'system_inst/display/diagnostics/result_mailbox', 512),
    ('advanced_mode_commands', 'advanced_mode_mailbox', 3),
    ('advanced_applied_status', 'advanced_status_mailbox', 8),
)

# Each pair is (source instance, source register, destination instance,
# destination register, width). Request and payload belong to ONE bundled-data
# assertion: payload-only skew does not bound its arrival relative to request.
BASELINE_CDC_GROUPS = tuple(
    (name, 'gray', ((instance, source, instance, destination, width),))
    for name, instance, source, destination, width in BASELINE_GRAY_BUSES
)
CDC_GROUPS = tuple(
    (name, 'gray', ((instance, source, instance, destination, width),))
    for name, instance, source, destination, width in GRAY_BUSES
) + tuple(
    (name, 'gray', ((source_instance, source, destination_instance, destination, width),))
    for name, source_instance, source, destination_instance, destination, width in CROSS_GRAY_BUSES
) + tuple(
    (name, 'bundled', ((instance, 'held_data', instance, 'destination_data', width),
                       (instance, 'request_toggle', instance, 'request_sync1', None)))
    for name, instance, width in BUNDLED_BUSES
)


def cell_pattern(instance, register):
    # ADS constraint import sees a vector register; later stages see individual
    # bits. Anchor the whole name, including brackets, to exclude wg_next/wg2.
    return '^' + re.escape(instance + '/' + register) + r'\[[0-9]+(:[0-9]+)?\]$'


def group_cell_pattern(pairs, source):
    patterns = []
    for source_instance, source_register, destination_instance, destination_register, width in pairs:
        instance, register = ((source_instance, source_register) if source
                              else (destination_instance, destination_register))
        pattern = re.escape(instance + '/' + register)
        if width is not None:
            pattern += r'\[[0-9]+(:[0-9]+)?\]'
        patterns.append(pattern)
    return '^' + (patterns[0] if len(patterns) == 1 else '(?:' + '|'.join(patterns) + ')') + '$'


def gray_cdc_fdc():
    lines = [
        '# PDS native CDC bus-skew assertions (User Guide 6.4.1, p198).',
        '# Each independent bus/direction has its own assertion. Only source',
        '# Gray register -> FIRST destination synchronizer stage is selected.',
        '# Bundled mailboxes hold payload until acknowledgement; each group',
        '# includes request -> sync1 AND held payload -> destination capture.',
        '# 5 ns is half the shortest 10 ns core clock; camera is >=11.90 ns',
        '# and pixel is >=13.46 ns. This leaves margin without a 2 ns overconstraint.',
        '# set_max_skew remains effective under the asynchronous clock groups.',
        '# syn_preserve prevents Gray MSBs merging into binary pointer registers.',
        '# Max Skew Report must show these paths after routing before sign-off.',
    ]
    for name, kind, pairs in CDC_GROUPS:
        src = group_cell_pattern(pairs, True)
        dst = group_cell_pattern(pairs, False)
        width = sum(pair[4] or 1 for pair in pairs)
        lines += [
            f'# CDC_{kind.upper()} {name}: {width} bits, max skew 5.000 ns',
            f'set_max_skew -from [get_cells -hierarchical -hsc / -regexp {{{src}}}] '
            f'-to [get_cells -hierarchical -hsc / -regexp {{{dst}}}] 5.000',
        ]
    return '\n'.join(lines) + '\n'


def check_constraint_import(log):
    """Fail if an expected assertion was not accepted by the actual PDS import.

    Command success confirms nonempty endpoint matching during import. It does
    not replace checking routed Max Skew Report or count all matched bit paths.
    """
    expected = [line for line in gray_cdc_fdc().splitlines() if line.startswith('set_max_skew')]
    missing = [line for line in expected if not re.search(
        re.escape(line).replace(r'5\.000', r'5(?:\.0+)?') + r' successfully\.', log)]
    errors = re.findall(r'(?m)^[EC]:.*(?:SDC-|CommandTiming-|ConstraintEditor-0046).*$', log)
    if missing or errors:
        raise ValueError(f'CDC constraint import incomplete: {len(missing)} missing assertions; '
                         + '\n'.join(errors))
    return len(expected)


if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description='Verify PDS imported every Gray/bundled CDC assertion; not physical sign-off.')
    parser.add_argument('--synthesis-log', required=True, type=Path)
    args = parser.parse_args()
    count = check_constraint_import(args.synthesis_log.read_text(encoding='utf-8-sig', errors='replace'))
    print(f'CDC IMPORT PASS: {count} assertions accepted. Routed Max Skew Report and board timing still need review.')
