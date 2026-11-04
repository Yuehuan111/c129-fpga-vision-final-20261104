"""Read CRC-checked FPGA status. Never infer processing FPS from USB video FPS."""
import argparse
import json
import struct
import time
from camera_config import crc16

FIELDS = ('build_id', 'version_flags', 'captured', 'processed', 'underflow',
          'overflow', 'dropped', 'commits', 'rejected', 'map_cycles')
FIELDS_V2 = FIELDS + ('raw_valid_frames', 'sample_ticks')
DIAGNOSTIC_FIELDS = ('pclk_edges', 'vsync_rises', 'vsync_falls', 'href_rises',
                     'href_cycles', 'last_line_pixels', 'min_line_pixels', 'max_line_pixels',
                     'vsync_rise_lines', 'vsync_fall_lines', 'vsync_rise_pixels',
                     'vsync_fall_pixels', 'raw_drop_count', 'diagnostic_flags', 'diagnostic_age_cycles')
FIELDS_V3 = FIELDS_V2 + DIAGNOSTIC_FIELDS
SCCB_FIELDS = ('sccb_flags', 'sccb_accepted_seq', 'sccb_completed_seq', 'sccb_value_register',
               'sccb_rejected_count', 'sccb_crc_error_count', 'sccb_framing_error_count',
               'sccb_packet_error_count')
FIELDS_V4 = FIELDS_V3 + SCCB_FIELDS
THROUGHPUT_FIELDS = ('last_frame_cycles', 'engine_busy_cycles', 'camera_skipped_frames',
                     'repeated_frames', 'idle_no_source_cycles', 'idle_no_output_cycles',
                     'idle_map_wait_cycles', 'engine_cmd_wait_cycles', 'engine_memory_cycles',
                     'memory_active_cycles', 'ddr_cmd_wait_cycles', 'ddr_read_first_wait_cycles',
                     'ddr_read_gap_cycles', 'ddr_write_wait_cycles', 'ddr_read_bursts', 'ddr_data_beats')
FIELDS_V5 = FIELDS_V4 + THROUGHPUT_FIELDS
FORMATS = {(1, 40): FIELDS, (2, 48): FIELDS_V2, (3, 108): FIELDS_V3,
           (4, 140): FIELDS_V4, (5, 204): FIELDS_V5}
CORE_CLK_HZ = 100000000
# The current RTL refreshes its mailbox about every 10 ms. Allow five periods,
# including handshake latency; stale counters cannot establish an input rate.
DIAGNOSTIC_MAX_AGE_CYCLES = CORE_CLK_HZ // 20
DIAGNOSTIC_RATES = (('pclk_edges', 'observed_pclk_hz'),
                    ('vsync_rises', 'observed_vsync_rise_hz'),
                    ('vsync_falls', 'observed_vsync_fall_hz'),
                    ('href_rises', 'observed_href_lines_per_second'),
                    ('href_cycles', 'observed_href_pixels_per_second'))
THROUGHPUT_FRACTIONS = {key: key.removesuffix('_cycles') + '_fraction'
                       for key in THROUGHPUT_FIELDS[1:14] if key.endswith('_cycles')}
THROUGHPUT_DERIVED = tuple(THROUGHPUT_FRACTIONS.values()) + (
    'throughput_rate_status', 'last_frame_seconds', 'last_frame_service_fps',
    'observed_camera_skipped_fps', 'observed_repeated_fps',
    'ddr_read_first_wait_cycles_per_burst', 'ddr_data_bytes_per_second')

def extract_packets(buffer):
    packets = []
    while True:
        position = buffer.find(b'\x55\xaa')
        if position < 0:
            del buffer[:-3]
            break
        del buffer[:position]
        if len(buffer) < 4:
            break
        version, payload_length = buffer[2:4]
        fields = FORMATS.get((version, payload_length))
        if fields is None:
            del buffer[0]
            continue
        protected_length = 4 + payload_length
        frame_length = protected_length + 2
        if len(buffer) < frame_length:
            break
        frame = bytes(buffer[:frame_length])
        if crc16(frame[:protected_length]) != int.from_bytes(frame[protected_length:], 'big'):
            del buffer[0]
            continue
        data = dict(zip(fields, struct.unpack('<' + str(len(fields)) + 'I', frame[4:protected_length])))
        data['protocol_version'] = version
        data['config_version'] = data['version_flags'] & 65535
        data['flags'] = data.pop('version_flags') >> 16
        data['processing'] = bool(data['flags'] & 1)
        data['map_building'] = bool(data['flags'] & 2)
        data['error_sticky'] = bool(data['flags'] & 4)
        data['camera_error'] = bool(data['flags'] & 8)
        data['camera_config_done'] = bool(data['flags'] & 16)
        data['hdmi_config_done'] = bool(data['flags'] & 32)
        data['ddr_initialized'] = bool(data['flags'] & 64)
        if 'diagnostic_flags' in data:
            bits = data['diagnostic_flags']
            for field, bit in (('diagnostic_snapshot_valid', 0), ('vsync_sampled', 1),
                               ('href_sampled', 2), ('raw_error', 3), ('bayer_error', 4),
                               ('camera_reset_released', 5), ('diagnostic_pending', 6),
                               ('vsync_core_level', 8), ('href_core_level', 9)):
                data[field] = bool(bits & (1 << bit))
            # Keep the legacy field for callers. Bit 5 is FPGA cam_rst_n,
            # not the sensor's physical RESET pin; it is meaningful only in a
            # valid, fresh mailbox snapshot.
            data['capture_logic_reset_released'] = data['camera_reset_released']
            data['camera_data_sample'] = (bits >> 16) & 255
            data['diagnostic_age_seconds'] = data['diagnostic_age_cycles'] / CORE_CLK_HZ
        if 'sccb_flags' in data:
            bits = data['sccb_flags']
            for field, bit in (('sccb_runtime_owner', 0), ('sccb_busy', 1),
                               ('sccb_result_valid', 2), ('sccb_write', 3),
                               ('sccb_boot_config_done', 16), ('sccb_scl_high', 17),
                               ('sccb_sda_high', 18)):
                data[field] = bool(bits & (1 << bit))
            data['sccb_error'] = (bits >> 4) & 15
            data['sccb_nack_phase'] = (bits >> 8) & 15
            data['sccb_last_reject_reason'] = (bits >> 12) & 15
            data['sccb_register'] = data['sccb_value_register'] & 65535
            data['sccb_value'] = (data['sccb_value_register'] >> 16) & 255
        packets.append(data)
        del buffer[:frame_length]
    return packets

def _add_diagnostic_rates(data, previous):
    """Require two fresh mailbox snapshots, independent of frame-rate validity.

    Pending alone is normal during the next handshake. The age is measured at
    the UART snapshot, so subtracting its change recovers the mailbox sampling
    interval (apart from the small request/acknowledge synchronization latency).
    """
    data['diagnostic_rate_status'] = 'unavailable'
    if 'pclk_edges' not in data:
        return
    for sample, prefix in ((data, ''), (previous, 'previous_')):
        if sample is None or 'pclk_edges' not in sample:
            data['diagnostic_rate_status'] = 'awaiting_previous_sample'
            return
        if not sample.get('diagnostic_snapshot_valid', False):
            data['diagnostic_rate_status'] = prefix + 'snapshot_invalid'
            return
        age = sample.get('diagnostic_age_cycles')
        if age is None or age < 0:
            data['diagnostic_rate_status'] = prefix + 'snapshot_age_unavailable'
            return
        if age > DIAGNOSTIC_MAX_AGE_CYCLES:
            data['diagnostic_rate_status'] = prefix + 'snapshot_stale'
            return
    if 'sample_ticks' not in data or 'sample_ticks' not in previous:
        data['diagnostic_rate_status'] = 'board_ticks_unavailable'
        return
    ticks = (data['sample_ticks'] - previous['sample_ticks']) & 0xffffffff
    if data['build_id'] != previous['build_id'] or ticks >= 0x80000000:
        data['diagnostic_rate_status'] = 'reset_or_out_of_order'
        return
    if ticks == 0:
        data['diagnostic_rate_status'] = 'duplicate_sample'
        return
    interval_cycles = (ticks * CORE_CLK_HZ - data['diagnostic_age_cycles']
                       + previous['diagnostic_age_cycles'])
    if interval_cycles <= 0:
        data['diagnostic_rate_status'] = 'snapshot_interval_invalid'
        return
    pclk_delta = (data['pclk_edges'] - previous['pclk_edges']) & 0xffffffff
    if pclk_delta == 0:
        data['diagnostic_rate_status'] = 'snapshot_not_advanced'
        return
    if pclk_delta >= 0x80000000:
        data['diagnostic_rate_status'] = 'pclk_edges_counter_reset'
        return
    interval = interval_cycles / CORE_CLK_HZ
    data['diagnostic_sample_interval_seconds'] = interval
    warnings = []
    for counter, rate in DIAGNOSTIC_RATES:
        if counter not in data or counter not in previous:
            continue
        delta = (data[counter] - previous[counter]) & 0xffffffff
        if delta >= 0x80000000:
            warnings.append(counter + '_counter_reset')
        else:
            data[rate] = delta / interval
    data['diagnostic_rate_status'] = ','.join(warnings) if warnings else 'ok'


def _add_throughput_rates(data, previous):
    """Rates use board ticks. Cycle counters wrap in 42.95 s at 100 MHz, so
    reject intervals >= half that range instead of inventing an unwrap count.
    First-R wait/burst is a window aggregate (bursts may cross its boundaries).
    """
    for key in THROUGHPUT_DERIVED:
        data.pop(key, None)
    data['throughput_rate_status'] = 'unavailable'
    if 'last_frame_cycles' not in data:
        return
    if data['last_frame_cycles']:
        data['last_frame_seconds'] = data['last_frame_cycles'] / CORE_CLK_HZ
        data['last_frame_service_fps'] = CORE_CLK_HZ / data['last_frame_cycles']
    if previous is None or 'last_frame_cycles' not in previous:
        data['throughput_rate_status'] = 'awaiting_previous_sample'
        return
    ticks = (data['sample_ticks'] - previous['sample_ticks']) & 0xffffffff
    if data['build_id'] != previous['build_id'] or ticks >= 0x80000000:
        data['throughput_rate_status'] = 'reset_or_out_of_order'
        return
    if ticks == 0:
        data['throughput_rate_status'] = 'duplicate_sample'
        return
    clocks = ticks * CORE_CLK_HZ
    if clocks >= 0x80000000:
        data['throughput_rate_status'] = 'interval_too_long_for_cycle_counters'
        return
    deltas = {}
    warnings = []
    for key in THROUGHPUT_FIELDS[1:]:
        delta = (data[key] - previous[key]) & 0xffffffff
        if delta >= 0x80000000:
            warnings.append(key + '_counter_reset')
        else:
            deltas[key] = delta
    for counter, fraction in THROUGHPUT_FRACTIONS.items():
        if counter in deltas:
            data[fraction] = deltas[counter] / clocks
    for counter, field in (('camera_skipped_frames', 'observed_camera_skipped_fps'),
                           ('repeated_frames', 'observed_repeated_fps')):
        if counter in deltas:
            data[field] = deltas[counter] / ticks
    if 'ddr_data_beats' in deltas:
        data['ddr_data_bytes_per_second'] = deltas['ddr_data_beats'] * 32 / ticks
    if deltas.get('ddr_read_bursts') and 'ddr_read_first_wait_cycles' in deltas:
        data['ddr_read_first_wait_cycles_per_burst'] = deltas['ddr_read_first_wait_cycles'] / deltas['ddr_read_bursts']
    data['throughput_rate_status'] = ','.join(warnings) if warnings else 'ok'


def add_frame_rates(data, previous):
    """Add board-clock rates in place; host receive timing is deliberately unused.

    Several old packets can arrive in one USB read. Their tick differences still
    describe the actual sampling interval. Counter resets/out-of-order packets
    and legacy packets never produce a manufactured frame rate.
    """
    for key in ('observed_new_fps', 'observed_raw_fps', 'sample_interval_seconds', 'rate_status',
                'diagnostic_rate_status', 'diagnostic_sample_interval_seconds',
                'observed_pclk_hz', 'observed_vsync_rise_hz', 'observed_vsync_fall_hz',
                'observed_href_lines_per_second', 'observed_href_pixels_per_second'):
        data.pop(key, None)
    _add_diagnostic_rates(data, previous)
    _add_throughput_rates(data, previous)
    if previous is None:
        return data
    if 'sample_ticks' not in data or 'sample_ticks' not in previous:
        data['rate_status'] = 'board_ticks_unavailable'
        return data
    ticks = (data['sample_ticks'] - previous['sample_ticks']) & 0xffffffff
    if data['build_id'] != previous['build_id'] or ticks >= 0x80000000:
        data['rate_status'] = 'reset_or_out_of_order'
        return data
    if ticks == 0:
        data['rate_status'] = 'duplicate_sample'
        return data
    data['sample_interval_seconds'] = ticks
    warnings = []
    for counter, rate in (('processed', 'observed_new_fps'),
                          ('raw_valid_frames', 'observed_raw_fps')):
        if counter not in data or counter not in previous:
            continue
        delta = (data[counter] - previous[counter]) & 0xffffffff
        if delta >= 0x80000000:
            warnings.append(counter + '_counter_reset')
        else:
            data[rate] = delta / ticks
    data['rate_status'] = ','.join(warnings) if warnings else 'ok'
    return data

def main():
    import serial
    from serial.tools import list_ports
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args()
    matches = [p for p in list_ports.comports() if p.device.upper() == args.port.upper()
               and p.vid == 0x10c4 and p.pid == 0xea60]
    if len(matches) != 1:
        raise SystemExit('The selected port is not the verified CP210x UART.')
    previous = None
    buffer = bytearray()
    with serial.Serial(args.port, 115200, timeout=.2) as port:
        begin = time.monotonic()
        while time.monotonic() - begin < args.seconds:
            buffer.extend(port.read(256))
            for data in extract_packets(buffer):
                add_frame_rates(data, previous)
                previous = data.copy()
                print(json.dumps(data), flush=True)
    if previous is None:
        raise SystemExit('No valid status packet; device enumeration does not prove FPGA communication.')

if __name__ == '__main__':
    main()
