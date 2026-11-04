"""Restore the vendor's wide-FOV720 RGB565 mode in a separate PDS project.

Keep the same sensor window and original720 binning/offset/PLL settings.
No FPGA image enlargement, PC mapping upload or geometric sensor correction.
"""
from __future__ import annotations
import re

MODE_ID = 'ov5640_dvp_rgb565_wide_1280x720_30_candidate_v1'
PROFILE = [
    (0x3008, 0x42),
    (0x3017, 0xFF), (0x3018, 0xFF), (0x300E, 0x58),
    # Exact vendor720 PLL,8-bit DVP transports each565 pixel in two clocks.
    # XVCLK24MHz -> VCO840MHz -> SCLK42MHz; expected DVP PCLK84MHz.
    (0x3034, 0x1A), (0x3035, 0x21), (0x3036, 0x69),
    (0x3037, 0x13), (0x3108, 0x01),
    (0x3820, 0x47), (0x3821, 0x00),
    (0x3814, 0x31), (0x3815, 0x31),
    (0x3800, 0x00), (0x3801, 0x00), (0x3802, 0x00), (0x3803, 0xFA),
    (0x3804, 0x0A), (0x3805, 0x3F), (0x3806, 0x06), (0x3807, 0xA9),
    (0x3808, 0x05), (0x3809, 0x00), (0x380A, 0x02), (0x380B, 0xD0),
    (0x380C, 0x07), (0x380D, 0x64), (0x380E, 0x02), (0x380F, 0xE4),
    (0x3810, 0x00), (0x3811, 0x10), (0x3812, 0x00), (0x3813, 0x04),
    (0x3618, 0x00), (0x3612, 0x29), (0x3708, 0x64),
    (0x3709, 0x52), (0x370C, 0x03),
    (0x4001, 0x02), (0x4004, 0x02), (0x4005, 0x1A),
    (0x3002, 0x1C), (0x3006, 0xC3),
    (0x4300, 0x60), (0x501F, 0x01),
    (0x5000, 0xA7), (0x5001, 0x83),
    # Keep the vendor's automatic DVP division for this exact output mode.
    (0x460B, 0x37), (0x460C, 0x20), (0x3824, 0x04),
    (0x4740, 0x20), (0x4745, 0x00),
    # Fixed nominal frame length; do not turn dark scenes into night-mode5fps.
    (0x3A00, 0x78), (0x3A02, 0x02), (0x3A03, 0xE0),
    (0x3A14, 0x02), (0x3A15, 0xE0),
    # Retain original vendor anti-flicker tuning. Validate under actual light.
    (0x3A08, 0x00), (0x3A09, 0x6F), (0x3A0A, 0x00), (0x3A0B, 0x5C),
    (0x3A0D, 0x08), (0x3A0E, 0x06), (0x3503, 0x00),
    (0x4201, 0x00), (0x4202, 0x00), (0x4901, 0x00), (0x4902, 0x00),
    (0x503D, 0x00), (0x3008, 0x02),
]


def profile_metadata():
    return {
        'mode_id': MODE_ID, 'hardware_verified': False,
        'status': 'wide720p30_rgb565_candidate_offline_only',
        'interface': '8-bit DVP sensor D[9:2]',
        'format': 'BGR565 high byte first; FPGA swaps R/B',
        'width': 1280, 'height': 720, 'target_new_frames_per_second': 30,
        'xvclk_hz_assumed_from_module_schematic': 24000000,
        'candidate_sclk_hz': 42000000, 'candidate_pclk_hz': 84000000,
        'hts': 1892, 'vts': 740,
        'configured_sensor_rate_hz_estimate': 42000000 / (1892 * 740),
        'pclk_division': {'method': 'vendor automatic', '0x460C': '0x20', '0x3824': '0x04'},
        'analog_crop': {'x': 0, 'y': 250, 'width': 2624, 'height': 1456},
        'isp_offset': {'x': 16, 'y': 4}, 'binning': True,
        'sampling_increments': ['0x31', '0x31'],
        'orientation': {'0x3820': '0x47', '0x3821': '0x00', 'vflip_bits_preserved': True},
        'pre_scale_dimensions': [1280, 720], 'sensor_scale_ratio': [1, 1],
        'fpga_upscale_from_720p': False,
        'reference_capture': {'original_output': [1280, 720], 'original_isp_offset': [16, 4],
            'effective_physical_field_model': {'x': 32, 'y': 258, 'width': 2560, 'height': 1440}},
        'calibration_basis': 'C97 Q16.16 derivative for same field using explicit pixel-center2/3 coordinate conversion; not a new optical calibration.',
        'warnings': [
            'Camera dimensions/FOV/colour/84MHz PCLK/nominal30fps require board readback and measurement.',
            'Source SCLK and DVP byte PCLK are different: frame estimate uses42MHz, not84MHz.',
            'HDMI720p60 refresh is separate from30 new camera/processed frames per second.',
            'Vendor SCCB completion does not prove ACK/readback or optical calibration accuracy.'
        ],
        'register_writes': [{'address': f'0x{a:04x}', 'value': f'0x{v:02x}'} for a, v in PROFILE],
    }


def apply_profile(source):
    if MODE_ID in source:
        raise ValueError('Already applied; start from the intact vendor source')
    match = re.search(r'if\s*\(\s*reg_index\s*<\s*(\d+)\s*\)', source)
    if match is None:
        raise ValueError('Missing vendor register-table bound')
    first = int(match[1]); total = first + len(PROFILE)
    if total > 512:
        raise ValueError('Register table exceeds9-bit index')
    source = source[:match.start(1)] + str(total) + source[match.end(1):]
    anchor = re.search(r'(?m)^\s*default\s*:\s*reg_data', source)
    if anchor is None:
        raise ValueError('Missing vendor table default')
    addition = f'\n// {MODE_ID}; HARDWARE_VERIFIED=0\n'
    addition += '\n'.join(f"  {first+i}: reg_data <= 24'h{a:04x}{v:02x};" for i, (a, v) in enumerate(PROFILE)) + '\n'
    return source[:anchor.start()] + addition + source[anchor.start():]

