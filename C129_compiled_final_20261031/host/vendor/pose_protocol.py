"""C125 atomic pose transactions and POS1 event snapshots; no hardware I/O.

H is a normalized output-ray to source-ray matrix, never K R K^-1.
The board validates every generated pixel and tile before publication. Domain
invalid pixels become black; setup, ROI, all-invalid and DDR failures retain
the previously published bank. Capability bits are mandatory for that policy.
"""
from numbers import Integral

from camera_config import packet
from complete_geometry import validate_words

REALTIME_BUILD_ID = 0x0720C125
VIEW_BUILD_ID = 0x0720C127
SCAN_BUILD_ID = 0x0720C128
DETAIL_BUILD_ID = 0x0720C129
SCAN_BUILD_IDS = (SCAN_BUILD_ID, DETAIL_BUILD_ID)
VIEW_BUILD_IDS = (VIEW_BUILD_ID, *SCAN_BUILD_IDS)
REALTIME_BUILD_IDS = (REALTIME_BUILD_ID, *VIEW_BUILD_IDS)
POSE_MAGIC = 0x504F5331
POSE_FIELDS = ('magic', 'build_id', 'version_flags', 'commits', 'rejected',
               'map_cycles', 'board_time_us', 'captured', 'processed',
               'control_failures', 'map_invalid_pixels', 'map_domain_pixels',
               'guard_state')
POSE_CAPABILITIES = 0x70000


def decode_pose(words):
    if len(words) not in (13, 14):
        raise ValueError('Pose status requires 13 (C125) or 14 (C127) words')
    data = dict(zip(POSE_FIELDS, words[:13]))
    if len(words) == 14:
        data['view_context'] = words[13]
    data['config_version'] = data['version_flags'] & 65535
    data['flags'] = data.pop('version_flags') >> 16
    data.update(protocol_version=8 if len(words) == 14 else 7, packet_type='pose_status')
    for field, bit in (('processing', 0), ('map_building', 1),
                       ('error_sticky', 2), ('camera_error', 3),
                       ('camera_config_done', 4), ('hdmi_config_done', 5),
                       ('ddr_initialized', 6)):
        data[field] = bool(data['flags'] & (1 << bit))
    guard = data['guard_state']
    data.update(reject_flags=guard & 15, saved_valid=bool(guard & 256),
                raw_active=bool(guard & 512), key_busy=bool(guard & 1024),
                hardware_admission_capable=(guard & POSE_CAPABILITIES) == POSE_CAPABILITIES)
    if len(words) == 14:
        data.update(view_capable=bool(guard & (1 << 19)),
                    committed_modes=(guard >> 11) & 3,
                    requested_modes=(guard >> 13) & 3,
                    plane_valid=bool(guard & (1 << 15)),
                    matrix_valid=bool(guard & (1 << 20)),
                    mode_reject=bool(guard & (1 << 21)),
                    last_publish_from_key=bool(guard & (1 << 22)),
                    software_route_capable=bool(guard & (1 << 23)),
                    observation_capable=bool(guard & (1 << 24)),
                    visual_matrix_capable=bool(guard & (1 << 25)),
                    observation_active=bool(guard & (1 << 26)),
                    detail_capable=bool(guard & (1 << 27)),
                    detail_active=bool(guard & (1 << 28)))
    return data


def verify_pose(status):
    if ((status.get('build_id'), status.get('protocol_version')) not in
            ((REALTIME_BUILD_ID, 7), *((build, 8) for build in VIEW_BUILD_IDS)) or status.get('magic') != POSE_MAGIC):
        raise ValueError('CRC-valid C125/V7 or C127-C129/V8 pose status is required')
    return status


def pose_packets(words, base_version):
    """FD PO|version, ordered nine H words, FF COMT: exactly 88 bytes."""
    words = validate_words(words)
    if isinstance(base_version, bool) or not isinstance(base_version, Integral) or not 0 <= base_version <= 65535:
        raise ValueError('Base version must be an unsigned 16-bit integer')
    if words[16] != 1:
        raise ValueError('Pose-only transactions require corrected mode')
    return (packet(0xFD, 0x504F0000 | int(base_version)) +
            b''.join(packet(address, word) for address, word in zip(range(0x20, 0x29), words[18:])) +
            packet(0xFF, 0x434F4D54))


def visual_packets(words, base_version):
    """C128 FC VI|version: an explicitly visual 3x3 request, not an IMU pose.

    It shares the ordered atomic matrix/commit format and full-map guard.
    Firmware also requires the raw observation route and static factors off.
    """
    ordinary = pose_packets(words, base_version)
    return packet(0xFC, 0x56490000 | int(base_version)) + ordinary[8:]
