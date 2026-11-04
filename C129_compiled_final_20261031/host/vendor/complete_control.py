"""Pure C124/C125 protocol adapter; imports never open hardware.

Geometry and wire format reuse the preserved C121 host reference. C124 identity
and enhancement flag semantics are verified against its delivered board RTL.
The persistent IMU session owns transport; there are no one-shot serial helpers.
"""
from camera_config import packet
from complete_geometry import ADDRESSES, BUILD_ID, validate_words

MODE_NAMES = ('bypass', 'auto', 'shadow', 'highlight', 'histogram')
DIRECTION_NAMES = ('unchanged', 'brighten', 'darken', 'reserved')


def verify_firmware(status):
    from pose_protocol import REALTIME_BUILD_IDS, verify_pose
    if status.get('build_id') not in (BUILD_ID, *REALTIME_BUILD_IDS):
        raise ValueError('Firmware must be C124, C125, C127, C128 or C129.')
    if status.get('protocol_version') in (7, 8):
        verify_pose(status)
    elif status.get('protocol_version') != 5:
        raise ValueError('A CRC-valid V5 or C125 POS1 status is required.')
    return status


def enhancement_status(status):
    verify_firmware(status)
    flags = status['flags']
    mode, direction, gamma = (flags >> 12) & 7, (flags >> 10) & 3, (flags >> 7) & 7
    if mode > 4:
        raise ValueError('Undefined C124 enhancement mode.')
    return dict(enhancement_mode=mode, enhancement_mode_name=MODE_NAMES[mode],
                enhancement_direction=direction,
                enhancement_direction_name=DIRECTION_NAMES[direction], gamma_id=gamma)


def configuration_packets(words):
    words = validate_words(words)
    return (packet(0xfe, 0x434f4d50)
            + b''.join(packet(address, word) for address, word in zip(ADDRESSES, words))
            + packet(0xff, 0x434f4d54))
