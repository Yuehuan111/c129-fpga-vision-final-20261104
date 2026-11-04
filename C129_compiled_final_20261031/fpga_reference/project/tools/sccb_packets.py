"""Pure runtime SCCB packet construction; no device access.

Promoted from experiments/sccb_runtime/tools/sccb_packets.py. The byte layout is
the existing eight-byte UART/CRC16 protocol, with dedicated E0/E1 opcodes.
"""
from camera_config import packet


def _check(value, bits, name):
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value < 1 << bits:
        raise ValueError(f'{name} must be an unsigned {bits}-bit integer')


def read_packet(register):
    _check(register, 16, 'register')
    return packet(0xe0, register)


def write_packet(register, value):
    _check(register, 16, 'register')
    _check(value, 8, 'value')
    return packet(0xe1, register | value << 16)
