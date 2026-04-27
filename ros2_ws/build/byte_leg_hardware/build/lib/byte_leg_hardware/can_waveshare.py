"""Waveshare USB-CAN framing + ODrive CAN Simple helpers.

Mirrors the protocol used by `tune.py` so the ROS bridge and the bench
tuning tool speak the same dialect to the ODrive. No ROS dependencies.

Waveshare envelope:  0xAA  0xC0|len  arb_lo  arb_hi  <data...>  0x55
ODrive arbitration:  (node_id << 5) | cmd_id
"""
from __future__ import annotations

import struct
import time
from typing import BinaryIO, Callable, Iterator, Optional


SYNC_START = 0xAA
SYNC_END = 0x55
FRAME_PREFIX = 0xC0  # upper nibble of the length byte

# ODrive CAN Simple command IDs (subset used here).
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_INPUT_POS = 0x0C
CMD_GET_IQ = 0x14

# Axis states.
AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8


def init_waveshare(ser: BinaryIO) -> None:
    """Put the Waveshare adapter into fixed-mode pass-through.

    This is a 20-byte Waveshare config packet. Required once per session
    before any CAN frame will be accepted by the adapter.
    """
    config = [0xAA, 0x55, 0x12, 0x03, 0x01] + [0x00] * 10
    checksum = sum(config[2:]) & 0xFF
    config.append(checksum)
    ser.write(bytes(config))
    ser.flush()
    time.sleep(0.5)


def _wrap_frame(arb_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError(f'CAN payload > 8 bytes: {len(data)}')
    header = bytes([
        SYNC_START,
        FRAME_PREFIX | len(data),
        arb_id & 0xFF,
        (arb_id >> 8) & 0xFF,
    ])
    return header + bytes(data) + bytes([SYNC_END])


def build_set_input_pos(node_id: int, pos_rev: float,
                        vel_ff: float = 0.0,
                        torque_ff: float = 0.0) -> bytes:
    """Return the wrapped Waveshare frame for a SET_INPUT_POS command.

    Lets the caller coalesce several frames into a single ``ser.write``
    so that all three motor commands for one tick land in the OS write
    buffer as one USB transfer.
    """
    vel_ff_int = int(vel_ff * 1000)
    torque_ff_int = int(torque_ff * 1000)
    data = struct.pack('<fhh', pos_rev, vel_ff_int, torque_ff_int)
    return _wrap_frame(arb_id_for(node_id, CMD_SET_INPUT_POS), data)


def build_set_axis_state(node_id: int, state: int) -> bytes:
    return _wrap_frame(arb_id_for(node_id, CMD_SET_AXIS_STATE),
                       struct.pack('<I', state))


def send_can(ser: BinaryIO, node_id: int, cmd_id: int,
             data: bytes = b'', flush: bool = False) -> None:
    """Write a CAN frame. flush=False by default: on USB-serial,
    ``ser.flush()`` calls ``tcdrain()`` which can block for a USB poll
    interval (~1-2ms) and adds up fast at 100+ frames/sec. The OS still
    transmits buffered writes promptly. Set flush=True for one-shot
    operations where you must know the bus has seen the frame before
    the next call (e.g. axis-state transitions during arm).
    """
    arb_id = (node_id << 5) | cmd_id
    ser.write(_wrap_frame(arb_id, data))
    if flush:
        ser.flush()


def set_axis_state(ser: BinaryIO, node_id: int, state: int) -> None:
    # State changes are infrequent and order-sensitive — flush.
    send_can(ser, node_id, CMD_SET_AXIS_STATE,
             struct.pack('<I', state), flush=True)


def set_input_pos(ser: BinaryIO, node_id: int, pos_rev: float,
                  vel_ff: float = 0.0, torque_ff: float = 0.0) -> None:
    # High-rate setpoint stream — skip flush. Latest setpoint wins.
    ser.write(build_set_input_pos(node_id, pos_rev, vel_ff, torque_ff))


def iter_frames(buf: bytearray,
                on_resync: Optional[Callable[[], None]] = None
                ) -> Iterator[tuple[int, bytes]]:
    """Consume complete Waveshare frames from ``buf`` in place.

    Yields ``(arb_id, payload_bytes)`` for each complete frame. Incomplete
    trailing bytes are left in ``buf`` for the next call. Bytes that don't
    start with the sync byte are dropped one-by-one — this is the same
    recovery pattern tune.py uses to re-align after a partial/corrupted
    frame on a noisy serial line.

    ``on_resync`` is invoked once per dropped byte. The bridge uses it to
    populate ``/real_leg/diag`` so a noisy bus shows up as a counter that
    moves, instead of just silent jank.
    """
    while len(buf) >= 5:
        if buf[0] != SYNC_START:
            del buf[0]
            if on_resync is not None:
                on_resync()
            continue
        data_len = buf[1] & 0x0F
        packet_len = 5 + data_len
        if len(buf) < packet_len:
            return  # wait for more bytes
        if buf[packet_len - 1] != SYNC_END:
            # Bad trailing sync — drop one byte and resync.
            del buf[0]
            if on_resync is not None:
                on_resync()
            continue
        arb_id = buf[2] | (buf[3] << 8)
        payload = bytes(buf[4:4 + data_len])
        del buf[:packet_len]
        yield arb_id, payload


def decode_encoder_estimates(payload: bytes) -> tuple[float, float]:
    """Parse a GET_ENCODER_ESTIMATES payload: (pos_rev, vel_rev_s)."""
    if len(payload) != 8:
        raise ValueError(f'encoder estimates expects 8 bytes, got {len(payload)}')
    return struct.unpack('<ff', payload)


def decode_iq(payload: bytes) -> tuple[float, float]:
    """Parse a GET_IQ payload: (iq_setpoint, iq_measured)."""
    if len(payload) != 8:
        raise ValueError(f'iq expects 8 bytes, got {len(payload)}')
    return struct.unpack('<ff', payload)


def arb_id_for(node_id: int, cmd_id: int) -> int:
    return (node_id << 5) | cmd_id


def split_arb_id(arb_id: int) -> tuple[int, int]:
    return arb_id >> 5, arb_id & 0x1F


# ---- Unit conversion ------------------------------------------------------
#
# Hardware units are *motor* revolutions (the encoder is on the motor side
# of the 8:1 cycloidal reduction). Everything above the bridge speaks
# joint radians per URDF convention.

_TAU = 6.283185307179586  # 2*pi


def joint_rad_to_motor_rev(rad: float, *, gear_ratio: float, sign: float,
                           zero_offset_rev: float) -> float:
    """Convert a commanded joint angle (rad, URDF frame) to motor rev."""
    return zero_offset_rev + sign * gear_ratio * rad / _TAU


def motor_rev_to_joint_rad(rev: float, *, gear_ratio: float, sign: float,
                           zero_offset_rev: float) -> float:
    """Convert a measured motor-rev reading to joint angle (rad, URDF frame)."""
    return (rev - zero_offset_rev) * _TAU / (sign * gear_ratio)


__all__ = [
    'CMD_SET_AXIS_STATE', 'CMD_GET_ENCODER_ESTIMATES', 'CMD_SET_INPUT_POS',
    'CMD_GET_IQ', 'AXIS_IDLE', 'AXIS_CLOSED_LOOP',
    'init_waveshare', 'send_can', 'set_axis_state', 'set_input_pos',
    'build_set_input_pos', 'build_set_axis_state',
    'iter_frames', 'decode_encoder_estimates', 'decode_iq',
    'arb_id_for', 'split_arb_id',
    'joint_rad_to_motor_rev', 'motor_rev_to_joint_rad',
]
