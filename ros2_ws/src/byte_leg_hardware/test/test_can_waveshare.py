import io
import math
import struct

import pytest

from byte_leg_hardware.can_waveshare import (
    CMD_GET_ENCODER_ESTIMATES,
    CMD_SET_AXIS_STATE,
    CMD_SET_INPUT_POS,
    arb_id_for,
    decode_encoder_estimates,
    iter_frames,
    joint_rad_to_motor_rev,
    motor_rev_to_joint_rad,
    send_can,
    set_axis_state,
    set_input_pos,
    split_arb_id,
)


class _FakeSerial(io.BytesIO):
    def flush(self) -> None:  # pragma: no cover - passthrough
        pass


def _bytes_written(ser: _FakeSerial) -> bytes:
    return ser.getvalue()


def test_send_can_roundtrip_minimal() -> None:
    ser = _FakeSerial()
    send_can(ser, node_id=3, cmd_id=CMD_SET_INPUT_POS, data=struct.pack('<fhh', 1.25, 0, 0))
    buf = bytearray(_bytes_written(ser))
    frames = list(iter_frames(buf))
    assert len(frames) == 1
    arb, payload = frames[0]
    assert arb == arb_id_for(3, CMD_SET_INPUT_POS)
    node_id, cmd_id = split_arb_id(arb)
    assert node_id == 3
    assert cmd_id == CMD_SET_INPUT_POS
    pos, vel, tq = struct.unpack('<fhh', payload)
    assert pos == pytest.approx(1.25)
    assert vel == 0
    assert tq == 0


def test_set_axis_state_frames_correctly() -> None:
    ser = _FakeSerial()
    set_axis_state(ser, node_id=5, state=8)  # CLOSED_LOOP
    buf = bytearray(_bytes_written(ser))
    (arb, payload), = list(iter_frames(buf))
    node_id, cmd_id = split_arb_id(arb)
    assert (node_id, cmd_id) == (5, CMD_SET_AXIS_STATE)
    (state,) = struct.unpack('<I', payload)
    assert state == 8


def test_set_input_pos_quantises_feedforwards() -> None:
    ser = _FakeSerial()
    set_input_pos(ser, node_id=1, pos_rev=0.5, vel_ff=1.234, torque_ff=-2.5)
    (arb, payload), = list(iter_frames(bytearray(_bytes_written(ser))))
    assert arb == arb_id_for(1, CMD_SET_INPUT_POS)
    pos, vel_q, tq_q = struct.unpack('<fhh', payload)
    assert pos == pytest.approx(0.5)
    assert vel_q == int(1.234 * 1000)
    assert tq_q == int(-2.5 * 1000)


def test_encoder_request_is_zero_length() -> None:
    ser = _FakeSerial()
    from byte_leg_hardware.can_waveshare import request_encoder_estimates
    request_encoder_estimates(ser, node_id=3)
    (arb, payload), = list(iter_frames(bytearray(_bytes_written(ser))))
    assert arb == arb_id_for(3, CMD_GET_ENCODER_ESTIMATES)
    assert payload == b''


def test_iter_frames_recovers_from_leading_garbage() -> None:
    ser = _FakeSerial()
    send_can(ser, node_id=3, cmd_id=CMD_SET_INPUT_POS, data=b'\x01\x02\x03\x04')
    good = _bytes_written(ser)
    # Prepend arbitrary noise that would not begin with 0xAA.
    noise = b'\x00\xFF\x12\x34\x56'
    buf = bytearray(noise + good)
    frames = list(iter_frames(buf))
    assert len(frames) == 1
    assert frames[0][1] == b'\x01\x02\x03\x04'
    # All bytes consumed.
    assert len(buf) == 0


def test_iter_frames_handles_false_sync() -> None:
    # A lone 0xAA byte followed by non-frame bytes must not consume a valid
    # following frame.
    ser = _FakeSerial()
    send_can(ser, node_id=1, cmd_id=CMD_SET_INPUT_POS, data=b'\xaa\xbb\xcc\xdd')
    good = _bytes_written(ser)
    buf = bytearray(b'\xAA\x00\x00\x00\x99' + good)
    # The spurious 0xAA at the start: data_len=0, packet_len=5,
    # trailing byte 0x99 != 0x55, so parser drops and resyncs.
    frames = list(iter_frames(buf))
    assert len(frames) == 1
    assert frames[0][1] == b'\xaa\xbb\xcc\xdd'


def test_iter_frames_holds_partial_tail() -> None:
    ser = _FakeSerial()
    send_can(ser, node_id=3, cmd_id=CMD_SET_INPUT_POS, data=b'\x01\x02\x03\x04')
    full = _bytes_written(ser)
    partial = bytearray(full[:-2])  # missing trailing bytes
    frames = list(iter_frames(partial))
    assert frames == []
    # Partial tail preserved for next round.
    assert bytes(partial) == full[:-2]


def test_decode_encoder_estimates_roundtrip() -> None:
    payload = struct.pack('<ff', 1.25, -3.5)
    pos, vel = decode_encoder_estimates(payload)
    assert pos == pytest.approx(1.25)
    assert vel == pytest.approx(-3.5)


@pytest.mark.parametrize('sign', [1.0, -1.0])
@pytest.mark.parametrize('gear', [1.0, 8.0, 21.0])
@pytest.mark.parametrize('offset', [0.0, 5.3, -17.1])
@pytest.mark.parametrize('angle', [-1.4, -0.3, 0.0, 0.7, 1.9])
def test_rad_rev_roundtrip(sign: float, gear: float, offset: float,
                           angle: float) -> None:
    rev = joint_rad_to_motor_rev(angle, gear_ratio=gear, sign=sign,
                                 zero_offset_rev=offset)
    back = motor_rev_to_joint_rad(rev, gear_ratio=gear, sign=sign,
                                  zero_offset_rev=offset)
    assert math.isclose(back, angle, abs_tol=1e-9)


def test_rad_to_rev_sign_behaviour() -> None:
    # Positive joint rotation with sign=+1 -> positive motor rev.
    assert joint_rad_to_motor_rev(1.0, gear_ratio=8.0, sign=1.0,
                                  zero_offset_rev=0.0) > 0
    # Same joint rotation with sign=-1 -> motor rev of opposite sign.
    fwd = joint_rad_to_motor_rev(1.0, gear_ratio=8.0, sign=1.0,
                                 zero_offset_rev=0.0)
    rev = joint_rad_to_motor_rev(1.0, gear_ratio=8.0, sign=-1.0,
                                 zero_offset_rev=0.0)
    assert fwd == pytest.approx(-rev)


def test_rad_to_rev_gear_multiplies() -> None:
    # 8:1 gear -> one full joint turn = 8 motor turns.
    assert joint_rad_to_motor_rev(2 * math.pi, gear_ratio=8.0, sign=1.0,
                                  zero_offset_rev=0.0) == pytest.approx(8.0)
    assert joint_rad_to_motor_rev(2 * math.pi, gear_ratio=1.0, sign=1.0,
                                  zero_offset_rev=0.0) == pytest.approx(1.0)
