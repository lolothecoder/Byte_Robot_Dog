"""Standalone diagnostic for the node-5 motor.

Mimics the can_relay bridge's arm + TX sequence for ONE node, using the
exact same can_waveshare framing the bridge uses. If this script can
drive a motor and the bridge can't, the difference is in the multi-node
coalescing or TX-loop interaction. If this script can't drive it but
tune.py can, the difference is in the bridge's per-node arm ordering
(IDLE -> SET_INPUT_POS while still IDLE -> CLOSED_LOOP) versus tune.py's
(IDLE -> CLOSED_LOOP -> SET_INPUT_POS).

Run inside the container, after stopping any other ros2 process that
might be holding /dev/ttyUSB0:

    python3 /ros2_ws/diagnose_node5.py

Sends a 5 s sinusoidal SET_INPUT_POS sweep around the current position
(amplitude 0.05 motor rev, period 1 s) and prints encoder estimates.
Always returns the node to IDLE on exit.
"""
from __future__ import annotations

import math
import sys
import time

# Reuse the bridge's framing module so any byte-level difference is
# limited to call ordering, not encoding.
sys.path.insert(0, '/ros2_ws/src/byte_leg_hardware')
from byte_leg_hardware import can_waveshare as cw  # noqa: E402

import serial  # noqa: E402

PORT = '/dev/ttyUSB0'
BAUD = 2_000_000
NODE = 5            # hip_pitch
SEED_REV = None     # filled in from first encoder estimate
AMPLITUDE_REV = 0.05
PERIOD_S = 1.0
TX_HZ = 50.0
DURATION_S = 5.0


def open_port() -> serial.Serial:
    print(f'Opening {PORT} @ {BAUD}')
    ser = serial.Serial(PORT, BAUD, timeout=0.01, exclusive=True)
    cw.init_waveshare(ser)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    return ser


def drain_for_position(ser: serial.Serial, timeout_s: float) -> float | None:
    """Block until we receive an encoder estimate from NODE, or timeout."""
    buf = bytearray()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        for arb_id, payload in cw.iter_frames(buf):
            node_id, cmd_id = cw.split_arb_id(arb_id)
            if (node_id == NODE
                    and cmd_id == cw.CMD_GET_ENCODER_ESTIMATES
                    and len(payload) == 8):
                pos, _vel = cw.decode_encoder_estimates(payload)
                return pos
    return None


def arm_bridge_style(ser: serial.Serial, seed_rev: float) -> None:
    """Replicate can_relay._srv_arm: IDLE -> 0.5s -> SET_INPUT_POS(seed)
    -> SET_AXIS_STATE(CLOSED_LOOP) -> 1.0s."""
    print(f'[arm] node {NODE}: IDLE')
    cw.set_axis_state(ser, NODE, cw.AXIS_IDLE)
    time.sleep(0.5)
    print(f'[arm] node {NODE}: pre-seed SET_INPUT_POS({seed_rev:+.4f} rev)')
    cw.set_input_pos(ser, NODE, seed_rev)
    print(f'[arm] node {NODE}: CLOSED_LOOP')
    cw.set_axis_state(ser, NODE, cw.AXIS_CLOSED_LOOP)
    time.sleep(1.0)


def tx_loop(ser: serial.Serial, seed_rev: float) -> None:
    """Replicate the bridge's _tx_loop body for one node."""
    dt = 1.0 / TX_HZ
    n_ticks = int(DURATION_S * TX_HZ)
    last_print = 0.0
    buf = bytearray()
    print(f'[tx] driving node {NODE}: '
          f'seed={seed_rev:+.4f} rev, amplitude={AMPLITUDE_REV}, '
          f'period={PERIOD_S}s, duration={DURATION_S}s')
    t0 = time.monotonic()
    for k in range(n_ticks):
        t = k * dt
        target = seed_rev + AMPLITUDE_REV * math.sin(2 * math.pi * t / PERIOD_S)
        ser.write(cw.build_set_input_pos(NODE, target))

        # Drain whatever encoder data has come back.
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        latest_pos = None
        for arb_id, payload in cw.iter_frames(buf):
            node_id, cmd_id = cw.split_arb_id(arb_id)
            if (node_id == NODE
                    and cmd_id == cw.CMD_GET_ENCODER_ESTIMATES
                    and len(payload) == 8):
                latest_pos, _ = cw.decode_encoder_estimates(payload)

        now = time.monotonic()
        if now - last_print > 0.1 and latest_pos is not None:
            err = latest_pos - target
            print(f'  t={now - t0:5.2f}s  cmd={target:+.4f}  '
                  f'meas={latest_pos:+.4f}  err={err:+.4f} rev')
            last_print = now

        # Sleep to next tick.
        next_t = t0 + (k + 1) * dt
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)


def main() -> int:
    ser = open_port()
    try:
        # First, IDLE the node so we always start from a known state.
        cw.set_axis_state(ser, NODE, cw.AXIS_IDLE)
        time.sleep(0.5)

        print('Waiting for an encoder estimate from node '
              f'{NODE}... (requires encoder_msg_rate_ms != 0 saved on the ODrive)')
        seed = drain_for_position(ser, timeout_s=2.0)
        if seed is None:
            print(f'No encoder data from node {NODE} in 2s. '
                  'Either node 5 is not on the bus, the node_id mismatch, '
                  'or encoder_msg_rate_ms is 0 on that ODrive.')
            return 2
        print(f'Current position of node {NODE}: {seed:+.4f} rev')

        arm_bridge_style(ser, seed)
        tx_loop(ser, seed)
        print('Done. Returning node to IDLE.')
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted.')
        return 130
    finally:
        try:
            cw.set_axis_state(ser, NODE, cw.AXIS_IDLE)
        except Exception as e:
            print(f'WARNING: could not send IDLE on exit: {e}')
        try:
            ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
