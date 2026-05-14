"""Bench joystick teleop — new leg, all motors at body.

Fork of joy_drive.py for the leg revision where the knee motor sits at
the body. Independent per-motor control (no parallel-4-bar mixing).

  Hold RB to enable. While held:
    LX (axis 0)  ->  hip_abduct  motor (node 2)
    RY (axis 4)  ->  hip_pitch   motor (node 4)
    B / X        ->  knee        motor (node 6, jog forward/back)

  Release RB: each motor smoothly decays back to its startup position.
  Ctrl-C: every motor is IDLE'd (with retries).

Same startup/shutdown sequence as joy_drive.py — IDLE -> CLOSED_LOOP ->
read fresh encoder -> SET_INPUT_POS to it; shutdown sends IDLE 3× with
explicit flushes so node 6 doesn't end up stuck hot.

Dependencies: pyserial, pyusb.
"""
import math
import signal
import struct
import sys
import threading
import time

import serial
import usb.core
import usb.util

# ---- Hardware constants ---------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 2_000_000

NODE_HIP_ABDUCT = 2
NODE_HIP_PITCH  = 4
NODE_KNEE       = 6

GEAR_RATIO = 8.0  # GIM8010-8 = 8:1 motor revs per joint rev
SIGN_HIP_ABDUCT = +1.0
SIGN_HIP_PITCH  = -1.0  # carried over from joy_drive.py; verify on bring-up
SIGN_KNEE       = +1.0

# ---- Teleop tuning --------------------------------------------------------
MAX_RATE_RAD_S = 0.4   # hips: joint rad/s at full stick deflection
DECAY_TIME_S   = 0.3   # ~63% return to home in this many seconds when RB released
DEADZONE       = 0.15
TICK_HZ        = 50.0

JOINT_LIMITS_RAD = {
    'hip_abduct': (-math.radians(20.0), math.radians(20.0)),
    'hip_pitch':  (-math.radians(40.0), math.radians(5.0)),
    'knee':       (-math.radians(90.0), math.radians(90.0)),
}

# ---- F710 USB IDs (DirectInput mode, slider on "D") -----------------------
VENDOR_ID = 0x046D
PRODUCT_ID = 0xC219

TAU = 2.0 * math.pi


# =============================================================================
#  Waveshare CAN framing — identical bytes-on-the-wire as tune.py / joy_drive.py
# =============================================================================
def init_waveshare(ser):
    config = [0xAA, 0x55, 0x12, 0x03, 0x01] + [0x00] * 10
    config.append(sum(config[2:]) & 0xFF)
    ser.write(bytes(config))
    time.sleep(0.5)


def send_can(ser, node_id, cmd_id, data):
    arb_id = (node_id << 5) | cmd_id
    pkt = [0xAA, 0xC0 | len(data), arb_id & 0xFF, (arb_id >> 8) & 0xFF]
    pkt.extend(data)
    pkt.append(0x55)
    ser.write(bytes(pkt))


def set_axis_state(ser, node_id, state):
    send_can(ser, node_id, 0x07, struct.pack('<I', state))


def set_input_pos(ser, node_id, pos_rev):
    send_can(ser, node_id, 0x0C, struct.pack('<fhh', pos_rev, 0, 0))


def read_position(ser, node_id, timeout=3.0, min_readings=5):
    """Block until `min_readings` encoder broadcasts arrive from `node_id`.

    Requires axis0.config.can.encoder_rate_ms = 10 saved on the ODrive
    (CyberBeast 0.6.4 attribute name; older/other ODrive firmware calls it
    encoder_msg_rate_ms — check `odrv0.axis0.config.can` to see which).
    """
    expected = (node_id << 5) | 0x09
    buf = bytearray()
    readings = []
    t0 = time.time()
    while time.time() - t0 < timeout and len(readings) < min_readings:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
        while len(buf) >= 5:
            if buf[0] != 0xAA:
                buf.pop(0)
                continue
            n = buf[1] & 0x0F
            pkt_len = 5 + n
            if len(buf) < pkt_len:
                break
            if buf[pkt_len - 1] != 0x55:
                buf.pop(0)
                continue
            aid = buf[2] | (buf[3] << 8)
            if aid == expected and n == 8:
                pos, _vel = struct.unpack('<ff', buf[4:12])
                readings.append(pos)
            del buf[:pkt_len]
        time.sleep(0.005)
    return readings[-1] if readings else None


def idle_all(ser, retries=3, node_delay=0.05):
    """IDLE every node with retries. Long node_delay on shutdown stops one
    node's burst from interfering with another's IDLE acknowledgement."""
    for node in (NODE_HIP_ABDUCT, NODE_HIP_PITCH, NODE_KNEE):
        for _ in range(retries):
            try:
                set_axis_state(ser, node, 1)  # AXIS_STATE_IDLE
            except Exception:
                pass
        try:
            ser.flush()
        except Exception:
            pass
        time.sleep(node_delay)


# =============================================================================
#  F710 reader (libusb, bypasses the kernel HID layer)
# =============================================================================
class F710:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if self.dev is None:
            raise RuntimeError(
                f'F710 ({VENDOR_ID:04x}:{PRODUCT_ID:04x}) not found. '
                'Set the slider on the back of the gamepad to "D" '
                '(DirectInput) and check the receiver is plugged in.'
            )
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        self.dev.set_configuration()
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]
        self.ep = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: (
                usb.util.endpoint_direction(e.bEndpointAddress)
                == usb.util.ENDPOINT_IN
                and usb.util.endpoint_type(e.bmAttributes)
                == usb.util.ENDPOINT_TYPE_INTR
            ),
        )
        if self.ep is None:
            raise RuntimeError('F710: no interrupt-IN endpoint found')

        self._latest = bytes(self.ep.wMaxPacketSize)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ep.read(self.ep.wMaxPacketSize, timeout=200)
                with self._lock:
                    self._latest = bytes(data)
            except usb.core.USBError as e:
                if getattr(e, 'errno', None) != 110:  # 110 = ETIMEDOUT, idle stick
                    pass

    def state(self):
        """Return (lx, ly, ry, rb_held) with deadzone applied."""
        with self._lock:
            d = self._latest
        if len(d) < 8:
            return 0.0, 0.0, 0.0, False
        lx, ly, _rx, ry = d[1], d[2], d[3], d[4]
        shoulder = d[6]

        def ax(v, invert=False):
            x = (v - 128) / 127.0
            if abs(x) < DEADZONE:
                return 0.0
            return -x if invert else x

        rb_held = bool((shoulder >> 1) & 1)
        return ax(lx, invert=True), ax(ly, invert=True), ax(ry, invert=True), rb_held

    def raw(self):
        with self._lock:
            d = self._latest
        if len(d) < 8:
            return 0, 0, 0, 0
        return d[1], d[2], d[3], d[4]  # lx, ly, rx, ry

    def buttons(self):
        """(x_held, b_held, face_byte). DInput face-button best-guess."""
        with self._lock:
            d = self._latest
        if len(d) < 8:
            return False, False, 0
        face = d[5]
        x_held = bool((face >> 4) & 1)
        b_held = bool((face >> 6) & 1)
        return x_held, b_held, face

    def stop(self):
        self._stop.set()
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:
            pass


# =============================================================================
#  Main
# =============================================================================
def main():
    print(f"Opening {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    init_waveshare(ser)

    # 1. Hard IDLE first, regardless of prior session state.
    idle_all(ser)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 2. Energize. Encoder frames are stale in IDLE on the CyberBeast,
    #    so we have to flip CLOSED_LOOP first to get fresh estimates.
    print("Energizing (CLOSED_LOOP)...")
    for node in (NODE_HIP_ABDUCT, NODE_HIP_PITCH, NODE_KNEE):
        set_axis_state(ser, node, 8)  # AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # 3. Read fresh encoder positions; these become each joint's "home".
    print("Reading current positions...")
    home = {}
    for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                       ('hip_pitch',  NODE_HIP_PITCH),
                       ('knee',       NODE_KNEE)):
        p = read_position(ser, node, timeout=3.0)
        if p is None:
            print(f"ERROR: no encoder data for {name} (node {node}). "
                  "Check that axis0.config.can.encoder_rate_ms = 10 is "
                  "saved on this ODrive (or encoder_msg_rate_ms on older "
                  "firmware). Aborting (motors -> IDLE).")
            idle_all(ser)
            ser.close()
            return 1
        home[name] = p
        print(f"  {name:11s} (node {node}): {p:+.4f} rev")

    # 4. Lock each motor at its freshly-read position. The offset
    #    accumulator only moves the target at MAX_RATE_RAD_S, so even a
    #    deflected stick at startup can't snap the leg.
    for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                       ('hip_pitch',  NODE_HIP_PITCH),
                       ('knee',       NODE_KNEE)):
        set_input_pos(ser, node, home[name])
    ser.flush()
    time.sleep(0.2)

    # 5. Open the F710 last — no point waiting for a gamepad if the motors
    #    weren't going to respond anyway.
    print("Opening F710...")
    pad = F710()

    # Hips use the analog accumulator. Knee uses discrete jogs (B/X held).
    JOINTS = [
        ('lx', SIGN_HIP_ABDUCT, GEAR_RATIO, NODE_HIP_ABDUCT, home['hip_abduct'], 'hip_abduct'),
        ('ry', SIGN_HIP_PITCH,  GEAR_RATIO, NODE_HIP_PITCH,  home['hip_pitch'],  'hip_pitch'),
    ]
    offset_rad = {j[5]: 0.0 for j in JOINTS}

    # Knee jog state. Hold B/X to move continuously at KNEE_SPEED_REV_S.
    KNEE_SPEED_REV_S = 15.0
    KNEE_UPDATE_HZ   = 30.0
    knee_step_rev    = KNEE_SPEED_REV_S / KNEE_UPDATE_HZ
    knee_period_s    = 1.0 / KNEE_UPDATE_HZ
    knee_target_rev  = home['knee']
    last_knee_send   = 0.0

    knee_lo_rad, knee_hi_rad = JOINT_LIMITS_RAD['knee']
    knee_min_rev = home['knee'] + SIGN_KNEE * GEAR_RATIO * knee_lo_rad / TAU
    knee_max_rev = home['knee'] + SIGN_KNEE * GEAR_RATIO * knee_hi_rad / TAU
    if knee_min_rev > knee_max_rev:
        knee_min_rev, knee_max_rev = knee_max_rev, knee_min_rev

    dt = 1.0 / TICK_HZ
    decay_alpha = min(1.0, dt / max(DECAY_TIME_S, 1e-3))

    # Robust shutdown: SIGINT, SIGTERM, exception path all converge here.
    shutting_down = threading.Event()

    def shutdown(*_):
        if shutting_down.is_set():
            return
        shutting_down.set()
        print("\nShutting down: idling all motors...")
        try:
            for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                               ('hip_pitch',  NODE_HIP_PITCH),
                               ('knee',       NODE_KNEE)):
                set_input_pos(ser, node, home[name])
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        idle_all(ser, retries=3, node_delay=5.0)
        try:
            pad.stop()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("Idle. Bye.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print()
    print("Hold RB to drive. Release RB to glide back to home. Ctrl-C to quit.")
    print(f"  hips: {MAX_RATE_RAD_S:.2f} rad/s   knee: {KNEE_SPEED_REV_S:.1f} motor rev/s")
    print(f"  limits: abduct ±20°, pitch −40°/+5°, knee ±90°")
    print()

    next_tick = time.monotonic()
    last_print = 0.0
    PRINT_PERIOD = 0.1
    while True:
        lx, _ly, ry, rb = pad.state()
        x_held, b_held, _face = pad.buttons()
        sticks = {'lx': lx, 'ry': ry}

        # Hips: continuous accumulator driven by analog sticks.
        for stick_key, sign, gear, node, home_rev, name in JOINTS:
            stick = sticks[stick_key]
            lo, hi = JOINT_LIMITS_RAD[name]
            if rb:
                offset_rad[name] += stick * MAX_RATE_RAD_S * dt
                offset_rad[name] = max(lo, min(hi, offset_rad[name]))
            else:
                offset_rad[name] *= (1.0 - decay_alpha)

            offset_rev = sign * gear * offset_rad[name] / TAU
            set_input_pos(ser, node, home_rev + offset_rev)

        # Knee: hold B (forward) or X (backward) to move at KNEE_SPEED_REV_S.
        knee_now = time.monotonic()
        if rb and (knee_now - last_knee_send) >= knee_period_s:
            direction = (1 if b_held else 0) - (1 if x_held else 0)
            if direction != 0:
                knee_target_rev += direction * SIGN_KNEE * knee_step_rev
                knee_target_rev = max(knee_min_rev, min(knee_max_rev, knee_target_rev))
                set_input_pos(ser, NODE_KNEE, knee_target_rev)
                last_knee_send = knee_now

        ser.flush()

        now = time.monotonic()
        if now - last_print > PRINT_PERIOD:
            rlx, _, _, rry = pad.raw()
            sys.stdout.write(
                f"\rRB={'1' if rb else '0'}  "
                f"LX={rlx:3d} RY={rry:3d}  "
                f"X={'1' if x_held else '0'} B={'1' if b_held else '0'}  "
                f"knee_tgt={knee_target_rev:+.3f}    "
            )
            sys.stdout.flush()
            last_print = now

        next_tick += dt
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
