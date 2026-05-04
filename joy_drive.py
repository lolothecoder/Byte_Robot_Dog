"""Bench joystick teleop — three motors, RB-as-deadman, no ROS.

Runs on the Jetson with the F710 in DirectInput mode (slider on the back
set to "D") and the Waveshare USB-CAN adapter on /dev/ttyUSB0.

  Hold RB to enable. While held:
    LX (axis 0)  ->  hip_abduct  (node 3)
    LY (axis 1)  ->  knee        (node 1)
    RY (axis 4)  ->  hip_pitch   (node 5)

  Release RB: each joint smoothly decays back to its startup position.
  Ctrl-C: every motor is IDLE'd (with retries, so a single dropped
  serial write doesn't leave node 5 hot — which sometimes happens with
  tune.py / small_walk.py).

Startup sequence (matches tune.py and the can_relay arm path):
  IDLE  ->  CLOSED_LOOP  ->  read fresh encoder  ->  SET_INPUT_POS to it
The CyberBeast's encoder frames are stale while in IDLE, so the read
must happen post-energize to get the real rotor position. Once locked
to the freshly-read position, the offset accumulator never moves the
target faster than MAX_RATE_RAD_S, so a stick that's already deflected
when you arm doesn't snap the leg.

Shutdown sends IDLE 3× with explicit flushes — fixes the recurring
"Ctrl-C didn't idle node 5" symptom that tune.py and small_walk.py have.

Dependencies: pyserial, pyusb. Already installed if f710_usb_joy works.
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

# ---- Hardware constants (mirror byte_leg_hardware/config/hardware.yaml) -----
PORT = "/dev/ttyUSB0"
BAUD = 2_000_000

NODE_HIP_ABDUCT = 3
NODE_HIP_PITCH = 5
NODE_KNEE = 1

GEAR_RATIO = 8.0  # GIM8010-8 = 8:1 motor revs per joint rev
SIGN_HIP_ABDUCT = 1.0
SIGN_HIP_PITCH = -1.0  # reversed vs URDF axis
SIGN_KNEE = 1.0

# ---- Teleop tuning ---------------------------------------------------------
MAX_RATE_RAD_S = 0.4   # joint rad/s at full stick deflection
MAX_OFFSET_RAD = 0.5   # cap accumulated offset per joint (workspace clamp)
DECAY_TIME_S = 0.3     # ~63% return to home per this many seconds when RB is released
DEADZONE = 0.15
TICK_HZ = 50.0

# ---- F710 USB IDs (DirectInput mode) ---------------------------------------
VENDOR_ID = 0x046D
PRODUCT_ID = 0xC219

TAU = 2.0 * math.pi


# =============================================================================
#  Waveshare CAN framing — identical bytes-on-the-wire as tune.py / small_walk.py
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
    """Block until we get `min_readings` encoder broadcasts from `node_id`.

    Requires axis0.config.can.encoder_msg_rate_ms = 10 saved on the ODrive
    so the cyclic broadcast fires even in IDLE.
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


def idle_all(ser, retries=3):
    """Send IDLE to every node, multiple times, with flushes.

    Belt-and-suspenders: the symptom of "Ctrl-C didn't idle node 5" is
    one frame getting eaten; retrying is cheap and decisive.
    """
    for _ in range(retries):
        for node in (NODE_HIP_ABDUCT, NODE_HIP_PITCH, NODE_KNEE):
            try:
                set_axis_state(ser, node, 1)  # AXIS_STATE_IDLE
            except Exception:
                pass
        try:
            ser.flush()
        except Exception:
            pass
        time.sleep(0.05)


# =============================================================================
#  F710 reader (libusb, bypasses the kernel HID layer like f710_usb_joy.py)
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
                # 110 = ETIMEDOUT, expected when the stick is idle
                if getattr(e, 'errno', None) != 110:
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
        return ax(lx), ax(ly, invert=True), ax(ry, invert=True), rb_held

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

    # 3. Read fresh encoder values now that the motors are emitting
    #    real-time estimates.
    print("Reading current positions...")
    home = {}
    for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                       ('hip_pitch',  NODE_HIP_PITCH),
                       ('knee',       NODE_KNEE)):
        p = read_position(ser, node, timeout=3.0)
        if p is None:
            print(f"ERROR: no encoder data for {name} (node {node}). "
                  "Check that axis0.config.can.encoder_msg_rate_ms = 10 "
                  "is saved on this ODrive. Aborting (motors -> IDLE).")
            idle_all(ser)
            ser.close()
            return 1
        home[name] = p
        print(f"  {name:11s} (node {node}): {p:+.4f} rev")

    # 4. Lock each motor at the freshly-read position. From here, the
    #    offset accumulator only moves the target at MAX_RATE_RAD_S,
    #    so even a deflected stick at startup can't snap the leg.
    for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                       ('hip_pitch',  NODE_HIP_PITCH),
                       ('knee',       NODE_KNEE)):
        set_input_pos(ser, node, home[name])
    ser.flush()
    time.sleep(0.2)

    # 5. Open the F710 last, so we don't burn 30 s waiting for a missing
    #    gamepad just to discover the motors weren't going to respond.
    print("Opening F710...")
    pad = F710()

    # Per-joint config: (stick_key, sign, gear, node, home_rev, name).
    JOINTS = [
        ('lx', SIGN_HIP_ABDUCT, GEAR_RATIO, NODE_HIP_ABDUCT, home['hip_abduct'], 'hip_abduct'),
        ('ly', SIGN_KNEE,       GEAR_RATIO, NODE_KNEE,       home['knee'],       'knee'),
        ('ry', SIGN_HIP_PITCH,  GEAR_RATIO, NODE_HIP_PITCH,  home['hip_pitch'],  'hip_pitch'),
    ]
    offset_rad = {j[5]: 0.0 for j in JOINTS}

    dt = 1.0 / TICK_HZ
    decay_alpha = min(1.0, dt / max(DECAY_TIME_S, 1e-3))

    # Robust shutdown: SIGINT, SIGTERM, exception path all converge here.
    shutting_down = threading.Event()

    def shutdown(*_):
        if shutting_down.is_set():
            return
        shutting_down.set()
        print("\nShutting down: idling all motors...")
        # First, snap commanded position back to home so the controller
        # is at rest before we cut the loop.
        try:
            for name, node in (('hip_abduct', NODE_HIP_ABDUCT),
                               ('hip_pitch',  NODE_HIP_PITCH),
                               ('knee',       NODE_KNEE)):
                set_input_pos(ser, node, home[name])
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        idle_all(ser, retries=4)
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
    print(f"  speed: {MAX_RATE_RAD_S:.2f} rad/s   workspace: ±{MAX_OFFSET_RAD:.2f} rad")
    print()

    next_tick = time.monotonic()
    while True:
        lx, ly, ry, rb = pad.state()
        sticks = {'lx': lx, 'ly': ly, 'ry': ry}

        for stick_key, sign, gear, node, home_rev, name in JOINTS:
            stick = sticks[stick_key]
            if rb:
                offset_rad[name] += stick * MAX_RATE_RAD_S * dt
                offset_rad[name] = max(-MAX_OFFSET_RAD,
                                       min(MAX_OFFSET_RAD, offset_rad[name]))
            else:
                offset_rad[name] *= (1.0 - decay_alpha)

            offset_rev = sign * gear * offset_rad[name] / TAU
            set_input_pos(ser, node, home_rev + offset_rev)

        ser.flush()

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
        # The signal handler should have run; this is just defense in depth.
        pass
