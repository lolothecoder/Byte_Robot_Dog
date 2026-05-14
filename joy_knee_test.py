"""Knee + hip_pitch test: analog accumulator via F710.

Controls node 6 (knee) and node 4 (hip_pitch). Same analog-accumulator
pattern for both — every tick, the commanded position grows toward where
the user is pushing, motor tracks smoothly via ODrive's position loop.

  Hold RB (deadman). While held:
    B / X  ->  knee     (node 6)   up/down
    RY     ->  hip_pitch (node 4)  forward/back

Releasing a control leaves that joint's target where it is. Ctrl-C idles
and exits.
"""
import signal
import struct
import sys
import threading
import time

import serial
import usb.core
import usb.util

# ---- Hardware ------------------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
NODE_KNEE      = 6
NODE_HIP_PITCH = 4
SIGN_HIP_PITCH = -1.0  # flips if RY-forward drives the wrong direction

# ---- Speed config --------------------------------------------------------
KNEE_SPEED_REV_S      = 2.0   # motor rev/s while B (up) or X (down) is held
HIP_PITCH_SPEED_REV_S = 0.4   # motor rev/s at full RY deflection
DEADZONE              = 0.15
TICK_HZ               = 50.0  # update rate of the main loop

# ---- F710 USB IDs (DirectInput mode, slider on "D") ----------------------
VENDOR_ID = 0x046D
PRODUCT_ID = 0xC219


# =============================================================================
#  Waveshare CAN framing — identical to tune.py / joy_drive*.py
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


# =============================================================================
#  F710 reader (libusb, bypasses kernel HID layer)
# =============================================================================
class F710:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if self.dev is None:
            raise RuntimeError(
                f'F710 ({VENDOR_ID:04x}:{PRODUCT_ID:04x}) not found. '
                'Set the slider on the back to "D" (DirectInput).'
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
                if getattr(e, 'errno', None) != 110:  # 110 = ETIMEDOUT
                    pass

    def state(self):
        """(rb_held, x_held, b_held, ry, face_byte). ry is in [-1, 1] with
        deadzone applied; stick-forward is positive."""
        with self._lock:
            d = self._latest
        if len(d) < 8:
            return False, False, False, 0.0, 0
        face = d[5]
        shoulder = d[6]
        rb_held = bool((shoulder >> 1) & 1)
        x_held = bool((face >> 4) & 1)
        b_held = bool((face >> 6) & 1)
        ry_raw = (d[4] - 128) / 127.0
        ry = -ry_raw if abs(ry_raw) >= DEADZONE else 0.0  # invert: forward = +
        return rb_held, x_held, b_held, ry, face

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

    # IDLE → CLOSED_LOOP → read encoder → lock. Same as tune.py.
    print("Setting nodes to IDLE...")
    for node in (NODE_KNEE, NODE_HIP_PITCH):
        set_axis_state(ser, node, 1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("Energizing (CLOSED_LOOP)...")
    for node in (NODE_KNEE, NODE_HIP_PITCH):
        set_axis_state(ser, node, 8)
        time.sleep(0.3)
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Reading current positions...")
    home = {}
    for name, node in (('knee', NODE_KNEE), ('hip_pitch', NODE_HIP_PITCH)):
        p = read_position(ser, node, timeout=3.0)
        if p is None:
            print(f"ERROR: no encoder data for {name} (node {node}). "
                  "Aborting (motors -> IDLE).")
            for n in (NODE_KNEE, NODE_HIP_PITCH):
                set_axis_state(ser, n, 1)
            ser.close()
            return 1
        home[name] = p
        print(f"  {name:10s} (node {node}): {p:+.4f} rev")

    knee_target_rev = home['knee']
    hip_pitch_target_rev = home['hip_pitch']
    for node, val in ((NODE_KNEE, knee_target_rev),
                      (NODE_HIP_PITCH, hip_pitch_target_rev)):
        set_input_pos(ser, node, val)
    ser.flush()
    time.sleep(0.2)

    print("Opening F710...")
    pad = F710()

    shutting_down = threading.Event()

    def shutdown(*_):
        if shutting_down.is_set():
            return
        shutting_down.set()
        print("\nShutting down: idling motors...")
        try:
            set_input_pos(ser, NODE_KNEE, home['knee'])
            set_input_pos(ser, NODE_HIP_PITCH, home['hip_pitch'])
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        for node in (NODE_KNEE, NODE_HIP_PITCH):
            for _ in range(3):
                try:
                    set_axis_state(ser, node, 1)
                except Exception:
                    pass
            try:
                ser.flush()
            except Exception:
                pass
            time.sleep(0.05)
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
    print(f"Hold RB + B / X     -> knee up/down at {KNEE_SPEED_REV_S} motor rev/s")
    print(f"Hold RB + RY        -> hip_pitch fwd/back at {HIP_PITCH_SPEED_REV_S} motor rev/s")
    print("Release: targets hold. Ctrl-C to quit.")
    print()

    dt = 1.0 / TICK_HZ
    knee_step      = KNEE_SPEED_REV_S * dt        # motor rev per tick at full speed
    hip_pitch_step = HIP_PITCH_SPEED_REV_S * dt   # motor rev per tick at full deflection
    next_tick = time.monotonic()
    last_print = 0.0
    PRINT_PERIOD = 0.1
    while True:
        rb, x_held, b_held, ry, face = pad.state()

        # Analog-accumulator pattern for both joints.
        if rb:
            knee_dir = (1 if b_held else 0) - (1 if x_held else 0)
            knee_target_rev += knee_dir * knee_step
            hip_pitch_target_rev += SIGN_HIP_PITCH * ry * hip_pitch_step

        set_input_pos(ser, NODE_KNEE, knee_target_rev)
        set_input_pos(ser, NODE_HIP_PITCH, hip_pitch_target_rev)
        ser.flush()

        now = time.monotonic()
        if now - last_print > PRINT_PERIOD:
            sys.stdout.write(
                f"\rRB={'1' if rb else '0'}  "
                f"X={'1' if x_held else '0'} B={'1' if b_held else '0'} "
                f"RY={ry:+.2f}  "
                f"knee={knee_target_rev:+.4f} "
                f"pitch={hip_pitch_target_rev:+.4f}    "
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
