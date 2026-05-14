"""Minimal knee-only test: tune.py-style discrete steps via the F710.

Controls node 6 (knee) ONLY. No hips, no continuous accumulator, no 30 Hz
update loop. Each rising edge of B (up) or X (down) sends exactly ONE
set_input_pos(NODE_KNEE, current_target + STEP_SIZE), exactly the way
`tune.py s 0.1` does it. ODrive's internal position loop then handles
the smooth traversal — no script-side stream of incremental commands.

Goal: verify node 6 moves cleanly with this pattern before re-introducing
the hips.

Hold RB (deadman) for any button to register. Ctrl-C idles and exits.
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
NODE_KNEE = 6

# ---- Step config ---------------------------------------------------------
STEP_REV  = 0.1   # motor rev per button press — same as `tune.py s 0.1`

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
        """(rb_held, x_held, b_held, face_byte)."""
        with self._lock:
            d = self._latest
        if len(d) < 8:
            return False, False, False, 0
        face = d[5]
        shoulder = d[6]
        rb_held = bool((shoulder >> 1) & 1)
        x_held = bool((face >> 4) & 1)
        b_held = bool((face >> 6) & 1)
        return rb_held, x_held, b_held, face

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
    print("Setting node 6 to IDLE...")
    set_axis_state(ser, NODE_KNEE, 1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("Energizing (CLOSED_LOOP)...")
    set_axis_state(ser, NODE_KNEE, 8)
    time.sleep(0.8)
    ser.reset_input_buffer()

    print("Reading current position...")
    p = read_position(ser, NODE_KNEE, timeout=3.0)
    if p is None:
        print("ERROR: no encoder data. Aborting (motor -> IDLE).")
        set_axis_state(ser, NODE_KNEE, 1)
        ser.close()
        return 1
    home_rev = p
    target_rev = p
    print(f"  node {NODE_KNEE}: {home_rev:+.4f} rev")

    set_input_pos(ser, NODE_KNEE, target_rev)
    ser.flush()
    time.sleep(0.2)

    print("Opening F710...")
    pad = F710()

    shutting_down = threading.Event()

    def shutdown(*_):
        if shutting_down.is_set():
            return
        shutting_down.set()
        print("\nShutting down: idling motor...")
        try:
            set_input_pos(ser, NODE_KNEE, home_rev)
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        for _ in range(3):
            try:
                set_axis_state(ser, NODE_KNEE, 1)
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
    print(f"Hold RB + press B  -> step +{STEP_REV} rev (up)")
    print(f"Hold RB + press X  -> step -{STEP_REV} rev (down)")
    print("Ctrl-C to quit.")
    print()

    prev_b = False
    prev_x = False
    last_print = 0.0
    PRINT_PERIOD = 0.1
    while True:
        rb, x_held, b_held, face = pad.state()

        # Rising-edge detection. STEP only fires once per press.
        if rb and b_held and not prev_b:
            target_rev += STEP_REV
            set_input_pos(ser, NODE_KNEE, target_rev)
            ser.flush()
            print(f"\n  step +{STEP_REV} -> target = {target_rev:+.4f} rev")
        if rb and x_held and not prev_x:
            target_rev -= STEP_REV
            set_input_pos(ser, NODE_KNEE, target_rev)
            ser.flush()
            print(f"\n  step -{STEP_REV} -> target = {target_rev:+.4f} rev")

        prev_b = b_held and rb
        prev_x = x_held and rb

        now = time.monotonic()
        if now - last_print > PRINT_PERIOD:
            sys.stdout.write(
                f"\rRB={'1' if rb else '0'}  "
                f"X={'1' if x_held else '0'} B={'1' if b_held else '0'} "
                f"face=0x{face:02X}  "
                f"target={target_rev:+.4f}    "
            )
            sys.stdout.flush()
            last_print = now

        time.sleep(0.02)  # 50 Hz button-edge polling


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
