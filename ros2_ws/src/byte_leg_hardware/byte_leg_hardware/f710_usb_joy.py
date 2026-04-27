"""Userspace USB-to-Joy bridge for the Logitech F710 in DirectInput mode.

Bypasses the kernel HID layer entirely — needed on Tegra kernels where
hid-logitech fails to probe and no /dev/input/jsX appears. Reads HID
interrupt reports directly via libusb (pyusb) and republishes them as
sensor_msgs/Joy on /joy, so the rest of the stack (joy_teleop, ik_node,
can_relay) is unchanged.
"""
import struct
import threading

import rclpy
import usb.core
import usb.util
from rclpy.node import Node
from sensor_msgs.msg import Joy

VENDOR_ID = 0x046D
PRODUCT_ID = 0xC219


class F710Bridge(Node):
    def __init__(self) -> None:
        super().__init__('f710_usb_joy')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('log_raw', False)

        self._pub = self.create_publisher(Joy, '/joy', 10)

        self._dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if self._dev is None:
            raise RuntimeError(
                f'F710 ({VENDOR_ID:04x}:{PRODUCT_ID:04x}) not found on USB. '
                'Ensure the receiver is plugged in and the slider on the back '
                'of the gamepad is set to "D" (DirectInput).'
            )

        try:
            if self._dev.is_kernel_driver_active(0):
                self._dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass

        self._dev.set_configuration()
        cfg = self._dev.get_active_configuration()
        intf = cfg[(0, 0)]
        self._ep = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: (
                usb.util.endpoint_direction(e.bEndpointAddress)
                == usb.util.ENDPOINT_IN
                and usb.util.endpoint_type(e.bmAttributes)
                == usb.util.ENDPOINT_TYPE_INTR
            ),
        )
        if self._ep is None:
            raise RuntimeError('No interrupt-IN endpoint found on F710.')

        self._latest = bytes(self._ep.wMaxPacketSize)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f'F710 bridge online ({VENDOR_ID:04x}:{PRODUCT_ID:04x}), '
            f'publishing /joy @ {rate:.0f} Hz'
        )

    def _read_loop(self) -> None:
        log_raw = bool(self.get_parameter('log_raw').value)
        while not self._stop.is_set():
            try:
                data = self._ep.read(self._ep.wMaxPacketSize, timeout=200)
                with self._lock:
                    self._latest = bytes(data)
                if log_raw:
                    self.get_logger().info(
                        ' '.join(f'{b:02x}' for b in data)
                    )
            except usb.core.USBError as e:
                # 110 = ETIMEDOUT, expected when stick is idle on some firmwares
                if getattr(e, 'errno', None) != 110:
                    self.get_logger().warning(f'USB read error: {e}')

    def _publish(self) -> None:
        with self._lock:
            data = self._latest
        if len(data) < 8:
            return

        # F710 DirectInput report (8 bytes, observed):
        #   0: LX  (0..255, center 128)
        #   1: LY  (0..255, center 128, 0 = up)
        #   2: RX  (0..255, center 128)
        #   3: RY  (0..255, center 128, 0 = up)
        #   4: hat (low nibble: 0..7 = dir CW from N, 8 = released)
        #   5: buttons_lo  (X=0, A=1, B=2, Y=3, LB=4, RB=5, LT=6, RT=7)
        #   6: buttons_hi  (BACK=0, START=1, LStick=2, RStick=3)
        #   7: reserved
        lx, ly, rx, ry, hat, b_lo, b_hi, _ = struct.unpack('BBBBBBBB', data[:8])

        def ax(v: int, invert: bool = False) -> float:
            x = (v - 128) / 127.0
            return -x if invert else x

        # Match the axis layout the joy package emits by default
        # (axis 0 = LX right=+, 1 = LY up=+, 3 = RX right=+, 4 = RY up=+),
        # and place D-pad on axes 6/7 to match xpad-style layout.
        axes = [
            ax(lx),                     # 0  left X  (right +)
            ax(ly, invert=True),        # 1  left Y  (up +)
            0.0,                        # 2  LT (analog) — F710 reports digital, leave 0
            ax(rx),                     # 3  right X (right +)
            ax(ry, invert=True),        # 4  right Y (up +)
            0.0,                        # 5  RT (analog) — same caveat
            0.0,                        # 6  D-pad X
            0.0,                        # 7  D-pad Y
        ]
        if hat != 0x08:
            # Hat directions, CW from N: 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW
            dx_lookup = (0, 1, 1, 1, 0, -1, -1, -1)
            dy_lookup = (1, 1, 0, -1, -1, -1, 0, 1)
            axes[6] = float(dx_lookup[hat & 0x07])
            axes[7] = float(dy_lookup[hat & 0x07])

        # Buttons: 8 from b_lo, 4 from b_hi, then 0-pad to 11 to match xpad's
        # default button count (the LT/RT digital bits are already in b_lo).
        buttons_packed = b_lo | (b_hi << 8)
        buttons = [(buttons_packed >> i) & 1 for i in range(12)]

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        msg.axes = axes
        msg.buttons = buttons
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            usb.util.dispose_resources(self._dev)
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = F710Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
