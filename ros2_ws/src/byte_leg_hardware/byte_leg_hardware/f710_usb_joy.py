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

        # F710 DirectInput report (observed on this unit):
        #   0: report ID (always 0x01)
        #   1: LX  (0..255, center 128, right = 0xFF)
        #   2: LY  (0..255, center 128, up = 0x00)
        #   3: RX  (0..255, center 128)
        #   4: RY  (0..255, center 128, up = 0x00)
        #   5: low nibble = hat (0..7 CW from N, 8 = released)
        #      high nibble = face buttons: X bit4, A bit5, B bit6, Y bit7
        #   6: shoulder/stick: LB(0) RB(1) LT(2) RT(3)
        #                      BACK(4) START(5) LStick(6) RStick(7)
        #   7: battery / status (ignored)
        _id, lx, ly, rx, ry, hat_face, shoulder, _ = struct.unpack(
            'BBBBBBBB', data[:8]
        )

        def ax(v: int, invert: bool = False) -> float:
            x = (v - 128) / 127.0
            return -x if invert else x

        axes = [
            ax(lx),                     # 0  left X  (right +)
            ax(ly, invert=True),        # 1  left Y  (up +)
            0.0,                        # 2  LT (digital on F710)
            ax(rx),                     # 3  right X
            ax(ry, invert=True),        # 4  right Y
            0.0,                        # 5  RT (digital)
            0.0,                        # 6  D-pad X
            0.0,                        # 7  D-pad Y
        ]
        hat = hat_face & 0x0F
        if hat != 0x08:
            # CW from N: 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW
            dx_lookup = (0, 1, 1, 1, 0, -1, -1, -1)
            dy_lookup = (1, 1, 0, -1, -1, -1, 0, 1)
            axes[6] = float(dx_lookup[hat & 0x07])
            axes[7] = float(dy_lookup[hat & 0x07])

        # xpad-style button order so joy_teleop's defaults
        # (button_home=0=A, button_enable=5=RB) match without remapping.
        face_X = (hat_face >> 4) & 1
        face_A = (hat_face >> 5) & 1
        face_B = (hat_face >> 6) & 1
        face_Y = (hat_face >> 7) & 1
        buttons = [
            face_A, face_B, face_X, face_Y,
            (shoulder >> 0) & 1,        # 4  LB
            (shoulder >> 1) & 1,        # 5  RB
            (shoulder >> 4) & 1,        # 6  BACK
            (shoulder >> 5) & 1,        # 7  START
            0,                          # 8  LOGO placeholder (no button on F710)
            (shoulder >> 6) & 1,        # 9  LStick
            (shoulder >> 7) & 1,        # 10 RStick
            (shoulder >> 2) & 1,        # 11 LT (digital)
            (shoulder >> 3) & 1,        # 12 RT (digital)
        ]

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
