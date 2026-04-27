"""Cartesian foot teleop with RB-as-deadman.

Hold RB and push the sticks: stick deflection sets *velocity* of the foot
in base_link frame. Left stick X/Y drives foot X (forward) / Y (lateral),
right stick Y drives foot Z (height). When RB is released the foot decays
smoothly back to home_pose. A is an instant home reset.
"""
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__('joy_teleop')

        self.declare_parameter('speed_xy', 0.20)        # m/s at full stick
        self.declare_parameter('speed_z', 0.15)         # m/s at full stick
        self.declare_parameter('decay_time', 0.3)       # sec to return ~63% home
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('home_pose', [0.0, 0.0, -0.55])
        self.declare_parameter('workspace_xy', 0.15)    # ± m from home
        self.declare_parameter('workspace_z', 0.10)     # ± m from home
        self.declare_parameter('axis_forward', 1)
        self.declare_parameter('axis_lateral', 0)
        self.declare_parameter('axis_vertical', 4)
        self.declare_parameter('button_home', 0)
        self.declare_parameter('button_enable', 5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('target_topic', '/foot/target_point')

        self._home = list(self.get_parameter('home_pose').value)
        self._offset = [0.0, 0.0, 0.0]
        self._enable_held = False
        self._last_stick = (0.0, 0.0, 0.0)
        self._reset_request = False

        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self._pub = self.create_publisher(
            PointStamped,
            self.get_parameter('target_topic').value,
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

    @staticmethod
    def _dead(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    def _on_joy(self, msg: Joy) -> None:
        axis_fwd = int(self.get_parameter('axis_forward').value)
        axis_lat = int(self.get_parameter('axis_lateral').value)
        axis_ver = int(self.get_parameter('axis_vertical').value)
        btn_home = int(self.get_parameter('button_home').value)
        btn_enable = int(self.get_parameter('button_enable').value)
        dz = float(self.get_parameter('deadzone').value)

        if len(msg.buttons) > btn_home and msg.buttons[btn_home] == 1:
            self._reset_request = True

        self._enable_held = (
            len(msg.buttons) > btn_enable
            and msg.buttons[btn_enable] == 1
        )

        def axis(i: int) -> float:
            return msg.axes[i] if i < len(msg.axes) else 0.0

        self._last_stick = (
            self._dead(axis(axis_fwd), dz),
            self._dead(axis(axis_lat), dz),
            self._dead(axis(axis_ver), dz),
        )

    def _tick(self) -> None:
        if self._reset_request:
            self._offset = [0.0, 0.0, 0.0]
            self._reset_request = False

        speed_xy = float(self.get_parameter('speed_xy').value)
        speed_z = float(self.get_parameter('speed_z').value)
        ws_xy = float(self.get_parameter('workspace_xy').value)
        ws_z = float(self.get_parameter('workspace_z').value)

        if self._enable_held:
            fwd, lat, ver = self._last_stick
            # REP-103: x forward, y left, z up. Joy: LY+ = up = forward,
            # LX+ = right = -y, RY+ = up = +z.
            self._offset[0] += fwd * speed_xy * self._dt
            self._offset[1] -= lat * speed_xy * self._dt
            self._offset[2] += ver * speed_z * self._dt
            self._offset[0] = max(-ws_xy, min(ws_xy, self._offset[0]))
            self._offset[1] = max(-ws_xy, min(ws_xy, self._offset[1]))
            self._offset[2] = max(-ws_z, min(ws_z, self._offset[2]))
        else:
            decay = float(self.get_parameter('decay_time').value)
            alpha = min(1.0, self._dt / max(decay, 1e-3))
            self._offset = [v * (1.0 - alpha) for v in self._offset]

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.point.x = self._home[0] + self._offset[0]
        msg.point.y = self._home[1] + self._offset[1]
        msg.point.z = self._home[2] + self._offset[2]
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
