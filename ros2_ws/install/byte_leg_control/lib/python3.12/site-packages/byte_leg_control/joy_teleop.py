import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped


class JoyTeleop(Node):
    """Drive a stepping gait for the foot based on Xbox stick input.

    Holding the left stick in a direction advances a gait phase so the foot
    traces a stepping cycle in that direction (forward-swing -> lift ->
    back-stance). The foot keeps cycling for as long as the stick is held
    (and the RB deadman is pressed), giving a "walking" feel rather than a
    fixed target that hits a workspace wall.

    Right stick Y adjusts standing height (home z offset), integrated.
    A resets phase and height offset. RB (button 5) is the deadman.
    """

    def __init__(self) -> None:
        super().__init__('joy_teleop')

        self.declare_parameter('step_amplitude', 0.10)
        self.declare_parameter('step_height', 0.06)
        self.declare_parameter('step_frequency', 1.2)
        self.declare_parameter('height_speed', 0.15)
        self.declare_parameter('decay_time', 0.4)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('home_pose', [0.0, 0.0, -0.55])
        self.declare_parameter('height_min', -0.15)
        self.declare_parameter('height_max', 0.15)
        self.declare_parameter('axis_forward', 1)
        self.declare_parameter('axis_lateral', 0)
        self.declare_parameter('axis_vertical', 4)
        self.declare_parameter('button_home', 0)
        self.declare_parameter('button_enable', 5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('target_topic', '/foot/target_point')

        self._home = list(self.get_parameter('home_pose').value)
        self._phase = 0.0
        self._height_offset = 0.0
        # Last-applied cartesian offset from home. Used to decay smoothly
        # back to home when the stick is released.
        self._offset = [0.0, 0.0, 0.0]
        self._enable_held = False
        self._last_stick = (0.0, 0.0, 0.0)

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
        if abs(value) < threshold:
            return 0.0
        return value

    def _on_joy(self, msg: Joy) -> None:
        axis_fwd = int(self.get_parameter('axis_forward').value)
        axis_lat = int(self.get_parameter('axis_lateral').value)
        axis_ver = int(self.get_parameter('axis_vertical').value)
        btn_home = int(self.get_parameter('button_home').value)
        btn_enable = int(self.get_parameter('button_enable').value)
        dz = float(self.get_parameter('deadzone').value)

        if len(msg.buttons) > btn_home and msg.buttons[btn_home] == 1:
            self._phase = 0.0
            self._height_offset = 0.0

        self._enable_held = (len(msg.buttons) > btn_enable
                             and msg.buttons[btn_enable] == 1)

        def axis(i: int) -> float:
            return msg.axes[i] if i < len(msg.axes) else 0.0

        self._last_stick = (
            self._dead(axis(axis_fwd), dz),
            self._dead(axis(axis_lat), dz),
            self._dead(axis(axis_ver), dz),
        )

    def _tick(self) -> None:
        fwd, lat, ver = self._last_stick
        active = self._enable_held and math.hypot(fwd, lat) > 0.0

        if self._enable_held:
            hs = float(self.get_parameter('height_speed').value)
            self._height_offset += ver * hs * self._dt
            h_min = float(self.get_parameter('height_min').value)
            h_max = float(self.get_parameter('height_max').value)
            self._height_offset = max(h_min, min(h_max, self._height_offset))

        if active:
            freq = float(self.get_parameter('step_frequency').value)
            amp = float(self.get_parameter('step_amplitude').value)
            lift = float(self.get_parameter('step_height').value)

            self._phase = (self._phase + 2.0 * math.pi * freq * self._dt) \
                % (2.0 * math.pi)

            mag = math.hypot(fwd, lat)
            dir_x = fwd / mag
            dir_y = lat / mag
            scale = min(mag, 1.0)

            step_x = -amp * math.cos(self._phase) * dir_x * scale
            step_y = -amp * math.cos(self._phase) * dir_y * scale
            step_z = lift * max(0.0, math.sin(self._phase)) * scale

            self._offset = [step_x, step_y, step_z]
        else:
            decay_time = float(self.get_parameter('decay_time').value)
            # Critically-damped-ish exponential decay toward 0 each tick.
            alpha = min(1.0, self._dt / max(decay_time, 1e-3))
            self._offset = [v * (1.0 - alpha) for v in self._offset]
            # Freeze phase so it resumes smoothly when the stick returns.

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.point.x = self._home[0] + self._offset[0]
        msg.point.y = self._home[1] + self._offset[1]
        msg.point.z = self._home[2] + self._height_offset + self._offset[2]
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
