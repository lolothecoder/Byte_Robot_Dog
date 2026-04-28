"""Direct per-joint teleop, bypassing IK.

Useful for validating one motor at a time. Stick mapping:

  Left stick forward/back  (LY, axis 1) -> knee        (node 1)
  Right stick forward/back (RY, axis 4) -> hip_pitch   (node 5)
  Left stick left/right    (LX, axis 0) -> hip_abduct  (node 3)

Hold RB to enable motion; release to decay each joint back to 0.
A: instant snap to zero.
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JoyPerJoint(Node):
    def __init__(self) -> None:
        super().__init__('joy_per_joint')

        self.declare_parameter('speed_rad_s', 1.0)         # rad/s at full stick
        self.declare_parameter('max_offset_rad', 1.6)      # cap per joint (bridge clamps each one to its own max_joint_rad anyway)
        self.declare_parameter('decay_time', 0.3)
        self.declare_parameter('deadzone', 0.15)
        # SWAPPED for diagnostic: hip_pitch on left stick, knee on right stick.
        # If hip_pitch now responds to the left stick, the issue is with the
        # right stick / axis 4. If it still doesn't, the issue is the motor.
        self.declare_parameter('axis_knee', 4)         # RY (was 1)
        self.declare_parameter('axis_hip_pitch', 1)    # LY (was 4)
        self.declare_parameter('axis_hip_abduct', 0)   # LX (unchanged)
        self.declare_parameter('button_enable', 5)
        self.declare_parameter('button_home', 0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('time_from_start', 0.05)
        self.declare_parameter('controller_topic',
                               '/leg_controller/joint_trajectory')

        self._offsets = {'hip_abduct': 0.0, 'hip_pitch': 0.0, 'knee': 0.0}
        self._stick = {'hip_abduct': 0.0, 'hip_pitch': 0.0, 'knee': 0.0}
        self._enable_held = False
        self._reset_request = False

        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self._pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter('controller_topic').value,
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

    @staticmethod
    def _dead(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    def _on_joy(self, msg: Joy) -> None:
        ax_k = int(self.get_parameter('axis_knee').value)
        ax_hp = int(self.get_parameter('axis_hip_pitch').value)
        ax_ha = int(self.get_parameter('axis_hip_abduct').value)
        btn_enable = int(self.get_parameter('button_enable').value)
        btn_home = int(self.get_parameter('button_home').value)
        dz = float(self.get_parameter('deadzone').value)

        if len(msg.buttons) > btn_home and msg.buttons[btn_home] == 1:
            self._reset_request = True

        self._enable_held = (
            len(msg.buttons) > btn_enable
            and msg.buttons[btn_enable] == 1
        )

        def axis(i: int) -> float:
            return msg.axes[i] if i < len(msg.axes) else 0.0

        self._stick = {
            'knee': self._dead(axis(ax_k), dz),
            'hip_pitch': self._dead(axis(ax_hp), dz),
            'hip_abduct': self._dead(axis(ax_ha), dz),
        }

    def _tick(self) -> None:
        if self._reset_request:
            for k in self._offsets:
                self._offsets[k] = 0.0
            self._reset_request = False

        speed = float(self.get_parameter('speed_rad_s').value)
        max_off = float(self.get_parameter('max_offset_rad').value)

        if self._enable_held:
            for k in self._offsets:
                self._offsets[k] += self._stick[k] * speed * self._dt
                self._offsets[k] = max(-max_off, min(max_off, self._offsets[k]))
        else:
            decay = float(self.get_parameter('decay_time').value)
            alpha = min(1.0, self._dt / max(decay, 1e-3))
            for k in self._offsets:
                self._offsets[k] *= (1.0 - alpha)

        traj = JointTrajectory()
        traj.joint_names = ['hip_abduct', 'hip_pitch', 'knee']
        point = JointTrajectoryPoint()
        point.positions = [
            self._offsets['hip_abduct'],
            self._offsets['hip_pitch'],
            self._offsets['knee'],
        ]
        point.time_from_start = Duration(
            seconds=float(self.get_parameter('time_from_start').value)
        ).to_msg()
        traj.points = [point]
        self._pub.publish(traj)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyPerJoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
