import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from byte_leg_control.kinematics import LegGeometry, inverse_kinematics


class IKNode(Node):
    def __init__(self) -> None:
        super().__init__('ik_node')
        self.declare_parameter('thigh_length', 0.30)
        self.declare_parameter('shank_length', 0.30)
        self.declare_parameter('controller_topic',
                               '/leg_controller/joint_trajectory')
        self.declare_parameter('target_topic', '/foot/target_point')
        self.declare_parameter('time_from_start', 0.05)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.geom = LegGeometry(
            thigh_length=self.get_parameter('thigh_length').value,
            shank_length=self.get_parameter('shank_length').value,
        )
        self.time_from_start = float(
            self.get_parameter('time_from_start').value)

        self._latest_target: tuple[float, float, float] | None = None

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_topic').value,
            self._on_target,
            10,
        )
        self._pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter('controller_topic').value,
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

    def _on_target(self, msg: PointStamped) -> None:
        self._latest_target = (msg.point.x, msg.point.y, msg.point.z)

    def _tick(self) -> None:
        if self._latest_target is None:
            return
        x, y, z = self._latest_target
        q_haa, q_hp, q_k = inverse_kinematics(x, y, z, self.geom)

        traj = JointTrajectory()
        traj.joint_names = ['hip_abduct', 'hip_pitch', 'knee']
        point = JointTrajectoryPoint()
        point.positions = [q_haa, q_hp, q_k]
        point.time_from_start = Duration(
            seconds=self.time_from_start).to_msg()
        traj.points = [point]
        self._pub.publish(traj)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
