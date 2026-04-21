"""Top-level launch for the real byte_leg hardware (no Gazebo).

Defaults are intentionally safe: `dry_run:=true` so motors stay IDLE and
the CAN bridge only reads encoders + logs what it *would* send. To
actually drive the leg, relaunch with `dry_run:=false` and explicitly
call `/real_leg/arm`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory('byte_leg_description')
    control_share = get_package_share_directory('byte_leg_control')
    hardware_share = get_package_share_directory('byte_leg_hardware')

    xacro_file = os.path.join(description_share, 'urdf', 'leg.urdf.xacro')
    teleop_yaml = os.path.join(control_share, 'config', 'teleop.yaml')
    hardware_yaml = os.path.join(hardware_share, 'config', 'hardware.yaml')

    dry_run = LaunchConfiguration('dry_run')
    port = LaunchConfiguration('port')
    rviz = LaunchConfiguration('rviz')

    declare_dry_run = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='If true, can_relay reads encoders + logs only; no '
                    'position or state frames are written to the bus.'
    )
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial device for the Waveshare USB-CAN adapter.'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with the leg description.'
    )

    # robot_description is useful for RViz. Reuse the sim xacro; the
    # controllers_config arg has no effect when gz_ros2_control isn't
    # loaded, but xacro requires it to resolve the $(arg ...) call.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' controllers_config:=',
            os.path.join(control_share, 'config', 'controllers.yaml'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[teleop_yaml], output='screen',
    )
    teleop_node = Node(
        package='byte_leg_control', executable='joy_teleop',
        name='joy_teleop', parameters=[teleop_yaml], output='screen',
    )
    ik_node = Node(
        package='byte_leg_control', executable='ik_node',
        name='ik_node', parameters=[teleop_yaml], output='screen',
    )
    can_relay = Node(
        package='byte_leg_hardware', executable='can_relay',
        name='can_relay',
        parameters=[hardware_yaml, {'dry_run': dry_run, 'port': port}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='log',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_dry_run, declare_port, declare_rviz,
        robot_state_publisher,
        joy_node, teleop_node, ik_node,
        can_relay,
        rviz_node,
    ])
