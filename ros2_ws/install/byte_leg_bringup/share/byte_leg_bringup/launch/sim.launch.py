"""Top-level launch for the byte_leg Gazebo simulation.

Brings up gz sim, spawns the xacro-expanded URDF, starts ros2_control
spawners, bridges /clock, and launches the joy_node + teleop + IK pipeline.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory('byte_leg_description')
    control_share = get_package_share_directory('byte_leg_control')
    gazebo_share = get_package_share_directory('byte_leg_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(description_share, 'urdf', 'leg.urdf.xacro')
    controllers_yaml = os.path.join(control_share, 'config', 'controllers.yaml')
    teleop_yaml = os.path.join(control_share, 'config', 'teleop.yaml')
    world_file = os.path.join(gazebo_share, 'worlds', 'bench.sdf')
    bridge_yaml = os.path.join(gazebo_share, 'config', 'bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo.'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Run Gazebo with the GUI (false => headless).'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' controllers_config:=', controllers_yaml,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    gz_args = PythonExpression([
        "'-r ", world_file,
        " -v 2' + (' -s --headless-rendering' if '",
        gui, "' == 'false' else '')"
    ])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'byte_leg',
            '-z', '0.0',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_yaml}',
        ],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[teleop_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    teleop_node = Node(
        package='byte_leg_control',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[teleop_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    ik_node = Node(
        package='byte_leg_control',
        executable='ik_node',
        name='ik_node',
        parameters=[teleop_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Sequence: spawn_entity (robot in Gazebo) -> joint_state_broadcaster ->
    # leg_controller + teleop/IK/joy. gz_ros2_control sets up the hardware
    # interfaces from the URDF, but the controllers themselves only exist
    # once the external spawner loads them.
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner],
        ),
    )
    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[leg_controller_spawner, joy_node, teleop_node, ik_node],
        ),
    )

    return LaunchDescription([
        declare_sim_time,
        declare_gui,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        clock_bridge,
        after_spawn,
        after_jsb,
    ])
