from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rowboboat_control"),
            "config",
            "paddle_wheel.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name='joint_state_broadcaster_spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name='left_controller_spawner',
        arguments=["left_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name='right_controller_spawner',
        arguments=["right_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name='diff_drive_controller_spawner',
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    nodes = [        
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
#        left_controller_spawner,
#        right_controller_spawner,
    ]

    return LaunchDescription(nodes)