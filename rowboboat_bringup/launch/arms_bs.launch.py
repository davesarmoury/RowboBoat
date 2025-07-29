from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    left_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='left_relay_node',
        output='screen',
        namespace="left",
        arguments=['joint_ctrl_single', '/joint_states']
    )

    right_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='right_relay_node',
        output='screen',
        namespace="right",
        arguments=['joint_ctrl_single', '/joint_states']
    )

    # Return the LaunchDescription
    return LaunchDescription([
        left_relay_node,
        right_relay_node
    ])
