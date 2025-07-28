from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper node.'
    )

    left_piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        namespace="left",
        parameters=[{
            'can_port': 'can0',
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_exist': 'false',
            'arm_name': 'left_',
        }],
    )

    right_piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        namespace="right",
        parameters=[{
            'can_port': 'can1',
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_exist': 'false',
            'arm_name': 'right_',
        }],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        left_piper_node,
        right_piper_node
    ])
