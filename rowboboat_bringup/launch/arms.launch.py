from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    left_piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        namespace="left",
        parameters=[{
            'can_port': 'arm_left',
            'auto_enable': True,
            'gripper_exist': False,
            'arm_name': 'left_',
        }],
        remappings=[
            ('joint_states_single', '/joint_states')
        ]
    )

    right_piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        namespace="right",
        parameters=[{
            'can_port': 'arm_right',
            'auto_enable': True,
            'gripper_exist': False,
            'arm_name': 'right_',
        }],
        remappings=[
            ('joint_states_single', '/joint_states')
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        left_piper_node,
        right_piper_node
    ])
