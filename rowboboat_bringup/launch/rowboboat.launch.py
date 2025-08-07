from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    arms_launch = IncludeLaunchDescription(PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_bringup'), 'launch']), 'arms.launch.py']))

    desc_launch = TimerAction(period=10.0, actions=[IncludeLaunchDescription(PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_description'), 'launch']), 'description.launch.py']))])
    control_launch = TimerAction(period=10.0, actions=[IncludeLaunchDescription(PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_control'), 'launch']), 'arms_control.launch.py']))])
    teleop_launch = TimerAction(period=10.0, actions=[IncludeLaunchDescription(PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_bringup'), 'launch']), 'teleop.launch.py']))])

    # Return the LaunchDescription
    return LaunchDescription([
        desc_launch,
        arms_launch,
        control_launch,
        teleop_launch,
    ])

