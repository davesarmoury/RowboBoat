from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    desc_launch = IncludeLaunchDescription(
        PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_description'), 'launch']), 'description.launch.py'])
    )

    arms_launch = IncludeLaunchDescription(
        PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_bringup'), 'launch']), 'arms_bs.launch.py'])
    )

    control_launch = IncludeLaunchDescription(
        PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_control'), 'launch']), 'arms_control.launch.py'])
    )

    viz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([PathJoinSubstitution([FindPackageShare('rowboboat_viz'), 'launch']), 'view_robot.launch.py'])
    )

    # Return the LaunchDescription
    return LaunchDescription([
        desc_launch,
        arms_launch,
        control_launch,
        viz_launch
    ])
