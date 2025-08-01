import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_directory = os.path.join(get_package_share_directory('rowboboat_bringup'), 'config')
    params = os.path.join(config_directory, 'teleop.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[params]),

        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[params, {'publish_stamped_twist': True}],
            remappings={('/cmd_vel', '/diff_drive_controller/cmd_vel')},
            ),
        ])
