import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('shared_control_nav')
    twist_mux_config = os.path.join(package_dir, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config],
            remappings=[
                ('cmd_vel_out', '/cmd_vel')
            ],
            output='screen'
        ),

        Node(
            package='shared_control_nav',
            executable='shared_control',
            name='shared_control',
            output='screen',
        ),
    ])