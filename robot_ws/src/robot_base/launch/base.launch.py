import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_base')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/wheeltec_controller',
            description='C30D serial device',
        ),
        Node(
            package='robot_base',
            executable='c30d_driver',
            name='c30d_driver',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'base.yaml'),
                {'port': LaunchConfiguration('port')},
            ],
        ),
    ])
