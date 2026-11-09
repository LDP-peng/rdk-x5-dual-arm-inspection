import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_mission')
    default_waypoints = os.path.join(share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('alert_hold_sec', default_value='5.0'),
        DeclareLaunchArgument('return_on_alert', default_value='true'),
        DeclareLaunchArgument('loop', default_value='true'),
        Node(
            package='robot_mission',
            executable='patrol_manager',
            name='patrol_manager',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'alert_hold_sec': ParameterValue(
                    LaunchConfiguration('alert_hold_sec'), value_type=float
                ),
                'return_on_alert': ParameterValue(
                    LaunchConfiguration('return_on_alert'), value_type=bool
                ),
                'loop': ParameterValue(
                    LaunchConfiguration('loop'), value_type=bool
                ),
            }],
        ),
    ])