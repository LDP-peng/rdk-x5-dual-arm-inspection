import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_dir = get_package_share_directory('wheeltec_nav2')
    nav_launch = os.path.join(nav_dir, 'launch')

    map_default = os.path.join(os.path.expanduser('~'), 'robot_ws', 'maps', 'map.yaml')
    params_default = os.path.join(
        nav_dir, 'param', 'wheeltec_param', 'param_mini_diff.yaml'
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('params', default_value=params_default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])