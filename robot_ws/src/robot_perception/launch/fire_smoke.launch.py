import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_perception')
    home = os.path.expanduser('~')

    default_model = os.path.join(home, 'robot_ws', 'models', 'fire_smoke_yolov8n.pt')
    default_labels = os.path.join(home, 'robot_ws', 'models', 'fire_smoke.names')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='ultralytics'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('label_path', default_value=default_labels),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('conf_threshold', default_value='0.35'),
        DeclareLaunchArgument('alert_consecutive_frames', default_value='3'),
        Node(
            package='robot_perception',
            executable='fire_smoke_detector',
            name='fire_smoke_detector',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'fire_smoke.yaml'),
                {
                    'backend': LaunchConfiguration('backend'),
                    'model_path': LaunchConfiguration('model_path'),
                    'label_path': LaunchConfiguration('label_path'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'conf_threshold': ParameterValue(
                        LaunchConfiguration('conf_threshold'), value_type=float
                    ),
                    'alert_consecutive_frames': ParameterValue(
                        LaunchConfiguration('alert_consecutive_frames'), value_type=int
                    ),
                },
            ],
        ),
    ])