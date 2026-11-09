"""Full-stack bringup for the RDK X5 differential robot.

Starts everything the robot needs in a single launch:
  robot_base         C30D chassis driver   /odom, /imu/data, odom -> base_footprint
  robot_description  URDF + state publisher base_footprint -> base_link -> links
  lslidar_driver     N10P lidar on UART    /scan at 10 Hz in frame laser_link
  hobot_usb_cam      C70 USB camera        /image at 30 Hz in frame camera_link

Each block can be switched off from the command line, for example:
  ros2 launch robot_bringup robot.launch.py use_camera:=false use_lidar:=false
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def load_camera_arguments():
    camera_yaml = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'camera_c70.yaml')
    with open(camera_yaml, 'r', encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['hobot_usb_cam']['ros__parameters']
    return {
        'usb_camera_calibration_file_path': str(params['camera_calibration_file_path']),
        'usb_frame_id': str(params['frame_id']),
        'usb_framerate': str(params['framerate']),
        'usb_image_height': str(params['image_height']),
        'usb_image_width': str(params['image_width']),
        'usb_io_method': str(params['io_method']),
        'usb_pixel_format': str(params['pixel_format']),
        'usb_video_device': str(params['video_device']),
        'usb_zero_copy': str(params['zero_copy']),
    }


def generate_launch_description():
    use_base = LaunchConfiguration('use_base')
    use_description = LaunchConfiguration('use_description')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_perception = LaunchConfiguration('use_perception')
    use_mission = LaunchConfiguration('use_mission')
    base_port = LaunchConfiguration('base_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_base', default_value='true',
                              description='Start the C30D chassis driver'),
        DeclareLaunchArgument('use_description', default_value='true',
                              description='Start robot_state_publisher with the local URDF'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='Start the N10P lidar driver'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Start the C70 USB camera'),
        DeclareLaunchArgument('use_perception', default_value='false',
                              description='Start the fire/smoke detector'),
        DeclareLaunchArgument('use_mission', default_value='false',
                              description='Start the patrol manager (needs Nav2)'),
        DeclareLaunchArgument('base_port', default_value='/dev/wheeltec_controller',
                              description='C30D serial device'),

        include_launch('robot_base', 'base.launch.py',
                       {'port': base_port}, IfCondition(use_base)),
        include_launch('robot_description', 'description.launch.py',
                       None, IfCondition(use_description)),
        include_launch('lslidar_driver', 'lsn10p_launch.py',
                       None, IfCondition(use_lidar)),
        include_launch('hobot_usb_cam', 'hobot_usb_cam.launch.py',
                       load_camera_arguments(), IfCondition(use_camera)),
        include_launch('robot_perception', 'fire_smoke.launch.py',
                       None, IfCondition(use_perception)),
        include_launch('robot_mission', 'mission.launch.py',
                       None, IfCondition(use_mission)),
    ])
