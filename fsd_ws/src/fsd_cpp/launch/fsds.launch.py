"""C++ stack against the Formula Student Driverless Simulator.

    ros2 launch fsd_cpp fsds.launch.py

Prerequisites (see fsd_ws/README.md, FSDS section):
  1. FSDS running with fsd_ws/fsds/settings.json copied to
     ~/Formula-Student-Driverless-Simulator/settings.json
  2. The FSDS ROS2 bridge running (publishes /fsds/... topics).
  3. fs_msgs cloned into fsd_ws/src (message defs used by the bridge).

Camera/IMU topic names vary slightly between FSDS bridge versions —
check `ros2 topic list` and pass e.g.:
    ros2 launch fsd_cpp fsds.launch.py left_image:=/fsds/camera/<name>/image_color
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fsd_cpp'),
                          'config', 'fsds_params.yaml')

    left_image = LaunchConfiguration('left_image')
    right_image = LaunchConfiguration('right_image')
    imu_topic = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        # Topic names verified against a live FSDS ros2 bridge v2.2.0:
        # cameras use absolute frame ids (/fsds/cam_<side>/image_color), the
        # IMU is at the root namespace (/imu).
        DeclareLaunchArgument('left_image',
                              default_value='/fsds/cam_left/image_color'),
        DeclareLaunchArgument('right_image',
                              default_value='/fsds/cam_right/image_color'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

        Node(package='fsd_cpp', executable='stereo_cone_node',
             name='stereo_cone', parameters=[params], output='screen',
             remappings=[('/camera/left/image_raw', left_image),
                         ('/camera/right/image_raw', right_image)]),
        Node(package='fsd_cpp', executable='state_estimation_node',
             name='state_estimation', parameters=[params], output='screen',
             remappings=[('/imu/data', imu_topic)]),
        Node(package='fsd_cpp', executable='cone_mapping_node',
             name='cone_mapping', parameters=[params], output='screen'),
        Node(package='fsd_cpp', executable='path_planning_node',
             name='path_planning', parameters=[params], output='screen'),
        Node(package='fsd_cpp', executable='motion_control_node',
             name='motion_control', parameters=[params], output='screen'),
        Node(package='fsd_cpp', executable='safety_supervisor_node',
             name='safety_supervisor', parameters=[params], output='screen'),
        Node(package='fsd_cpp', executable='fsds_adapter_node',
             name='fsds_adapter', parameters=[params], output='screen'),

        # Mission-control dashboard (Python package): http://localhost:8321
        Node(package='fsd_stack', executable='dashboard',
             name='dashboard', output='screen'),
    ])
