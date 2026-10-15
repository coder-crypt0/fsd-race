"""Bounded RPC polling for the Windows FSDS / WSL demo.

Keep the upstream bridge read-only. Reference odometry is evaluation-only;
the autonomy stack consumes the camera, IMU and wheel encoders instead.
"""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    settings = json.loads((Path.home() / 'Formula-Student-Driverless-Simulator'
                           / 'settings.json').read_text())
    cameras = settings['Vehicles']['FSCar']['Cameras']
    if len(cameras) != 1 or any(c['CaptureSettings'][0]['ImageType'] != 0
                                for c in cameras.values()):
        raise ValueError('The supported demo profile requires exactly one RGB camera')
    host = LaunchConfiguration('host')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        *[Node(package='fsds_ros2_bridge', executable='fsds_ros2_bridge_camera',
               namespace='fsds/camera', name=name, output='screen',
               parameters=[{'camera_name': name, 'depthcamera': False,
                            'framerate': 20.0, 'host_ip': host}],
               on_exit=Shutdown()) for name in cameras],
        Node(package='fsds_ros2_bridge', executable='fsds_ros2_bridge',
             name='ros_bridge', output='screen', on_exit=Shutdown(),
             parameters=[{'host_ip': host, 'mission_name': 'trackdrive',
                          'track_name': 'A', 'competition_mode': False,
                          'manual_mode': False, 'UDP_control': False,
                          'update_odom_every_n_sec': 0.05,
                          'update_imu_every_n_sec': 0.01,
                          'update_wheel_states_every_n_sec': 0.02,
                          'update_gps_every_n_sec': 0.1,
                          'update_gss_every_n_sec': 0.05,
                          'publish_static_tf_every_n_sec': 1.0}]),
    ])
