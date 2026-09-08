"""Full stack on the real vehicle.

    ros2 launch fsd_stack vehicle.launch.py

Prerequisites:
  * camera driver publishing /camera/left/image_raw (+ right for stereo)
  * BNO055 driver publishing /imu/data
  * SocketCAN interface 'can0' up at 500 kbps:
        sudo ip link set can0 up type can bitrate 500000
  * STM32 ECU flashed with firmware/stm32_bridge

The CAN bridge publishes /wheel_speeds and /vehicle/status from ECU frames.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fsd_stack'),
                          'config', 'params.yaml')

    def node(executable):
        return Node(package='fsd_stack', executable=executable,
                    name=executable, parameters=[params], output='screen')

    return LaunchDescription([
        node('cone_detection'),
        node('cone_localization'),
        node('state_estimation'),
        node('cone_mapping'),
        node('path_planning'),
        node('motion_control'),
        node('safety_supervisor'),
        node('can_bridge'),
    ])
