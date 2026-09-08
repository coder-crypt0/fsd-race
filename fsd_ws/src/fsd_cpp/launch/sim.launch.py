"""The C++ PRODUCTION stack against the Python kinematic simulator.

    ros2 launch fsd_cpp sim.launch.py

Why this exists: fsd_cpp is what ships on the car, but until now it could only
be exercised against FSDS (needs a GPU and a Windows/Linux binary) or the car
itself. The kinematic sim publishes nothing but contract topics —
/perception/cones, /imu/data, /wheel_speeds, /vehicle/status — and consumes
/control/cmd, so the C++ nodes can be dropped straight onto it. That makes a
headless closed-loop regression test of the production stack possible anywhere,
which is what tools/docker_test.sh uses.

The sim stands in for the car AND for Blocks 1-2 (it publishes ideal cone
detections), so stereo_cone_node and the FSDS adapter are not launched and the
supervisor's required-node list excludes them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cpp_params = os.path.join(get_package_share_directory('fsd_cpp'),
                              'config', 'sim_params.yaml')
    sim_params = os.path.join(get_package_share_directory('fsd_stack'),
                              'config', 'sim_params.yaml')

    def cpp(executable, name):
        return Node(package='fsd_cpp', executable=executable, name=name,
                    parameters=[cpp_params], output='screen')

    return LaunchDescription([
        # The simulator itself is the Python package's node.
        Node(package='fsd_stack', executable='sim', name='sim',
             parameters=[sim_params], output='screen'),

        cpp('state_estimation_node', 'state_estimation'),
        cpp('cone_mapping_node', 'cone_mapping'),
        cpp('path_planning_node', 'path_planning'),
        cpp('motion_control_node', 'motion_control'),
        cpp('safety_supervisor_node', 'safety_supervisor'),
    ])
