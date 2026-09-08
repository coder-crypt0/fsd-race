"""Full closed-loop in simulation — no hardware, no cameras.

    ros2 launch fsd_stack sim.launch.py

The sim node stands in for the car and for Blocks 1-2 (it publishes ideal
/perception/cones), so the supervisor's required-node list excludes the
perception nodes and the CAN bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # sim_params.yaml, not params.yaml: the simulator runs the speeds that
    # actually exercise the explore -> race regime change and the obstacle
    # layer. params.yaml is the real car, where those stay timid.
    params = os.path.join(get_package_share_directory('fsd_stack'),
                          'config', 'sim_params.yaml')

    def node(executable, extra=None):
        p = [params] + ([extra] if extra else [])
        return Node(package='fsd_stack', executable=executable,
                    name=executable, parameters=p, output='screen')

    return LaunchDescription([
        node('sim'),
        node('state_estimation'),
        node('cone_mapping'),
        node('path_planning'),
        node('motion_control'),
        node('safety_supervisor', extra={
            'required_nodes': ['state_estimation', 'cone_mapping',
                               'path_planning', 'motion_control'],
        }),
    ])
