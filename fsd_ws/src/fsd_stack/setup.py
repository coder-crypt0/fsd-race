from setuptools import setup
import os
from glob import glob

package_name = 'fsd_stack'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FSD Team',
    maintainer_email='183348113+coder-crypt0@users.noreply.github.com',
    description='FSD autonomous stack v1 — see FSD_System_Interface_Specification.md',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_detection = fsd_stack.cone_detection_node:main',
            'cone_localization = fsd_stack.cone_localization_node:main',
            'state_estimation = fsd_stack.state_estimation_node:main',
            'cone_mapping = fsd_stack.cone_mapping_node:main',
            'path_planning = fsd_stack.path_planning_node:main',
            'motion_control = fsd_stack.motion_control_node:main',
            'safety_supervisor = fsd_stack.safety_supervisor_node:main',
            'can_bridge = fsd_stack.can_bridge_node:main',
            'sim = fsd_stack.sim_node:main',
            'dashboard = fsd_stack.dashboard_node:main',
        ],
    },
)
