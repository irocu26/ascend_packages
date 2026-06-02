from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ascend_mission_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IRoCU-26 Team',
    maintainer_email='irocu2026@ursc.gov.in',
    description='ASCEND autonomous mission control — FSM, survey planner, monitor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core FSM brain
            'fsm_node = ascend_mission_control.fsm_node:main',

            # Survey waypoint planner
            'survey_planner_node = ascend_mission_control.survey_planner_node:main',

            # Mission telemetry monitor (display-only)
            'mission_monitor_node = ascend_mission_control.mission_monitor_node:main',

            # FIXED: added ap_interface standalone test entry point
            # Run this alone (with SITL + DDS agent) to verify AP commands work
            # before integrating with the full FSM
            'ap_interface = ascend_mission_control.ap_interface:main',

           
        ],
    },
)