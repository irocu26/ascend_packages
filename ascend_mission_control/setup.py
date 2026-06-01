"""Setup configuration for ascend_mission_control."""

from setuptools import setup, find_packages

package_name = 'ascend_mission_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/mission.launch.py']),
        ('share/' + package_name + '/config',
         ['config/mission_params.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='ASCEND Robotics',
    maintainer_email='team@ascend.com',
    description='ASCEND Autonomous Mission Control with Micro-XRCE-DDS',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control = ascend_mission_control.ascend_mission_control_node:main',
            'fcu_bridge = ascend_mission_control.fcu_bridge:main',
            'mission_executor = ascend_mission_control.mission_executor:main',
            'dds_flight_manager = ascend_mission_control.dds_flight_manager:main',


        ],
    },
)