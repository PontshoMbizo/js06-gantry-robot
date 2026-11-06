import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gantry_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pontsho',
    maintainer_email='pontsho@example.com',
    description=(
        'Closed-loop control algorithm testbed for the gantry robot: '
        'position controller and unloaded motor transfer model '
        'driving ROS 2 velocity interfaces in Gazebo.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = gantry_control.control_node:main',
            'discretize_helper = gantry_control.discretize_helper:main',
        ],
    },
)
