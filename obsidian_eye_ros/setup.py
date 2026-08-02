from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obsidian_eye_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hajer',
    maintainer_email='example@email.com',
    description='Obsidian Eye ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_yolo_node = obsidian_eye_ros.camera_yolo_node:main',
            'rf_node = obsidian_eye_ros.rf_node:main',
            'acoustic_node = obsidian_eye_ros.acoustic_node:main',
            'decision_node = obsidian_eye_ros.decision_node:main',
            'tracking_action_server = obsidian_eye_ros.tracking_action_server:main',
            'mission_node = obsidian_eye_ros.mission_node:main',
        ],
    },
)
