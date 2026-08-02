#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Start the mission automatically
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([

        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start the mission automatically'
        ),

        # RF sensor
        Node(
            package='obsidian_eye_ros',
            executable='rf_node',
            name='rf_sensor_node',
            output='screen'
        ),

        # Acoustic sensor
        Node(
            package='obsidian_eye_ros',
            executable='acoustic_node',
            name='acoustic_sensor_node',
            output='screen'
        ),

        # Camera and YOLO verification
        Node(
            package='obsidian_eye_ros',
            executable='camera_yolo_node',
            name='camera_yolo_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.70,
                'device': 'cpu',
                'camera_topic':
                    '/world/external_world/model/x500_depth_0/'
                    'link/camera_link/sensor/IMX214/image'
            }]
        ),

        # Decision Engine
        Node(
            package='obsidian_eye_ros',
            executable='decision_node',
            name='decision_engine_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),

        # Tracking Action Server
        Node(
            package='obsidian_eye_ros',
            executable='tracking_action_server',
            name='tracking_action_server',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),

        # Mission Manager
        Node(
            package='obsidian_eye_ros',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'auto_start': auto_start
            }]
        ),
       
    ])