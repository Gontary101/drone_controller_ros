#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Adjust the path to your URDF file as needed:
    urdf_file = os.path.join(
        os.getenv('HOME'), 'drone_ws', 'drone_control', 'urdf', 'advanced_quadrotor.urdf'
    )
    return LaunchDescription([
        # Launch Gazebo (using the standard gazebo.launch.py from gazebo_ros)
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn the drone after waiting 5 seconds for Gazebo to start
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'advanced_quadrotor', '-file', urdf_file],
                    output='screen'
                )
            ]
        )
    ])
