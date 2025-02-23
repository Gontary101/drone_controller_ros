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
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
            output='screen'
        ),
        # Spawn the drone after waiting 5 seconds for Ignition Gazebo to start
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-file', urdf_file, '-name', 'advanced_quadrotor'],
                    output='screen'
                )
            ]
        )
    ])