#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use the package share directory to locate the URDF file
    urdf_file = os.path.join(get_package_share_directory('drone_control'), 'urdf', 'advanced_quadrotor.urdf')
    return LaunchDescription([
        # Launch Ignition Gazebo (using default world)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r'],
            output='screen'
        ),
        # Spawn the drone model after a 5-second delay for Gazebo to initialize
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
