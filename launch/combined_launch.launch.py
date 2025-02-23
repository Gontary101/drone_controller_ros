#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

this_dir = os.path.dirname(__file__)
gazebo_launch_file = os.path.join(this_dir, 'gazebo_launch.launch.py')

def generate_launch_description():
    return LaunchDescription([
        # Include the Ignition Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),
        # Bridge ROS 2 /drone/wrench to Ignition Gazebo topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/drone/wrench@geometry_msgs/msg/Wrench]model/advanced_quadrotor/link/base_link/wrench'],
            output='screen'
        ),
        # Launch the drone controller node after a delay to ensure sim is ready
        TimerAction(
            period=10.0,  # 10-second delay to allow Gazebo to start
            actions=[
                Node(
                    package='drone_control',
                    executable='drone_controller',
                    name='drone_controller',
                    output='screen'
                )
            ]
        )
    ])