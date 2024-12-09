#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    gazebo_params_file = os.path.join(get_package_share_directory('bot_description'),'config','gazebo_params.yaml')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file }.items()
        )


    return LaunchDescription([

        gazebo,

        Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            arguments = [
                '-entity', 'Ogmen',
                '-topic', 'robot_description',
                '-x','0',
                '-y','0',
                '-z','0.0'
            ],
            output = 'screen',
        ),
    ])