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
    
    xacro_file_path = os.path.join(get_package_share_directory('bot_description'), 'urdf', 'bot_urdf.xacro')
    gazebo_params_file = os.path.join(get_package_share_directory('bot_world'),'config','gazebo_params.yaml')
    world_file_path = os.path.join(get_package_share_directory('bot_world'),'world','simple_world.world')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'world':world_file_path,
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file }.items()
        )

    use_sim_time =  LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'use sim time if true'
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file_path]), 'use_sim_time' : use_sim_time}]
        ),

        gazebo,

        Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            arguments = [
                '-entity', 'Ogmen',
                '-topic', 'robot_description',
                '-x','0',
                '-y','0.0',
                '-z','0.0'
            ],
            output = 'screen',
        ),
    ])