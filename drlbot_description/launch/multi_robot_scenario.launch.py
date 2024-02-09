#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'td3.world'
    world = os.path.join(get_package_share_directory('drlbot_description'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('drlbot_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'paused': LaunchConfiguration('paused')
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/rsp.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic','/robot_description','-entity','spawn_urdf'],
        ),
        
    ])