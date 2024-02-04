#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command,PathJoinSubstitution,LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'td3_org.world'
    world = os.path.join(get_package_share_directory('drlbot_description'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('drlbot_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    #rviz_file = os.path.join(get_package_share_directory('td3'), 'launch', 'pioneer3dx.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rvizconfig', default_value=PathJoinSubstitution([share_dir, 'rviz', 'urdf.rviz']), description='RViz configuration file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        Node(package='drlbot_description',
             executable='drl_train',
             output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/rsp.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic','/robot_description','-entity','spawn_urdf'],
        ),
    ])