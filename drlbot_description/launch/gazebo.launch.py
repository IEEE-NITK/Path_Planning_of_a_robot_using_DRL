from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command,PathJoinSubstitution,LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    share_dir = get_package_share_directory('drlbot_description')

    xacro_file = os.path.join(share_dir, 'urdf','drlbot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    
    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('rvizconfig', default_value=PathJoinSubstitution([share_dir, 'rviz', 'urdf.rviz']), description='RViz configuration file'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic','/robot_description','-entity','spawn_urdf','-x','0.0','-y','0.0','-z','0.0'],
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gzserver.launch.py'
                ]),
            ]),
            launch_arguments={
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'headless': LaunchConfiguration('headless'),
                'debug': LaunchConfiguration('debug'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gzclient.launch.py'
                ]),
            ])
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     output='screen',
        #     arguments=['-d', LaunchConfiguration('rvizconfig')]
        # ),
    ])
