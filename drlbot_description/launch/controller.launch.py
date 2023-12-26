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
        DeclareLaunchArgument(
            'controller_config',
            default_value=os.path.join(get_package_share_directory('drlbot_description') ,'config', 'controller.yaml'),
            description='Path to the controller configuration file'
        ),

        # Start the controller spawner node
        Node(
            package='controller_manager',
            executable='spawner.py',
            name='spawner',
            namespace='drlbot',
            output='screen',
            arguments=['controller_config']
        ),

        #Launch the robot_state_publisher node with remapping
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}],
            remappings=[('/joint_states', '/drlbot/joint_states')],
        )
    ])
    
