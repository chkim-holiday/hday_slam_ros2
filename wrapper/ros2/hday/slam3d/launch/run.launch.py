#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='friday',
        description='Configuration name (e.g., friday)'
    )
    
    # Get package share directory
    package_name = 'hday_slam3d_ros2'
    config_name = LaunchConfiguration('config')
    
    # Parameter file path
    params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        config_name,
        'parameters.yaml'
    ])
    
    # Node
    slam_node = Node(
        package=package_name,
        executable='hday_slam3d_ros2_node',
        name='hday_slam3d_node',
        output='screen',
        parameters=[params_file],
        remappings=[]
    )
    
    return LaunchDescription([
        config_arg,
        slam_node,
    ])
