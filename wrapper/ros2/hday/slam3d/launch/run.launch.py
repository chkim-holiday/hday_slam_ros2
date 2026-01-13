#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml  # PyYAML 라이브러리

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
    parameter_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        config_name,
        'parameters.yaml'
    ])
    sensor_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        config_name,
        'sensors.yaml'
    ])
    
    # Nodes
    point_cloud_aggregator_node = Node(
        package=package_name,
        executable='point_cloud_aggregator_node',
        name='point_cloud_aggregator_node',
        output='screen',
        parameters=[parameter_file, sensor_file],
        remappings=[]
    )
    slam_system_node = Node(
        package=package_name,
        executable='slam_system_node',
        name='slam_system_node',
        output='screen',
        parameters=[parameter_file, sensor_file],
        remappings=[]
    )
    
    return LaunchDescription([
        config_arg,
        slam_system_node,
        point_cloud_aggregator_node
    ])
