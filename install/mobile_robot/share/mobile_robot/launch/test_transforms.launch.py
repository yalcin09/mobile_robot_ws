#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='mobile_robot').find('mobile_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mobile_robot.rviz')
    
    # Read URDF file content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # Static transforms for odom and map
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    ld = LaunchDescription()
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(static_transform_map_to_odom)
    ld.add_action(static_transform_odom_to_base_link)
    ld.add_action(rviz_cmd)
    
    return ld