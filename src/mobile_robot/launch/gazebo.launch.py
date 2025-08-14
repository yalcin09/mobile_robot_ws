#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('mobile_robot'))
    urdf_file_name = 'mobile_robot.urdf'
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)
    
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot')
        
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher is handled in manual_control.launch.py
    # robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_desc}])

    # Start Gazebo with resource path and verbose output
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_path, '..'),
            'GZ_VERBOSE': '1',
            'GZ_DEBUG': '1'
        })

    # Spawn robot using ros_gz_sim
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                  '-entity', 'mobile_robot',
                  '-x', x_pose,
                  '-y', y_pose,
                  '-z', '0.1'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add the actions
    # ld.add_action(robot_state_publisher_cmd)  # Handled in manual_control.launch.py
    ld.add_action(start_gazebo_cmd)
    
    # Delay spawning to allow Gazebo to start
    ld.add_action(TimerAction(
        period=3.0,
        actions=[spawn_entity_cmd]
    ))

    return ld