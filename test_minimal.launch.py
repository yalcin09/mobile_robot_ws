#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # URDF file path
    urdf_file = os.path.join(os.getcwd(), 'working_robot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    # Start Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen')

    # Spawn robot
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                  '-entity', 'test_robot',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1'],
        output='screen')

    # Bridge for cmd_vel
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    
    # Delay spawning to allow Gazebo to start
    ld.add_action(TimerAction(
        period=3.0,
        actions=[spawn_entity_cmd, bridge_cmd]
    ))

    return ld