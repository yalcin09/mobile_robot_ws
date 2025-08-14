#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='mobile_robot').find('mobile_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mobile_robot.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    use_tray = LaunchConfiguration('use_tray')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='True',
        description='Whether to launch Gazebo')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz')
    
    declare_use_teleop_cmd = DeclareLaunchArgument(
        'use_teleop',
        default_value='True',
        description='Whether to launch teleop keyboard')
    
    declare_use_tray_cmd = DeclareLaunchArgument(
        'use_tray',
        default_value='True',
        description='Whether to launch tray controller')
    
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
    
    # Add joint_state_publisher GUI for manual testing
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }],
        condition=IfCondition('true')  # Re-enabled for guaranteed transforms
    )
    
    # mobile_robot_controller_node = Node(
    #     package='mobile_robot',
    #     executable='mobile_robot_controller',
    #     name='mobile_robot_controller',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    teleop_keyboard_node = Node(
        package='mobile_robot',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        condition=IfCondition(use_teleop),
        parameters=[{'use_sim_time': use_sim_time}],
        prefix='xterm -e'
    )
    
    tray_controller_node = Node(
        package='mobile_robot',
        executable='tray_controller',
        name='tray_controller',
        output='screen',
        condition=IfCondition(use_tray),
        parameters=[{'use_sim_time': use_sim_time}],
        prefix='xterm -e'
    )
    
    # Joint state merger - disabled to use direct joint_states from Gazebo
    # joint_state_merger = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_merger',
    #     parameters=[{
    #         'robot_description': robot_desc,
    #         'use_sim_time': use_sim_time,
    #         'source_list': ['joint_states']
    #     }],
    #     remappings=[]
    # )
    
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mobile_robot'), '/launch', '/gazebo.launch.py'
        ]),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0'
        }.items()
    )
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    static_transform_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Static transforms for fixed joints (since robot_state_publisher might not handle them properly)
    static_transform_front_caster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_front_caster',
        arguments=['0.5', '0', '-0.04875', '0', '0', '0', 'base_link', 'frontCasterLink']
    )
    
    static_transform_back_caster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_back_caster',
        arguments=['-0.5', '0', '-0.04875', '0', '0', '0', 'base_link', 'backCasterLink']
    )
    
    static_transform_outer_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_outer_body',
        arguments=['0.56025', '0', '0.19001', '0', '0', '0', 'base_link', 'outerBodyLink']
    )
    
    # Static transforms for wheels
    static_transform_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_left_wheel',
        arguments=['0', '0.325', '0.0375', '0', '0', '0', 'base_link', 'leftWheeltLink']
    )
    
    static_transform_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_right_wheel',
        arguments=['0', '-0.325', '0.0375', '0', '0', '0', 'base_link', 'rightWheelLink']
    )
    
    # Static transform for tray (at default position)
    static_transform_tray = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_tray',
        arguments=['0', '0', '0.41', '0', '0', '0', 'base_link', 'trayLink']
    )
    
    # Bridge for Gazebo topics
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', 
            'odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            'tray_position_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            'joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Joint states bridge removed - using joint_state_publisher_gui
    
    # Separate tf bridge - let DiffDrive plugin handle tf publishing directly
    # tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge', 
    #     arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_teleop_cmd)
    ld.add_action(declare_use_tray_cmd)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    
    ld.add_action(gazebo_cmd)
    
    # ld.add_action(TimerAction(
    #     period=3.0,
    #     actions=[mobile_robot_controller_node]
    # ))
    
    # Joint state merger disabled - using direct joint_states from Gazebo
    # ld.add_action(TimerAction(
    #     period=4.0,
    #     actions=[joint_state_merger]
    # ))
    
    ld.add_action(TimerAction(
        period=5.0,
        actions=[rviz_cmd]
    ))
    
    ld.add_action(TimerAction(
        period=6.0,
        actions=[teleop_keyboard_node]
    ))
    
    ld.add_action(TimerAction(
        period=7.0,
        actions=[tray_controller_node]
    ))
    
    # Essential transforms for complete tf tree
    ld.add_action(static_transform_odom_to_map)
    
    # Add odom to base_link if DiffDrive plugin doesn't publish it
    # static_transform_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher', 
    #     name='static_transform_publisher_odom_to_base',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )
    # ld.add_action(static_transform_odom_to_base)
    
    ld.add_action(TimerAction(
        period=3.0,
        actions=[gz_ros2_bridge]
    ))
    
    # tf bridge removed - using robot_state_publisher for joint transforms
    # and DiffDrive plugin for odom->base_link
    
    return ld