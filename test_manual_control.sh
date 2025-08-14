#!/bin/bash

# Test script for manual control system
echo "=== Mobile Robot Manual Control Test ==="
echo "This script will test the manual control system for the mobile robot."
echo

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "1. Testing robot_state_publisher with URDF..."
timeout 5s ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/mobile_robot/urdf/mobile_robot.urdf)" &
ROBOT_STATE_PID=$!

echo "2. Testing mobile_robot_controller..."
timeout 5s ros2 run mobile_robot mobile_robot_controller &
CONTROLLER_PID=$!

sleep 2

echo "3. Checking active nodes..."
ros2 node list

echo
echo "4. Checking active topics..."
ros2 topic list

echo
echo "5. Testing cmd_vel topic (sending test command)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

sleep 2

echo
echo "6. Checking joint_states topic..."
ros2 topic echo /joint_states --once

echo
echo "7. Checking odom topic..."
ros2 topic echo /odom --once

# Clean up
echo
echo "Cleaning up test processes..."
kill $ROBOT_STATE_PID $CONTROLLER_PID 2>/dev/null
wait

echo "=== Test Complete ==="
echo "If you saw joint_states and odom data, the system is working correctly!"
echo
echo "To launch the full manual control system, use:"
echo "ros2 launch mobile_robot manual_control.launch.py"