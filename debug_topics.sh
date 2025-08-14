#!/bin/bash

echo "=== ROS2 Topic Debug ==="
echo "Listing all topics:"
ros2 topic list

echo ""
echo "=== Joint States Topic ==="
echo "Checking joint_states:"
timeout 3 ros2 topic echo /joint_states | head -20

echo ""
echo "=== CMD VEL Topic Test ==="
echo "Publishing test cmd_vel:"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' --once

echo ""
echo "=== Tray Position Test ==="
echo "Publishing test tray position:"
ros2 topic pub /tray_position_cmd std_msgs/msg/Float64 '{data: 0.1}' --once

echo ""
echo "=== Topic Info ==="
echo "cmd_vel topic info:"
ros2 topic info /cmd_vel

echo "tray_position_cmd topic info:"
ros2 topic info /tray_position_cmd

echo "joint_states topic info:"
ros2 topic info /joint_states