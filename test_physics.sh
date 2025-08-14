#!/bin/bash

echo "=== PHYSICS DEBUG TEST ==="

echo "1. Testing manual cmd_vel with high values:"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 5.0}}' --once

sleep 2

echo "2. Checking if Gazebo receives the command:"
timeout 3 ros2 topic echo /cmd_vel

echo "3. Checking joint states from Gazebo:"
timeout 3 ros2 topic echo /joint_states | head -20

echo "4. Testing reverse direction:"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -5.0}}' --once

sleep 2

echo "5. Testing angular motion:"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 3.0}}' --once

echo "=== TEST COMPLETE ==="