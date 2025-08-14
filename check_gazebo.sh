#!/bin/bash

echo "=== GAZEBO PLUGIN DEBUG ==="

echo "1. Checking Gazebo topics:"
gz topic -l | grep -E "(cmd_vel|joint_state|odom)"

echo ""
echo "2. Checking if DiffDrive plugin is subscribed to cmd_vel:"
gz topic -i -t /cmd_vel

echo ""
echo "3. Checking robot entity in Gazebo:"
gz model -l

echo ""
echo "4. Testing direct Gazebo command:"
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 1.0}'

echo ""
echo "5. Checking ROS2 topics:"
ros2 topic list | grep -E "(cmd_vel|joint|odom)"

echo ""
echo "6. Checking bridge status:"
ros2 node list | grep bridge

echo "=== DEBUG COMPLETE ==="