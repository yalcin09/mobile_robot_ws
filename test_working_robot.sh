#!/bin/bash

echo "=== MINIMAL WORKING ROBOT TEST ==="
echo "Bu basit robot çalışacak mı test edelim"
echo ""

echo "1. Gazebo'yu başlat ve minimal robot'u spawn et:"
echo "   gz sim empty.sdf &"
echo "   sleep 3"
echo "   gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"working_robot.urdf\"'"
echo ""

echo "2. Basit hareket komutu ver:"
echo "   gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}'"
echo ""

echo "Manuel test için:"
echo "gz sim empty.sdf"
echo "# Başka terminalde:"
echo "gz model --spawn-file working_robot.urdf --model-name test_robot"
echo "gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 1.0}'"

echo "=== TEST HAZIR ==="