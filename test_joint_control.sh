#!/bin/bash

echo "=== JOINT CONTROL TEST ==="
echo "Bu script joint controller'ları test eder"
echo ""

echo "1. Sol tekerleğe force uygula (ileri):"
ros2 topic pub /left_wheel_force std_msgs/msg/Float64 '{data: 100.0}' --once

echo "2. Sağ tekerleğe force uygula (ileri):"
ros2 topic pub /right_wheel_force std_msgs/msg/Float64 '{data: 100.0}' --once

sleep 3

echo "3. Force'ları durdur:"
ros2 topic pub /left_wheel_force std_msgs/msg/Float64 '{data: 0.0}' --once
ros2 topic pub /right_wheel_force std_msgs/msg/Float64 '{data: 0.0}' --once

echo "4. Ters yönde force:"
ros2 topic pub /left_wheel_force std_msgs/msg/Float64 '{data: -100.0}' --once
ros2 topic pub /right_wheel_force std_msgs/msg/Float64 '{data: -100.0}' --once

sleep 3

echo "5. Force'ları sıfırla:"
ros2 topic pub /left_wheel_force std_msgs/msg/Float64 '{data: 0.0}' --once
ros2 topic pub /right_wheel_force std_msgs/msg/Float64 '{data: 0.0}' --once

echo "=== TEST BITTI ==="
echo "Eğer robot hareket ettiyse joint controller çalışıyor demektir"
echo "Eğer hareket etmediyse sorun daha derinde..."