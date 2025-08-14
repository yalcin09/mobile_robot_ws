#!/bin/bash

echo "=== MOBILE ROBOT TEST SCRIPT ==="
echo "Bu script robotun hareket problemini tespit etmek için hazırlanmıştır."
echo ""

# ROS2 ortamı kontrolü
if ! command -v ros2 &> /dev/null; then
    echo "HATA: ROS2 bulunamadı! Lütfen ROS2 ortamını kurun:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  # veya ROS2'nin kurulu olduğu dizin"
    exit 1
fi

if ! command -v gz &> /dev/null; then
    echo "HATA: Gazebo bulunamadı! Lütfen Gazebo Ionic kurun:"
    echo "  sudo apt install gz-ionic"
    exit 1
fi

echo "1. Ortam hazırlanıyor..."
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || echo "ROS2 ortamı manuel olarak source edilmeli"

echo "2. Workspace build ediliyor..."
cd ~/mobile_robot_ws
colcon build --packages-select mobile_robot

echo "3. Workspace source ediliyor..."
source install/setup.bash

echo "4. Ana robot test ediliyor..."
echo "   Terminal 1: ros2 launch mobile_robot manual_control.launch.py"
echo "   Terminal 2: ros2 run mobile_robot teleop_keyboard"
echo ""
echo "5. Minimal robot test için:"
echo "   gz sim -v 4 -r minimal_test.urdf"
echo ""

echo "TEST ADIMLARI:"
echo "1. Launch dosyasını çalıştırın"
echo "2. Gazebo'da robotun spawn olup olmadığını kontrol edin"
echo "3. Teleop ile W tuşuna basın ve robotun hareket edip etmediğini gözlemleyin"
echo "4. Eğer hareket etmiyorsa, şu kontrolleri yapın:"
echo "   - ros2 topic list | grep cmd_vel"
echo "   - ros2 topic echo /cmd_vel"
echo "   - ros2 topic echo /joint_states"
echo ""

echo "SORUN GİDERME:"
echo "- Tekerlekler dönüyor ama robot hareket etmiyorsa: Sürtünme/temas problemi"
echo "- Tekerlekler hiç dönmüyorsa: Plugin/topic bridge problemi"
echo "- Robot Gazebo'da görünmüyorsa: URDF/mesh dosya problemi"
echo ""
echo "=== TEST SCRIPT HAZIR ==="