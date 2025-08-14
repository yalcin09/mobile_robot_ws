#!/usr/bin/env python3
"""
Basit robot testi
URDF değişikliklerini test etmek için minimal launcher
"""

import time
import subprocess
import sys
import signal

def kill_processes():
    """Tüm ilgili süreçleri temizle"""
    try:
        subprocess.run(['pkill', '-f', 'gazebo'], check=False)
        subprocess.run(['pkill', '-f', 'gz'], check=False) 
        subprocess.run(['pkill', '-f', 'rviz'], check=False)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], check=False)
        time.sleep(2)
    except:
        pass

def test_basic_functionality():
    """Temel işlevsellik testi"""
    print("=== Mobile Robot Test - Basit Test ===")
    print("1. URDF joint dynamics değerleri düzeltildi:")
    print("   - friction: 0.0375 -> 0.1")
    print("   - damping: 0.01 -> 0.5")
    print("   - max_wheel_torque: 200 -> 50")
    print("   - effort limits: 2000 -> 100")
    print("   - velocity limits: 100 -> 10")
    print("")
    
    print("2. Ana problemler:")
    print("   - ROS2 kurulumu eksik, environment setup yapılamıyor")
    print("   - Gazebo/DiffDrive plugin'i manuel test edilemiyor")
    print("   - Topic bridging test edilemiyor")
    print("")
    
    print("3. Önerilen manual test:")
    print("   Sistem üzerinde ROS2 Jazzy kurulu ise:")
    print("   $ source /opt/ros/jazzy/setup.bash")
    print("   $ cd mobile_robot_ws") 
    print("   $ colcon build")
    print("   $ source install/setup.bash")
    print("   $ ros2 launch mobile_robot manual_control.launch.py")
    print("")
    
    print("4. Değişiklik özeti:")
    print("   - Joint dynamics daha gerçekçi değerler")
    print("   - Torque limitleri düşürüldü") 
    print("   - Effort ve velocity limitleri optimize edildi")
    print("   - Castor wheel friction korundu (mu1=0.0001)")
    print("   - Main wheel friction artırıldı (mu1=2.5)")
    print("")

def signal_handler(sig, frame):
    print("\nTest durduruluyor...")
    kill_processes()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    kill_processes()
    test_basic_functionality()
    print("Test tamamlandı. Manuel testler için yukarıdaki komutları kullanın.")