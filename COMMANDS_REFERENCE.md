# Mobile Robot - Komut Referans Kılavuzu

Bu dosya mobile robot projesindeki tüm launch dosyaları, node'lar ve komutları içerir.

## 📋 Hızlı Referans

### 🚀 Ana Launch Komutları
```bash
# 1. RViz'de robot görselleştirme
ros2 launch mobile_robot display.launch.py

# 2. Gazebo'da robot simülasyonu
ros2 launch mobile_robot gazebo.launch.py

# 3. MANUEL KONTROL SİSTEMİ (YENİ!)
ros2 launch mobile_robot manual_control.launch.py
```

---

## 🛠️ Temel Kurulum Komutları

### Workspace Hazırlama
```bash
# Workspace'e git
cd ~/mobile_robot_ws

# Build işlemi
colcon build

# Environment'ı source et
source install/setup.bash

# ROS2 environment'ını source et (ihtiyaç halinde)
source /opt/ros/jazzy/setup.bash
```

---

## 📄 Launch Dosyaları

### 1. `display.launch.py` - RViz Görselleştirme
**Amaç**: Robot'u RViz'de görselleştirme ve joint kontrolü
**Konum**: `src/mobile_robot/launch/display.launch.py`

```bash
# Çalıştırma
ros2 launch mobile_robot display.launch.py

# Parametreli çalıştırma
ros2 launch mobile_robot display.launch.py use_rviz:=True use_robot_state_pub:=True
```

**Başlattığı Node'lar:**
- `robot_state_publisher`: URDF'yi ROS'a yayınlar
- `joint_state_publisher_gui`: Joint'leri manuel kontrol GUI'si
- `rviz2`: Robot görselleştirme arayüzü

**Parametreler:**
- `use_robot_state_pub` (default: True): Robot state publisher'ı başlat
- `use_rviz` (default: True): RViz'i başlat

### 2. `gazebo.launch.py` - Gazebo Simülasyonu
**Amaç**: Robot'u Gazebo'da simüle etme
**Konum**: `src/mobile_robot/launch/gazebo.launch.py`

```bash
# Çalıştırma
ros2 launch mobile_robot gazebo.launch.py

# Pozisyon parametreli çalıştırma
ros2 launch mobile_robot gazebo.launch.py x_pose:=1.0 y_pose:=2.0
```

**Başlattığı Process/Node'lar:**
- `gz sim`: Gazebo Ionic simülatörü
- `robot_state_publisher`: URDF'yi ROS'a yayınlar
- `ros_gz_sim create`: Robot'u Gazebo'ya spawn eder (3sn gecikme ile)

**Parametreler:**
- `x_pose` (default: 0.0): Robot'un X pozisyonu
- `y_pose` (default: 0.0): Robot'un Y pozisyonu

### 3. `manual_control.launch.py` - Manuel Kontrol Sistemi (YENİ!)
**Amaç**: Robot'u C++ ile manuel kontrol etme - Gazebo + RViz + Teleop + Tray
**Konum**: `src/mobile_robot/launch/manual_control.launch.py`

```bash
# Tam sistem ile çalıştırma
ros2 launch mobile_robot manual_control.launch.py

# Sadece belirli bileşenler ile
ros2 launch mobile_robot manual_control.launch.py use_teleop:=False use_tray:=False
ros2 launch mobile_robot manual_control.launch.py use_gazebo:=False use_rviz:=True
```

**Başlattığı C++ Node'lar:**
- `mobile_robot_controller`: Ana differential drive kontrolü ve odometry
- `teleop_keyboard`: WASD klavye kontrolü (ayrı terminal)
- `tray_controller`: R/F ile tepsi kontrolü (ayrı terminal)
- `robot_state_publisher`: URDF yayınlama
- `joint_state_publisher`: Joint state birleştirme
- `static_tf`: odom->map transform

**Parametreler:**
- `use_sim_time` (default: True): Simülasyon zamanı kullan
- `use_gazebo` (default: True): Gazebo'yu başlat
- `use_rviz` (default: True): RViz'i başlat  
- `use_teleop` (default: True): Klavye kontrolünü başlat
- `use_tray` (default: True): Tepsi kontrolünü başlat

**Klavye Kontrolleri:**
- **Robot Hareketi (teleop_keyboard terminali):**
  - W: İleri hareket
  - S: Geri hareket
  - A: Sola dönüş
  - D: Sağa dönüş
  - Q: Hız artırma
  - E: Hız azaltma
  - Space: Dur
  - X: Çıkış

- **Tepsi Kontrolü (tray_controller terminali):**
  - R: Tepsiyi kaldır
  - F: Tepsiyi indir
  - T: Tepsi hızını artır
  - G: Tepsi hızını azalt
  - Space: Tepsiyi durdur
  - C: Çıkış

---

## 🤖 Manuel Node Çalıştırma

### C++ Controller Node'ları (YENİ!)

#### Mobile Robot Controller
```bash
# Ana differential drive controller
ros2 run mobile_robot mobile_robot_controller
```
**Görev**: 
- `/cmd_vel` topic'ini dinler
- Wheel joint velocities hesaplar
- `/joint_states` ve `/odom` yayınlar
- `odom->base_link` TF transform yayınlar

**Yayınladığı Topic'ler:**
- `/joint_states` (sensor_msgs/JointState)
- `/odom` (nav_msgs/Odometry)

**Dinlediği Topic'ler:**
- `/cmd_vel` (geometry_msgs/Twist)

#### Teleop Keyboard Controller  
```bash
# WASD klavye kontrolü
ros2 run mobile_robot teleop_keyboard
```
**Görev**: WASD tuşları ile robot kontrol
**Yayınladığı Topic'ler:**
- `/cmd_vel` (geometry_msgs/Twist)

#### Tray Controller
```bash
# Tepsi prismatic joint kontrolü
ros2 run mobile_robot tray_controller  
```
**Görev**: R/F tuşları ile tepsi kontrol
**Yayınladığı Topic'ler:**
- `/tray_joint_states` (sensor_msgs/JointState)

### Test Script
```bash
# Hızlı sistem testi
./test_manual_control.sh
```

### Robot State Publisher
```bash
# Robot description'ı yayınla
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/mobile_robot/urdf/mobile_robot.urdf)"
```
**Görev**: URDF dosyasını `/robot_description` topic'ine yayınlar

### Joint State Publisher GUI
```bash
# Joint kontrolü için GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
**Görev**: Robot eklemlerini manuel olarak hareket ettirme arayüzü

### RViz2
```bash
# RViz'i aç
ros2 run rviz2 rviz2

# Config dosyası ile aç
ros2 run rviz2 rviz2 -d src/mobile_robot/config/mobile_robot.rviz
```
**Görev**: 3D robot görselleştirme ve analiz

### Manuel Gazebo Robot Spawn
```bash
# Gazebo çalışırken robot spawn et
ros2 run ros_gz_sim create -topic /robot_description -entity mobile_robot -x 0 -y 0 -z 0.1
```
**Görev**: Robot'u Gazebo'ya manuel olarak ekler

---

## 🎮 Robot Kontrol Komutları

### Keyboard Teleoperation
```bash
# Klavye ile robot kontrolü
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cmd_vel topic'ini dinle
ros2 topic echo /cmd_vel
```
**Görev**: Klavye ile robot hareket kontrolü

### Topic Kontrolleri
```bash
# Aktif topic'leri listele
ros2 topic list

# Robot description'ı kontrol et
ros2 topic echo /robot_description

# Joint states'i kontrol et
ros2 topic echo /joint_states

# Manuel kontrol topic'lerini izle (YENİ!)
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /tray_joint_states

# Topic frekanslarını kontrol et
ros2 topic hz /joint_states
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# TF tree'yi görüntüle
ros2 run tf2_tools view_frames
```

---

## 🔍 Debug ve Monitoring Komutları

### ROS2 Sistem Kontrolleri
```bash
# Node'ları listele
ros2 node list

# Belirli node'un info'su
ros2 node info /robot_state_publisher

# Service'leri listele
ros2 service list

# Parameter'leri listele
ros2 param list
```

### Gazebo Kontrolleri
```bash
# Gazebo dünyasındaki entity'leri listele
gz service -s /world/empty/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene

# Gazebo service'lerini listele
gz service --list

# Robot'u manuel olarak spawn et
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --req 'sdf_filename: "model.sdf", name: "mobile_robot"'
```

### Log Kontrolleri
```bash
# ROS2 log seviyesini ayarla
ros2 run rclcpp_components component_container --ros-args --log-level DEBUG

# Launch log dosyalarını kontrol et
ls ~/.ros/log/
```

---

## 🛠️ Bakım ve Güncelleme Komutları

### Workspace Bakımı
```bash
# Clean build
cd ~/mobile_robot_ws
rm -rf build install log
colcon build

# Sadece mobile_robot paketini build et
colcon build --packages-select mobile_robot

# Symlink install (development için)
colcon build --symlink-install
```

### URDF Kontrolleri
```bash
# URDF syntax kontrolü
check_urdf src/mobile_robot/urdf/mobile_robot.urdf

# URDF'yi Graphviz ile görselleştir
urdf_to_graphiz src/mobile_robot/urdf/mobile_robot.urdf
```

---

## 🚨 Hata Giderme Komutları

### Genel Sorun Giderme
```bash
# ROS2 environment kontrol
echo $ROS_DOMAIN_ID
echo $ROS_VERSION

# Package'ın kurulu olup olmadığını kontrol et
ros2 pkg list | grep mobile_robot

# Package path'ini kontrol et
ros2 pkg prefix mobile_robot
```

### Gazebo Sorun Giderme
```bash
# Gazebo çalışıyor mu kontrol et
ps aux | grep gz

# Gazebo'yu temizle
killall gz
```

### Mesh Dosyası Kontrolleri
```bash
# Mesh dosyalarının varlığını kontrol et
ls -la install/mobile_robot/share/mobile_robot/meshes/

# URDF'deki mesh yollarını kontrol et
grep -n "filename=" src/mobile_robot/urdf/mobile_robot.urdf
```

---

## 🔄 Tipik Çalışma Akışı

### 1. Günlük Başlangıç
```bash
cd ~/mobile_robot_ws
source install/setup.bash
```

### 2. Development Cycle
```bash
# Kod değişikliği sonrası
colcon build --packages-select mobile_robot
source install/setup.bash
ros2 launch mobile_robot display.launch.py
```

### 3. Test Sequence
```bash
# Terminal 1: RViz test
ros2 launch mobile_robot display.launch.py

# Terminal 2: Gazebo test
ros2 launch mobile_robot gazebo.launch.py

# Terminal 3: Topic monitoring
ros2 topic list
ros2 topic echo /joint_states
```

---

## 📊 Performans Monitoring

### Resource Usage
```bash
# CPU/Memory kullanımı
htop

# ROS2 node'larının resource kullanımı
ps aux | grep ros2

# Gazebo performance
gz stats
```

### Network Monitoring
```bash
# ROS2 network trafiği
ros2 topic hz /robot_description
ros2 topic bw /joint_states
```

---

## 🎯 Hızlı Erişim Komutları

### En Sık Kullanılan
```bash
# Hızlı başlangıç
alias mobile_setup='cd ~/mobile_robot_ws && source install/setup.bash'
alias mobile_rviz='ros2 launch mobile_robot display.launch.py'
alias mobile_gazebo='ros2 launch mobile_robot gazebo.launch.py'

# .bashrc'ye ekle:
echo "alias mobile_setup='cd ~/mobile_robot_ws && source install/setup.bash'" >> ~/.bashrc
echo "alias mobile_rviz='ros2 launch mobile_robot display.launch.py'" >> ~/.bashrc
echo "alias mobile_gazebo='ros2 launch mobile_robot gazebo.launch.py'" >> ~/.bashrc
```

---

## 📁 Dosya Yapısı Referansı

```
mobile_robot_ws/
├── src/mobile_robot/
│   ├── src/                       # C++ source files (YENİ!)
│   │   ├── mobile_robot_controller.cpp  # Ana differential drive controller
│   │   ├── teleop_keyboard.cpp          # WASD klavye kontrolü
│   │   └── tray_controller.cpp          # Tepsi prismatic joint kontrolü
│   ├── launch/
│   │   ├── display.launch.py            # RViz launcher
│   │   ├── gazebo.launch.py             # Gazebo launcher
│   │   └── manual_control.launch.py     # Manuel kontrol sistemi (YENİ!)
│   ├── urdf/
│   │   └── mobile_robot.urdf            # Robot tanımı
│   ├── meshes/                          # 3D model dosyaları
│   ├── config/
│   │   └── mobile_robot.rviz            # RViz konfigürasyonu
│   ├── package.xml                      # ROS2 paket tanımı (C++ deps eklendi)
│   └── CMakeLists.txt                  # Build config (C++ executables eklendi)
├── test_manual_control.sh               # Test script (YENİ!)
├── build/                               # Build dosyaları
├── install/                             # Install dosyaları
└── log/                                # Log dosyaları
```

---

## 🚀 Gelecek Eklentiler İçin Template

### Yeni Launch Dosyası Ekleme
```python
# src/mobile_robot/launch/new_feature.launch.py
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='executable_name',
            name='node_name',
            output='screen'
        )
    ])
```

### Yeni Node Ekleme
```bash
# Yeni node çalıştırma
ros2 run package_name executable_name

# Launch dosyasına ekleme
Node(
    package='package_name',
    executable='executable_name', 
    name='node_name',
    output='screen',
    parameters=[{'param_name': 'param_value'}]
)
```

---

*Bu dosya her yeni özellik eklendiğinde güncellenecektir.*

**Son güncelleme**: C++ Manuel Kontrol Sistemi tamamlandı
**Eklenen özellikler**: 
- Differential drive controller (C++)
- WASD klavye teleop (C++)  
- Tepsi prismatic joint kontrolü (C++)
- Odometry hesaplama ve publishing
- Integrated launch file (manual_control.launch.py)
- Test script (test_manual_control.sh)

**Sonraki eklentiler**: SLAM, Sensör entegrasyonu, Autonomous navigation, LLM AI integration