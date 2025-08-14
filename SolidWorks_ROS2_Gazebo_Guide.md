# SolidWorks Mobile Robot ROS2 Gazebo Entegrasyonu Rehberi

Bu rehber, SolidWorks'te tasarlanmış bir mobile robot'un ROS2 Jazzy ve Gazebo Ionic ortamında çalıştırılması sürecinde karşılaşılan sorunları ve çözümlerini detaylandırmaktadır.

## 📋 İçindekiler
1. [Proje Genel Bakış](#proje-genel-bakış)
2. [Karşılaşılan Ana Sorunlar](#karşılaşılan-ana-sorunlar)
3. [Çözüm Adımları](#çözüm-adımları)
4. [ROS2 Uyumluluk Rehberi](#ros2-uyumluluk-rehberi)
5. [Gazebo Entegrasyonu](#gazebo-entegrasyonu)
6. [RViz Konfigürasyonu](#rviz-konfigürasyonu)
7. [Dikkat Edilmesi Gerekenler](#dikkat-edilmesi-gerekenler)
8. [Test ve Doğrulama](#test-ve-doğrulama)

---

## 🤖 Proje Genel Bakış

**Robot Özellikleri:**
- **Platform**: Tekerlekli mobil robot
- **Tasarım**: SolidWorks
- **Format**: URDF (SolidWorks to URDF Exporter)
- **Hedef**: ROS2 Jazzy + Gazebo Ionic + Ubuntu 24.04 (WSL)

**Robot Bileşenleri:**
- `base_link`: Ana gövde
- `leftWheeltLink`, `rightWheelLink`: Tekerlekler (continuous joint)
- `frontCasterLink`, `backCasterLink`: Caster tekerlekleri (fixed joint)
- `outerBodyLink`: Dış gövde (fixed joint)
- `trayLink`: Yükselebilen tepsi (prismatic joint)

---

## ⚠️ Karşılaşılan Ana Sorunlar

### 1. **ROS1 vs ROS2 Uyumsuzluk**
```xml
<!-- SORUN: ROS1 formatı -->
<buildtool_depend>catkin</buildtool_depend>
<depend>roslaunch</depend>
<depend>rviz</depend>

<!-- ÇÖZÜM: ROS2 formatı -->
<buildtool_depend>ament_cmake</buildtool_depend>
<exec_depend>rviz2</exec_depend>
```

### 2. **Launch Dosyası Format Sorunu**
```xml
<!-- SORUN: ROS1 XML launch -->
<launch>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
</launch>

<!-- ÇÖZÜM: ROS2 Python launch -->
```

### 3. **Gazebo Mesh Yolu Sorunu**
```xml
<!-- SORUN: Gazebo mesh bulamıyor -->
filename="package://mobile_robot/meshes/base_link.STL"

<!-- ÇÖZÜM: Tam yol -->
filename="file:///home/user/workspace/install/mobile_robot/share/mobile_robot/meshes/base_link.STL"
```

### 4. **Dartsim Fizik Motoru Limitasyonu**
```
[Dbg] Mesh construction from an SDF has not been implemented yet for dartsim
```
**Sonuç**: Mesh collision'lar çalışmıyor, sadece visual mesh'ler görünüyor.

---

## 🔧 Çözüm Adımları

### Adım 1: Workspace Hazırlama
```bash
# ROS2 workspace oluştur
mkdir -p ~/mobile_robot_ws/src
cd ~/mobile_robot_ws/src

# SolidWorks URDF paketini kopyala
cp -r /path/to/mobile_robot ./
```

### Adım 2: package.xml Güncelleme
```xml
<?xml version="1.0"?>
<package format="3">
  <name>mobile_robot</name>
  <version>1.0.0</version>
  <description>
    URDF Description package for mobile_robot.
    This package contains configuration data, 3D models and launch files
    for mobile_robot robot
  </description>
  <author>TODO</author>
  <maintainer email="TODO@email.com">TODO</maintainer>
  <license>BSD</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_ros_pkgs</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>xacro</exec_depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Adım 3: CMakeLists.txt Güncelleme
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

### Adım 4: ROS2 Launch Dosyaları Oluşturma

**display.launch.py** (RViz için):
```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('mobile_robot'))
    urdf_file_name = 'mobile_robot.urdf'
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)
    
    # Launch configuration variables specific to simulation
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare the launch arguments
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Specify the actions
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'config', 'mobile_robot.rviz')])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any conditioned actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
```

**gazebo.launch.py** (Gazebo için):
```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('mobile_robot'))
    urdf_file_name = 'mobile_robot.urdf'
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)
    
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot')
        
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    # Start Gazebo with resource path
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_path, '..')})

    # Spawn robot using ros_gz_sim
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                  '-entity', 'mobile_robot',
                  '-x', x_pose,
                  '-y', y_pose,
                  '-z', '0.1'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add the actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    
    # Delay spawning to allow Gazebo to start
    ld.add_action(TimerAction(
        period=3.0,
        actions=[spawn_entity_cmd]
    ))

    return ld
```

### Adım 5: URDF Mesh Yollarını Düzeltme

**Problem**: Gazebo mesh dosyalarını bulamıyor
```xml
<!-- SolidWorks çıktısı -->
<mesh filename="package://mobile_robot/meshes/base_link.STL" />
```

**Çözüm**: Tam yol kullanma
```bash
# URDF'de tüm mesh yollarını güncelle
sed -i 's|package://mobile_robot/meshes/|file:///home/yalcinolgac/mobile_robot_ws/install/mobile_robot/share/mobile_robot/meshes/|g' urdf/mobile_robot.urdf
```

### Adım 6: Gazebo Materyallerini Ekleme

URDF sonuna ekle:
```xml
<!-- Gazebo material properties -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <selfCollide>true</selfCollide>
</gazebo>

<gazebo reference="leftWheeltLink">
  <material>Gazebo/Black</material>
  <mu1>0.5</mu1>
  <mu2>0.5</mu2>
  <selfCollide>true</selfCollide>
</gazebo>

<gazebo reference="rightWheelLink">
  <material>Gazebo/Black</material>
  <mu1>0.5</mu1>
  <mu2>0.5</mu2>
  <selfCollide>true</selfCollide>
</gazebo>
```

---

## 🎯 ROS2 Uyumluluk Rehberi

### 1. Package Format Kontrolü
```xml
<!-- ROS2 için format="3" kullan -->
<package format="3">
```

### 2. Build System
```xml
<!-- ROS1: catkin -->
<buildtool_depend>catkin</buildtool_depend>

<!-- ROS2: ament_cmake -->
<buildtool_depend>ament_cmake</buildtool_depend>
```

### 3. Dependencies
```xml
<!-- ROS1 → ROS2 Çeviriler -->
<depend>rviz</depend>          → <exec_depend>rviz2</exec_depend>
<depend>roslaunch</depend>     → (Launch dosyaları Python'a çevrilir)
<depend>gazebo</depend>        → <exec_depend>gazebo_ros_pkgs</exec_depend>
```

### 4. Launch Dosyaları
- **ROS1**: XML format (.launch)
- **ROS2**: Python format (.launch.py)

---

## 🌐 Gazebo Entegrasyonu

### 1. Gazebo Ionic Gereksinimleri
```bash
# Gerekli paketleri kontrol et
ros2 pkg list | grep gazebo
ros2 pkg list | grep ros_gz
```

### 2. Mesh Dosyaları
**Sorun**: `package://` protokolü Gazebo'da çalışmıyor
**Çözüm**: Tam dosya yolu kullan
```xml
filename="file:///tam/yol/mesh_dosyasi.STL"
```

### 3. Fizik Motoru Limitasyonları
- **Dartsim**: Mesh collision desteklemiyor
- **Bullet**: Mesh collision destekliyor (alternatif)
- **Visual mesh'ler**: Her iki motorda da çalışıyor

### 4. Materyal Konfigürasyonu
```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>  <!-- Sürtünme katsayısı -->
  <mu2>0.2</mu2>
  <selfCollide>true</selfCollide>
</gazebo>
```

---

## 📊 RViz Konfigürasyonu

### 1. RViz Config Dosyası Oluşturma
```yaml
# config/mobile_robot.rviz
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Value: /robot_description
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
  Global Options:
    Fixed Frame: base_link
```

### 2. Joint State Publisher GUI
Robot eklemlerini manuel kontrol için:
```python
start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen')
```

---

## ⚠️ Dikkat Edilmesi Gerekenler

### 1. **Dosya Yolu Sorunları**
- Windows WSL kullanırken yol ayrımcıları
- Build sonrası install klasöründeki yollar
- Mesh dosyalarının build sonrası konumu

### 2. **Joint Konfigürasyonu**
```xml
<!-- Continuous joint'ler için limit tanımla -->
<joint name="wheelJoint" type="continuous">
  <limit effort="100" velocity="100" />
</joint>

<!-- Prismatic joint'ler için range tanımla -->
<joint name="trayJoint" type="prismatic">
  <limit lower="0" upper="0.25" effort="1" velocity="0.1" />
</joint>
```

### 3. **Inertia Değerleri**
SolidWorks'ten gelen inertia değerleri genellikle doğrudur, ancak:
- Root link (base_link) inertia uyarısı normal
- Fizik simülasyonu için gerekli
- KDL parser uyarısı görmezden gelinebilir

### 4. **Mesh Dosyası Formatları**
- **STL**: SolidWorks varsayılan çıktısı
- **DAE**: Materyal bilgisi içerir (tercih edilen)
- **OBJ**: Alternatif format

### 5. **Build Sırası**
```bash
# Doğru sıra
cd workspace
colcon build          # Önce build
source install/setup.bash  # Sonra source
ros2 launch ...       # Son olarak launch
```

---

## 🧪 Test ve Doğrulama

### 1. RViz Testi
```bash
ros2 launch mobile_robot display.launch.py
```
**Kontrol listesi:**
- [ ] Robot görünüyor
- [ ] Joint State Publisher GUI açılıyor
- [ ] Eklemler hareket ettirilebiliyor
- [ ] TF frame'leri görünüyor

### 2. Gazebo Testi
```bash
ros2 launch mobile_robot gazebo.launch.py
```
**Kontrol listesi:**
- [ ] Gazebo açılıyor
- [ ] Robot spawn ediliyor (`Entity creation successful`)
- [ ] Mesh'ler yükleniyor (görsel)
- [ ] Entity Tree'de robot görünüyor

### 3. Entegrasyon Testi
```bash
# Terminal 1: Gazebo
ros2 launch mobile_robot gazebo.launch.py

# Terminal 2: RViz
ros2 launch mobile_robot display.launch.py

# Terminal 3: Robot topics kontrol
ros2 topic list
ros2 topic echo /robot_description
```

### 4. Ortak Hata Mesajları ve Çözümleri

**Hata**: `Package 'mobile_robot' not found`
```bash
# Çözüm
source install/setup.bash
```

**Hata**: `Unable to find file [package://mobile_robot/meshes/...]`
```bash
# Çözüm: URDF'de mesh yollarını güncelle
filename="file:///tam/yol/mesh_dosyasi.STL"
```

**Hata**: `Mesh construction from an SDF has not been implemented`
```
# Bu normal - dartsim limitation
# Visual mesh'ler çalışır, collision mesh'ler çalışmaz
```

---

## 📈 Performans Optimizasyonu

### 1. Mesh Optimizasyonu
- Polygon sayısını düşür
- LOD (Level of Detail) kullan
- Simple collision geometriler ekle

### 2. Gazebo Performansı
```python
# Düşük verbose level
cmd=['gz', 'sim', '-r', '-v', '1', 'empty.sdf']

# GPU rendering kullan
additional_env={'OGRE_RTT_MODE': 'Copy'}
```

---

## 🎯 Sonuç

Bu rehber, SolidWorks'ten ROS2 Gazebo'ya geçiş sürecindeki tüm kritik adımları kapsamaktadır. Ana başarı faktörleri:

1. **Doğru format dönüşümü** (ROS1 → ROS2)
2. **Mesh yolu problemi çözümü**
3. **Launch dosyası modernizasyonu**  
4. **Gazebo uyumluluk ayarları**

**Sonuç**: SolidWorks mobile robot başarıyla ROS2 Jazzy + Gazebo Ionic ortamında çalışır hale getirilmiştir.

---

## 📚 İleri Çalışmalar

1. **Robot Kontrolü**: Differential drive controller ekleme
2. **Sensör Entegrasyonu**: Lidar, kamera, IMU
3. **Navigation**: Nav2 stack entegrasyonu
4. **Autonomous Görevler**: SLAM, path planning

---

*Bu doküman, SolidWorks mobile robot ROS2 entegrasyonu sürecinde yaşanan tüm sorunları ve çözümlerini içermektedir. Gelecek projeler için referans olarak kullanılabilir.*