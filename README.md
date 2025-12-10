# Hướng dẫn PX4 cho Dự án Drone Bay Qua Mê Cung

## 📁 CẤU TRÚC THỨ MỤC QUAN TRỌNG

### 1. **Tools/simulation/gazebo-classic/sitl_gazebo-classic/**
Thư mục mô phỏng Gazebo Classic (dành cho ROS1 + Ubuntu 20.04)

#### **models/** - Các model drone và sensor
- `iris/` - Drone cơ bản với 4 cánh quạt
- `iris_depth_camera/` - Drone có camera depth
- `iris_vision/` - Drone có camera vision
- `depth_camera/` - Model camera 3D
- `fpv_cam/` - Camera FPV
- `lidar/` - Cảm biến Lidar
- `rplidar/` - Lidar RPLidar
- **`custom_maze_drone/`** - ✨ Drone tùy chỉnh của bạn (vừa tạo)

#### **worlds/** - Các thế giới/môi trường mô phỏng
- `empty.world` - Thế giới trống
- `warehouse.world` - Kho hàng (có tường)
- Bạn có thể tạo world mê cung riêng ở đây

#### **plugins/** - Các plugin Gazebo
- `gazebo_mavlink_interface` - Kết nối PX4 với Gazebo
- `gazebo_motor_model` - Mô phỏng động cơ
- `gazebo_camera_manager_plugin` - Quản lý camera
- `gazebo_lidar_plugin` - Plugin Lidar

---

### 2. **src/modules/** - Module điều khiển drone

#### **Điều khiển Multicopter:**
- **`mc_pos_control/`** - Điều khiển vị trí (X, Y, Z)
  - File quan trọng: `PositionControl.cpp`, `PositionControl.hpp`
  - Sử dụng để bay đến tọa độ cụ thể

- **`mc_att_control/`** - Điều khiển tư thế (roll, pitch, yaw)
  - File: `AttitudeControl.cpp`

- **`mc_rate_control/`** - Điều khiển tốc độ góc

#### **Navigation & Tránh vật cản:**
- **`navigator/`** - Điều hướng, waypoint
  - File quan trọng: `mission.cpp`, `loiter.cpp`
  - Dùng để lập trình đường bay

- **`collision_prevention/`** - Tránh va chạm (nếu có)

#### **Commander & Mode:**
- **`commander/`** - Quản lý chế độ bay (MANUAL, OFFBOARD, AUTO)
  - File: `Commander.cpp`

- **`flight_mode_manager/`** - Quản lý chế độ bay

#### **Sensors:**
- **`sensors/`** - Xử lý dữ liệu sensor (IMU, GPS, Camera)
- **`ekf2/`** - Extended Kalman Filter (ước lượng vị trí, tốc độ)

#### **MAVLink:**
- **`mavlink/`** - Giao tiếp MAVLink với ROS/Ground Station
  - File: `mavlink_receiver.cpp`, `mavlink_messages.cpp`

---

### 3. **msg/** - Định nghĩa Message

Các file `.msg` định nghĩa cấu trúc dữ liệu:
- `ActuatorOutputs.msg` - Output động cơ
- `VehicleAttitude.msg` - Tư thế drone
- `VehicleLocalPosition.msg` - Vị trí local
- `ObstacleDistance.msg` - Khoảng cách vật cản
- `CollisionConstraints.msg` - Ràng buộc va chạm

Bạn có thể thêm message mới cho dự án của mình.

---

### 4. **ROMFS/** - File cấu hình khởi động

- `ROMFS/px4fmu_common/init.d/` - Script khởi động
- `ROMFS/px4fmu_common/init.d/airframes/` - Cấu hình airframe
  - Bạn có thể tạo airframe mới cho drone của mình

---

### 5. **boards/** - Hardware board

Cấu hình cho các board phần cứng (Pixhawk, etc.)
- Nếu dùng simulation, không cần quan tâm nhiều

---

## 🚁 CẤU TRÚC FILE SDF DRONE

### **Các thành phần chính:**

```xml
<model name="drone_name">
  <!-- 1. THÂN DRONE (Base Link) -->
  <link name="base_link">
    <inertial>         <!-- Khối lượng, moment quán tính -->
      <mass>1.5</mass>
      <inertia>...</inertia>
    </inertial>
    <collision>        <!-- Hình dạng va chạm -->
      <geometry>
        <box><size>0.47 0.47 0.11</size></box>
      </geometry>
    </collision>
    <visual>           <!-- Hình dạng 3D -->
      <geometry>
        <mesh><uri>model://iris/meshes/iris.stl</uri></mesh>
      </geometry>
    </visual>
  </link>

  <!-- 2. IMU (Cảm biến quán tính) -->
  <link name="imu_link">
    <pose>0 0 0.02 0 0 0</pose>  <!-- Vị trí tương đối thân -->
  </link>
  <joint name="imu_joint" type="revolute">
    <child>imu_link</child>
    <parent>base_link</parent>
  </joint>

  <!-- 3. CÁNH QUẠT (4 rotors) -->
  <link name="rotor_0">
    <pose>0.13 -0.22 0.023 0 0 0</pose>  <!-- Vị trí cánh quạt 0 -->
    <inertial><mass>0.005</mass></inertial>
    <visual>
      <mesh><uri>model://iris/meshes/iris_prop_ccw.dae</uri></mesh>
    </visual>
  </link>
  <joint name="rotor_0_joint" type="revolute">
    <child>rotor_0</child>
    <parent>base_link</parent>
    <axis><xyz>0 0 1</xyz></axis>  <!-- Quay quanh trục Z -->
  </joint>
  <!-- Tương tự cho rotor_1, rotor_2, rotor_3 -->

  <!-- 4. CAMERA -->
  <include>
    <uri>model://depth_camera</uri>
    <pose>0.1 0 0 0 0 0</pose>  <!-- Camera phía trước 10cm -->
  </include>
  <joint name="camera_joint" type="revolute">
    <child>depth_camera::link</child>
    <parent>base_link</parent>
  </joint>

  <!-- 5. LIDAR/RADAR -->
  <include>
    <uri>model://lidar</uri>
    <pose>0 0 0.05 0 0 0</pose>  <!-- Lidar giữa drone -->
  </include>
  <joint name="lidar_joint" type="revolute">
    <child>lidar::link</child>
    <parent>base_link</parent>
  </joint>

  <!-- 6. GAZEBO PLUGINS -->
  <plugin name="mavlink_interface" filename="libgazebo_mavlink_interface.so">
    <!-- Kết nối PX4 -->
  </plugin>
  <plugin name="motor_model" filename="libgazebo_motor_model.so">
    <!-- Mô phỏng động cơ -->
  </plugin>
</model>
```

---

## 🎯 CÁC BƯỚC LÀM VIỆC VỚI DRONE

### **Bước 1: Tạo Model Drone**
✅ **Đã tạo:** `/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/custom_maze_drone/`

Gồm 2 file:
- `custom_maze_drone.sdf` - Model drone
- `model.config` - Metadata

### **Bước 2: Tạo World Mê Cung**

Tạo file `/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/maze.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="maze_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Tường mê cung - thêm các box làm tường -->
    <model name="wall_1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 4 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 4 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
    <!-- Thêm nhiều tường khác tương tự -->
  </world>
</sdf>
```

### **Bước 3: Build PX4**

```bash
cd /home/nguyenminh/PX4-Autopilot
make px4_sitl_default gazebo-classic
```

### **Bước 4: Chạy Simulation**

```bash
# Chạy với drone tùy chỉnh trong world mê cung
make px4_sitl_default gazebo-classic_custom_maze_drone__maze
```

Hoặc:

```bash
# Chạy PX4 SITL
cd /home/nguyenminh/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic

# Spawn drone trong Gazebo
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

# Chạy Gazebo với world mê cung
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/maze.world
```

### **Bước 5: Kết nối ROS**

Trong terminal mới:

```bash
# Launch MAVROS để kết nối PX4 với ROS
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### **Bước 6: Điều khiển Drone từ ROS**

Tạo node ROS Python để điều khiển:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# Set mode OFFBOARD
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
set_mode(custom_mode='OFFBOARD')

# Arm drone
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
arming(True)

# Gửi lệnh position
pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
pose = PoseStamped()
pose.pose.position.x = 5.0  # Bay đến vị trí (5, 0, 2)
pose.pose.position.y = 0.0
pose.pose.position.z = 2.0
pose_pub.publish(pose)
```

---

## 📚 TÀI LIỆU THAM KHẢO

1. **PX4 User Guide:** https://docs.px4.io/main/en/
2. **PX4 Developer Guide:** https://docs.px4.io/main/en/development/
3. **Gazebo SDF Format:** http://sdformat.org/
4. **MAVROS Documentation:** http://wiki.ros.org/mavros

---

## 🔧 CÁC LỆNH HỮU ÍCH

```bash
# Build PX4 cho SITL
make px4_sitl_default

# Build với Gazebo Classic
make px4_sitl_default gazebo-classic

# Build với drone cụ thể
make px4_sitl_default gazebo-classic_iris

# Clean build
make clean
make distclean

# List các model có sẵn
ls Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# List các world có sẵn
ls Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

---

## 💡 MẸO CHO DỰ ÁN MÊ CUNG

1. **Sử dụng Lidar** để phát hiện tường mê cung
   - Subscribe topic: `/scan` hoặc `/laser/scan`

2. **Sử dụng Depth Camera** để tránh vật cản
   - Subscribe topic: `/camera/depth/points`

3. **Thuật toán pathfinding:**
   - A* algorithm
   - Wall follower algorithm
   - RRT (Rapidly-exploring Random Tree)

4. **Điều khiển:**
   - Sử dụng OFFBOARD mode
   - Gửi position setpoint qua MAVROS
   - Hoặc dùng velocity setpoint để điều khiển mượt hơn

---

Chúc bạn thành công với dự án! 🚁
