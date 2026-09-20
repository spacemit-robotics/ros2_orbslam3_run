# ORB-SLAM3 Run

## 项目简介

本包提供基于 640×480 RGB-D 相机的 ORB-SLAM3 ROS 2 节点，可用于机器人视觉里程计和稀疏地图构建。默认使用 Intel RealSense D455 的 640×480@15 标定配置，同时提供 D415 配置。

## 功能特性

**支持：**

- RGB-D 视觉里程计，输出三维和二维里程计
- RGB-D 稀疏地图构建、轨迹与地图点发布
- 保存 ORB-SLAM3 Atlas 和 TUM 格式轨迹
- 从 `CameraInfo` 动态读取 640×480 相机内参与畸变参数
- 从 TF 动态读取机器人底盘到彩色相机光学坐标系的外参
- D455、D415 的 640×480@15 参数模板
- 可选里程计或地图坐标变换发布

## 快速开始

### 环境准备

- ROS 2 Humble
- ORB-SLAM3 和 Pangolin（默认安装前缀为 `/opt/orbslam3`）
- SpacemiT OpenCV（RVV 构建默认安装前缀为 `/opt/opencv-spacemit`）
- RGB-D 相机驱动，彩色图和对齐深度图均须为 640×480

**必需话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 640×480 彩色图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机标定信息 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 640×480、对齐到彩色图像的深度图 |

相机驱动还需发布从 `base_footprint` 到彩色相机光学坐标系的完整 TF 链， 见 linksee 的 tf 变换。

安装依赖以及 orbslam3 的库

```bash
sudo apt install git cmake ninja-build build-essential patchelf \
  libeigen3-dev opencv-spacemit=4.14.0-2bb4 \
  libboost-serialization-dev libssl-dev libsuitesparse-dev \
  libglew-dev libepoxy-dev libx11-dev libwayland-dev \
  libjpeg-dev libpng-dev libtiff-dev wget
```

```bash
wget https://archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/orbslam3_rvv/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/orbslam3.tar.gz
sudo tar xzf orbslam3.tar.gz -C /opt
echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

### 构建编译

在包含本包的 ROS 2 工作空间中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam3_run \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

非 RISC-V 主机调试时，需关闭默认的 RVV 编译选项：

```bash
colcon build --packages-select orbslam3_run \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_USE_RVV=OFF
```

### 启动相机

以下示例使用 RealSense 640×480@15 RGB-D 数据流：

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true enable_sync:=true
```

### 运行示例

**视觉里程计：**

```bash
ros2 launch orbslam3_run rgbd_odometry.launch.py
```

**SLAM 建图：**

```bash
ros2 launch orbslam3_run rgbd_slam.launch.py
```

默认使用 D455 参数模板。相机内参、畸变、图像尺寸在运行时从
`/camera/color/camera_info` 读取。使用 D415 时分别指定对应模板，以匹配其
双目基线等参数：

```bash
ros2 launch orbslam3_run rgbd_odometry.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_run)/share/orbslam3_run/config/d415_rgbd.yaml

ros2 launch orbslam3_run rgbd_slam.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_run)/share/orbslam3_run/config/d415_rgbd_slam.yaml
```

## 详细使用

### 视觉里程计

`rgbd_odometry.launch.py` 启动 `orbslam3_odometry_node`。三维里程计保留完整位姿；二维里程计仅保留 x、y 和 yaw，适合二维导航。跟踪状态值为 `2` 时表示跟踪正常。

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/orbslam3/odom` | `nav_msgs/Odometry` | 三维视觉里程计 |
| `/orbslam3/odom_2d` | `nav_msgs/Odometry` | 二维平面视觉里程计 |
| `/orbslam3/tracking_state` | `std_msgs/UInt8` | ORB-SLAM3 跟踪状态 |

### SLAM 建图

`rgbd_slam.launch.py` 启动 `orbslam3_slam_node`。ORB-SLAM3 生成稀疏视觉地图，不直接生成二维占据栅格；Nav2 的障碍物代价地图应使用深度点云或激光雷达数据。

**发布接口：**

| 名称 | 类型 | 说明 |
|------|------|------|
| `/orbslam3_slam/pose` | `geometry_msgs/PoseStamped` | 当前全局位姿 |
| `/orbslam3_slam/path` | `nav_msgs/Path` | 建图轨迹 |
| `/orbslam3_slam/map_points` | `sensor_msgs/PointCloud2` | 当前跟踪到的稀疏地图点 |
| `/orbslam3_slam/tracking_state` | `std_msgs/UInt8` | 跟踪状态，`2` 表示正常 |
| `orb_map -> base_footprint` | TF | 默认发布的地图坐标变换 |

**服务：**

| 服务 | 类型 | 说明 |
|------|------|------|
| `/orbslam3_slam/finish_mapping` | `std_srvs/srv/Trigger` | 结束建图并保存结果 |
| `/orbslam3_slam/reset_map` | `std_srvs/srv/Trigger` | 清空当前活动地图 |

保存建图结果：

```bash
ros2 service call /orbslam3_slam/finish_mapping std_srvs/srv/Trigger '{}'
```

默认保存目录为 `~/.ros/orbslam3_slam`，包含 `orbslam3_atlas.osa`、`CameraTrajectory.txt` 和 `KeyFrameTrajectory.txt`。

### 启动参数

所有话题、坐标系和保存目录均可通过 launch 参数覆盖。查看完整参数：

```bash
ros2 launch orbslam3_run rgbd_odometry.launch.py --show-args
ros2 launch orbslam3_run rgbd_slam.launch.py --show-args
```

## 常见问题

1. **持续提示图像尺寸错误**：确认 `CameraInfo`、彩色图和对齐深度图均为 640×480，本包不执行图像缩放。
2. **跟踪状态不为 2**：检查 RGB-D 时间同步、深度和彩色图对齐，并确认场景中有足够纹理和光照。
3. **启动时找不到 ORB-SLAM3**：确认 `/opt/orbslam3` 已安装库、头文件和 `share/orbslam3/ORBvoc.txt`；使用其他前缀时设置 `ORB_SLAM3_PREFIX`。
4. **没有里程计 TF**：里程计模式默认 `publish_tf:=false`，需要时在启动命令中显式设置为 `true`。
5. **一直等待相机外参 TF**：确认 `base_frame` 到彩色图像消息中 `frame_id` 的 TF 链完整，或通过启动参数修改 `base_frame`。

## 版本与发布

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.1.0 | 2026-09 | 支持 640×480 RGB-D 里程计和 SLAM 建图 |

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
