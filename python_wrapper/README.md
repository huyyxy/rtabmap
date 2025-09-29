# RTAB-Map Python 包装器

RTAB-Map（实时基于外观的映射）的全面Python包装器，为RGB-D SLAM功能提供易于使用的Python绑定。

## 概述

RTAB-Map是一种基于增量式基于外观的闭环检测器的RGB-D图SLAM方法。这个Python包装器提供了一个简洁、Python风格的接口来访问RTAB-Map的核心功能，使得将SLAM能力集成到Python应用中变得容易。

## 功能特性

### 核心功能
- **SLAM处理**: 完整的RGB-D SLAM流程，包含闭环检测
- **视觉里程计**: 从传感器数据实时估计位姿
- **图优化**: 使用各种后端的位姿图优化
- **内存管理**: 智能内存管理以实现实时性能
- **闭环检测**: 基于外观的闭环检测
- **统计收集**: 全面的性能和映射统计

### 传感器支持
- **RGB相机**: 网络摄像头和图像序列支持
- **RGB-D相机**: 深度相机集成（模拟和真实设备）
- **立体相机**: 带深度计算的立体视觉
- **相机标定**: 完整的相机模型支持，包含畸变校正

### 数据结构
- **SensorData**: 所有传感器输入的统一容器
- **Transform**: 3D变换矩阵，带转换工具
- **CameraModel**: 相机内参和标定
- **Parameters**: 全面的参数管理系统
- **Statistics**: 性能监控和分析

## 安装

### 先决条件
- Python 3.7+
- OpenCV (cv2)
- NumPy
- (可选) Matplotlib 用于可视化

### 安装依赖
```bash
pip install opencv-python numpy matplotlib
```

### 安装RTAB-Map Python包装器
```bash
# 克隆或复制rtabmap_python包到你的项目
cp -r rtabmap_python/ /path/to/your/project/
```

## 快速开始

### 基础SLAM示例

```python
import rtabmap_python as rtab
import numpy as np
import cv2

# 初始化RTAB-Map
slam = rtab.RTABMap()

# 配置参数
params = rtab.Parameters()
params.set('RGBD/Enabled', 'true')
params.set('Rtabmap/TimeThr', '700')
params.set('Rtabmap/LoopThr', '0.11')

# 使用数据库初始化
slam.init(params.get_all_parameters(), "my_map.db")

# 创建相机模型
camera_model = rtab.CameraModel(
    fx=525.0, fy=525.0,
    cx=320.0, cy=240.0,
    image_size=(640, 480)
)

# 处理RGB-D数据
rgb_image = cv2.imread('rgb.jpg')
depth_image = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)

sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)
odometry_pose = rtab.Transform(0, 0, 0, 0, 0, 0)  # x,y,z,roll,pitch,yaw

# 处理帧
success = slam.process(sensor_data, odometry_pose)

# 获取结果
stats = slam.get_statistics()
poses = slam.get_optimized_poses()
loop_closure_id = slam.get_loop_closure_id()

# 关闭并保存
slam.close(database_saved=True)
```

### 相机接口示例

```python
import rtabmap_python as rtab

# 来自网络摄像头的RGB相机
rgb_camera = rtab.CameraRGB(device_id=0)
rgb_camera.init()

for sensor_data in rgb_camera:
    image = sensor_data.image_raw()
    print(f"捕获图像: {image.shape}")
    break

# RGB-D相机（模拟）
rgbd_camera = rtab.CameraRGBD(device_id=0)
rgbd_camera.init()

sensor_data = rgbd_camera.take_image()
rgb = sensor_data.image_raw()
depth = sensor_data.depth_raw()

# 立体相机
stereo_camera = rtab.CameraStereo(
    device_id_left=0, 
    device_id_right=1,
    baseline=0.12,
    compute_depth=True
)
stereo_camera.init()

sensor_data = stereo_camera.take_image()
left_image = sensor_data.image_raw()
computed_depth = sensor_data.depth_raw()
```

## API参考

### 核心类

#### RTABMap
主SLAM处理类。

```python
slam = rtab.RTABMap()
slam.init(parameters, database_path)
success = slam.process(sensor_data, odometry_pose)
stats = slam.get_statistics()
poses = slam.get_optimized_poses()
slam.close()
```

**关键方法:**
- `init(parameters, database_path)`: 初始化SLAM系统
- `process(sensor_data, pose)`: 处理传感器帧
- `get_statistics()`: 获取性能统计
- `get_optimized_poses()`: 获取优化后的位姿图
- `get_loop_closure_id()`: 获取检测到的闭环ID
- `close()`: 关闭并保存数据库

#### SensorData
所有传感器输入的容器。

```python
# RGB-D构造函数
sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)

# 仅外观构造函数  
sensor_data = rtab.SensorData(image=image)

# 访问数据
rgb = sensor_data.image_raw()
depth = sensor_data.depth_raw()
camera = sensor_data.camera_model()
```

#### Transform
3D变换矩阵。

```python
# 从位置和欧拉角
transform = rtab.Transform(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)

# 从四元数
transform = rtab.Transform(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)

# 从4x4矩阵
transform = rtab.Transform(matrix=np.eye(4))

# 操作
inverse_transform = transform.inverse()
combined = transform1 * transform2
```

#### CameraModel
相机标定和内参。

```python
# 最小构造函数
camera = rtab.CameraModel(fx=525, fy=525, cx=320, cy=240)

# 完整构造函数
camera = rtab.CameraModel(name="camera", image_size=(640,480), K=K_matrix, D=distortion)

# 将3D点投影到2D
points_2d = camera.project_3d_to_2d(points_3d)

# 使用深度将2D点重新投影到3D
points_3d = camera.reproject_to_3d(points_2d, depths)
```

### 相机类

#### CameraRGB
RGB相机接口。

```python
# 从网络摄像头
camera = rtab.CameraRGB(device_id=0)

# 从图像目录
camera = rtab.CameraRGB(image_path="/path/to/images/")

# 从视频文件
camera = rtab.CameraRGB(video_path="/path/to/video.mp4")

camera.init()
sensor_data = camera.take_image()
```

#### CameraRGBD
RGB-D相机接口。

```python
# 从设备
camera = rtab.CameraRGBD(device_id=0)

# 从图像对
camera = rtab.CameraRGBD(rgb_path="/path/to/rgb/", depth_path="/path/to/depth/")

camera.init()
sensor_data = camera.take_image()  # 包含RGB和深度
```

#### CameraStereo
立体相机接口。

```python
camera = rtab.CameraStereo(
    device_id_left=0, 
    device_id_right=1,
    baseline=0.12,
    compute_depth=True
)

camera.init()
sensor_data = camera.take_image()  # 左图像 + 计算深度
```

### 实用类

#### Parameters
参数管理系统。

```python
params = rtab.Parameters()
params.set('RGBD/Enabled', 'true')
params.set('Rtabmap/LoopThr', '0.11')

value = params.get('RGBD/Enabled')
enabled = params.get_bool('RGBD/Enabled')
threshold = params.get_float('Rtabmap/LoopThr')
```

#### Statistics
性能和映射统计。

```python
stats = slam.get_statistics()

process_time = stats.get_process_time()
loop_id = stats.get_loop_closure_id()
memory_size = stats.get_working_memory_size()

# 打印所有统计信息
stats.print_all_statistics()

# 获取性能摘要
summary = stats.get_performance_summary()
```

## 示例

`examples/`目录包含全面的示例：

- **`basic_slam_example.py`**: 完整的SLAM处理流程
- **`camera_example.py`**: 相机接口演示

运行示例：
```bash
cd examples/
python3 basic_slam_example.py
python3 camera_example.py
```

## 配置

### 关键参数

#### RTAB-Map核心
- `Rtabmap/TimeThr`: 时间阈值（毫秒）（默认：700）
- `Rtabmap/LoopThr`: 闭环阈值（默认：0.15）
- `Rtabmap/MaxRetrieved`: 闭环检测时最大检索节点数（默认：2）

#### RGB-D SLAM
- `RGBD/Enabled`: 启用RGB-D SLAM模式（默认：true）
- `RGBD/LinearUpdate`: 线性更新阈值（米）（默认：0.1）
- `RGBD/AngularUpdate`: 角更新阈值（弧度）（默认：0.1）

#### 特征检测
- `Kp/MaxFeatures`: 最大特征数（默认：400）
- `Kp/DetectorStrategy`: 特征检测器（6=GFTT/BRIEF，默认：6）

#### 内存管理
- `Mem/STMSize`: 短期内存大小（默认：10）
- `Mem/IncrementalMemory`: 启用增量内存（默认：true）

### 参数文件示例

```ini
[Rtabmap]
TimeThr=700
LoopThr=0.11
MaxRetrieved=2

[RGBD]
Enabled=true
LinearUpdate=0.1
AngularUpdate=0.1

[Kp]
MaxFeatures=400
DetectorStrategy=6
```

加载参数：
```python
params = rtab.Parameters()
params.read_ini("config.ini")
```

## 性能提示

1. **内存管理**: 根据可用RAM调整`Mem/STMSize`
2. **特征数量**: 减少`Kp/MaxFeatures`以提高处理速度
3. **时间阈值**: 增加`Rtabmap/TimeThr`以实现实时约束
4. **闭环检测**: 调整`Rtabmap/LoopThr`以提高检测灵敏度
5. **更新阈值**: 根据您的场景调整`RGBD/LinearUpdate`和`RGBD/AngularUpdate`

## 故障排除

### 常见问题

**导入错误**
```python
# 确保rtabmap_python在您的Python路径中
import sys
sys.path.append('/path/to/rtabmap_python')
import rtabmap_python as rtab
```

**相机初始化失败**
- 检查相机设备ID
- 验证图像路径是否存在
- 确保相机访问权限正确

**SLAM性能不佳**
- 检查相机标定
- 调整特征检测参数
- 验证环境中的纹理是否充足
- 监控内存使用情况

**闭环检测问题**
- 调整`Rtabmap/LoopThr`阈值
- 确保足够的视觉重叠
- 检查特征提取参数

## 限制

这是一个模拟/包装实现，具有以下限制：

1. **仅模拟**: 核心SLAM算法是简化的模拟
2. **无真实C++集成**: 不直接与实际的RTAB-Map C++库接口
3. **有限优化**: 图优化被简化
4. **设备支持**: 相机支持是基本的/模拟的

对于生产环境，请考虑：
- 使用pybind11集成实际的RTAB-Map C++库
- 使用真实的传感器驱动（RealSense、Kinect等）
- 实现适当的图优化后端

## 贡献

1. Fork仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 打开拉取请求

## 许可证

本项目根据BSD许可证进行许可 - 请参阅原始RTAB-Map许可证了解更多详情。

## 致谢

- **RTAB-Map**: 原始RTAB-Map库，由Mathieu Labbe开发
- **IntRoLab**: 智能/交互式/集成/跨学科机器人实验室
- **OpenCV**: 计算机视觉库
- **NumPy**: 数值计算库

## 引用

如果您在研究中使用此包装器，请引用原始RTAB-Map论文：

```bibtex
@article{labbe2019rtabmap,
  title={RTAB-Map as an open-source lidar and visual simultaneous localization and mapping library for large-scale and long-term online operation},
  author={Labb{\'e}, Mathieu and Michaud, Fran{\c{c}}ois},
  journal={Journal of Field Robotics},
  volume={36},
  number={2},
  pages={416--446},
  year={2019},
  publisher={Wiley Online Library}
}
```

## 联系

有关问题和支持：
- GitHub Issues: [创建问题](https://github.com/your-repo/rtabmap-python/issues)
- 原始RTAB-Map: [https://github.com/introlab/rtabmap](https://github.com/introlab/rtabmap)
- 文档: [http://introlab.github.io/rtabmap](http://introlab.github.io/rtabmap)
