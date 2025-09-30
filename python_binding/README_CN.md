# RTAB-Map Python 绑定

**使用 pybind11 为官方 RTAB-Map C++ 库提供的真实 Python 绑定**

本包为 RTAB-Map（基于实时外观的建图）提供真正的 Python 绑定，这是一个基于增量外观回环检测器的 RGB-D 图优化 SLAM 方法。与模拟实现不同，这些绑定直接与实际的 RTAB-Map C++ 库接口。

## 功能特性

### 核心 SLAM 功能
- **完整 RGB-D SLAM**：具有回环检测的完整 SLAM 处理
- **实时性能**：直接 C++ 集成以获得最大性能
- **图优化**：访问优化的位姿图和约束
- **内存管理**：实时操作的智能内存管理
- **回环检测**：鲁棒的外观回环检测
- **综合统计**：详细的性能和图构建统计

### 传感器支持
- **RGB-D 相机**：支持深度相机（RealSense、Kinect 等）
- **立体相机**：具有深度计算的立体视觉
- **相机标定**：完整的相机模型支持，包括畸变校正
- **IMU 集成**：惯性测量单元数据融合
- **GPS 集成**：用于全局定位的 GPS 数据

### 数据结构
- **SensorData**：所有传感器输入的统一容器
- **Transform**：具有完整矩阵运算的 3D 变换
- **CameraModel**：相机内参和标定参数
- **Parameters**：综合参数管理系统
- **Statistics**：性能监控和分析

## 安装

### 前置条件

1. **RTAB-Map C++ 库**：您必须先安装 RTAB-Map
   ```bash
   # 在 Ubuntu/Debian 上
   sudo apt install ros-*-rtabmap ros-*-rtabmap-ros
   
   # 或从源码构建
   git clone https://github.com/introlab/rtabmap.git
   cd rtabmap/build
   cmake ..
   make -j4
   sudo make install
   ```

2. **系统依赖**：
   ```bash
   # Ubuntu/Debian
   sudo apt install python3-dev python3-pip cmake pkg-config
   sudo apt install libopencv-dev libeigen3-dev
   
   # macOS
   brew install python cmake pkg-config opencv eigen
   ```

3. **Python 依赖**：
   ```bash
   pip install numpy>=1.19.0 opencv-python>=4.5.0 pybind11>=2.6.0
   ```

### 构建和安装

1. **从源码安装**（推荐）：
   ```bash
   cd rtabmap/python_binding
   pip install -r requirements.txt
   pip install .
   ```

2. **开发安装**：
   ```bash
   pip install -e .  # 可编辑安装
   pip install -e ".[dev]"  # 包含开发工具
   ```

3. **使用 CMake**（高级）：
   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

## 快速开始

### 基础 RGB-D SLAM

```python
import rtabmap_python as rtab
import numpy as np
import cv2

# 初始化 RTAB-Map
slam = rtab.Rtabmap()

# 配置参数
params = rtab.ParametersMap()
params[rtab.Param.kRGBDEnabled] = "true"
params[rtab.Param.kRtabmapTimeThr] = "700"
params[rtab.Param.kRtabmapLoopThr] = "0.11"

# 使用数据库初始化
slam.init(params, "my_map.db")

# 创建相机模型
camera_model = rtab.CameraModel(
    fx=525.0, fy=525.0,
    cx=320.0, cy=240.0
)

# 处理 RGB-D 数据
rgb_image = cv2.imread('rgb.jpg')
depth_image = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)

sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)
odometry_pose = rtab.Transform(0, 0, 0, 0, 0, 0)  # x,y,z,roll,pitch,yaw

# 处理帧
success = slam.process(sensor_data, odometry_pose)

# 获取结果
stats = slam.getStatistics()
poses = slam.getOptimizedPoses()
loop_closure_id = slam.getLoopClosureId()

print(f"处理时间: {stats.getProcessTime():.2f}ms")
print(f"检测到回环: {loop_closure_id}")
print(f"位姿数量: {len(poses)}")

# 关闭并保存
slam.close(database_saved=True)
```

### 使用 NumPy 数组

```python
import rtabmap_python as rtab
import numpy as np

# 初始化 SLAM
slam = rtab.Rtabmap()
params = rtab.ParametersMap()
params[rtab.Param.kRGBDEnabled] = "true"
slam.init(params)

# 相机模型
camera_model = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)

# RGB-D 数据作为 numpy 数组
rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)

# 变换
pose = rtab.Transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.1)

# 使用便捷方法处理
success = slam.processRGBD(rgb, depth, camera_model, pose)

# 获取统计信息
stats = slam.getStatistics()
print(f"提取的特征: {stats.getFeaturesExtracted()}")
print(f"工作内存大小: {stats.getWorkingMemorySize()}")
```

### 参数管理

```python
import rtabmap_python as rtab

# 获取所有默认参数
default_params = rtab.Parameters.getDefaultParameters()
print(f"总参数数量: {len(default_params)}")

# 创建参数映射
params = rtab.ParametersMap()

# 设置 RGB-D SLAM 参数
params[rtab.Param.kRGBDEnabled] = "true"
params[rtab.Param.kRGBDLinearUpdate] = "0.1"
params[rtab.Param.kRGBDAngularUpdate] = "0.1"

# 设置特征检测参数
params[rtab.Param.kKpMaxFeatures] = "400"
params[rtab.Param.kKpDetectorStrategy] = "6"  # GFTT/BRIEF

# 设置回环检测参数
params[rtab.Param.kRtabmapLoopThr] = "0.11"
params[rtab.Param.kRtabmapMaxRetrieved] = "2"

# 设置内存管理
params[rtab.Param.kMemRehearsalSimilarity] = "0.6"

# 验证参数
validated_params = rtab.Parameters.parse(params)
print("参数验证成功！")
```

## API 参考

### 核心类

#### Rtabmap
主要的 SLAM 处理类。

```python
slam = rtab.Rtabmap()
slam.init(parameters, database_path)
success = slam.process(sensor_data, odometry_pose)
stats = slam.getStatistics()
poses = slam.getOptimizedPoses()
slam.close()
```

#### SensorData
传感器输入容器。

```python
# RGB-D 构造函数
sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)

# 从 numpy 数组创建
sensor_data = rtab.SensorData.create(rgb_array, depth_array, camera_model)

# 访问数据
rgb = sensor_data.imageRaw()
depth = sensor_data.depthRaw()
```

#### Transform
3D 变换矩阵。

```python
# 从位置和欧拉角
transform = rtab.Transform(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)

# 从 4x4 矩阵
transform = rtab.Transform.fromEigen4d(matrix)

# 运算
inverse = transform.inverse()
combined = transform1 * transform2
distance = transform1.getDistance(transform2)
```

#### CameraModel
相机标定参数。

```python
# 基础构造函数
camera = rtab.CameraModel(fx=525, fy=525, cx=320, cy=240)

# 从标定矩阵
camera = rtab.CameraModel.fromEigen(K_matrix, D_vector, image_size)

# 投影运算
points_2d = camera.project(points_3d)
points_3d = camera.reproject(points_2d, depths)
```

### 配置

#### 关键参数

```python
# 核心 RTAB-Map 参数
rtab.Param.kRtabmapTimeThr          # 时间阈值 (ms)
rtab.Param.kRtabmapLoopThr          # 回环检测阈值
rtab.Param.kRtabmapMaxRetrieved     # 最大检索节点数

# RGB-D SLAM 参数
rtab.Param.kRGBDEnabled             # 启用 RGB-D 模式
rtab.Param.kRGBDLinearUpdate        # 线性更新阈值 (m)
rtab.Param.kRGBDAngularUpdate       # 角度更新阈值 (rad)

# 特征检测参数
rtab.Param.kKpMaxFeatures           # 最大特征数
rtab.Param.kKpDetectorStrategy      # 特征检测器类型

# 内存管理参数
rtab.Param.kMemRehearsalSimilarity  # 排练相似度阈值
```

## 示例

`examples/` 目录包含综合示例：

- **`basic_slam_example.py`**：完整的 RGB-D SLAM 工作流程
- **`camera_integration_example.py`**：真实相机集成
- **`parameter_tuning_example.py`**：参数优化
- **`localization_example.py`**：定位模式使用

## 性能优化建议

1. **参数调优**：根据您的环境调整参数
   - 为实时约束增加 `TimeThr`
   - 为更快处理减少 `MaxFeatures`
   - 调整 `LoopThr` 以控制检测灵敏度

2. **内存管理**：监控内存使用
   - 使用 `getMemoryUsed()` 跟踪内存
   - 根据可用 RAM 调整 `STMSize`
   - 为建图启用 `IncrementalMemory`

3. **相机标定**：正确的标定至关重要
   - 使用高质量的标定数据
   - 包含畸变参数
   - 如需要验证校正

## 故障排除

### 常见问题

**导入错误**：`ImportError: No module named 'rtabmap_python'`
```bash
# 确保 RTAB-Map C++ 库已安装
pkg-config --exists rtabmap && echo "找到 RTAB-Map" || echo "未找到 RTAB-Map"

# 重新安装绑定
pip install --force-reinstall rtabmap-python
```

**构建错误**：`fatal error: rtabmap/core/Rtabmap.h: No such file`
```bash
# 安装 RTAB-Map 开发头文件
sudo apt install librtabmap-dev  # Ubuntu/Debian
# 或从源码构建 RTAB-Map
```

**运行时错误**：`Segmentation fault`
- 确保 RTAB-Map 库版本与绑定匹配
- 检查相机模型有效性
- 验证传感器数据完整性

**性能问题**：
- 减少 `MaxFeatures` 参数
- 为实时操作增加 `TimeThr`
- 使用 `getMemoryUsed()` 监控内存使用

## 与模拟实现的区别

这是一个**真实实现**，具有以下特点：

✅ **直接 C++ 集成**：与实际 RTAB-Map C++ 库接口  
✅ **完整 SLAM 功能**：具有所有功能的完整 RGB-D SLAM  
✅ **真实性能**：生产就绪的性能特征  
✅ **综合 API**：访问所有 RTAB-Map 功能  
✅ **真实回环检测**：实际的外观回环检测  
✅ **图优化**：真实的位姿图优化  

与模拟 SLAM 行为的模拟实现不同。

## 贡献

1. Fork 仓库
2. 创建功能分支（`git checkout -b feature/amazing-feature`）
3. 提交更改（`git commit -m 'Add amazing feature'`）
4. 推送到分支（`git push origin feature/amazing-feature`）
5. 打开 Pull Request

## 许可证

本项目采用与 RTAB-Map 相同的许可证（BSD 许可证）。

## 引用

如果您在研究中使用这些绑定，请引用原始 RTAB-Map 论文：

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

## 联系方式

- **问题反馈**：[GitHub Issues](https://github.com/introlab/rtabmap/issues)
- **文档**：[RTAB-Map Wiki](http://introlab.github.io/rtabmap)
- **原始项目**：[RTAB-Map](https://github.com/introlab/rtabmap)
