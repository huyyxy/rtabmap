# RTAB-Map Python 绑定

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-0.20%2B-orange.svg)](https://github.com/introlab/rtabmap)

**基于 pybind11 的 RTAB-Map C++ 库官方 Python 绑定**

RTAB-Map 是一个基于实时外观的建图（Real-Time Appearance-Based Mapping）的 RGB-D SLAM 库。本 Python 绑定提供了对完整 RTAB-Map C++ 库的直接访问，支持实时 SLAM、回环检测和图优化。

## ✨ 主要特性

- 🚀 **完整 SLAM 功能**：RGB-D SLAM、回环检测、图优化
- ⚡ **高性能**：直接 C++ 集成，实时处理能力
- 📷 **多传感器支持**：RGB-D 相机、立体相机、IMU、GPS
- 🎯 **精确标定**：完整的相机模型和畸变校正
- 📊 **详细统计**：性能监控和 SLAM 分析
- 🔧 **灵活配置**：丰富的参数调优选项

## 📦 安装

### 系统要求

- Python 3.7+
- RTAB-Map C++ 库
- OpenCV 4.5+
- Eigen3
- pybind11

### 快速安装

```bash
# 1. 安装 RTAB-Map C++ 库
# Ubuntu/Debian
sudo apt install ros-*-rtabmap ros-*-rtabmap-ros

# macOS
brew install rtabmap

# 2. 安装 Python 依赖
pip install numpy opencv-python pybind11

# 3. 安装 Python 绑定
cd rtabmap/python_binding
pip install .
```

### 开发安装

```bash
# 可编辑安装
pip install -e .

# 包含开发工具
pip install -e ".[dev]"
```

## 🚀 快速开始

### 基础 RGB-D SLAM

```python
import rtabmap_python as rtab
import numpy as np
import cv2

# 初始化 SLAM
slam = rtab.Rtabmap()

# 配置参数
params = rtab.ParametersMap()
params[rtab.Param.kRGBDEnabled] = "true"
params[rtab.Param.kRtabmapTimeThr] = "700"
params[rtab.Param.kRtabmapLoopThr] = "0.11"

# 初始化
slam.init(params, "my_map.db")

# 创建相机模型
camera = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)

# 处理 RGB-D 数据
rgb = cv2.imread('rgb.jpg')
depth = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)
pose = rtab.Transform(0, 0, 0, 0, 0, 0)

sensor_data = rtab.SensorData(rgb, depth, camera)
success = slam.process(sensor_data, pose)

# 获取结果
stats = slam.getStatistics()
print(f"处理时间: {stats.getProcessTime():.2f}ms")
print(f"回环检测: {slam.getLoopClosureId()}")

# 保存并关闭
slam.close(database_saved=True)
```

### 使用 NumPy 数组

```python
# 创建模拟数据
rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)

# 便捷处理
success = slam.processRGBD(rgb, depth, camera, pose)

# 获取统计信息
stats = slam.getStatistics()
print(f"提取特征: {stats.getFeaturesExtracted()}")
print(f"工作内存: {stats.getWorkingMemorySize()}")
```

## 📚 核心 API

### Rtabmap 类

主要的 SLAM 处理类。

```python
# 初始化
slam = rtab.Rtabmap()
slam.init(parameters, database_path="")

# 处理数据
success = slam.process(sensor_data, odometry_pose)
success = slam.processRGBD(rgb, depth, camera, pose)

# 获取结果
stats = slam.getStatistics()
poses = slam.getLocalOptimizedPoses()
constraints = slam.getLocalConstraints()

# 状态查询
process_time = slam.getLastProcessTime()
loop_closure_id = slam.getLoopClosureId()
wm_size = slam.getWMSize()

# 关闭
slam.close(database_saved=True)
```

### SensorData 类

传感器数据容器。

```python
# 创建传感器数据
sensor_data = rtab.SensorData(rgb, depth, camera, id=0, stamp=0.0)

# 数据访问
rgb_array = sensor_data.imageRaw()
depth_array = sensor_data.depthRaw()
camera_models = sensor_data.cameraModels()

# 数据验证
is_valid = sensor_data.isValid()
has_image = sensor_data.hasImage()
has_depth = sensor_data.hasDepth()
```

### Transform 类

3D 变换操作。

```python
# 创建变换
transform = rtab.Transform(x, y, z, roll, pitch, yaw)
transform = rtab.Transform(matrix_4x4)

# 变换运算
inverse = transform.inverse()
combined = transform1 * transform2
distance = transform1.getDistance(transform2)

# 位置和旋转
x, y, z = transform.x(), transform.y(), transform.z()
roll, pitch, yaw = transform.roll(), transform.pitch(), transform.yaw()
```

### CameraModel 类

相机标定和投影。

```python
# 创建相机模型
camera = rtab.CameraModel(fx, fy, cx, cy, image_size=rtab.Size(640, 480))

# 投影运算
x, y, z = camera.project(u, v, depth)  # 2D -> 3D
u, v = camera.reproject(x, y, z)       # 3D -> 2D

# 图像校正
rectified = camera.rectifyImage(raw_image)
```

### Statistics 类

性能统计和分析。

```python
# 获取统计信息
stats = slam.getStatistics()

# 基础统计
process_time = stats.getProcessTime()
wm_size = stats.getWorkingMemorySize()
features = stats.getFeaturesExtracted()

# 性能摘要
summary = stats.getPerformanceSummary()

# 字典式访问
value = stats["Process/time/ms"]
keys = stats.keys()
```

## ⚙️ 参数配置

### 核心参数

```python
params = rtab.ParametersMap()

# SLAM 参数
params[rtab.Param.kRtabmapTimeThr] = "700"      # 时间阈值 (ms)
params[rtab.Param.kRtabmapLoopThr] = "0.11"     # 回环检测阈值
params[rtab.Param.kRtabmapMaxRetrieved] = "2"   # 最大检索节点

# RGB-D 参数
params[rtab.Param.kRGBDEnabled] = "true"        # 启用 RGB-D
params[rtab.Param.kRGBDLinearUpdate] = "0.1"    # 线性更新阈值
params[rtab.Param.kRGBDAngularUpdate] = "0.1"   # 角度更新阈值

# 特征检测参数
params[rtab.Param.kKpMaxFeatures] = "400"       # 最大特征数
params[rtab.Param.kKpDetectorStrategy] = "6"    # 检测器策略
params[rtab.Param.kKpNndrRatio] = "0.6"         # NNDR 比率

# 内存管理参数
params[rtab.Param.kMemRehearsalSimilarity] = "0.6"  # 排练相似度
params[rtab.Param.kMemImageKept] = "true"           # 保留图像
```

### 参数管理

```python
# 获取默认参数
default_params = rtab.Parameters.getDefaultParameters()

# 获取特定组参数
rgbd_params = rtab.Parameters.getDefaultParametersForGroup("RGBD")

# 参数验证
validated_params = rtab.Parameters.parse(params)
```

## 📖 使用示例

### 完整 SLAM 工作流程

```python
import rtabmap_python as rtab
import numpy as np
import cv2

def run_slam_example():
    # 初始化
    slam = rtab.Rtabmap()
    params = rtab.ParametersMap()
    params[rtab.Param.kRGBDEnabled] = "true"
    params[rtab.Param.kRtabmapTimeThr] = "700"
    slam.init(params, "example_map.db")
    
    # 相机模型
    camera = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)
    
    # 处理循环
    for i in range(100):
        # 模拟数据
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)
        pose = rtab.Transform(i*0.1, 0, 0, 0, 0, 0)
        
        # 处理
        sensor_data = rtab.SensorData.create(rgb, depth, camera, id=i)
        success = slam.process(sensor_data, pose)
        
        # 统计
        stats = slam.getStatistics()
        print(f"帧 {i}: 时间 {stats.getProcessTime():.1f}ms, "
              f"特征 {stats.getFeaturesExtracted()}, "
              f"内存 {stats.getWorkingMemorySize()}")
    
    # 结果
    poses = slam.getLocalOptimizedPoses()
    constraints = slam.getLocalConstraints()
    print(f"总位姿: {len(poses)}, 总约束: {len(constraints)}")
    
    # 导出
    slam.exportPoses("poses.txt", optimized=True, global=True)
    slam.close(database_saved=True)

if __name__ == "__main__":
    run_slam_example()
```

### 相机标定和投影

```python
def camera_projection_example():
    # 创建相机模型
    camera = rtab.CameraModel(
        name="my_camera",
        fx=525.0, fy=525.0,
        cx=320.0, cy=240.0,
        image_size=rtab.Size(640, 480)
    )
    
    # 2D 到 3D 投影
    u, v = 320, 240  # 图像中心
    depth = 1000.0   # 深度值 (mm)
    x, y, z = camera.project(u, v, depth)
    print(f"像素 ({u}, {v}) 深度 {depth} -> 3D点 ({x:.2f}, {y:.2f}, {z:.2f})")
    
    # 3D 到 2D 重投影
    u_proj, v_proj = camera.reproject(x, y, z)
    print(f"3D点 ({x:.2f}, {y:.2f}, {z:.2f}) -> 像素 ({u_proj:.2f}, {v_proj:.2f})")
    
    # 图像校正
    raw_image = cv2.imread("raw_image.jpg")
    rectified_image = camera.rectifyImage(raw_image, cv2.INTER_LINEAR)
    cv2.imwrite("rectified_image.jpg", rectified_image)
```

### 变换操作

```python
def transform_example():
    # 创建变换
    transform1 = rtab.Transform(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    transform2 = rtab.Transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # 变换运算
    combined = transform1 * transform2
    inverse = transform1.inverse()
    distance = transform1.getDistance(transform2)
    
    print(f"组合变换: {combined}")
    print(f"逆变换: {inverse}")
    print(f"距离: {distance:.3f}")
    
    # 插值
    t = 0.5
    interpolated = transform1.interpolate(t, transform2)
    print(f"插值变换: {interpolated}")
```

### 立体相机模型

```python
def stereo_camera_example():
    # 创建立体相机模型
    stereo_camera = rtab.StereoCameraModel(
        name="stereo_camera",
        fx=525.0, fy=525.0,
        cx=320.0, cy=240.0,
        baseline=0.1,  # 10cm 基线
        image_size=rtab.Size(640, 480)
    )
    
    # 访问左右相机
    left_camera = stereo_camera.left()
    right_camera = stereo_camera.right()
    print(f"基线: {stereo_camera.baseline():.3f}m")
    
    # 立体运算
    disparity = 50.0
    depth = stereo_camera.computeDepth(disparity)
    print(f"视差 {disparity} -> 深度 {depth:.3f}m")
    
    depth = 2.0
    disparity = stereo_camera.computeDisparity(depth)
    print(f"深度 {depth}m -> 视差 {disparity:.2f}")
```

## 🔧 性能优化

### 参数调优建议

1. **实时性能优化**
   ```python
   params[rtab.Param.kRtabmapTimeThr] = "500"      # 降低时间阈值
   params[rtab.Param.kKpMaxFeatures] = "200"       # 减少特征数
   params[rtab.Param.kRGBDLinearUpdate] = "0.2"    # 增加更新阈值
   ```

2. **精度优化**
   ```python
   params[rtab.Param.kRtabmapLoopThr] = "0.08"     # 降低回环阈值
   params[rtab.Param.kKpMaxFeatures] = "800"       # 增加特征数
   params[rtab.Param.kKpNndrRatio] = "0.7"         # 提高匹配质量
   ```

3. **内存管理**
   ```python
   params[rtab.Param.kMemRehearsalSimilarity] = "0.5"  # 降低相似度阈值
   params[rtab.Param.kMemImageKept] = "false"          # 不保留图像
   ```

### 监控和调试

```python
def monitor_performance(slam):
    stats = slam.getStatistics()
    
    # 性能指标
    print(f"处理时间: {stats.getProcessTime():.2f}ms")
    print(f"工作内存: {stats.getWorkingMemorySize()}")
    print(f"提取特征: {stats.getFeaturesExtracted()}")
    print(f"匹配特征: {stats.getFeaturesMatched()}")
    print(f"内点数: {stats.getInliers()}")
    
    # 回环检测
    loop_id = slam.getLoopClosureId()
    if loop_id > 0:
        print(f"检测到回环: {loop_id}")
    
    # 内存使用
    memory = slam.getMemory()
    db_memory = memory.getDatabaseMemoryUsed()
    print(f"数据库内存: {db_memory / 1024 / 1024:.2f} MB")
```

## 🐛 故障排除

### 常见问题

**1. 导入错误**
```bash
ImportError: No module named 'rtabmap_python'
```
解决方案：
```bash
# 检查 RTAB-Map 安装
pkg-config --exists rtabmap && echo "找到 RTAB-Map" || echo "未找到 RTAB-Map"

# 重新安装
pip install --force-reinstall rtabmap-python
```

**2. 构建错误**
```bash
fatal error: rtabmap/core/Rtabmap.h: No such file
```
解决方案：
```bash
# Ubuntu/Debian
sudo apt install librtabmap-dev

# 或从源码构建 RTAB-Map
```

**3. 运行时错误**
```bash
Segmentation fault
```
解决方案：
- 确保 RTAB-Map 库版本匹配
- 检查相机模型有效性
- 验证传感器数据完整性

**4. 性能问题**
解决方案：
- 减少 `MaxFeatures` 参数
- 增加 `TimeThr` 参数
- 监控内存使用情况

### 调试技巧

```python
# 启用详细日志
import logging
logging.basicConfig(level=logging.DEBUG)

# 检查数据有效性
def validate_sensor_data(sensor_data):
    if not sensor_data.isValid():
        print("传感器数据无效")
        return False
    
    if not sensor_data.hasImage():
        print("缺少 RGB 图像")
        return False
    
    if not sensor_data.hasDepth():
        print("缺少深度数据")
        return False
    
    return True

# 检查相机模型
def validate_camera_model(camera):
    if not camera.isValidForProjection():
        print("相机模型投影无效")
        return False
    
    if camera.imageWidth() <= 0 or camera.imageHeight() <= 0:
        print("图像尺寸无效")
        return False
    
    return True
```

## 📁 项目结构

```
python_binding/
├── src/                    # 源代码
├── examples/               # 示例代码
├── requirements.txt        # Python 依赖
├── setup.py               # 安装脚本
├── CMakeLists.txt         # CMake 配置
└── README_CN.md          # 中文文档
```

## 🤝 贡献

欢迎贡献代码！请遵循以下步骤：

1. Fork 仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

## 📄 许可证

本项目采用 BSD 许可证，与 RTAB-Map 保持一致。

## 📚 参考资料

- [RTAB-Map 官网](https://github.com/introlab/rtabmap)
- [RTAB-Map 文档](http://introlab.github.io/rtabmap)
- [pybind11 文档](https://pybind11.readthedocs.io/)

## 📖 引用

如果您在研究中使用本绑定，请引用原始 RTAB-Map 论文：

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

## 📞 联系方式

- **问题反馈**: [GitHub Issues](https://github.com/introlab/rtabmap/issues)
- **文档**: [RTAB-Map Wiki](http://introlab.github.io/rtabmap)
- **原始项目**: [RTAB-Map](https://github.com/introlab/rtabmap)

---

**注意**: 这是一个真实的 RTAB-Map C++ 库绑定，提供完整的 SLAM 功能，与模拟实现不同。