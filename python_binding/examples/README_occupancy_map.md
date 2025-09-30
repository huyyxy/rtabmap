# RTAB-Map Python 2D Occupancy Map Examples

这个目录包含了使用RTAB-Map Python绑定从RGB-D数据集生成2D占用地图的示例。

## 文件说明

### 1. `simple_occupancy_map.py` (推荐)
最简单的示例，专注于核心功能：
- 加载Freiburg RGB-D数据集
- 使用RTAB-Map进行SLAM处理
- 导出数据库用于地图生成

### 2. `freiburg_occupancy_map.py`
基础示例，包含更多功能：
- 完整的参数配置
- 详细的统计信息
- 错误处理

### 3. `freiburg_occupancy_map_complete.py`
完整示例，包含：
- 自动调用rtabmap-export工具
- 地图可视化功能
- 完整的错误处理

## 使用方法

### 前提条件

1. 确保RTAB-Map已安装并编译了Python绑定
2. 确保rtabmap-export工具可用
3. 准备Freiburg RGB-D数据集

### 运行示例

#### 方法1：使用简单示例（推荐）

```bash
# 进入示例目录
cd python_binding/examples

# 运行简单示例
python simple_occupancy_map.py --dataset ../../rgbd_dataset_freiburg1_room --max_frames 50

# 生成2D占用地图
rtabmap-export --2d_map --output freiburg_map freiburg_slam.db
```

#### 方法2：使用完整示例

```bash
# 运行完整示例（自动生成地图）
python freiburg_occupancy_map_complete.py --dataset ../../rgbd_dataset_freiburg1_room --max_frames 100 --visualize
```

### 参数说明

- `--dataset`: Freiburg数据集路径
- `--max_frames`: 处理的最大帧数
- `--output`: 输出地图名称
- `--visualize`: 是否可视化生成的地图

## 输出文件

运行完成后会生成以下文件：

1. **数据库文件**: `freiburg_slam.db`
   - 包含SLAM处理的所有数据
   - 可用于后续的地图生成和分析

2. **位姿文件**: `freiburg_poses.txt`
   - 包含机器人的轨迹位姿
   - 可用于轨迹分析和可视化

3. **占用地图文件**:
   - `freiburg_map.pgm`: 占用地图图像文件
   - `freiburg_map.yaml`: 地图元数据文件

## 地图格式

生成的占用地图使用标准的ROS地图格式：

- **PGM文件**: 灰度图像，像素值表示占用概率
  - 0: 自由空间
  - 100: 障碍物
  - 255: 未知空间

- **YAML文件**: 包含地图元数据
  - `resolution`: 地图分辨率（米/像素）
  - `origin`: 地图原点坐标
  - `occupied_thresh`: 障碍物阈值
  - `free_thresh`: 自由空间阈值

## 可视化地图

可以使用以下工具可视化生成的占用地图：

### 使用OpenCV（Python）
```python
import cv2
import numpy as np

# 加载地图
map_image = cv2.imread('freiburg_map.pgm', cv2.IMREAD_GRAYSCALE)

# 创建彩色版本
colored_map = cv2.applyColorMap(map_image, cv2.COLORMAP_JET)

# 显示地图
cv2.imshow('Occupancy Map', colored_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

### 使用ROS工具
```bash
# 使用rviz可视化
rosrun map_server map_server freiburg_map.yaml
rviz
```

## 故障排除

### 常见问题

1. **导入错误**: 确保RTAB-Map Python绑定已正确安装
2. **数据集路径错误**: 检查数据集路径是否正确
3. **rtabmap-export未找到**: 确保RTAB-Map工具已安装并在PATH中
4. **内存不足**: 减少处理的帧数或增加系统内存

### 调试技巧

1. 使用较少的帧数进行测试
2. 检查生成的数据库文件大小
3. 查看控制台输出的统计信息
4. 使用rtabmap-databaseViewer查看数据库内容

## 扩展功能

### 自定义参数

可以修改示例中的参数来优化性能：

```python
# 调整地图分辨率
"--Grid/CellSize", "0.02",  # 2cm cell size

# 调整检测器
"--Kp/DetectorStrategy", "5",  # SIFT detector

# 调整循环闭合检测
"--Rtabmap/LoopThr", "0.15",  # 更严格的循环闭合阈值
```

### 处理其他数据集

可以修改代码来处理其他RGB-D数据集：

1. 调整相机参数
2. 修改数据加载函数
3. 调整时间戳处理

## 性能优化

1. **减少帧数**: 使用`--max_frames`参数
2. **跳帧处理**: 修改代码中的跳帧间隔
3. **调整参数**: 根据数据集特点调整RTAB-Map参数
4. **使用GPU**: 如果可用，启用GPU加速

## 参考资料

- [RTAB-Map官方文档](https://github.com/introlab/rtabmap)
- [Freiburg RGB-D数据集](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [ROS地图格式规范](http://wiki.ros.org/map_server)
