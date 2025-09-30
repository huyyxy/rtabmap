# RTAB-Map Python 绑定安装指南

本指南提供了构建和安装真实 RTAB-Map Python 绑定的详细说明。

## 前置条件

### 1. RTAB-Map C++ 库

**最重要的要求**：您必须首先安装 RTAB-Map C++ 库。

#### 选项 A：从包管理器安装（Ubuntu/Debian）

```bash
# 安装 RTAB-Map 和开发头文件
sudo apt update
sudo apt install librtabmap-dev

# 或者如果您使用 ROS
sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros  # ROS Noetic
sudo apt install ros-humble-rtabmap ros-humble-rtabmap-ros  # ROS 2 Humble
```

#### 选项 B：从源码构建（推荐，可获得最新功能）

```bash
# 安装依赖项
sudo apt install cmake git pkg-config libopencv-dev libeigen3-dev
sudo apt install libpcl-dev libg2o-dev libsuitesparse-dev

# 克隆并构建 RTAB-Map
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install

# 更新库缓存
sudo ldconfig
```

### 2. 系统依赖项

#### Ubuntu/Debian
```bash
sudo apt install python3-dev python3-pip cmake pkg-config
sudo apt install libopencv-dev libeigen3-dev
```

#### macOS
```bash
brew install python cmake pkg-config opencv eigen
```

#### Windows
- 安装带有 C++ 支持的 Visual Studio
- 安装 CMake
- 安装 vcpkg 用于依赖项管理
- 使用 vcpkg 构建 RTAB-Map

### 3. Python 依赖项

```bash
pip install numpy>=1.19.0 opencv-python>=4.5.0 pybind11>=2.6.0
```

## 安装方法

### 方法 1：自动化构建脚本（推荐）

最简单的构建和安装方式：

```bash
cd rtabmap/python_binding
chmod +x build.sh
./build.sh
```

脚本将：
- 检查所有依赖项
- 安装 Python 要求
- 构建绑定
- 安装包
- 运行测试以验证安装

### 方法 2：使用 setup.py 手动构建

```bash
cd rtabmap/python_binding

# 安装 Python 依赖项
pip install -r requirements.txt

# 构建并安装
pip install .

# 或者用于开发（可编辑安装）
pip install -e .
```

### 方法 3：CMake 构建（高级）

```bash
cd rtabmap/python_binding

# 创建构建目录
mkdir build && cd build

# 配置
cmake .. -DCMAKE_BUILD_TYPE=Release

# 构建
make -j$(nproc)

# 编译的模块将在 build 目录中
```

## 验证

测试您的安装：

```python
import rtabmap_python as rtab

# 测试基本功能
slam = rtab.Rtabmap()
print(f"RTAB-Map 版本: {slam.getVersion()}")

# 测试数据结构
transform = rtab.Transform(1, 2, 3, 0, 0, 0)
camera = rtab.CameraModel(525, 525, 320, 240)
params = rtab.ParametersMap()

print("✅ RTAB-Map Python 绑定工作正常！")
```

或者运行测试脚本：

```bash
cd examples/
python3 basic_slam_example.py
```

## 故障排除

### 常见问题

#### 1. ImportError: No module named 'rtabmap_python'

**原因**：模块未正确构建或安装。

**解决方案**：
```bash
# 检查 RTAB-Map C++ 库是否已安装
pkg-config --exists rtabmap && echo "找到" || echo "未找到"

# 重新安装绑定
pip uninstall rtabmap-python
pip install . --force-reinstall

# 检查 Python 路径
python3 -c "import sys; print('\n'.join(sys.path))"
```

#### 2. fatal error: rtabmap/core/Rtabmap.h: No such file

**原因**：未找到 RTAB-Map 开发头文件。

**解决方案**：
```bash
# 安装开发包
sudo apt install librtabmap-dev

# 或检查头文件是否存在
find /usr -name "Rtabmap.h" 2>/dev/null

# 如果从源码构建，确保您运行了 'sudo make install'
```

#### 3. undefined symbol: _ZN6rtabmap7Rtabmap4initE

**原因**：运行时未找到 RTAB-Map 库或版本不匹配。

**解决方案**：
```bash
# 检查库是否已安装
ldconfig -p | grep rtabmap

# 更新库缓存
sudo ldconfig

# 检查库路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

#### 4. CMake Error: Could not find pybind11

**原因**：pybind11 未安装或 CMake 找不到。

**解决方案**：
```bash
# 安装 pybind11
pip install pybind11

# 或安装系统包
sudo apt install pybind11-dev

# 手动指定 pybind11 路径
cmake .. -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
```

#### 5. 运行时段错误

**原因**：通常是 ABI 不匹配或库链接错误。

**解决方案**：
- 确保 RTAB-Map 和绑定使用相同编译器构建
- 检查 OpenCV 版本兼容性
- 使用相同配置重新构建 RTAB-Map 和绑定

#### 6. 性能问题

**解决方案**：
```bash
# 以 Release 模式构建
cmake .. -DCMAKE_BUILD_TYPE=Release

# 使用优化的 OpenCV
pip uninstall opencv-python
pip install opencv-contrib-python

# 减少特征点数量以进行实时操作
params[rtab.Param.kKpMaxFeatures] = "200"
```

### 构建配置选项

#### 自定义 RTAB-Map 安装路径

如果 RTAB-Map 安装在非标准位置：

```bash
# 设置环境变量
export RTABMap_ROOT=/path/to/rtabmap/install
export PKG_CONFIG_PATH=/path/to/rtabmap/install/lib/pkgconfig:$PKG_CONFIG_PATH

# 或在 CMake 中指定
cmake .. -DRTABMap_ROOT=/path/to/rtabmap/install
```

#### 自定义 OpenCV 安装

```bash
# 指定 OpenCV 路径
cmake .. -DOpenCV_DIR=/path/to/opencv/build

# 或使用特定 OpenCV 版本
export OpenCV_DIR=/usr/local/lib/cmake/opencv4
```

#### 调试构建

```bash
# 用于调试绑定
cmake .. -DCMAKE_BUILD_TYPE=Debug -DPYBIND11_DETAILED_ERROR_MESSAGES=ON
```

## 平台特定说明

### Ubuntu 20.04/22.04

```bash
# 安装所有依赖项
sudo apt install cmake git pkg-config python3-dev python3-pip
sudo apt install libopencv-dev libeigen3-dev libpcl-dev
sudo apt install libg2o-dev libsuitesparse-dev libceres-dev

# 安装 RTAB-Map
sudo apt install librtabmap-dev
# 或从源码构建以获得最新版本
```

### macOS

```bash
# 使用 Homebrew 安装依赖项
brew install cmake pkg-config opencv eigen pcl

# 从源码构建 RTAB-Map（包管理器版本可能过时）
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
sudo make install
```

### Windows（高级）

1. 安装带有 C++ 支持的 Visual Studio 2019/2022
2. 安装 CMake 和 Git
3. 使用 vcpkg 安装依赖项：
   ```cmd
   vcpkg install opencv eigen3 pcl
   ```
4. 使用 vcpkg 工具链构建 RTAB-Map
5. 使用相同工具链构建 Python 绑定

## 开发设置

用于开发绑定：

```bash
# 克隆仓库
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/python_binding

# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装开发依赖项
pip install -e ".[dev]"

# 运行测试
python3 -m pytest tests/

# 格式化代码
black src/
flake8 src/
```

## 获取帮助

如果您遇到问题：

1. **检查日志**：使用详细输出构建：
   ```bash
   pip install . -v
   ```

2. **验证 RTAB-Map 安装**：
   ```bash
   pkg-config --cflags --libs rtabmap
   ```

3. **检查系统库**：
   ```bash
   ldd /path/to/rtabmap_python.so
   ```

4. **创建问题**：如果问题持续存在，请在 [RTAB-Map GitHub 仓库](https://github.com/introlab/rtabmap/issues) 创建问题，包含：
   - 您的操作系统和版本
   - RTAB-Map 版本
   - Python 版本
   - 完整错误消息
   - 构建日志

## 下一步

成功安装后：

1. **运行示例**：
   ```bash
   cd examples/
   python3 basic_slam_example.py
   python3 camera_integration_example.py
   ```

2. **阅读文档**：查看 `README.md` 获取 API 参考

3. **校准您的相机**：对于实际应用，正确的相机校准至关重要

4. **调整参数**：根据您的特定用例调整 RTAB-Map 参数
