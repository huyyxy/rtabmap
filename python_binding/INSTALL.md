# Installation Guide for RTAB-Map Python Bindings

This guide provides detailed instructions for building and installing the real RTAB-Map Python bindings.

## Prerequisites

### 1. RTAB-Map C++ Library

**The most important requirement**: You must have the RTAB-Map C++ library installed first.

#### Option A: Install from Package Manager (Ubuntu/Debian)

```bash
# Install RTAB-Map and development headers
sudo apt update
sudo apt install librtabmap-dev

# Or if you're using ROS
sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros  # ROS Noetic
sudo apt install ros-humble-rtabmap ros-humble-rtabmap-ros  # ROS 2 Humble
```

#### Option B: Build from Source (Recommended for latest features)

```bash
# Install dependencies
sudo apt install cmake git pkg-config libopencv-dev libeigen3-dev
sudo apt install libpcl-dev libg2o-dev libsuitesparse-dev

# Clone and build RTAB-Map
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install

# Update library cache
sudo ldconfig
```

### 2. System Dependencies

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
- Install Visual Studio with C++ support
- Install CMake
- Install vcpkg for dependencies
- Build RTAB-Map using vcpkg

### 3. Python Dependencies

```bash
pip install numpy>=1.19.0 opencv-python>=4.5.0 pybind11>=2.6.0
```

## Installation Methods

### Method 1: Automated Build Script (Recommended)

The easiest way to build and install:

```bash
cd rtabmap/python_binding
chmod +x build.sh
./build.sh
```

The script will:
- Check all dependencies
- Install Python requirements
- Build the bindings
- Install the package
- Run tests to verify installation

### Method 2: Manual Build with setup.py

```bash
cd rtabmap/python_binding

# Install Python dependencies
pip install -r requirements.txt

# Build and install
pip install .

# Or for development (editable install)
pip install -e .
```

### Method 3: CMake Build (Advanced)

```bash
cd rtabmap/python_binding

# Create build directory
mkdir build && cd build

# Configure
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

# The compiled module will be in the build directory
```

## Verification

Test your installation:

```python
import rtabmap_python as rtab

# Test basic functionality
slam = rtab.Rtabmap()
print(f"RTAB-Map version: {slam.getVersion()}")

# Test data structures
transform = rtab.Transform(1, 2, 3, 0, 0, 0)
camera = rtab.CameraModel(525, 525, 320, 240)
params = rtab.ParametersMap()

print("✅ RTAB-Map Python bindings working correctly!")
```

Or run the test script:

```bash
cd examples/
python3 basic_slam_example.py
```

## Troubleshooting

### Common Issues

#### 1. ImportError: No module named 'rtabmap_python'

**Cause**: The module wasn't built or installed correctly.

**Solutions**:
```bash
# Check if RTAB-Map C++ library is installed
pkg-config --exists rtabmap && echo "Found" || echo "Not found"

# Reinstall the bindings
pip uninstall rtabmap-python
pip install . --force-reinstall

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"
```

#### 2. fatal error: rtabmap/core/Rtabmap.h: No such file

**Cause**: RTAB-Map development headers not found.

**Solutions**:
```bash
# Install development package
sudo apt install librtabmap-dev

# Or check if headers exist
find /usr -name "Rtabmap.h" 2>/dev/null

# If building from source, make sure you ran 'sudo make install'
```

#### 3. undefined symbol: _ZN6rtabmap7Rtabmap4initE

**Cause**: RTAB-Map library not found at runtime or version mismatch.

**Solutions**:
```bash
# Check if library is installed
ldconfig -p | grep rtabmap

# Update library cache
sudo ldconfig

# Check library path
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

#### 4. CMake Error: Could not find pybind11

**Cause**: pybind11 not installed or not found by CMake.

**Solutions**:
```bash
# Install pybind11
pip install pybind11

# Or install system package
sudo apt install pybind11-dev

# Specify pybind11 path manually
cmake .. -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
```

#### 5. Segmentation fault during runtime

**Cause**: Usually ABI mismatch or incorrect library linking.

**Solutions**:
- Ensure RTAB-Map and bindings are built with same compiler
- Check OpenCV version compatibility
- Rebuild both RTAB-Map and bindings with same configuration

#### 6. Performance Issues

**Solutions**:
```bash
# Build in Release mode
cmake .. -DCMAKE_BUILD_TYPE=Release

# Use optimized OpenCV
pip uninstall opencv-python
pip install opencv-contrib-python

# Reduce feature count for real-time operation
params[rtab.Param.kKpMaxFeatures] = "200"
```

### Build Configuration Options

#### Custom RTAB-Map Installation Path

If RTAB-Map is installed in a non-standard location:

```bash
# Set environment variables
export RTABMap_ROOT=/path/to/rtabmap/install
export PKG_CONFIG_PATH=/path/to/rtabmap/install/lib/pkgconfig:$PKG_CONFIG_PATH

# Or specify in CMake
cmake .. -DRTABMap_ROOT=/path/to/rtabmap/install
```

#### Custom OpenCV Installation

```bash
# Specify OpenCV path
cmake .. -DOpenCV_DIR=/path/to/opencv/build

# Or use specific OpenCV version
export OpenCV_DIR=/usr/local/lib/cmake/opencv4
```

#### Debug Build

```bash
# For debugging the bindings
cmake .. -DCMAKE_BUILD_TYPE=Debug -DPYBIND11_DETAILED_ERROR_MESSAGES=ON
```

## Platform-Specific Notes

### Ubuntu 20.04/22.04

```bash
# Install all dependencies
sudo apt install cmake git pkg-config python3-dev python3-pip
sudo apt install libopencv-dev libeigen3-dev libpcl-dev
sudo apt install libg2o-dev libsuitesparse-dev libceres-dev

# Install RTAB-Map
sudo apt install librtabmap-dev
# OR build from source for latest version
```

### macOS

```bash
# Install dependencies with Homebrew
brew install cmake pkg-config opencv eigen pcl

# Build RTAB-Map from source (package manager version may be outdated)
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
sudo make install
```

### Windows (Advanced)

1. Install Visual Studio 2019/2022 with C++ support
2. Install CMake and Git
3. Use vcpkg to install dependencies:
   ```cmd
   vcpkg install opencv eigen3 pcl
   ```
4. Build RTAB-Map with vcpkg toolchain
5. Build Python bindings with same toolchain

## Development Setup

For developing the bindings:

```bash
# Clone the repository
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/python_binding

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install development dependencies
pip install -e ".[dev]"

# Run tests
python3 -m pytest tests/

# Format code
black src/
flake8 src/
```

## Getting Help

If you encounter issues:

1. **Check the logs**: Build with verbose output:
   ```bash
   pip install . -v
   ```

2. **Verify RTAB-Map installation**:
   ```bash
   pkg-config --cflags --libs rtabmap
   ```

3. **Check system libraries**:
   ```bash
   ldd /path/to/rtabmap_python.so
   ```

4. **Create an issue**: If problems persist, create an issue on the [RTAB-Map GitHub repository](https://github.com/introlab/rtabmap/issues) with:
   - Your operating system and version
   - RTAB-Map version
   - Python version
   - Complete error messages
   - Build logs

## Next Steps

Once installed successfully:

1. **Run examples**:
   ```bash
   cd examples/
   python3 basic_slam_example.py
   python3 camera_integration_example.py
   ```

2. **Read the documentation**: Check `README.md` for API reference

3. **Calibrate your camera**: For real applications, proper camera calibration is crucial

4. **Tune parameters**: Adjust RTAB-Map parameters for your specific use case
