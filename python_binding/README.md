# RTAB-Map Python Bindings

**Real Python bindings for the official RTAB-Map C++ library using pybind11**

This package provides true Python bindings for RTAB-Map (Real-Time Appearance-Based Mapping), a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. Unlike mock implementations, these bindings directly interface with the actual RTAB-Map C++ library.

## Features

### Core SLAM Functionality
- **Full RGB-D SLAM**: Complete SLAM processing with loop closure detection
- **Real-time Performance**: Direct C++ integration for maximum performance
- **Graph Optimization**: Access to optimized pose graphs and constraints
- **Memory Management**: Intelligent memory management for real-time operation
- **Loop Closure Detection**: Robust appearance-based loop closure detection
- **Comprehensive Statistics**: Detailed performance and mapping statistics

### Sensor Support
- **RGB-D Cameras**: Support for depth cameras (RealSense, Kinect, etc.)
- **Stereo Cameras**: Stereo vision with depth computation
- **Camera Calibration**: Full camera model support with distortion correction
- **IMU Integration**: Inertial measurement unit data fusion
- **GPS Integration**: GPS data for global localization

### Data Structures
- **SensorData**: Unified container for all sensor inputs
- **Transform**: 3D transformations with full matrix operations
- **CameraModel**: Camera intrinsics and calibration parameters
- **Parameters**: Comprehensive parameter management system
- **Statistics**: Performance monitoring and analysis

## Installation

### Prerequisites

1. **RTAB-Map C++ Library**: You must have RTAB-Map installed first
   ```bash
   # On Ubuntu/Debian
   sudo apt install ros-*-rtabmap ros-*-rtabmap-ros
   
   # Or build from source
   git clone https://github.com/introlab/rtabmap.git
   cd rtabmap/build
   cmake ..
   make -j4
   sudo make install
   ```

2. **System Dependencies**:
   ```bash
   # Ubuntu/Debian
   sudo apt install python3-dev python3-pip cmake pkg-config
   sudo apt install libopencv-dev libeigen3-dev
   
   # macOS
   brew install python cmake pkg-config opencv eigen
   ```

3. **Python Dependencies**:
   ```bash
   pip install numpy>=1.19.0 opencv-python>=4.5.0 pybind11>=2.6.0
   ```

### Build and Install

1. **From Source** (recommended):
   ```bash
   cd rtabmap/python_binding
   pip install -r requirements.txt
   pip install .
   ```

2. **Development Installation**:
   ```bash
   pip install -e .  # Editable install
   pip install -e ".[dev]"  # With development tools
   ```

3. **Using CMake** (advanced):
   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

## Quick Start

### Basic RGB-D SLAM

```python
import rtabmap_python as rtab
import numpy as np
import cv2

# Initialize RTAB-Map
slam = rtab.Rtabmap()

# Configure parameters
params = rtab.ParametersMap()
params[rtab.Param.kRGBDEnabled] = "true"
params[rtab.Param.kRtabmapTimeThr] = "700"
params[rtab.Param.kRtabmapLoopThr] = "0.11"

# Initialize with database
slam.init(params, "my_map.db")

# Create camera model
camera_model = rtab.CameraModel(
    fx=525.0, fy=525.0,
    cx=320.0, cy=240.0
)

# Process RGB-D data
rgb_image = cv2.imread('rgb.jpg')
depth_image = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)

sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)
odometry_pose = rtab.Transform(0, 0, 0, 0, 0, 0)  # x,y,z,roll,pitch,yaw

# Process frame
success = slam.process(sensor_data, odometry_pose)

# Get results
stats = slam.getStatistics()
poses = slam.getOptimizedPoses()
loop_closure_id = slam.getLoopClosureId()

print(f"Processing time: {stats.getProcessTime():.2f}ms")
print(f"Loop closure detected: {loop_closure_id}")
print(f"Number of poses: {len(poses)}")

# Close and save
slam.close(database_saved=True)
```

### Using NumPy Arrays

```python
import rtabmap_python as rtab
import numpy as np

# Initialize SLAM
slam = rtab.Rtabmap()
params = rtab.ParametersMap()
params[rtab.Param.kRGBDEnabled] = "true"
slam.init(params)

# Camera model
camera_model = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)

# RGB-D data as numpy arrays
rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)

# Transform
pose = rtab.Transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.1)

# Process using convenience method
success = slam.processRGBD(rgb, depth, camera_model, pose)

# Get statistics
stats = slam.getStatistics()
print(f"Features extracted: {stats.getFeaturesExtracted()}")
print(f"Working memory size: {stats.getWorkingMemorySize()}")
```

### Parameter Management

```python
import rtabmap_python as rtab

# Get all default parameters
default_params = rtab.Parameters.getDefaultParameters()
print(f"Total parameters: {len(default_params)}")

# Create parameter map
params = rtab.ParametersMap()

# Set RGB-D SLAM parameters
params[rtab.Param.kRGBDEnabled] = "true"
params[rtab.Param.kRGBDLinearUpdate] = "0.1"
params[rtab.Param.kRGBDAngularUpdate] = "0.1"

# Set feature detection parameters
params[rtab.Param.kKpMaxFeatures] = "400"
params[rtab.Param.kKpDetectorStrategy] = "6"  # GFTT/BRIEF

# Set loop closure parameters
params[rtab.Param.kRtabmapLoopThr] = "0.11"
params[rtab.Param.kRtabmapMaxRetrieved] = "2"

# Set memory management
params[rtab.Param.kMemRehearsalSimilarity] = "0.6"

# Validate parameters
validated_params = rtab.Parameters.parse(params)
print("Parameters validated successfully!")
```

## API Reference

### Core Classes

#### Rtabmap
Main SLAM processing class.

```python
slam = rtab.Rtabmap()
slam.init(parameters, database_path)
success = slam.process(sensor_data, odometry_pose)
stats = slam.getStatistics()
poses = slam.getOptimizedPoses()
slam.close()
```

#### SensorData
Container for sensor inputs.

```python
# RGB-D constructor
sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)

# From numpy arrays
sensor_data = rtab.SensorData.create(rgb_array, depth_array, camera_model)

# Access data
rgb = sensor_data.imageRaw()
depth = sensor_data.depthRaw()
```

#### Transform
3D transformation matrices.

```python
# From position and Euler angles
transform = rtab.Transform(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)

# From 4x4 matrix
transform = rtab.Transform.fromEigen4d(matrix)

# Operations
inverse = transform.inverse()
combined = transform1 * transform2
distance = transform1.getDistance(transform2)
```

#### CameraModel
Camera calibration parameters.

```python
# Basic constructor
camera = rtab.CameraModel(fx=525, fy=525, cx=320, cy=240)

# From calibration matrices
camera = rtab.CameraModel.fromEigen(K_matrix, D_vector, image_size)

# Projection operations
points_2d = camera.project(points_3d)
points_3d = camera.reproject(points_2d, depths)
```

### Configuration

#### Key Parameters

```python
# Core RTAB-Map parameters
rtab.Param.kRtabmapTimeThr          # Time threshold (ms)
rtab.Param.kRtabmapLoopThr          # Loop closure threshold
rtab.Param.kRtabmapMaxRetrieved     # Max retrieved nodes

# RGB-D SLAM parameters
rtab.Param.kRGBDEnabled             # Enable RGB-D mode
rtab.Param.kRGBDLinearUpdate        # Linear update threshold (m)
rtab.Param.kRGBDAngularUpdate       # Angular update threshold (rad)

# Feature detection parameters
rtab.Param.kKpMaxFeatures           # Maximum features
rtab.Param.kKpDetectorStrategy      # Feature detector type

# Memory management parameters
rtab.Param.kMemRehearsalSimilarity  # Rehearsal similarity threshold
```

## Examples

The `examples/` directory contains comprehensive examples:

- **`basic_slam_example.py`**: Complete RGB-D SLAM workflow
- **`camera_integration_example.py`**: Real camera integration
- **`parameter_tuning_example.py`**: Parameter optimization
- **`localization_example.py`**: Localization mode usage

## Performance Tips

1. **Parameter Tuning**: Adjust parameters based on your environment
   - Increase `TimeThr` for real-time constraints
   - Decrease `MaxFeatures` for faster processing
   - Tune `LoopThr` for detection sensitivity

2. **Memory Management**: Monitor memory usage
   - Use `getMemoryUsed()` to track memory
   - Adjust `STMSize` based on available RAM
   - Enable `IncrementalMemory` for mapping

3. **Camera Calibration**: Proper calibration is crucial
   - Use high-quality calibration data
   - Include distortion parameters
   - Verify rectification if needed

## Troubleshooting

### Common Issues

**Import Error**: `ImportError: No module named 'rtabmap_python'`
```bash
# Make sure RTAB-Map C++ library is installed
pkg-config --exists rtabmap && echo "RTAB-Map found" || echo "RTAB-Map not found"

# Reinstall bindings
pip install --force-reinstall rtabmap-python
```

**Build Error**: `fatal error: rtabmap/core/Rtabmap.h: No such file`
```bash
# Install RTAB-Map development headers
sudo apt install librtabmap-dev  # Ubuntu/Debian
# Or build RTAB-Map from source
```

**Runtime Error**: `Segmentation fault`
- Ensure RTAB-Map library version matches bindings
- Check camera model validity
- Verify sensor data integrity

**Performance Issues**:
- Reduce `MaxFeatures` parameter
- Increase `TimeThr` for real-time operation
- Monitor memory usage with `getMemoryUsed()`

## Differences from Mock Implementation

This is a **real implementation** that:

✅ **Direct C++ Integration**: Interfaces with actual RTAB-Map C++ library  
✅ **Full SLAM Functionality**: Complete RGB-D SLAM with all features  
✅ **Real Performance**: Production-ready performance characteristics  
✅ **Comprehensive API**: Access to all RTAB-Map functionality  
✅ **True Loop Closure**: Actual appearance-based loop closure detection  
✅ **Graph Optimization**: Real pose graph optimization  

Unlike mock implementations that simulate SLAM behavior.

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

This project is licensed under the same license as RTAB-Map (BSD License).

## Citation

If you use these bindings in research, please cite the original RTAB-Map paper:

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

## Contact

- **Issues**: [GitHub Issues](https://github.com/introlab/rtabmap/issues)
- **Documentation**: [RTAB-Map Wiki](http://introlab.github.io/rtabmap)
- **Original Project**: [RTAB-Map](https://github.com/introlab/rtabmap)
