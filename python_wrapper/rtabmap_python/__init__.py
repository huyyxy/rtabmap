"""
RTAB-Map Python Wrapper

This package provides Python bindings for RTAB-Map (Real-Time Appearance-Based Mapping),
a RGB-D SLAM approach with real-time constraints.

RTAB-Map is a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector.
The loop closure detector uses a bag-of-words approach to determinate how likely a new image comes from a previous location or a new location.
When a loop closure hypothesis is accepted, a new constraint is added to the map's graph, then a graph optimizer minimizes the errors in the map.
A memory management approach is used to limit the number of locations used for loop closure detection and graph optimization,
so that real-time constraints on large-scale environments are always respected.

Main Classes:
- RTABMap: Main SLAM class for processing sensor data
- SensorData: Container for sensor input (RGB-D images, laser scans, IMU, etc.)
- Transform: 3D transformation matrix utilities
- CameraModel: Camera intrinsic parameters and calibration
- Statistics: Performance and mapping statistics
- Parameters: Configuration parameters for RTAB-Map

Author: Based on RTAB-Map library by Mathieu Labbe
License: BSD
"""

__version__ = "0.1.0"
__author__ = "RTAB-Map Python Wrapper"

# Core classes
from .core import (
    RTABMap,
    SensorData,
    Transform,
    CameraModel,
    Statistics,
    Parameters
)

# Camera and sensor classes
from .sensors import (
    CameraRGB,
    CameraRGBD,
    CameraStereo
)

# Utility classes
from .utils import (
    Odometry,
    Graph,
    Memory
)

__all__ = [
    # Core
    'RTABMap',
    'SensorData', 
    'Transform',
    'CameraModel',
    'Statistics',
    'Parameters',
    
    # Sensors
    'CameraRGB',
    'CameraRGBD', 
    'CameraStereo',
    
    # Utils
    'Odometry',
    'Graph',
    'Memory'
]
