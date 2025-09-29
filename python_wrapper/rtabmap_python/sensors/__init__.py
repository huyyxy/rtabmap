"""
Sensor and camera interfaces for RTAB-Map.
"""

from .camera_rgb import CameraRGB
from .camera_rgbd import CameraRGBD
from .camera_stereo import CameraStereo

__all__ = [
    'CameraRGB',
    'CameraRGBD',
    'CameraStereo'
]
