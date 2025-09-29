"""
Python wrapper for RTAB-Map SensorData class.

This module provides the SensorData class that wraps sensor input data
including RGB images, depth images, laser scans, IMU data, GPS, etc.
"""

import numpy as np
import cv2
from typing import Optional, List, Dict, Any
import time

from .camera_model import CameraModel
from .transform import Transform


class SensorData:
    """
    Python wrapper for rtabmap::SensorData class.
    
    Container for sensor input data including RGB-D images, laser scans,
    IMU data, GPS coordinates, and other sensor information.
    
    Example:
        >>> # RGB-D constructor
        >>> rgb = cv2.imread('rgb.jpg')
        >>> depth = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)
        >>> camera_model = CameraModel(fx=525, fy=525, cx=320, cy=240)
        >>> sensor_data = SensorData(rgb, depth, camera_model)
        >>> 
        >>> # Appearance-only constructor
        >>> image = cv2.imread('image.jpg')
        >>> sensor_data = SensorData(image=image)
    """
    
    def __init__(self,
                 image: Optional[np.ndarray] = None,
                 depth: Optional[np.ndarray] = None,
                 camera_model: Optional[CameraModel] = None,
                 image_id: int = 0,
                 timestamp: float = 0.0,
                 laser_scan: Optional[np.ndarray] = None,
                 imu_data: Optional[Dict[str, Any]] = None,
                 gps_data: Optional[Dict[str, float]] = None,
                 user_data: Optional[np.ndarray] = None):
        """
        Initialize SensorData.
        
        Args:
            image: RGB image (H x W x 3, uint8)
            depth: Depth image (H x W, uint16 or float32)
            camera_model: Camera calibration parameters
            image_id: Unique identifier (auto-generated if 0)
            timestamp: Timestamp in seconds (current time if 0.0)
            laser_scan: Laser scan data (N x 3, float32)
            imu_data: IMU measurements dictionary
            gps_data: GPS coordinates dictionary
            user_data: Additional user data
        """
        # Generate ID and timestamp if not provided
        self._id = image_id if image_id != 0 else int(time.time() * 1000000) % 1000000
        self._timestamp = timestamp if timestamp != 0.0 else time.time()
        
        # Image data
        self._image_raw = image.copy() if image is not None else np.array([])
        self._depth_raw = depth.copy() if depth is not None else np.array([])
        self._image_compressed = np.array([])  # Placeholder for compressed data
        self._depth_compressed = np.array([])  # Placeholder for compressed data
        
        # Camera model
        self._camera_model = camera_model if camera_model else CameraModel()
        
        # Laser scan data
        self._laser_scan_raw = laser_scan.copy() if laser_scan is not None else np.array([])
        self._laser_scan_compressed = np.array([])
        
        # IMU data
        self._imu_data = imu_data.copy() if imu_data else {}
        
        # GPS data  
        self._gps_data = gps_data.copy() if gps_data else {}
        
        # User data
        self._user_data_raw = user_data.copy() if user_data is not None else np.array([])
        self._user_data_compressed = np.array([])
        
        # Features (keypoints and descriptors)
        self._keypoints = []  # List of cv2.KeyPoint
        self._keypoints_3d = np.array([])  # N x 3 array
        self._descriptors = np.array([])  # N x descriptor_size array
        
        # Ground truth and global pose
        self._ground_truth = Transform()
        self._global_pose = Transform() 
        self._global_pose_covariance = np.eye(6, dtype=np.float64)
        
        # Validation
        self._validate_data()
        
    def _validate_data(self) -> None:
        """Validate input data consistency."""
        # Check image and depth dimensions match
        if (self._image_raw.size > 0 and self._depth_raw.size > 0):
            if self._image_raw.shape[:2] != self._depth_raw.shape[:2]:
                raise ValueError(
                    f"Image and depth dimensions don't match: "
                    f"{self._image_raw.shape[:2]} vs {self._depth_raw.shape[:2]}"
                )
                
        # Check camera model is valid for RGB-D data
        if (self._image_raw.size > 0 and self._depth_raw.size > 0 and 
            not self._camera_model.is_valid()):
            raise ValueError("Valid camera model required for RGB-D data")
            
    # Property getters
    def id(self) -> int:
        """Get sensor data ID."""
        return self._id
        
    def timestamp(self) -> float:
        """Get timestamp in seconds."""
        return self._timestamp
        
    def image_raw(self) -> np.ndarray:
        """Get raw RGB image."""
        return self._image_raw
        
    def depth_raw(self) -> np.ndarray:
        """Get raw depth image.""" 
        return self._depth_raw
        
    def image_compressed(self) -> np.ndarray:
        """Get compressed RGB image data."""
        return self._image_compressed
        
    def depth_compressed(self) -> np.ndarray:
        """Get compressed depth image data."""
        return self._depth_compressed
        
    def laser_scan_raw(self) -> np.ndarray:
        """Get raw laser scan data."""
        return self._laser_scan_raw
        
    def laser_scan_compressed(self) -> np.ndarray:
        """Get compressed laser scan data."""
        return self._laser_scan_compressed
        
    def camera_model(self) -> CameraModel:
        """Get camera model."""
        return self._camera_model
        
    def imu_data(self) -> Dict[str, Any]:
        """Get IMU data."""
        return self._imu_data.copy()
        
    def gps_data(self) -> Dict[str, float]:
        """Get GPS data."""
        return self._gps_data.copy()
        
    def user_data_raw(self) -> np.ndarray:
        """Get raw user data."""
        return self._user_data_raw
        
    def keypoints(self) -> List[cv2.KeyPoint]:
        """Get detected keypoints."""
        return self._keypoints.copy()
        
    def keypoints_3d(self) -> np.ndarray:
        """Get 3D keypoints."""
        return self._keypoints_3d.copy()
        
    def descriptors(self) -> np.ndarray:
        """Get feature descriptors."""
        return self._descriptors.copy()
        
    def ground_truth(self) -> Transform:
        """Get ground truth pose."""
        return self._ground_truth
        
    def global_pose(self) -> Transform:
        """Get global pose."""
        return self._global_pose
        
    def global_pose_covariance(self) -> np.ndarray:
        """Get global pose covariance matrix."""
        return self._global_pose_covariance.copy()
        
    # Property setters
    def set_id(self, sensor_id: int) -> None:
        """Set sensor data ID."""
        self._id = sensor_id
        
    def set_timestamp(self, timestamp: float) -> None:
        """Set timestamp."""
        self._timestamp = timestamp
        
    def set_rgb_image(self, image: np.ndarray, clear_previous: bool = True) -> None:
        """
        Set RGB image data.
        
        Args:
            image: RGB image array
            clear_previous: Clear previous compressed data
        """
        self._image_raw = image.copy()
        if clear_previous:
            self._image_compressed = np.array([])
            
    def set_depth_image(self, depth: np.ndarray, clear_previous: bool = True) -> None:
        """
        Set depth image data.
        
        Args:
            depth: Depth image array
            clear_previous: Clear previous compressed data
        """
        self._depth_raw = depth.copy()
        if clear_previous:
            self._depth_compressed = np.array([])
            
    def set_rgbd_image(self,
                       rgb: np.ndarray,
                       depth: np.ndarray, 
                       camera_model: CameraModel,
                       clear_previous: bool = True) -> None:
        """
        Set RGB-D image data together.
        
        Args:
            rgb: RGB image
            depth: Depth image
            camera_model: Camera calibration
            clear_previous: Clear previous data
        """
        self._image_raw = rgb.copy()
        self._depth_raw = depth.copy()
        self._camera_model = camera_model
        
        if clear_previous:
            self._image_compressed = np.array([])
            self._depth_compressed = np.array([])
            
        self._validate_data()
        
    def set_laser_scan(self, laser_scan: np.ndarray) -> None:
        """Set laser scan data."""
        self._laser_scan_raw = laser_scan.copy()
        
    def set_imu_data(self, imu_data: Dict[str, Any]) -> None:
        """Set IMU data."""
        self._imu_data = imu_data.copy()
        
    def set_gps_data(self, gps_data: Dict[str, float]) -> None:
        """Set GPS data."""
        self._gps_data = gps_data.copy()
        
    def set_user_data(self, user_data: np.ndarray) -> None:
        """Set user data."""
        self._user_data_raw = user_data.copy()
        
    def set_features(self,
                     keypoints: List[cv2.KeyPoint],
                     descriptors: np.ndarray,
                     keypoints_3d: Optional[np.ndarray] = None) -> None:
        """
        Set detected features.
        
        Args:
            keypoints: List of 2D keypoints
            descriptors: Feature descriptors
            keypoints_3d: Corresponding 3D points (optional)
        """
        self._keypoints = keypoints.copy()
        self._descriptors = descriptors.copy()
        if keypoints_3d is not None:
            self._keypoints_3d = keypoints_3d.copy()
            
    def set_ground_truth(self, pose: Transform) -> None:
        """Set ground truth pose."""
        self._ground_truth = pose
        
    def set_global_pose(self, pose: Transform, covariance: Optional[np.ndarray] = None) -> None:
        """Set global pose."""
        self._global_pose = pose
        if covariance is not None:
            self._global_pose_covariance = covariance.copy()
            
    # Utility methods
    def is_valid(self) -> bool:
        """
        Check if sensor data is valid.
        
        Returns:
            True if data contains meaningful sensor information
        """
        return (self._id != 0 or
                self._timestamp != 0.0 or
                self._image_raw.size > 0 or
                self._image_compressed.size > 0 or
                self._depth_raw.size > 0 or
                self._depth_compressed.size > 0 or
                self._laser_scan_raw.size > 0 or
                self._laser_scan_compressed.size > 0 or
                len(self._keypoints) > 0 or
                self._descriptors.size > 0 or
                len(self._imu_data) > 0)
                
    def has_image(self) -> bool:
        """Check if has RGB image data."""
        return self._image_raw.size > 0 or self._image_compressed.size > 0
        
    def has_depth(self) -> bool:
        """Check if has depth data."""
        return self._depth_raw.size > 0 or self._depth_compressed.size > 0
        
    def has_laser_scan(self) -> bool:
        """Check if has laser scan data."""
        return self._laser_scan_raw.size > 0 or self._laser_scan_compressed.size > 0
        
    def has_imu(self) -> bool:
        """Check if has IMU data."""
        return len(self._imu_data) > 0
        
    def has_gps(self) -> bool:
        """Check if has GPS data."""
        return len(self._gps_data) > 0
        
    def has_features(self) -> bool:
        """Check if has extracted features."""
        return len(self._keypoints) > 0 and self._descriptors.size > 0
        
    def is_rgb_d(self) -> bool:
        """Check if this is RGB-D data."""
        return self.has_image() and self.has_depth()
        
    def uncompress_data(self) -> None:
        """
        Uncompress compressed data.
        
        Note: In this simulation, we don't actually compress/uncompress data.
        This method is provided for API compatibility.
        """
        # In real implementation, would decompress JPEG/PNG/etc data
        pass
        
    def compress_data(self) -> None:
        """
        Compress raw data for storage.
        
        Note: In this simulation, we don't actually compress data.
        This method is provided for API compatibility.
        """
        # In real implementation, would compress to JPEG/PNG/etc
        pass
        
    def get_image_size(self) -> tuple:
        """Get image dimensions (height, width)."""
        if self._image_raw.size > 0:
            return self._image_raw.shape[:2]
        elif self._depth_raw.size > 0:
            return self._depth_raw.shape[:2]
        else:
            return (0, 0)
            
    def clone(self) -> 'SensorData':
        """Create a deep copy of this SensorData."""
        cloned = SensorData()
        cloned._id = self._id
        cloned._timestamp = self._timestamp
        cloned._image_raw = self._image_raw.copy()
        cloned._depth_raw = self._depth_raw.copy()
        cloned._image_compressed = self._image_compressed.copy()
        cloned._depth_compressed = self._depth_compressed.copy()
        cloned._camera_model = self._camera_model
        cloned._laser_scan_raw = self._laser_scan_raw.copy()
        cloned._laser_scan_compressed = self._laser_scan_compressed.copy()
        cloned._imu_data = self._imu_data.copy()
        cloned._gps_data = self._gps_data.copy()
        cloned._user_data_raw = self._user_data_raw.copy()
        cloned._user_data_compressed = self._user_data_compressed.copy()
        cloned._keypoints = self._keypoints.copy()
        cloned._keypoints_3d = self._keypoints_3d.copy()
        cloned._descriptors = self._descriptors.copy()
        cloned._ground_truth = self._ground_truth
        cloned._global_pose = self._global_pose
        cloned._global_pose_covariance = self._global_pose_covariance.copy()
        return cloned
        
    def __repr__(self) -> str:
        """String representation."""
        return (f"SensorData(id={self._id}, timestamp={self._timestamp:.3f}, "
                f"image={self._image_raw.shape if self._image_raw.size > 0 else 'None'}, "
                f"depth={self._depth_raw.shape if self._depth_raw.size > 0 else 'None'}, "
                f"laser_scan={self._laser_scan_raw.shape if self._laser_scan_raw.size > 0 else 'None'})")
