"""
Python wrapper for RTAB-Map CameraModel class.

This module provides camera calibration parameters and utilities
for RGB-D and stereo camera systems.
"""

import numpy as np
import cv2
from typing import Tuple, Optional
import math

from .transform import Transform


class CameraModel:
    """
    Python wrapper for rtabmap::CameraModel class.
    
    Represents camera intrinsic parameters, distortion coefficients,
    and local transformation for a single camera.
    
    Example:
        >>> # Minimal constructor
        >>> camera = CameraModel(fx=525, fy=525, cx=320, cy=240)
        >>> 
        >>> # Full constructor with distortion
        >>> K = np.array([[525, 0, 320], [0, 525, 240], [0, 0, 1]])
        >>> D = np.array([0.1, -0.2, 0.001, 0.002, 0.1])
        >>> camera = CameraModel("camera", (640, 480), K, D)
    """
    
    def __init__(self,
                 name: str = "",
                 image_size: Tuple[int, int] = (0, 0),
                 K: Optional[np.ndarray] = None,
                 D: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 P: Optional[np.ndarray] = None,
                 local_transform: Optional[Transform] = None,
                 fx: Optional[float] = None,
                 fy: Optional[float] = None,
                 cx: Optional[float] = None,
                 cy: Optional[float] = None,
                 Tx: float = 0.0):
        """
        Initialize CameraModel.
        
        Args:
            name: Camera name/identifier
            image_size: Image dimensions (width, height)
            K: 3x3 camera intrinsic matrix
            D: Distortion coefficients (1x4, 1x5, or 1x8)
            R: 3x3 rectification matrix (default: identity)
            P: 3x4 projection matrix (default: [K [0 0 0]'])
            local_transform: Transform from base frame to camera frame
            fx, fy: Focal lengths (alternative to K matrix)
            cx, cy: Principal point coordinates (alternative to K matrix)
            Tx: Baseline for stereo (meters, negative for right camera)
        """
        self._name = name
        self._image_size = image_size
        self._Tx = Tx
        
        # Set local transform (default: optical rotation)
        self._local_transform = local_transform if local_transform else Transform.optical_rotation()
        
        # Initialize matrices
        if K is not None:
            self._K = np.array(K, dtype=np.float64)
            if self._K.shape != (3, 3):
                raise ValueError("K matrix must be 3x3")
        elif all(param is not None for param in [fx, fy, cx, cy]):
            self._K = np.array([
                [fx,  0, cx],
                [ 0, fy, cy],
                [ 0,  0,  1]
            ], dtype=np.float64)
        else:
            self._K = np.eye(3, dtype=np.float64)
            
        # Distortion coefficients
        if D is not None:
            self._D = np.array(D, dtype=np.float64).flatten()
            if self._D.size not in [4, 5, 8]:
                raise ValueError("D must have 4, 5, or 8 coefficients")
        else:
            self._D = np.zeros(5, dtype=np.float64)
            
        # Rectification matrix
        if R is not None:
            self._R = np.array(R, dtype=np.float64)
            if self._R.shape != (3, 3):
                raise ValueError("R matrix must be 3x3")
        else:
            self._R = np.eye(3, dtype=np.float64)
            
        # Projection matrix
        if P is not None:
            self._P = np.array(P, dtype=np.float64)
            if self._P.shape != (3, 4):
                raise ValueError("P matrix must be 3x4")
        else:
            # Default: P = [K [0 0 0]']
            self._P = np.zeros((3, 4), dtype=np.float64)
            self._P[:3, :3] = self._K
            self._P[0, 3] = Tx * self._K[0, 0]  # Tx * fx for stereo baseline
            
    # Property getters
    def name(self) -> str:
        """Get camera name."""
        return self._name
        
    def image_size(self) -> Tuple[int, int]:
        """Get image size (width, height)."""
        return self._image_size
        
    def image_width(self) -> int:
        """Get image width."""
        return self._image_size[0]
        
    def image_height(self) -> int:
        """Get image height."""
        return self._image_size[1]
        
    def K(self) -> np.ndarray:
        """Get camera intrinsic matrix (3x3)."""
        return self._K.copy()
        
    def D(self) -> np.ndarray:
        """Get distortion coefficients."""
        return self._D.copy()
        
    def R(self) -> np.ndarray:
        """Get rectification matrix (3x3)."""
        return self._R.copy()
        
    def P(self) -> np.ndarray:
        """Get projection matrix (3x4)."""
        return self._P.copy()
        
    def local_transform(self) -> Transform:
        """Get local transform from base frame to camera frame."""
        return self._local_transform
        
    def fx(self) -> float:
        """Get focal length in x direction."""
        return float(self._K[0, 0])
        
    def fy(self) -> float:
        """Get focal length in y direction."""
        return float(self._K[1, 1])
        
    def cx(self) -> float:
        """Get principal point x coordinate."""
        return float(self._K[0, 2])
        
    def cy(self) -> float:
        """Get principal point y coordinate."""
        return float(self._K[1, 2])
        
    def Tx(self) -> float:
        """Get baseline (for stereo cameras)."""
        return self._Tx
        
    # Validation
    def is_valid(self) -> bool:
        """Check if camera model is valid."""
        return (self.fx() > 0 and self.fy() > 0 and 
                self.cx() >= 0 and self.cy() >= 0 and
                self._image_size[0] > 0 and self._image_size[1] > 0)
                
    def is_rectified(self) -> bool:
        """Check if camera is rectified (R != Identity)."""
        return not np.allclose(self._R, np.eye(3), atol=1e-6)
        
    def has_distortion(self) -> bool:
        """Check if camera has distortion coefficients."""
        return not np.allclose(self._D, 0, atol=1e-6)
        
    # Coordinate transformations
    def project_3d_to_2d(self, points_3d: np.ndarray) -> np.ndarray:
        """
        Project 3D points to 2D image coordinates.
        
        Args:
            points_3d: Nx3 array of 3D points
            
        Returns:
            Nx2 array of 2D image coordinates
        """
        points_3d = np.array(points_3d)
        if points_3d.ndim == 1:
            points_3d = points_3d.reshape(1, -1)
            
        if points_3d.shape[1] != 3:
            raise ValueError("Points must be Nx3")
            
        # Filter out points behind camera (z <= 0)
        valid_mask = points_3d[:, 2] > 1e-6
        points_2d = np.full((points_3d.shape[0], 2), np.nan)
        
        if np.any(valid_mask):
            valid_points = points_3d[valid_mask]
            
            # Project using pinhole model: [u, v, 1]' = K * [X/Z, Y/Z, 1]'
            projected = valid_points / valid_points[:, 2:3]  # Normalize by Z
            projected_2d = (self._K @ projected.T).T
            
            points_2d[valid_mask] = projected_2d[:, :2]
            
        return points_2d
        
    def reproject_to_3d(self, 
                       points_2d: np.ndarray,
                       depths: np.ndarray) -> np.ndarray:
        """
        Reproject 2D points to 3D using depth values.
        
        Args:
            points_2d: Nx2 array of 2D image coordinates
            depths: N array of depth values
            
        Returns:
            Nx3 array of 3D points
        """
        points_2d = np.array(points_2d)
        depths = np.array(depths)
        
        if points_2d.ndim == 1:
            points_2d = points_2d.reshape(1, -1)
        if depths.ndim == 0:
            depths = depths.reshape(1)
            
        if points_2d.shape[1] != 2:
            raise ValueError("Points must be Nx2")
        if points_2d.shape[0] != depths.shape[0]:
            raise ValueError("Number of points and depths must match")
            
        # Filter out invalid depths
        valid_mask = depths > 1e-6
        points_3d = np.full((points_2d.shape[0], 3), np.nan)
        
        if np.any(valid_mask):
            valid_2d = points_2d[valid_mask]
            valid_depths = depths[valid_mask]
            
            # Inverse projection: [X, Y, Z]' = Z * K^-1 * [u, v, 1]'
            K_inv = np.linalg.inv(self._K)
            homogeneous = np.column_stack([valid_2d, np.ones(valid_2d.shape[0])])
            normalized = (K_inv @ homogeneous.T).T
            points_3d_valid = normalized * valid_depths.reshape(-1, 1)
            
            points_3d[valid_mask] = points_3d_valid
            
        return points_3d
        
    def is_in_image(self, 
                   points_2d: np.ndarray,
                   border: int = 0) -> np.ndarray:
        """
        Check if 2D points are within image bounds.
        
        Args:
            points_2d: Nx2 array of 2D points
            border: Border margin in pixels
            
        Returns:
            N boolean array indicating which points are valid
        """
        points_2d = np.array(points_2d)
        if points_2d.ndim == 1:
            points_2d = points_2d.reshape(1, -1)
            
        width, height = self._image_size
        
        valid = ((points_2d[:, 0] >= border) & 
                (points_2d[:, 0] < width - border) &
                (points_2d[:, 1] >= border) & 
                (points_2d[:, 1] < height - border))
                
        return valid
        
    # Calibration utilities
    def undistort_points(self, points_2d: np.ndarray) -> np.ndarray:
        """
        Undistort 2D points using camera distortion model.
        
        Args:
            points_2d: Nx2 array of distorted 2D points
            
        Returns:
            Nx2 array of undistorted 2D points
        """
        if not self.has_distortion():
            return np.array(points_2d)
            
        points_2d = np.array(points_2d, dtype=np.float32)
        if points_2d.ndim == 1:
            points_2d = points_2d.reshape(1, -1)
            
        # Use OpenCV undistortPoints
        undistorted = cv2.undistortPoints(
            points_2d.reshape(-1, 1, 2),
            self._K.astype(np.float32),
            self._D.astype(np.float32),
            R=self._R.astype(np.float32),
            P=self._P.astype(np.float32)
        )
        
        return undistorted.reshape(-1, 2)
        
    def rectify_image(self, image: np.ndarray) -> np.ndarray:
        """
        Rectify image using calibration parameters.
        
        Args:
            image: Input image
            
        Returns:
            Rectified image
        """
        if not self.is_rectified() and not self.has_distortion():
            return image
            
        # Create rectification maps
        map1, map2 = cv2.initUndistortRectifyMap(
            self._K.astype(np.float32),
            self._D.astype(np.float32),
            self._R.astype(np.float32),
            self._P.astype(np.float32),
            self._image_size,
            cv2.CV_32FC1
        )
        
        # Apply rectification
        rectified = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        return rectified
        
    # Utility methods
    def horizontal_fov(self) -> float:
        """Calculate horizontal field of view in radians."""
        return 2.0 * math.atan(self._image_size[0] / (2.0 * self.fx()))
        
    def vertical_fov(self) -> float:
        """Calculate vertical field of view in radians."""
        return 2.0 * math.atan(self._image_size[1] / (2.0 * self.fy()))
        
    def diagonal_fov(self) -> float:
        """Calculate diagonal field of view in radians."""
        diagonal = math.sqrt(self._image_size[0]**2 + self._image_size[1]**2)
        f_avg = (self.fx() + self.fy()) / 2.0
        return 2.0 * math.atan(diagonal / (2.0 * f_avg))
        
    def clone(self) -> 'CameraModel':
        """Create a copy of this camera model."""
        return CameraModel(
            name=self._name,
            image_size=self._image_size,
            K=self._K,
            D=self._D,
            R=self._R,
            P=self._P,
            local_transform=self._local_transform,
            Tx=self._Tx
        )
        
    # Static methods
    @staticmethod
    def optical_rotation() -> Transform:
        """Get optical rotation transform."""
        return Transform.optical_rotation()
        
    def __repr__(self) -> str:
        """String representation."""
        return (f"CameraModel(name='{self._name}', size={self._image_size}, "
                f"fx={self.fx():.1f}, fy={self.fy():.1f}, "
                f"cx={self.cx():.1f}, cy={self.cy():.1f}, Tx={self._Tx:.3f})")
                
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = [
            f"CameraModel: {self._name}",
            f"  Image size: {self._image_size}",
            f"  Intrinsics (fx, fy, cx, cy): {self.fx():.2f}, {self.fy():.2f}, {self.cx():.2f}, {self.cy():.2f}",
            f"  Baseline Tx: {self._Tx:.6f}",
            f"  Has distortion: {self.has_distortion()}",
            f"  Is rectified: {self.is_rectified()}",
            f"  Horizontal FOV: {math.degrees(self.horizontal_fov()):.1f}°",
            f"  Vertical FOV: {math.degrees(self.vertical_fov()):.1f}°"
        ]
        return "\n".join(lines)
