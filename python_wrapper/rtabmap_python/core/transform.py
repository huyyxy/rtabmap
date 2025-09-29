"""
Python wrapper for RTAB-Map Transform class.

This module provides the Transform class for 3D transformations
including translation, rotation, and coordinate frame conversions.
"""

import numpy as np
import math
from typing import List, Tuple, Optional, Union
import cv2


class Transform:
    """
    Python wrapper for rtabmap::Transform class.
    
    Represents a 3D transformation matrix (4x4 homogeneous matrix)
    for position and orientation in 3D space.
    
    The transformation matrix format is:
    [r11 r12 r13 tx ]
    [r21 r22 r23 ty ]
    [r31 r32 r33 tz ]
    [0   0   0   1  ]
    
    Example:
        >>> # Create from position and Euler angles
        >>> t = Transform(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)
        >>> 
        >>> # Create from quaternion
        >>> t = Transform(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        >>> 
        >>> # Create from 4x4 matrix
        >>> matrix = np.eye(4)
        >>> matrix[:3, 3] = [1, 2, 3]
        >>> t = Transform(matrix)
    """
    
    def __init__(self,
                 r11: Optional[float] = None, r12: Optional[float] = None, r13: Optional[float] = None, o14: Optional[float] = None,
                 r21: Optional[float] = None, r22: Optional[float] = None, r23: Optional[float] = None, o24: Optional[float] = None,
                 r31: Optional[float] = None, r32: Optional[float] = None, r33: Optional[float] = None, o34: Optional[float] = None,
                 matrix: Optional[np.ndarray] = None,
                 x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None,
                 roll: Optional[float] = None, pitch: Optional[float] = None, yaw: Optional[float] = None,
                 qx: Optional[float] = None, qy: Optional[float] = None, qz: Optional[float] = None, qw: Optional[float] = None,
                 theta: Optional[float] = None):
        """
        Initialize Transform with various input formats.
        
        Args:
            r11-r33, o14-o34: Individual matrix elements
            matrix: 4x4 or 3x4 transformation matrix
            x, y, z: Translation components
            roll, pitch, yaw: Euler angles in radians
            qx, qy, qz, qw: Quaternion components
            theta: 2D rotation angle (for x, y, theta constructor)
        """
        # Initialize as 3x4 matrix (CV_32FC1 format like rtabmap)
        self._data = np.zeros((3, 4), dtype=np.float32)
        self._is_null = True
        
        # Determine construction method
        if matrix is not None:
            self._from_matrix(matrix)
        elif all(v is not None for v in [r11, r12, r13, o14, r21, r22, r23, o24, r31, r32, r33, o34]):
            self._from_elements(r11, r12, r13, o14, r21, r22, r23, o24, r31, r32, r33, o34)
        elif all(v is not None for v in [x, y, z]) and all(v is not None for v in [qx, qy, qz, qw]):
            self._from_translation_quaternion(x, y, z, qx, qy, qz, qw)
        elif all(v is not None for v in [x, y, z]) and all(v is not None for v in [roll, pitch, yaw]):
            self._from_translation_euler(x, y, z, roll, pitch, yaw)
        elif all(v is not None for v in [x, y]) and theta is not None:
            self._from_2d(x, y, theta)
        else:
            # Default: identity transform
            self.set_identity()
            
    def _from_matrix(self, matrix: np.ndarray) -> None:
        """Initialize from transformation matrix."""
        matrix = np.array(matrix, dtype=np.float32)
        
        if matrix.shape == (4, 4):
            self._data = matrix[:3, :4].copy()
        elif matrix.shape == (3, 4):
            self._data = matrix.copy()
        else:
            raise ValueError(f"Matrix must be 4x4 or 3x4, got {matrix.shape}")
            
        self._is_null = False
        
    def _from_elements(self, r11: float, r12: float, r13: float, o14: float,
                      r21: float, r22: float, r23: float, o24: float,
                      r31: float, r32: float, r33: float, o34: float) -> None:
        """Initialize from individual matrix elements."""
        self._data[0, :] = [r11, r12, r13, o14]
        self._data[1, :] = [r21, r22, r23, o24]
        self._data[2, :] = [r31, r32, r33, o34]
        self._is_null = False
        
    def _from_translation_euler(self, x: float, y: float, z: float,
                               roll: float, pitch: float, yaw: float) -> None:
        """Initialize from translation and Euler angles."""
        # Create rotation matrix from Euler angles (ZYX convention)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        # Rotation matrix (ZYX Euler angles)
        self._data[0, 0] = cy * cp
        self._data[0, 1] = cy * sp * sr - sy * cr
        self._data[0, 2] = cy * sp * cr + sy * sr
        self._data[0, 3] = x
        
        self._data[1, 0] = sy * cp
        self._data[1, 1] = sy * sp * sr + cy * cr
        self._data[1, 2] = sy * sp * cr - cy * sr
        self._data[1, 3] = y
        
        self._data[2, 0] = -sp
        self._data[2, 1] = cp * sr
        self._data[2, 2] = cp * cr
        self._data[2, 3] = z
        
        self._is_null = False
        
    def _from_translation_quaternion(self, x: float, y: float, z: float,
                                    qx: float, qy: float, qz: float, qw: float) -> None:
        """Initialize from translation and quaternion."""
        # Normalize quaternion
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 0:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # Convert quaternion to rotation matrix
        xx, xy, xz = qx*qx, qx*qy, qx*qz
        xw, yy, yz = qx*qw, qy*qy, qy*qz
        yw, zz, zw = qy*qw, qz*qz, qz*qw
        
        self._data[0, 0] = 1 - 2*(yy + zz)
        self._data[0, 1] = 2*(xy - zw)
        self._data[0, 2] = 2*(xz + yw)
        self._data[0, 3] = x
        
        self._data[1, 0] = 2*(xy + zw)
        self._data[1, 1] = 1 - 2*(xx + zz)
        self._data[1, 2] = 2*(yz - xw)
        self._data[1, 3] = y
        
        self._data[2, 0] = 2*(xz - yw)
        self._data[2, 1] = 2*(yz + xw)
        self._data[2, 2] = 1 - 2*(xx + yy)
        self._data[2, 3] = z
        
        self._is_null = False
        
    def _from_2d(self, x: float, y: float, theta: float) -> None:
        """Initialize from 2D pose (x, y, theta)."""
        c, s = math.cos(theta), math.sin(theta)
        
        self._data[0, :] = [c, -s, 0, x]
        self._data[1, :] = [s,  c, 0, y]
        self._data[2, :] = [0,  0, 1, 0]
        
        self._is_null = False
        
    # Element access
    def r11(self) -> float: return float(self._data[0, 0])
    def r12(self) -> float: return float(self._data[0, 1])
    def r13(self) -> float: return float(self._data[0, 2])
    def r21(self) -> float: return float(self._data[1, 0])
    def r22(self) -> float: return float(self._data[1, 1])
    def r23(self) -> float: return float(self._data[1, 2])
    def r31(self) -> float: return float(self._data[2, 0])
    def r32(self) -> float: return float(self._data[2, 1])
    def r33(self) -> float: return float(self._data[2, 2])
    
    def o14(self) -> float: return float(self._data[0, 3])
    def o24(self) -> float: return float(self._data[1, 3])
    def o34(self) -> float: return float(self._data[2, 3])
    
    def x(self) -> float: return float(self._data[0, 3])
    def y(self) -> float: return float(self._data[1, 3])
    def z(self) -> float: return float(self._data[2, 3])
    
    # State queries
    def is_null(self) -> bool:
        """Check if transform is null (uninitialized)."""
        return self._is_null
        
    def is_identity(self) -> bool:
        """Check if transform is identity."""
        if self._is_null:
            return False
            
        identity = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ], dtype=np.float32)
        
        return np.allclose(self._data, identity, atol=1e-6)
        
    # State setters
    def set_null(self) -> None:
        """Set transform to null state."""
        self._data.fill(0)
        self._is_null = True
        
    def set_identity(self) -> None:
        """Set transform to identity."""
        self._data = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ], dtype=np.float32)
        self._is_null = False
        
    # Data access
    def data_matrix(self) -> np.ndarray:
        """Get 3x4 transformation matrix."""
        return self._data.copy()
        
    def to_matrix_4x4(self) -> np.ndarray:
        """Get 4x4 homogeneous transformation matrix."""
        matrix = np.zeros((4, 4), dtype=np.float32)
        matrix[:3, :] = self._data
        matrix[3, 3] = 1.0
        return matrix
        
    def data(self) -> np.ndarray:
        """Get flattened matrix data (12 elements)."""
        return self._data.flatten()
        
    # Geometric queries
    def theta(self) -> float:
        """Get 2D rotation angle."""
        return math.atan2(self.r21(), self.r11())
        
    def get_translation(self) -> np.ndarray:
        """Get translation vector [x, y, z]."""
        return self._data[:, 3].copy()
        
    def get_rotation_matrix(self) -> np.ndarray:
        """Get 3x3 rotation matrix."""
        return self._data[:, :3].copy()
        
    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Get Euler angles (roll, pitch, yaw) in radians.
        
        Returns:
            Tuple of (roll, pitch, yaw) angles
        """
        # Extract angles from rotation matrix (ZYX convention)
        r11, r12, r13 = self.r11(), self.r12(), self.r13()
        r21, r22, r23 = self.r21(), self.r22(), self.r23()
        r31, r32, r33 = self.r31(), self.r32(), self.r33()
        
        # Pitch (y-axis rotation)
        pitch = math.asin(-r31)
        
        # Roll (x-axis rotation) and Yaw (z-axis rotation)
        if abs(math.cos(pitch)) > 1e-6:
            roll = math.atan2(r32, r33)
            yaw = math.atan2(r21, r11)
        else:
            # Gimbal lock case
            roll = 0.0
            yaw = math.atan2(-r12, r22)
            
        return (roll, pitch, yaw)
        
    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Get quaternion representation (qx, qy, qz, qw).
        
        Returns:
            Tuple of (qx, qy, qz, qw) quaternion components
        """
        # Extract rotation matrix elements
        r11, r12, r13 = self.r11(), self.r12(), self.r13()
        r21, r22, r23 = self.r21(), self.r22(), self.r23()
        r31, r32, r33 = self.r31(), self.r32(), self.r33()
        
        # Convert rotation matrix to quaternion
        trace = r11 + r22 + r33
        
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (r32 - r23) / s
            qy = (r13 - r31) / s
            qz = (r21 - r12) / s
        elif r11 > r22 and r11 > r33:
            s = math.sqrt(1.0 + r11 - r22 - r33) * 2  # s = 4 * qx
            qw = (r32 - r23) / s
            qx = 0.25 * s
            qy = (r12 + r21) / s
            qz = (r13 + r31) / s
        elif r22 > r33:
            s = math.sqrt(1.0 + r22 - r11 - r33) * 2  # s = 4 * qy
            qw = (r13 - r31) / s
            qx = (r12 + r21) / s
            qy = 0.25 * s
            qz = (r23 + r32) / s
        else:
            s = math.sqrt(1.0 + r33 - r11 - r22) * 2  # s = 4 * qz
            qw = (r21 - r12) / s
            qx = (r13 + r31) / s
            qy = (r23 + r32) / s
            qz = 0.25 * s
            
        return (qx, qy, qz, qw)
        
    def norm(self) -> float:
        """Get translation norm (distance from origin)."""
        return float(np.linalg.norm(self.get_translation()))
        
    def get_distance_squared(self) -> float:
        """Get squared distance from origin."""
        trans = self.get_translation()
        return float(np.dot(trans, trans))
        
    def get_angle(self) -> float:
        """Get rotation angle magnitude."""
        qx, qy, qz, qw = self.get_quaternion()
        return 2.0 * math.acos(min(1.0, abs(qw)))
        
    # Transformations
    def inverse(self) -> 'Transform':
        """Get inverse transformation."""
        if self._is_null:
            return Transform()
            
        # For homogeneous transformation matrix:
        # T^-1 = [R^T  -R^T*t]
        #        [0    1     ]
        
        R = self.get_rotation_matrix()
        t = self.get_translation()
        
        R_inv = R.T
        t_inv = -R_inv @ t
        
        result = Transform()
        result._data[:, :3] = R_inv
        result._data[:, 3] = t_inv
        result._is_null = False
        
        return result
        
    def clone(self) -> 'Transform':
        """Create a copy of this transform."""
        result = Transform()
        result._data = self._data.copy()
        result._is_null = self._is_null
        return result
        
    # Operators
    def __mul__(self, other: 'Transform') -> 'Transform':
        """Multiply two transforms (composition)."""
        if self._is_null or other._is_null:
            return Transform()
            
        # Convert to 4x4 matrices for multiplication
        m1 = self.to_matrix_4x4()
        m2 = other.to_matrix_4x4()
        result_matrix = m1 @ m2
        
        return Transform(matrix=result_matrix)
        
    def __eq__(self, other: 'Transform') -> bool:
        """Check equality with another transform."""
        if not isinstance(other, Transform):
            return False
            
        if self._is_null and other._is_null:
            return True
        elif self._is_null or other._is_null:
            return False
        else:
            return np.allclose(self._data, other._data, atol=1e-6)
            
    # Static methods
    @staticmethod
    def get_identity() -> 'Transform':
        """Get identity transform."""
        t = Transform()
        t.set_identity()
        return t
        
    @staticmethod
    def optical_rotation() -> 'Transform':
        """
        Get optical rotation transform.
        
        Transforms image coordinate frame (x->right, y->down, z->forward)
        to robot coordinate frame (x->forward, y->left, z->up).
        """
        return Transform(
            0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0
        )
        
    # String representation
    def __repr__(self) -> str:
        """String representation."""
        if self._is_null:
            return "Transform(null)"
        else:
            x, y, z = self.x(), self.y(), self.z()
            roll, pitch, yaw = self.get_euler_angles()
            return (f"Transform(x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                   f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f})")
                   
    def __str__(self) -> str:
        """Detailed string representation."""
        if self._is_null:
            return "Transform: null"
        else:
            return f"Transform:\n{self._data}"
