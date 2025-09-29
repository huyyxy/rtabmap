"""
Python wrapper for stereo camera functionality.
"""

import numpy as np
import cv2
import time
import os
import glob
from typing import Optional, List, Tuple, Iterator

from ..core.sensor_data import SensorData
from ..core.camera_model import CameraModel
from ..core.transform import Transform


class CameraStereo:
    """
    Python wrapper for stereo camera capture.
    
    Provides interface for capturing synchronized left and right stereo images
    and computing depth through stereo matching.
    
    Example:
        >>> # From stereo image pairs
        >>> camera = CameraStereo(left_path="/path/to/left/", right_path="/path/to/right/")
        >>> camera.init()
        >>> sensor_data = camera.take_image()
        >>> 
        >>> # Process stereo pairs
        >>> for data in camera:
        ...     left = data.image_raw()
        ...     right = data.depth_raw()  # Right image stored in depth field
        ...     process_stereo(left, right)
    """
    
    def __init__(self,
                 device_id_left: Optional[int] = None,
                 device_id_right: Optional[int] = None,
                 left_path: Optional[str] = None,
                 right_path: Optional[str] = None,
                 frame_rate: float = 30.0,
                 baseline: float = 0.12,  # 12cm typical stereo baseline
                 compute_depth: bool = True,
                 local_transform: Optional[Transform] = None):
        """
        Initialize stereo camera.
        
        Args:
            device_id_left: Left camera device ID
            device_id_right: Right camera device ID  
            left_path: Path to directory containing left images
            right_path: Path to directory containing right images
            frame_rate: Desired frame rate (Hz)
            baseline: Stereo baseline in meters
            compute_depth: Whether to compute depth from stereo matching
            local_transform: Transform from base frame to camera frame
        """
        self._device_id_left = device_id_left
        self._device_id_right = device_id_right
        self._left_path = left_path
        self._right_path = right_path
        self._frame_rate = frame_rate
        self._baseline = baseline
        self._compute_depth = compute_depth
        self._local_transform = local_transform or Transform.optical_rotation()
        
        # Internal state
        self._cap_left = None
        self._cap_right = None
        self._left_files = []
        self._right_files = []
        self._current_index = 0
        self._initialized = False
        self._camera_model = CameraModel()
        self._last_capture_time = 0.0
        
        # Stereo matching
        self._stereo_matcher = None
        self._disparity_to_depth_scale = 1.0
        
        # Determine capture mode
        if device_id_left is not None and device_id_right is not None:
            self._mode = 'devices'
        elif left_path is not None and right_path is not None:
            self._mode = 'images'
        else:
            self._mode = 'devices'
            self._device_id_left = 0
            self._device_id_right = 1
            
    def init(self, calibration_folder: str = "", camera_name: str = "") -> bool:
        """
        Initialize stereo camera.
        
        Args:
            calibration_folder: Path to calibration files
            camera_name: Camera name for calibration lookup
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._mode == 'devices':
                return self._init_devices()
            elif self._mode == 'images':
                return self._init_images()
            else:
                return False
                
        except Exception as e:
            print(f"Error initializing stereo camera: {e}")
            return False
            
    def _init_devices(self) -> bool:
        """Initialize stereo device capture."""
        self._cap_left = cv2.VideoCapture(self._device_id_left)
        self._cap_right = cv2.VideoCapture(self._device_id_right)
        
        if not self._cap_left.isOpened():
            print(f"Failed to open left camera {self._device_id_left}")
            return False
            
        if not self._cap_right.isOpened():
            print(f"Failed to open right camera {self._device_id_right}")
            return False
            
        # Get camera properties from left camera
        width = int(self._cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Create stereo camera model
        fx = width * 0.8  # Rough estimate
        fy = height * 0.8
        cx = width / 2.0
        cy = height / 2.0
        
        self._camera_model = CameraModel(
            name=f"stereo_{self._device_id_left}_{self._device_id_right}",
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            Tx=-self._baseline * fx,  # Negative for right camera
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        # Initialize stereo matcher
        if self._compute_depth:
            self._init_stereo_matcher()
            
        self._initialized = True
        print(f"Stereo cameras {self._device_id_left}, {self._device_id_right} initialized: {width}x{height}")
        return True
        
    def _init_images(self) -> bool:
        """Initialize stereo image pair capture."""
        if not os.path.exists(self._left_path):
            print(f"Left image path not found: {self._left_path}")
            return False
            
        if not os.path.exists(self._right_path):
            print(f"Right image path not found: {self._right_path}")
            return False
            
        # Find left image files
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
        self._left_files = []
        
        for ext in extensions:
            pattern = os.path.join(self._left_path, ext)
            self._left_files.extend(glob.glob(pattern))
            pattern = os.path.join(self._left_path, ext.upper())
            self._left_files.extend(glob.glob(pattern))
            
        if not self._left_files:
            print(f"No left images found in: {self._left_path}")
            return False
            
        # Find right image files
        self._right_files = []
        
        for ext in extensions:
            pattern = os.path.join(self._right_path, ext)
            self._right_files.extend(glob.glob(pattern))
            pattern = os.path.join(self._right_path, ext.upper())
            self._right_files.extend(glob.glob(pattern))
            
        if not self._right_files:
            print(f"No right images found in: {self._right_path}")
            return False
            
        # Sort files
        self._left_files.sort()
        self._right_files.sort()
        
        # Check if we have matching pairs
        if len(self._left_files) != len(self._right_files):
            print(f"Warning: Left ({len(self._left_files)}) and right ({len(self._right_files)}) "
                  f"image counts don't match")
            
        self._current_index = 0
        
        # Read first left image to get dimensions
        first_left = cv2.imread(self._left_files[0])
        if first_left is None:
            print(f"Failed to read first left image: {self._left_files[0]}")
            return False
            
        height, width = first_left.shape[:2]
        
        # Create stereo camera model
        fx = width * 0.8
        fy = height * 0.8
        cx = width / 2.0
        cy = height / 2.0
        
        self._camera_model = CameraModel(
            name=f"stereo_images_{os.path.basename(self._left_path)}",
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            Tx=-self._baseline * fx,
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        # Initialize stereo matcher
        if self._compute_depth:
            self._init_stereo_matcher()
            
        self._initialized = True
        print(f"Stereo image pairs initialized: {len(self._left_files)} pairs, {width}x{height}")
        return True
        
    def _init_stereo_matcher(self) -> None:
        """Initialize stereo matching algorithm."""
        # Create stereo matcher (using OpenCV's StereoBM)
        self._stereo_matcher = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        
        # Calculate disparity to depth scale factor
        # depth = (fx * baseline) / disparity
        fx = self._camera_model.fx()
        self._disparity_to_depth_scale = fx * self._baseline
        
    def take_image(self) -> SensorData:
        """
        Capture next stereo image pair.
        
        Returns:
            SensorData containing left image (in image field) and right image 
            (in depth field), or computed depth if compute_depth=True
        """
        if not self._initialized:
            print("Stereo camera not initialized")
            return SensorData()
            
        # Respect frame rate timing
        current_time = time.time()
        if self._frame_rate > 0:
            min_interval = 1.0 / self._frame_rate
            elapsed = current_time - self._last_capture_time
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)
                current_time = time.time()
                
        try:
            if self._mode == 'devices':
                return self._capture_from_devices()
            elif self._mode == 'images':
                return self._capture_from_images()
            else:
                return SensorData()
                
        except Exception as e:
            print(f"Error capturing stereo images: {e}")
            return SensorData()
        finally:
            self._last_capture_time = time.time()
            
    def _capture_from_devices(self) -> SensorData:
        """Capture stereo pair from devices."""
        ret_left, left_frame = self._cap_left.read()
        ret_right, right_frame = self._cap_right.read()
        
        if not ret_left or not ret_right or left_frame is None or right_frame is None:
            return SensorData()  # Capture failed
            
        return self._process_stereo_pair(left_frame, right_frame)
        
    def _capture_from_images(self) -> SensorData:
        """Capture stereo pair from image files."""
        if (self._current_index >= len(self._left_files) or 
            self._current_index >= len(self._right_files)):
            return SensorData()  # End of sequence
            
        left_file = self._left_files[self._current_index]
        right_file = self._right_files[self._current_index]
        
        # Read left image
        left_frame = cv2.imread(left_file)
        if left_frame is None:
            print(f"Failed to read left image: {left_file}")
            self._current_index += 1
            return SensorData()
            
        # Read right image
        right_frame = cv2.imread(right_file)
        if right_frame is None:
            print(f"Failed to read right image: {right_file}")
            self._current_index += 1
            return SensorData()
            
        self._current_index += 1
        return self._process_stereo_pair(left_frame, right_frame)
        
    def _process_stereo_pair(self, left_frame: np.ndarray, right_frame: np.ndarray) -> SensorData:
        """Process stereo pair and optionally compute depth."""
        # Generate timestamp and ID
        timestamp = time.time()
        image_id = self._current_index if self._mode == 'images' else int(timestamp * 1000000) % 1000000
        
        if self._compute_depth and self._stereo_matcher is not None:
            # Compute depth from stereo matching
            depth = self._compute_depth_from_stereo(left_frame, right_frame)
            
            return SensorData(
                image=left_frame,
                depth=depth,
                camera_model=self._camera_model,
                image_id=image_id,
                timestamp=timestamp
            )
        else:
            # Store right image in depth field for access
            # Convert right image to single channel if needed
            if len(right_frame.shape) == 3:
                right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            else:
                right_gray = right_frame
                
            return SensorData(
                image=left_frame,
                depth=right_gray.astype(np.uint16),  # Store as uint16 for consistency
                camera_model=self._camera_model,
                image_id=image_id,
                timestamp=timestamp
            )
            
    def _compute_depth_from_stereo(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Compute depth map from stereo pair."""
        # Convert to grayscale if needed
        if len(left.shape) == 3:
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left
            
        if len(right.shape) == 3:
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        else:
            right_gray = right
            
        # Compute disparity
        disparity = self._stereo_matcher.compute(left_gray, right_gray)
        
        # Convert disparity to depth
        # Avoid division by zero
        valid_mask = disparity > 0
        depth = np.zeros_like(disparity, dtype=np.float32)
        depth[valid_mask] = self._disparity_to_depth_scale / disparity[valid_mask]
        
        # Convert to millimeters and clip to uint16 range
        depth_mm = np.clip(depth * 1000, 0, 65535).astype(np.uint16)
        
        return depth_mm
        
    def get_stereo_images(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get the last captured stereo pair.
        
        Returns:
            Tuple of (left_image, right_image) or (None, None) if not available
        """
        sensor_data = self.take_image()
        if not sensor_data.is_valid():
            return None, None
            
        left = sensor_data.image_raw()
        
        if self._compute_depth:
            # If we computed depth, we need to capture again to get right image
            # This is a limitation of the current design
            return left, None
        else:
            # Right image is stored in depth field
            right = sensor_data.depth_raw()
            return left, right
            
    def is_valid(self) -> bool:
        """Check if stereo camera is valid and ready."""
        return self._initialized
        
    def get_camera_model(self) -> CameraModel:
        """Get camera model."""
        return self._camera_model
        
    def set_camera_model(self, camera_model: CameraModel) -> None:
        """Set camera model."""
        self._camera_model = camera_model
        
    def get_baseline(self) -> float:
        """Get stereo baseline in meters."""
        return self._baseline
        
    def set_baseline(self, baseline: float) -> None:
        """Set stereo baseline and update camera model."""
        self._baseline = baseline
        fx = self._camera_model.fx()
        # Update Tx in camera model
        self._camera_model = CameraModel(
            name=self._camera_model.name(),
            image_size=self._camera_model.image_size(),
            K=self._camera_model.K(),
            D=self._camera_model.D(),
            R=self._camera_model.R(),
            P=self._camera_model.P(),
            local_transform=self._camera_model.local_transform(),
            Tx=-baseline * fx
        )
        
        # Update stereo matcher scale
        if self._stereo_matcher:
            self._disparity_to_depth_scale = fx * baseline
            
    def get_frame_rate(self) -> float:
        """Get frame rate."""
        return self._frame_rate
        
    def set_frame_rate(self, frame_rate: float) -> None:
        """Set frame rate."""
        self._frame_rate = frame_rate
        
    def set_compute_depth(self, compute_depth: bool) -> None:
        """Enable/disable depth computation."""
        self._compute_depth = compute_depth
        if compute_depth and not self._stereo_matcher:
            self._init_stereo_matcher()
            
    def get_image_count(self) -> int:
        """Get total number of stereo pairs."""
        if self._mode == 'images':
            return min(len(self._left_files), len(self._right_files))
        else:
            return -1  # Unknown for devices
            
    def get_current_index(self) -> int:
        """Get current image index."""
        return self._current_index
        
    def seek(self, index: int) -> bool:
        """Seek to specific stereo pair index."""
        if self._mode == 'images':
            max_index = min(len(self._left_files), len(self._right_files))
            if 0 <= index < max_index:
                self._current_index = index
                return True
            return False
        else:
            return False  # Not supported for device mode
            
    def close(self) -> None:
        """Close cameras and release resources."""
        if self._cap_left:
            self._cap_left.release()
            self._cap_left = None
            
        if self._cap_right:
            self._cap_right.release()
            self._cap_right = None
            
        self._initialized = False
        self._left_files = []
        self._right_files = []
        self._current_index = 0
        
    def __iter__(self) -> Iterator[SensorData]:
        """Iterator interface for processing all stereo pairs."""
        return self
        
    def __next__(self) -> SensorData:
        """Get next stereo pair in iteration."""
        data = self.take_image()
        if not data.is_valid():
            raise StopIteration
        return data
        
    def __enter__(self):
        """Context manager entry."""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        
    def __repr__(self) -> str:
        """String representation."""
        return (f"CameraStereo(mode={self._mode}, initialized={self._initialized}, "
                f"baseline={self._baseline:.3f}m, compute_depth={self._compute_depth})")
                
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = [f"CameraStereo ({self._mode} mode):"]
        
        if self._mode == 'devices':
            lines.append(f"  Left device: {self._device_id_left}")
            lines.append(f"  Right device: {self._device_id_right}")
        elif self._mode == 'images':
            lines.append(f"  Left path: {self._left_path}")
            lines.append(f"  Right path: {self._right_path}")
            lines.append(f"  Left images: {len(self._left_files)}")
            lines.append(f"  Right images: {len(self._right_files)}")
            lines.append(f"  Current index: {self._current_index}")
            
        lines.append(f"  Initialized: {self._initialized}")
        lines.append(f"  Frame rate: {self._frame_rate} Hz")
        lines.append(f"  Baseline: {self._baseline:.3f} m")
        lines.append(f"  Compute depth: {self._compute_depth}")
        
        if self._initialized:
            size = self._camera_model.image_size()
            lines.append(f"  Image size: {size[0]}x{size[1]}")
            
        return "\n".join(lines)
