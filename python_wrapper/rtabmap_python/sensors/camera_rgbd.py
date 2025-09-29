"""
Python wrapper for RGB-D camera functionality.
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


class CameraRGBD:
    """
    Python wrapper for RGB-D camera capture.
    
    Provides interface for capturing synchronized RGB and depth images
    from various sources including RGB-D cameras and image pairs.
    
    Example:
        >>> # From RGB-D camera (simulated with paired images)
        >>> camera = CameraRGBD(rgb_path="/path/to/rgb/", depth_path="/path/to/depth/")
        >>> camera.init()
        >>> sensor_data = camera.take_image()
        >>> 
        >>> # Process all RGB-D pairs
        >>> for data in camera:
        ...     rgb = data.image_raw()
        ...     depth = data.depth_raw()
        ...     process_rgbd(rgb, depth)
    """
    
    def __init__(self,
                 device_id: Optional[int] = None,
                 rgb_path: Optional[str] = None,
                 depth_path: Optional[str] = None,
                 frame_rate: float = 30.0,
                 depth_scale: float = 1000.0,
                 local_transform: Optional[Transform] = None):
        """
        Initialize RGB-D camera.
        
        Args:
            device_id: Camera device ID (for RGB-D cameras like RealSense)
            rgb_path: Path to directory containing RGB images
            depth_path: Path to directory containing depth images
            frame_rate: Desired frame rate (Hz)
            depth_scale: Scale factor for depth values (e.g., 1000 for mm to m)
            local_transform: Transform from base frame to camera frame
        """
        self._device_id = device_id
        self._rgb_path = rgb_path
        self._depth_path = depth_path
        self._frame_rate = frame_rate
        self._depth_scale = depth_scale
        self._local_transform = local_transform or Transform.optical_rotation()
        
        # Internal state
        self._cap = None
        self._rgb_files = []
        self._depth_files = []
        self._current_index = 0
        self._initialized = False
        self._camera_model = CameraModel()
        self._last_capture_time = 0.0
        
        # Determine capture mode
        if device_id is not None:
            self._mode = 'device'
        elif rgb_path is not None and depth_path is not None:
            self._mode = 'images'
        else:
            self._mode = 'device'
            self._device_id = 0
            
    def init(self, calibration_folder: str = "", camera_name: str = "") -> bool:
        """
        Initialize RGB-D camera.
        
        Args:
            calibration_folder: Path to calibration files
            camera_name: Camera name for calibration lookup
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._mode == 'device':
                return self._init_device()
            elif self._mode == 'images':
                return self._init_images()
            else:
                return False
                
        except Exception as e:
            print(f"Error initializing RGB-D camera: {e}")
            return False
            
    def _init_device(self) -> bool:
        """Initialize RGB-D device capture."""
        # Note: In a real implementation, this would interface with
        # actual RGB-D cameras like RealSense, Kinect, etc.
        # For this simulation, we'll create a mock interface
        
        print(f"Simulating RGB-D device {self._device_id}")
        
        # Create default camera model for RGB-D device
        self._camera_model = CameraModel(
            name=f"rgbd_device_{self._device_id}",
            fx=525.0,  # Typical RGB-D camera parameters
            fy=525.0,
            cx=320.0,
            cy=240.0,
            image_size=(640, 480),
            local_transform=self._local_transform
        )
        
        self._initialized = True
        print(f"RGB-D device {self._device_id} initialized (simulated)")
        return True
        
    def _init_images(self) -> bool:
        """Initialize RGB-D image pair capture."""
        if not os.path.exists(self._rgb_path):
            print(f"RGB path not found: {self._rgb_path}")
            return False
            
        if not os.path.exists(self._depth_path):
            print(f"Depth path not found: {self._depth_path}")
            return False
            
        # Find RGB image files
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
        self._rgb_files = []
        
        for ext in extensions:
            pattern = os.path.join(self._rgb_path, ext)
            self._rgb_files.extend(glob.glob(pattern))
            pattern = os.path.join(self._rgb_path, ext.upper())
            self._rgb_files.extend(glob.glob(pattern))
            
        if not self._rgb_files:
            print(f"No RGB images found in: {self._rgb_path}")
            return False
            
        # Find depth image files
        depth_extensions = ['*.png', '*.tiff', '*.tif', '*.pgm', '*.exr']
        self._depth_files = []
        
        for ext in depth_extensions:
            pattern = os.path.join(self._depth_path, ext)
            self._depth_files.extend(glob.glob(pattern))
            pattern = os.path.join(self._depth_path, ext.upper())
            self._depth_files.extend(glob.glob(pattern))
            
        if not self._depth_files:
            print(f"No depth images found in: {self._depth_path}")
            return False
            
        # Sort files
        self._rgb_files.sort()
        self._depth_files.sort()
        
        # Check if we have matching pairs
        if len(self._rgb_files) != len(self._depth_files):
            print(f"Warning: RGB ({len(self._rgb_files)}) and depth ({len(self._depth_files)}) "
                  f"image counts don't match")
            
        self._current_index = 0
        
        # Read first RGB image to get dimensions
        first_rgb = cv2.imread(self._rgb_files[0])
        if first_rgb is None:
            print(f"Failed to read first RGB image: {self._rgb_files[0]}")
            return False
            
        height, width = first_rgb.shape[:2]
        
        # Create default camera model
        self._camera_model = CameraModel(
            name=f"rgbd_images_{os.path.basename(self._rgb_path)}",
            fx=width * 0.8,
            fy=height * 0.8,
            cx=width / 2.0,
            cy=height / 2.0,
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        self._initialized = True
        print(f"RGB-D image pairs initialized: {len(self._rgb_files)} pairs, {width}x{height}")
        return True
        
    def take_image(self) -> SensorData:
        """
        Capture next RGB-D image pair.
        
        Returns:
            SensorData containing RGB and depth images, or empty if failed
        """
        if not self._initialized:
            print("RGB-D camera not initialized")
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
            if self._mode == 'device':
                return self._capture_from_device()
            elif self._mode == 'images':
                return self._capture_from_images()
            else:
                return SensorData()
                
        except Exception as e:
            print(f"Error capturing RGB-D image: {e}")
            return SensorData()
        finally:
            self._last_capture_time = time.time()
            
    def _capture_from_device(self) -> SensorData:
        """Capture RGB-D from device (simulated)."""
        # Note: In a real implementation, this would capture from actual device
        # For simulation, we'll generate synthetic data
        
        width, height = self._camera_model.image_size()
        
        # Generate synthetic RGB image
        rgb = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        
        # Generate synthetic depth image (in millimeters)
        depth = np.random.randint(500, 5000, (height, width), dtype=np.uint16)
        
        # Generate timestamp and ID
        timestamp = time.time()
        image_id = int(timestamp * 1000000) % 1000000
        
        return SensorData(
            image=rgb,
            depth=depth,
            camera_model=self._camera_model,
            image_id=image_id,
            timestamp=timestamp
        )
        
    def _capture_from_images(self) -> SensorData:
        """Capture RGB-D from image pairs."""
        if (self._current_index >= len(self._rgb_files) or 
            self._current_index >= len(self._depth_files)):
            return SensorData()  # End of sequence
            
        rgb_file = self._rgb_files[self._current_index]
        depth_file = self._depth_files[self._current_index]
        
        # Read RGB image
        rgb = cv2.imread(rgb_file)
        if rgb is None:
            print(f"Failed to read RGB image: {rgb_file}")
            self._current_index += 1
            return SensorData()
            
        # Read depth image
        depth = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH)
        if depth is None:
            print(f"Failed to read depth image: {depth_file}")
            self._current_index += 1
            return SensorData()
            
        # Apply depth scaling if needed
        if self._depth_scale != 1.0:
            depth = (depth.astype(np.float32) / self._depth_scale).astype(np.uint16)
            
        # Generate timestamp and ID
        timestamp = time.time()
        image_id = self._current_index + 1
        
        self._current_index += 1
        
        return SensorData(
            image=rgb,
            depth=depth,
            camera_model=self._camera_model,
            image_id=image_id,
            timestamp=timestamp
        )
        
    def is_valid(self) -> bool:
        """Check if camera is valid and ready."""
        return self._initialized
        
    def get_camera_model(self) -> CameraModel:
        """Get camera model."""
        return self._camera_model
        
    def set_camera_model(self, camera_model: CameraModel) -> None:
        """Set camera model."""
        self._camera_model = camera_model
        
    def get_frame_rate(self) -> float:
        """Get frame rate."""
        return self._frame_rate
        
    def set_frame_rate(self, frame_rate: float) -> None:
        """Set frame rate."""
        self._frame_rate = frame_rate
        
    def get_depth_scale(self) -> float:
        """Get depth scale factor."""
        return self._depth_scale
        
    def set_depth_scale(self, scale: float) -> None:
        """Set depth scale factor."""
        self._depth_scale = scale
        
    def get_local_transform(self) -> Transform:
        """Get local transform."""
        return self._local_transform
        
    def set_local_transform(self, transform: Transform) -> None:
        """Set local transform."""
        self._local_transform = transform
        
    def get_image_count(self) -> int:
        """Get total number of image pairs."""
        if self._mode == 'images':
            return min(len(self._rgb_files), len(self._depth_files))
        else:
            return -1  # Unknown for device
            
    def get_current_index(self) -> int:
        """Get current image index."""
        return self._current_index
        
    def seek(self, index: int) -> bool:
        """
        Seek to specific image pair index.
        
        Args:
            index: Target index
            
        Returns:
            True if successful, False otherwise
        """
        if self._mode == 'images':
            max_index = min(len(self._rgb_files), len(self._depth_files))
            if 0 <= index < max_index:
                self._current_index = index
                return True
            return False
        else:
            return False  # Not supported for device mode
            
    def close(self) -> None:
        """Close camera and release resources."""
        if self._cap:
            self._cap.release()
            self._cap = None
            
        self._initialized = False
        self._rgb_files = []
        self._depth_files = []
        self._current_index = 0
        
    def __iter__(self) -> Iterator[SensorData]:
        """Iterator interface for processing all image pairs."""
        return self
        
    def __next__(self) -> SensorData:
        """Get next image pair in iteration."""
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
        return (f"CameraRGBD(mode={self._mode}, initialized={self._initialized}, "
                f"frame_rate={self._frame_rate}, depth_scale={self._depth_scale})")
                
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = [f"CameraRGBD ({self._mode} mode):"]
        
        if self._mode == 'device':
            lines.append(f"  Device ID: {self._device_id}")
        elif self._mode == 'images':
            lines.append(f"  RGB path: {self._rgb_path}")
            lines.append(f"  Depth path: {self._depth_path}")
            lines.append(f"  RGB images: {len(self._rgb_files)}")
            lines.append(f"  Depth images: {len(self._depth_files)}")
            lines.append(f"  Current index: {self._current_index}")
            
        lines.append(f"  Initialized: {self._initialized}")
        lines.append(f"  Frame rate: {self._frame_rate} Hz")
        lines.append(f"  Depth scale: {self._depth_scale}")
        
        if self._initialized:
            size = self._camera_model.image_size()
            lines.append(f"  Image size: {size[0]}x{size[1]}")
            
        return "\n".join(lines)
