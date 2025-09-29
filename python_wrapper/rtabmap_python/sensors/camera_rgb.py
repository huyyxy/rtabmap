"""
Python wrapper for RGB camera functionality.
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


class CameraRGB:
    """
    Python wrapper for RGB camera capture.
    
    Provides interface for capturing RGB images from various sources
    including webcams, image sequences, and video files.
    
    Example:
        >>> # From webcam
        >>> camera = CameraRGB(device_id=0)
        >>> camera.init()
        >>> sensor_data = camera.take_image()
        >>> 
        >>> # From image directory
        >>> camera = CameraRGB(image_path="/path/to/images/")
        >>> camera.init()
        >>> for data in camera:
        ...     process(data)
    """
    
    def __init__(self,
                 device_id: Optional[int] = None,
                 image_path: Optional[str] = None,
                 video_path: Optional[str] = None,
                 frame_rate: float = 30.0,
                 local_transform: Optional[Transform] = None):
        """
        Initialize RGB camera.
        
        Args:
            device_id: Camera device ID (for webcam)
            image_path: Path to directory containing images
            video_path: Path to video file
            frame_rate: Desired frame rate (Hz)
            local_transform: Transform from base frame to camera frame
        """
        self._device_id = device_id
        self._image_path = image_path
        self._video_path = video_path
        self._frame_rate = frame_rate
        self._local_transform = local_transform or Transform.optical_rotation()
        
        # Internal state
        self._cap = None
        self._image_files = []
        self._current_index = 0
        self._initialized = False
        self._camera_model = CameraModel()
        self._last_capture_time = 0.0
        
        # Determine capture mode
        if device_id is not None:
            self._mode = 'webcam'
        elif image_path is not None:
            self._mode = 'images'
        elif video_path is not None:
            self._mode = 'video'
        else:
            self._mode = 'webcam'
            self._device_id = 0
            
    def init(self, calibration_folder: str = "", camera_name: str = "") -> bool:
        """
        Initialize camera.
        
        Args:
            calibration_folder: Path to calibration files
            camera_name: Camera name for calibration lookup
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._mode == 'webcam':
                return self._init_webcam()
            elif self._mode == 'video':
                return self._init_video()
            elif self._mode == 'images':
                return self._init_images()
            else:
                return False
                
        except Exception as e:
            print(f"Error initializing camera: {e}")
            return False
            
    def _init_webcam(self) -> bool:
        """Initialize webcam capture."""
        self._cap = cv2.VideoCapture(self._device_id)
        if not self._cap.isOpened():
            print(f"Failed to open webcam {self._device_id}")
            return False
            
        # Get camera properties
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Create default camera model
        self._camera_model = CameraModel(
            name=f"webcam_{self._device_id}",
            fx=width * 0.8,  # Rough estimate
            fy=height * 0.8,
            cx=width / 2.0,
            cy=height / 2.0,
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        self._initialized = True
        print(f"Webcam {self._device_id} initialized: {width}x{height}")
        return True
        
    def _init_video(self) -> bool:
        """Initialize video file capture."""
        if not os.path.exists(self._video_path):
            print(f"Video file not found: {self._video_path}")
            return False
            
        self._cap = cv2.VideoCapture(self._video_path)
        if not self._cap.isOpened():
            print(f"Failed to open video: {self._video_path}")
            return False
            
        # Get video properties
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        # Override frame rate with video FPS if not specified
        if self._frame_rate <= 0:
            self._frame_rate = fps if fps > 0 else 30.0
            
        # Create default camera model
        self._camera_model = CameraModel(
            name=f"video_{os.path.basename(self._video_path)}",
            fx=width * 0.8,
            fy=height * 0.8,
            cx=width / 2.0,
            cy=height / 2.0,
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        self._initialized = True
        print(f"Video initialized: {width}x{height}, {frame_count} frames, {fps:.1f} FPS")
        return True
        
    def _init_images(self) -> bool:
        """Initialize image sequence capture."""
        if not os.path.exists(self._image_path):
            print(f"Image path not found: {self._image_path}")
            return False
            
        # Find image files
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
        self._image_files = []
        
        for ext in extensions:
            pattern = os.path.join(self._image_path, ext)
            self._image_files.extend(glob.glob(pattern))
            pattern = os.path.join(self._image_path, ext.upper())
            self._image_files.extend(glob.glob(pattern))
            
        if not self._image_files:
            print(f"No image files found in: {self._image_path}")
            return False
            
        # Sort files
        self._image_files.sort()
        self._current_index = 0
        
        # Read first image to get dimensions
        first_image = cv2.imread(self._image_files[0])
        if first_image is None:
            print(f"Failed to read first image: {self._image_files[0]}")
            return False
            
        height, width = first_image.shape[:2]
        
        # Create default camera model
        self._camera_model = CameraModel(
            name=f"images_{os.path.basename(self._image_path)}",
            fx=width * 0.8,
            fy=height * 0.8,
            cx=width / 2.0,
            cy=height / 2.0,
            image_size=(width, height),
            local_transform=self._local_transform
        )
        
        self._initialized = True
        print(f"Image sequence initialized: {len(self._image_files)} images, {width}x{height}")
        return True
        
    def take_image(self) -> SensorData:
        """
        Capture next image.
        
        Returns:
            SensorData containing captured image, or empty SensorData if failed
        """
        if not self._initialized:
            print("Camera not initialized")
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
            if self._mode in ['webcam', 'video']:
                return self._capture_from_video()
            elif self._mode == 'images':
                return self._capture_from_images()
            else:
                return SensorData()
                
        except Exception as e:
            print(f"Error capturing image: {e}")
            return SensorData()
        finally:
            self._last_capture_time = time.time()
            
    def _capture_from_video(self) -> SensorData:
        """Capture frame from video source."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return SensorData()  # End of video or capture failed
            
        # Generate timestamp and ID
        timestamp = time.time()
        image_id = int(timestamp * 1000000) % 1000000
        
        return SensorData(
            image=frame,
            camera_model=self._camera_model,
            image_id=image_id,
            timestamp=timestamp
        )
        
    def _capture_from_images(self) -> SensorData:
        """Capture frame from image sequence."""
        if self._current_index >= len(self._image_files):
            return SensorData()  # End of sequence
            
        image_file = self._image_files[self._current_index]
        image = cv2.imread(image_file)
        
        if image is None:
            print(f"Failed to read image: {image_file}")
            self._current_index += 1
            return SensorData()
            
        # Generate timestamp and ID
        timestamp = time.time()
        image_id = self._current_index + 1
        
        self._current_index += 1
        
        return SensorData(
            image=image,
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
        
    def get_local_transform(self) -> Transform:
        """Get local transform."""
        return self._local_transform
        
    def set_local_transform(self, transform: Transform) -> None:
        """Set local transform."""
        self._local_transform = transform
        
    def get_image_count(self) -> int:
        """Get total number of images (for image sequence mode)."""
        if self._mode == 'images':
            return len(self._image_files)
        elif self._mode == 'video' and self._cap:
            return int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        else:
            return -1  # Unknown for webcam
            
    def get_current_index(self) -> int:
        """Get current image index."""
        if self._mode == 'images':
            return self._current_index
        elif self._mode == 'video' and self._cap:
            return int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
        else:
            return -1
            
    def seek(self, index: int) -> bool:
        """
        Seek to specific frame/image index.
        
        Args:
            index: Target index
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if self._mode == 'images':
                if 0 <= index < len(self._image_files):
                    self._current_index = index
                    return True
                return False
            elif self._mode == 'video' and self._cap:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                return True
            else:
                return False
                
        except Exception as e:
            print(f"Error seeking to index {index}: {e}")
            return False
            
    def close(self) -> None:
        """Close camera and release resources."""
        if self._cap:
            self._cap.release()
            self._cap = None
            
        self._initialized = False
        self._image_files = []
        self._current_index = 0
        
    def __iter__(self) -> Iterator[SensorData]:
        """Iterator interface for processing all images."""
        return self
        
    def __next__(self) -> SensorData:
        """Get next image in iteration."""
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
        return (f"CameraRGB(mode={self._mode}, initialized={self._initialized}, "
                f"frame_rate={self._frame_rate})")
                
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = [f"CameraRGB ({self._mode} mode):"]
        
        if self._mode == 'webcam':
            lines.append(f"  Device ID: {self._device_id}")
        elif self._mode == 'video':
            lines.append(f"  Video path: {self._video_path}")
        elif self._mode == 'images':
            lines.append(f"  Image path: {self._image_path}")
            lines.append(f"  Image count: {len(self._image_files)}")
            lines.append(f"  Current index: {self._current_index}")
            
        lines.append(f"  Initialized: {self._initialized}")
        lines.append(f"  Frame rate: {self._frame_rate} Hz")
        
        if self._initialized:
            size = self._camera_model.image_size()
            lines.append(f"  Image size: {size[0]}x{size[1]}")
            
        return "\n".join(lines)
