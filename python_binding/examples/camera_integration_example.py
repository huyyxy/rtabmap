#!/usr/bin/env python3
"""
Camera integration example for RTAB-Map Python bindings.

This example demonstrates how to integrate real cameras with RTAB-Map
for live RGB-D SLAM processing.
"""

import sys
import os
import numpy as np
import cv2
import time
import argparse

# Add the parent directory to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("Successfully imported RTAB-Map Python bindings!")
except ImportError as e:
    print(f"Failed to import RTAB-Map: {e}")
    sys.exit(1)

class RGBDCamera:
    """Simple RGB-D camera interface."""
    
    def __init__(self, rgb_device=0, depth_device=None, use_realsense=False):
        self.rgb_device = rgb_device
        self.depth_device = depth_device
        self.use_realsense = use_realsense
        self.rgb_cap = None
        self.depth_cap = None
        self.pipeline = None
        
    def init(self):
        """Initialize camera."""
        if self.use_realsense:
            return self._init_realsense()
        else:
            return self._init_opencv()
    
    def _init_realsense(self):
        """Initialize RealSense camera."""
        try:
            import pyrealsense2 as rs
            
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Configure streams
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            profile = self.pipeline.start(config)
            
            # Get camera intrinsics
            depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
            depth_intrinsics = depth_profile.get_intrinsics()
            
            self.camera_model = rtab.CameraModel(
                fx=depth_intrinsics.fx,
                fy=depth_intrinsics.fy,
                cx=depth_intrinsics.ppx,
                cy=depth_intrinsics.ppy
            )
            
            print("RealSense camera initialized successfully!")
            return True
            
        except ImportError:
            print("pyrealsense2 not available. Install with: pip install pyrealsense2")
            return False
        except Exception as e:
            print(f"Failed to initialize RealSense: {e}")
            return False
    
    def _init_opencv(self):
        """Initialize OpenCV cameras."""
        try:
            # Initialize RGB camera
            self.rgb_cap = cv2.VideoCapture(self.rgb_device)
            if not self.rgb_cap.isOpened():
                print(f"Failed to open RGB camera {self.rgb_device}")
                return False
            
            # Set camera properties
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.rgb_cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Create default camera model (you should calibrate your camera!)
            self.camera_model = rtab.CameraModel(
                fx=525.0, fy=525.0, cx=320.0, cy=240.0
            )
            
            print(f"OpenCV camera {self.rgb_device} initialized successfully!")
            print("Warning: Using default camera calibration. Please calibrate your camera for better results!")
            return True
            
        except Exception as e:
            print(f"Failed to initialize OpenCV camera: {e}")
            return False
    
    def capture_frame(self):
        """Capture RGB-D frame."""
        if self.use_realsense:
            return self._capture_realsense()
        else:
            return self._capture_opencv()
    
    def _capture_realsense(self):
        """Capture frame from RealSense."""
        try:
            import pyrealsense2 as rs
            
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None, None
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            rgb_image = np.asanyarray(color_frame.get_data())
            
            return rgb_image, depth_image
            
        except Exception as e:
            print(f"Failed to capture RealSense frame: {e}")
            return None, None
    
    def _capture_opencv(self):
        """Capture frame from OpenCV camera (RGB only, simulate depth)."""
        try:
            ret, rgb_image = self.rgb_cap.read()
            if not ret:
                return None, None
            
            # Simulate depth image (you would need a real depth camera)
            # This creates a fake depth image for demonstration
            height, width = rgb_image.shape[:2]
            depth_image = np.full((height, width), 2000, dtype=np.uint16)  # 2m depth
            
            # Add some depth variation based on image intensity
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            depth_variation = (gray.astype(np.float32) / 255.0 * 1000).astype(np.uint16)
            depth_image = depth_image + depth_variation
            
            return rgb_image, depth_image
            
        except Exception as e:
            print(f"Failed to capture OpenCV frame: {e}")
            return None, None
    
    def release(self):
        """Release camera resources."""
        if self.use_realsense and self.pipeline:
            self.pipeline.stop()
        if self.rgb_cap:
            self.rgb_cap.release()
        if self.depth_cap:
            self.depth_cap.release()

def main():
    """Main function for camera integration example."""
    
    parser = argparse.ArgumentParser(description='RTAB-Map Camera Integration Example')
    parser.add_argument('--camera', type=int, default=0, help='Camera device ID (default: 0)')
    parser.add_argument('--realsense', action='store_true', help='Use RealSense camera')
    parser.add_argument('--frames', type=int, default=100, help='Number of frames to process (default: 100)')
    parser.add_argument('--display', action='store_true', help='Display camera feed')
    args = parser.parse_args()
    
    print("=== RTAB-Map Camera Integration Example ===\n")
    
    # Initialize camera
    print("Initializing camera...")
    camera = RGBDCamera(rgb_device=args.camera, use_realsense=args.realsense)
    
    if not camera.init():
        print("Failed to initialize camera!")
        return 1
    
    # Initialize RTAB-Map
    print("Initializing RTAB-Map...")
    slam = rtab.Rtabmap()
    
    # Configure parameters for real-time operation
    params = rtab.ParametersMap()
    params[rtab.Param.kRGBDEnabled] = "true"
    params[rtab.Param.kRtabmapTimeThr] = "0"  # No time limit for real-time
    params[rtab.Param.kRtabmapLoopThr] = "0.11"
    params[rtab.Param.kKpMaxFeatures] = "400"
    params[rtab.Param.kKpDetectorStrategy] = "6"  # GFTT/BRIEF (fast)
    params[rtab.Param.kRGBDLinearUpdate] = "0.1"
    params[rtab.Param.kRGBDAngularUpdate] = "0.1"
    
    # Initialize SLAM
    database_path = "live_camera_map.db"
    if not slam.init(params, database_path):
        print("Failed to initialize RTAB-Map!")
        camera.release()
        return 1
    
    print(f"RTAB-Map initialized with database: {database_path}")
    print(f"Camera model: {camera.camera_model}")
    
    # Main processing loop
    print(f"\nStarting live SLAM processing...")
    print("Press 'q' to quit, 's' to save map, 'r' to reset")
    
    frame_count = 0
    total_process_time = 0.0
    loop_closures = 0
    
    try:
        while frame_count < args.frames:
            # Capture frame
            rgb_image, depth_image = camera.capture_frame()
            if rgb_image is None or depth_image is None:
                print("Failed to capture frame, retrying...")
                continue
            
            # Create sensor data
            timestamp = time.time()
            sensor_data = rtab.SensorData(rgb_image, depth_image, camera.camera_model, 
                                        frame_count, timestamp)
            
            # Simple odometry (identity transform - in practice you'd use visual odometry)
            odometry_pose = rtab.Transform()  # Identity transform
            
            # Process frame
            start_time = time.time()
            added_to_map = slam.process(sensor_data, odometry_pose)
            process_time = (time.time() - start_time) * 1000
            
            total_process_time += process_time
            frame_count += 1
            
            # Get statistics
            stats = slam.getStatistics()
            loop_id = slam.getLoopClosureId()
            
            if loop_id > 0:
                loop_closures += 1
                print(f"Loop closure detected! Linked to node {loop_id}")
            
            # Print progress
            if frame_count % 10 == 0:
                avg_time = total_process_time / frame_count
                print(f"Frame {frame_count}: avg_time={avg_time:.1f}ms, "
                      f"features={stats.getFeaturesExtracted()}, "
                      f"wm_size={stats.getWorkingMemorySize()}, "
                      f"loops={loop_closures}")
            
            # Display images if requested
            if args.display:
                # Create display image
                display_rgb = rgb_image.copy()
                
                # Add frame info
                cv2.putText(display_rgb, f"Frame: {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_rgb, f"Features: {stats.getFeaturesExtracted()}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_rgb, f"Process: {process_time:.1f}ms", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if loop_id > 0:
                    cv2.putText(display_rgb, f"LOOP CLOSURE: {loop_id}", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Show depth as colormap
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                # Combine images
                combined = np.hstack((display_rgb, depth_colormap))
                cv2.imshow('RTAB-Map Live SLAM (RGB | Depth)', combined)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Quit requested by user")
                    break
                elif key == ord('s'):
                    print("Saving map...")
                    slam.close(database_saved=True)
                    slam.init(params, database_path)  # Reinitialize
                elif key == ord('r'):
                    print("Resetting SLAM...")
                    slam.close(database_saved=False)
                    slam.init(params, ":memory:")  # Use in-memory database
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    # Cleanup
    print(f"\nCleaning up...")
    
    # Final statistics
    poses = slam.getOptimizedPoses()
    constraints = slam.getConstraints()
    
    print(f"\n=== Final Results ===")
    print(f"Processed frames: {frame_count}")
    print(f"Average process time: {total_process_time/max(frame_count,1):.2f}ms")
    print(f"Total poses: {len(poses)}")
    print(f"Total constraints: {len(constraints)}")
    print(f"Loop closures detected: {loop_closures}")
    print(f"Memory used: {slam.getMemoryUsed()/(1024*1024):.2f} MB")
    
    # Save results
    slam.close(database_saved=True)
    print(f"Map saved to: {database_path}")
    
    # Release camera
    camera.release()
    cv2.destroyAllWindows()
    
    print("Camera integration example completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
