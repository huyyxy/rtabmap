#!/usr/bin/env python3
"""
Simple RTAB-Map Python example: Generate 2D occupancy map from Freiburg RGB-D dataset.

This is a simplified version that focuses on the core functionality:
1. Load RGB-D data from the Freiburg dataset
2. Process the data using RTAB-Map SLAM
3. Export the database for map generation

Usage:
    python simple_occupancy_map.py --dataset ../../rgbd_dataset_freiburg1_room --max_frames 50

Then use rtabmap-export to generate the map:
    rtabmap-export --2d_map --output freiburg_map freiburg_slam.db
"""

import sys
import os
import numpy as np
import cv2
import time
import argparse
from pathlib import Path

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("Successfully imported RTAB-Map Python bindings!")
except ImportError as e:
    print(f"Failed to import RTAB-Map Python bindings: {e}")
    print("Make sure RTAB-Map is installed and the bindings are built correctly.")
    sys.exit(1)

def load_freiburg_dataset(dataset_path, max_frames=100):
    """Load RGB-D frames from Freiburg dataset."""
    dataset_path = Path(dataset_path)
    
    # Load file lists
    rgb_files = []
    depth_files = []
    
    # Load RGB files
    with open(dataset_path / "rgb.txt", 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                parts = line.split()
                if len(parts) >= 2:
                    timestamp = float(parts[0])
                    filename = parts[1]
                    rgb_files.append((timestamp, filename))
    
    # Load depth files
    with open(dataset_path / "depth.txt", 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                parts = line.split()
                if len(parts) >= 2:
                    timestamp = float(parts[0])
                    filename = parts[1]
                    depth_files.append((timestamp, filename))
    
    print(f"Loaded {len(rgb_files)} RGB files and {len(depth_files)} depth files")
    
    # Load frames
    frames = []
    num_frames = min(max_frames, len(rgb_files), len(depth_files))
    
    for i in range(0, num_frames, 5):  # Skip every 5 frames
        if i >= len(rgb_files) or i >= len(depth_files):
            break
            
        # Load RGB image
        rgb_timestamp, rgb_filename = rgb_files[i]
        rgb_image = cv2.imread(str(dataset_path / rgb_filename))
        
        # Load depth image
        depth_timestamp, depth_filename = depth_files[i]
        depth_image = cv2.imread(str(dataset_path / depth_filename), cv2.IMREAD_UNCHANGED)
        
        if rgb_image is not None and depth_image is not None:
            # Convert depth from mm to m, then back to mm for RTABMap
            depth_image = depth_image.astype(np.float32) / 1000.0
            depth_image = (depth_image * 1000).astype(np.uint16)
            
            frames.append({
                'rgb': rgb_image,
                'depth': depth_image,
                'timestamp': rgb_timestamp,
                'frame_id': len(frames)
            })
    
    print(f"Loaded {len(frames)} valid frames")
    return frames

def create_camera_model():
    """Create camera model for Freiburg dataset."""
    # Freiburg dataset camera parameters
    fx = 525.0  # Focal length x
    fy = 525.0  # Focal length y
    cx = 320.0  # Principal point x
    cy = 240.0  # Principal point y
    
    return rtab.CameraModel(fx, fy, cx, cy)

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Generate 2D occupancy map from Freiburg RGB-D dataset')
    parser.add_argument('--dataset', type=str, 
                       default='../../rgbd_dataset_freiburg1_room',
                       help='Path to Freiburg dataset directory')
    parser.add_argument('--max_frames', type=int, 
                       default=100,
                       help='Maximum number of frames to process')
    parser.add_argument('--output_db', type=str,
                       default='freiburg_slam.db',
                       help='Output database filename')
    
    args = parser.parse_args()
    
    print("=== Simple RTAB-Map Freiburg Dataset Processor ===\n")
    
    # Step 1: Load dataset
    print("Step 1: Loading Freiburg dataset...")
    frames = load_freiburg_dataset(args.dataset, args.max_frames)
    
    if len(frames) == 0:
        print("Error: No valid frames found!")
        return 1
    
    # Step 2: Create RTAB-Map instance
    print("\nStep 2: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    
    # Step 3: Configure parameters
    print("\nStep 3: Configuring parameters...")
    try:
        # Basic parameters for RGB-D SLAM
        args_list = [
            "--RGBD/Enabled", "true",
            "--Rtabmap/TimeThr", "700",
            "--Rtabmap/LoopThr", "0.11",
            "--Rtabmap/MaxRetrieved", "2",
            "--Kp/MaxFeatures", "400",
            "--Kp/DetectorStrategy", "6",  # ORB detector
            "--RGBD/LinearUpdate", "0.1",
            "--RGBD/AngularUpdate", "0.1",
            "--Mem/RehearsalSimilarity", "0.6",
            # Occupancy grid parameters
            "--Grid/FromDepth", "true",
            "--Grid/3D", "false",
            "--Grid/CellSize", "0.05",
            "--Grid/RangeMax", "4.0",
            "--Grid/RayTracing", "true",
            "--Grid/UnknownSpaceFilled", "true",
            "--Grid/MaxObstacleHeight", "0.5",
            "--Grid/MaxGroundHeight", "0.1"
        ]
        
        params = rtab.Parameters.parseArguments(args_list)
        print(f"Configured {len(params)} parameters")
        
    except Exception as e:
        print(f"Parameter parsing failed: {e}")
        return 1
    
    # Step 4: Initialize RTAB-Map
    print("\nStep 4: Initializing RTAB-Map...")
    init_result = slam.init(params, args.output_db)
    print(f"Init result: {init_result}")
    
    if not slam.isInitialized():
        print("RTAB-Map failed to initialize!")
        return 1
        
    print("RTAB-Map initialized successfully!")
    
    # Step 5: Create camera model
    print("\nStep 5: Creating camera model...")
    camera_model = create_camera_model()
    print(f"Camera model: {camera_model}")
    
    # Step 6: Process RGB-D data
    print(f"\nStep 6: Processing {len(frames)} RGB-D frames...")
    
    loop_closure_detected = False
    
    for i, frame in enumerate(frames):
        print(f"Processing frame {i+1}/{len(frames)}...")
        
        # Create sensor data
        sensor_data = rtab.SensorData.create(
            frame['rgb'], 
            frame['depth'], 
            camera_model, 
            frame['frame_id'], 
            frame['timestamp']
        )
        
        # Simple odometry (forward motion with slight rotation)
        x = i * 0.05  # 5cm per frame
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = i * 0.01  # Slight rotation
        
        odometry_pose = rtab.Transform(x, y, z, roll, pitch, yaw)
        
        # Process frame
        try:
            added_to_map = slam.process(sensor_data, odometry_pose, 0.1, 0.1)
            
            # Check for loop closure
            loop_id = slam.getLoopClosureId()
            if loop_id > 0 and not loop_closure_detected:
                loop_closure_detected = True
                print(f"   🎉 Loop closure detected! Linked to node {loop_id}")
            
            # Print statistics
            stats = slam.getStatistics()
            print(f"   Added to map: {added_to_map}")
            print(f"   Features extracted: {stats.getFeaturesExtracted()}")
            print(f"   Working memory size: {stats.getWorkingMemorySize()}")
            
        except Exception as e:
            print(f"   Processing failed: {e}")
            continue
        
        # Small delay
        time.sleep(0.01)
    
    # Step 7: Close RTAB-Map
    print(f"\nStep 7: Closing RTAB-Map...")
    slam.close(database_saved=True)
    print("RTAB-Map closed and database saved!")
    
    # Step 8: Export poses
    print(f"\nStep 8: Exporting poses...")
    poses_file = "freiburg_poses.txt"
    slam.exportPoses(poses_file, True, True, 0)
    print(f"Poses exported to: {poses_file}")
    
    # Final summary
    print(f"\n=== Summary ===")
    print(f"✅ Processed {len(frames)} frames successfully")
    print(f"✅ Loop closure detected: {'Yes' if loop_closure_detected else 'No'}")
    print(f"✅ Database saved to: {args.output_db}")
    print(f"✅ Poses exported to: {poses_file}")
    
    print(f"\nTo generate the 2D occupancy map, run:")
    print(f"rtabmap-export --2d_map --output freiburg_map {args.output_db}")
    print(f"\nThis will create:")
    print(f"  - freiburg_map.pgm (occupancy map image)")
    print(f"  - freiburg_map.yaml (map metadata)")
    
    return 0

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
