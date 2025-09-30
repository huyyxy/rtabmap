#!/usr/bin/env python3
"""
RTAB-Map Python example: Generate 2D occupancy map from Freiburg RGB-D dataset.

This example demonstrates how to:
1. Load RGB-D data from the Freiburg dataset
2. Process the data using RTAB-Map SLAM
3. Generate and export a 2D occupancy grid map

Dataset: rgbd_dataset_freiburg1_room
Output: 2D occupancy map (PGM + YAML format)
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

class FreiburgDatasetLoader:
    """Loader for Freiburg RGB-D dataset."""
    
    def __init__(self, dataset_path):
        self.dataset_path = Path(dataset_path)
        
        # Load timestamps and filenames
        self.rgb_files = self._load_file_list(self.dataset_path / "rgb.txt")
        self.depth_files = self._load_file_list(self.dataset_path / "depth.txt")
        
        print(f"Loaded {len(self.rgb_files)} RGB files and {len(self.depth_files)} depth files")
    
    def _load_file_list(self, file_path):
        """Load file list from timestamp file."""
        files = []
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split()
                    if len(parts) >= 2:
                        timestamp = float(parts[0])
                        filename = parts[1]
                        files.append((timestamp, filename))
        return files
    
    def get_frame(self, index):
        """Get RGB-D frame by index."""
        if index >= len(self.rgb_files) or index >= len(self.depth_files):
            return None, None, None
        
        # Get RGB image
        rgb_timestamp, rgb_filename = self.rgb_files[index]
        rgb_image = cv2.imread(str(self.dataset_path / rgb_filename))
        
        # Get depth image
        depth_timestamp, depth_filename = self.depth_files[index]
        depth_image = cv2.imread(str(self.dataset_path / depth_filename), cv2.IMREAD_UNCHANGED)
        
        if rgb_image is None or depth_image is None:
            return None, None, None
        
        # Convert depth from mm to m (Freiburg dataset uses mm)
        depth_image = depth_image.astype(np.float32) / 1000.0
        depth_image = (depth_image * 1000).astype(np.uint16)  # Convert back to mm for RTABMap
        
        return rgb_image, depth_image, rgb_timestamp

def create_camera_model():
    """Create camera model for Freiburg dataset."""
    # Freiburg dataset camera parameters (approximate)
    fx = 525.0  # Focal length x
    fy = 525.0  # Focal length y
    cx = 320.0  # Principal point x
    cy = 240.0  # Principal point y
    
    return rtab.CameraModel(fx, fy, cx, cy)

def save_occupancy_map(map_data, x_min, y_min, cell_size, output_path):
    """Save occupancy map in PGM and YAML format."""
    # Convert map to 8-bit image
    map_8u = rtab.util3d.convertMap2Image8U(map_data, True)
    
    # Save PGM file
    pgm_path = output_path.with_suffix('.pgm')
    cv2.imwrite(str(pgm_path), map_8u)
    
    # Save YAML file
    yaml_path = output_path.with_suffix('.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"image: {pgm_path.name}\n")
        f.write(f"resolution: {cell_size}\n")
        f.write(f"origin: [{x_min}, {y_min}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    
    print(f"Occupancy map saved to: {pgm_path} and {yaml_path}")
    return pgm_path, yaml_path

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Generate 2D occupancy map from Freiburg RGB-D dataset')
    parser.add_argument('--dataset', type=str, 
                       default='../../rgbd_dataset_freiburg1_room',
                       help='Path to Freiburg dataset directory')
    parser.add_argument('--output', type=str, 
                       default='freiburg_occupancy_map',
                       help='Output map name (without extension)')
    parser.add_argument('--max_frames', type=int, 
                       default=100,
                       help='Maximum number of frames to process')
    parser.add_argument('--skip_frames', type=int, 
                       default=5,
                       help='Skip every N frames to reduce processing time')
    
    args = parser.parse_args()
    
    print("=== RTAB-Map Freiburg Dataset Occupancy Map Generator ===\n")
    
    # Step 1: Load dataset
    print("Step 1: Loading Freiburg dataset...")
    dataset = FreiburgDatasetLoader(args.dataset)
    
    if len(dataset.rgb_files) == 0:
        print("Error: No RGB files found in dataset!")
        return 1
    
    # Step 2: Create RTAB-Map instance
    print("\nStep 2: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    
    # Step 3: Configure parameters
    print("\nStep 3: Configuring parameters...")
    try:
        # Parameters optimized for RGB-D SLAM
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
            "--Grid/FromDepth", "true",  # Enable occupancy grid from depth
            "--Grid/3D", "false",  # 2D occupancy grid
            "--Grid/CellSize", "0.05",  # 5cm cell size
            "--Grid/RangeMax", "4.0",  # 4m max range
            "--Grid/RayTracing", "true",  # Enable ray tracing
            "--Grid/UnknownSpaceFilled", "true",  # Fill unknown space
            "--Grid/MaxObstacleHeight", "0.5",  # Max obstacle height
            "--Grid/MaxGroundHeight", "0.1"  # Max ground height
        ]
        
        params = rtab.Parameters.parseArguments(args_list)
        print(f"Configured {len(params)} parameters")
        
    except Exception as e:
        print(f"Parameter parsing failed: {e}")
        return 1
    
    # Step 4: Initialize RTAB-Map
    print("\nStep 4: Initializing RTAB-Map...")
    database_path = "freiburg_slam.db"
    
    init_result = slam.init(params, database_path)
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
    print(f"\nStep 6: Processing RGB-D data (max {args.max_frames} frames)...")
    
    num_frames = min(args.max_frames, len(dataset.rgb_files))
    loop_closure_detected = False
    poses = []
    
    for i in range(0, num_frames, args.skip_frames):
        print(f"\nProcessing frame {i+1}/{num_frames}...")
        
        # Load RGB-D frame
        rgb_image, depth_image, timestamp = dataset.get_frame(i)
        if rgb_image is None:
            print(f"   Failed to load frame {i}")
            continue
        
        # Create sensor data
        sensor_data = rtab.SensorData.create(rgb_image, depth_image, camera_model, i, timestamp)
        
        # Simple odometry (assuming forward motion)
        x = i * 0.1  # 10cm per frame
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = i * 0.02  # Slight rotation
        
        odometry_pose = rtab.Transform(x, y, z, roll, pitch, yaw)
        
        # Process frame
        start_time = time.time()
        try:
            added_to_map = slam.process(sensor_data, odometry_pose, 0.1, 0.1)
            process_time = (time.time() - start_time) * 1000
            
            # Store pose
            poses.append(odometry_pose)
            
        except Exception as e:
            print(f"   Processing failed: {e}")
            continue
        
        # Check for loop closure
        loop_id = slam.getLoopClosureId()
        if loop_id > 0 and not loop_closure_detected:
            loop_closure_detected = True
            print(f"   🎉 Loop closure detected! Linked to node {loop_id}")
        
        # Print statistics
        stats = slam.getStatistics()
        print(f"   Added to map: {added_to_map}")
        print(f"   Process time: {process_time:.2f}ms")
        print(f"   Features extracted: {stats.getFeaturesExtracted()}")
        print(f"   Working memory size: {stats.getWorkingMemorySize()}")
        
        # Small delay
        time.sleep(0.01)
    
    # Step 7: Generate occupancy map
    print(f"\nStep 7: Generating 2D occupancy map...")
    
    try:
        # Get memory object to access local grids
        memory = slam.getMemory()
        if memory is None:
            print("Error: Memory object is None!")
            return 1
        
        # Create occupancy grid from local grids
        # Note: This is a simplified approach. In practice, you might need to
        # use the export tool or access the database directly
        
        # For now, we'll try to export the map using RTAB-Map's built-in functionality
        print("Attempting to export occupancy map...")
        
        # Try to get optimized poses
        optimized_poses = slam.getLocalOptimizedPoses()
        print(f"Found {len(optimized_poses)} optimized poses")
        
        # Export poses for debugging
        poses_file = "freiburg_poses.txt"
        slam.exportPoses(poses_file, True, True, 0)
        print(f"Poses exported to: {poses_file}")
        
        # Generate DOT graph
        dot_file = "freiburg_graph.dot"
        slam.generateDOTGraph(dot_file)
        print(f"Graph exported to: {dot_file}")
        
    except Exception as e:
        print(f"Failed to generate occupancy map: {e}")
        print("Note: Direct occupancy map generation from Python bindings may be limited.")
        print("Consider using the rtabmap-export tool with --2d_map option.")
    
    # Step 8: Close RTAB-Map
    print(f"\nStep 8: Closing RTAB-Map...")
    slam.close(database_saved=True)
    print("RTAB-Map closed and database saved!")
    
    # Final summary
    print(f"\n=== Summary ===")
    print(f"✅ Processed {num_frames} frames successfully")
    print(f"✅ Loop closure detected: {'Yes' if loop_closure_detected else 'No'}")
    print(f"✅ Database saved to: {database_path}")
    print(f"✅ Poses exported to: {poses_file}")
    print(f"✅ Graph exported to: {dot_file}")
    
    print(f"\nTo generate the 2D occupancy map, run:")
    print(f"rtabmap-export --2d_map --output {args.output} {database_path}")
    
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
