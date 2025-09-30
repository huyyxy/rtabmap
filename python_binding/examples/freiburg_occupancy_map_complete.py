#!/usr/bin/env python3
"""
Complete RTAB-Map Python example: Generate 2D occupancy map from Freiburg RGB-D dataset.

This example demonstrates how to:
1. Load RGB-D data from the Freiburg dataset
2. Process the data using RTAB-Map SLAM
3. Use RTAB-Map's export functionality to generate 2D occupancy grid map
4. Load and visualize the generated map

Dataset: rgbd_dataset_freiburg1_room
Output: 2D occupancy map (PGM + YAML format)
"""

import sys
import os
import numpy as np
import cv2
import time
import argparse
import subprocess
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

def visualize_occupancy_map(pgm_path, yaml_path):
    """Visualize the generated occupancy map."""
    try:
        # Load the map
        map_image = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            print(f"Failed to load map image: {pgm_path}")
            return
        
        # Load YAML info
        yaml_info = {}
        with open(yaml_path, 'r') as f:
            for line in f:
                if ':' in line:
                    key, value = line.strip().split(':', 1)
                    yaml_info[key.strip()] = value.strip()
        
        print(f"\nMap Information:")
        print(f"  Resolution: {yaml_info.get('resolution', 'N/A')}")
        print(f"  Origin: {yaml_info.get('origin', 'N/A')}")
        print(f"  Image size: {map_image.shape}")
        
        # Create a colored version for better visualization
        colored_map = cv2.applyColorMap(map_image, cv2.COLORMAP_JET)
        
        # Show the map
        cv2.imshow("2D Occupancy Map", colored_map)
        print("Press any key to close the map visualization...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Failed to visualize map: {e}")

def run_rtabmap_export(database_path, output_name, output_dir):
    """Run rtabmap-export to generate 2D occupancy map."""
    try:
        # Check if rtabmap-export exists
        result = subprocess.run(['which', 'rtabmap-export'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print("rtabmap-export not found in PATH. Trying to find it...")
            # Try common locations
            possible_paths = [
                '/usr/local/bin/rtabmap-export',
                '/usr/bin/rtabmap-export',
                'rtabmap-export'
            ]
            
            export_cmd = None
            for path in possible_paths:
                result = subprocess.run(['which', path], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    export_cmd = path
                    break
            
            if export_cmd is None:
                print("Error: rtabmap-export not found!")
                print("Please install RTAB-Map tools or add them to PATH")
                return False
        else:
            export_cmd = 'rtabmap-export'
        
        # Run export command
        cmd = [
            export_cmd,
            '--2d_map',
            '--output', output_name,
            '--output_dir', str(output_dir),
            str(database_path)
        ]
        
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            print("Export completed successfully!")
            print("STDOUT:", result.stdout)
            return True
        else:
            print("Export failed!")
            print("STDERR:", result.stderr)
            return False
            
    except Exception as e:
        print(f"Failed to run rtabmap-export: {e}")
        return False

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Generate 2D occupancy map from Freiburg RGB-D dataset')
    parser.add_argument('--dataset', type=str, 
                       default='../../rgbd_dataset_freiburg1_room',
                       help='Path to Freiburg dataset directory')
    parser.add_argument('--output', type=str, 
                       default='freiburg_occupancy_map',
                       help='Output map name (without extension)')
    parser.add_argument('--output_dir', type=str, 
                       default='.',
                       help='Output directory for generated files')
    parser.add_argument('--max_frames', type=int, 
                       default=200,
                       help='Maximum number of frames to process')
    parser.add_argument('--skip_frames', type=int, 
                       default=3,
                       help='Skip every N frames to reduce processing time')
    parser.add_argument('--visualize', action='store_true',
                       help='Visualize the generated occupancy map')
    
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
        # Parameters optimized for RGB-D SLAM with occupancy grid
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
            "--Grid/MaxGroundHeight", "0.1",  # Max ground height
            "--Grid/GlobalMinSize", "10.0",  # Minimum map size
            "--Grid/GlobalEroded", "true",  # Erode obstacles
            "--Grid/GlobalFootprintRadius", "0.2"  # Robot footprint
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
    processed_frames = 0
    
    for i in range(0, num_frames, args.skip_frames):
        print(f"\nProcessing frame {i+1}/{num_frames}...")
        
        # Load RGB-D frame
        rgb_image, depth_image, timestamp = dataset.get_frame(i)
        if rgb_image is None:
            print(f"   Failed to load frame {i}")
            continue
        
        # Create sensor data
        sensor_data = rtab.SensorData.create(rgb_image, depth_image, camera_model, i, timestamp)
        
        # Simple odometry (assuming forward motion with some rotation)
        x = i * 0.05  # 5cm per frame
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = i * 0.01  # Slight rotation
        
        odometry_pose = rtab.Transform(x, y, z, roll, pitch, yaw)
        
        # Process frame
        start_time = time.time()
        try:
            added_to_map = slam.process(sensor_data, odometry_pose, 0.1, 0.1)
            process_time = (time.time() - start_time) * 1000
            
            processed_frames += 1
            
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
    
    # Step 7: Close RTAB-Map
    print(f"\nStep 7: Closing RTAB-Map...")
    slam.close(database_saved=True)
    print("RTAB-Map closed and database saved!")
    
    # Step 8: Generate occupancy map using rtabmap-export
    print(f"\nStep 8: Generating 2D occupancy map...")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    
    success = run_rtabmap_export(database_path, args.output, output_dir)
    
    if success:
        # Check if map files were created
        pgm_path = output_dir / f"{args.output}.pgm"
        yaml_path = output_dir / f"{args.output}.yaml"
        
        if pgm_path.exists() and yaml_path.exists():
            print(f"✅ 2D occupancy map generated successfully!")
            print(f"   PGM file: {pgm_path}")
            print(f"   YAML file: {yaml_path}")
            
            if args.visualize:
                print("\nVisualizing occupancy map...")
                visualize_occupancy_map(pgm_path, yaml_path)
        else:
            print("❌ Map files not found after export")
            return 1
    else:
        print("❌ Failed to generate occupancy map")
        return 1
    
    # Final summary
    print(f"\n=== Summary ===")
    print(f"✅ Processed {processed_frames} frames successfully")
    print(f"✅ Loop closure detected: {'Yes' if loop_closure_detected else 'No'}")
    print(f"✅ Database saved to: {database_path}")
    print(f"✅ 2D occupancy map generated: {args.output}.pgm")
    
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
