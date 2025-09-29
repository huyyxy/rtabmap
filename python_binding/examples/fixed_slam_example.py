#!/usr/bin/env python3
"""
Fixed RTAB-Map SLAM example that avoids the recursion issue.
"""

import sys
import os
import numpy as np
import cv2
import time

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("Successfully imported real RTAB-Map Python bindings!")
except ImportError as e:
    print(f"Failed to import RTAB-Map Python bindings: {e}")
    sys.exit(1)

def create_synthetic_rgbd_data(frame_id, image_size=(640, 480)):
    """
    Create synthetic RGB-D data for testing.
    """
    width, height = image_size
    
    # Create RGB image with some geometric patterns
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Add some colored rectangles and circles as "features"
    cv2.rectangle(rgb_image, (50, 50), (150, 150), (0, 255, 0), -1)  # Green rectangle
    cv2.rectangle(rgb_image, (200, 100), (300, 200), (255, 0, 0), -1)  # Blue rectangle
    cv2.rectangle(rgb_image, (400, 50), (500, 150), (0, 0, 255), -1)  # Red rectangle
    
    # Add some circles
    cv2.circle(rgb_image, (100, 300), 50, (255, 255, 0), -1)  # Cyan circle
    cv2.circle(rgb_image, (300, 350), 40, (255, 0, 255), -1)  # Magenta circle
    cv2.circle(rgb_image, (500, 300), 60, (0, 255, 255), -1)  # Yellow circle
    
    # Add some text as features
    cv2.putText(rgb_image, f"Frame {frame_id}", (50, 400), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Add some noise to make it more realistic
    noise = np.random.randint(0, 50, (height, width, 3), dtype=np.uint8)
    rgb_image = cv2.add(rgb_image, noise)
    
    # Create depth image (simulating objects at different distances)
    depth_image = np.full((height, width), 3000, dtype=np.uint16)  # 3 meters background
    
    # Add depth variation for the objects
    depth_image[50:150, 50:150] = 1500   # Green rectangle at 1.5m
    depth_image[100:200, 200:300] = 2000  # Blue rectangle at 2m
    depth_image[50:150, 400:500] = 1000   # Red rectangle at 1m
    
    # Circles at different depths
    cv2.circle(depth_image, (100, 300), 50, 2500, -1)  # Cyan circle at 2.5m
    cv2.circle(depth_image, (300, 350), 40, 1800, -1)  # Magenta circle at 1.8m
    cv2.circle(depth_image, (500, 300), 60, 1200, -1)  # Yellow circle at 1.2m
    
    # Add some depth noise
    depth_noise = np.random.randint(-100, 100, (height, width), dtype=np.int16)
    depth_image = np.clip(depth_image.astype(np.int16) + depth_noise, 
                         500, 5000).astype(np.uint16)
    
    return rgb_image, depth_image

def main():
    """Main function demonstrating RTAB-Map usage."""
    
    print("=== Fixed RTAB-Map Python Bindings Example ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    print(f"Initial state: {slam}")
    
    # Step 2: Configure parameters using direct string keys (avoiding recursion issue)
    print("\nStep 2: Configuring parameters...")
    params = rtab.ParametersMap()
    
    # Enable RGB-D SLAM mode - use string keys directly
    params["RGBD/Enabled"] = "true"
    params["Rtabmap/TimeThr"] = "700"
    params["Rtabmap/LoopThr"] = "0.11"
    params["Rtabmap/MaxRetrieved"] = "2"
    params["Kp/MaxFeatures"] = "400"
    params["Kp/DetectorStrategy"] = "6"  # GFTT/BRIEF
    params["RGBD/LinearUpdate"] = "0.1"    # 10cm movement threshold
    params["RGBD/AngularUpdate"] = "0.1"   # ~5.7 degree rotation threshold
    params["Mem/RehearsalSimilarity"] = "0.6"
    
    print("Configured parameters successfully")
    
    # Step 3: Initialize RTAB-Map
    print("\nStep 3: Initializing RTAB-Map...")
    database_path = "example_map.db"
    success = slam.init(params, database_path)
    
    if not success:
        print("Failed to initialize RTAB-Map!")
        return 1
        
    print(f"RTAB-Map initialized successfully with database: {database_path}")
    print(f"Initialized state: {slam}")
    
    # Step 4: Create camera model
    print("\nStep 4: Creating camera model...")
    # Typical RGB-D camera parameters (similar to Kinect/RealSense)
    camera_model = rtab.CameraModel(
        fx=525.0,  # Focal length x
        fy=525.0,  # Focal length y  
        cx=320.0,  # Principal point x
        cy=240.0   # Principal point y
    )
    print(f"Camera model: {camera_model}")
    
    # Step 5: Process synthetic RGB-D data
    print("\nStep 5: Processing synthetic RGB-D data...")
    print("Simulating camera movement and processing frames...")
    
    num_frames = 20  # Reduced number for testing
    loop_closure_detected = False
    
    for i in range(num_frames):
        print(f"\nProcessing frame {i+1}/{num_frames}...")
        
        # Generate synthetic RGB-D data
        rgb_image, depth_image = create_synthetic_rgbd_data(i)
        
        # Create sensor data
        timestamp = time.time()
        sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model, i, timestamp)
        
        # Simulate odometry (camera moving forward with slight rotation)
        x = i * 0.1          # Move forward 10cm per frame
        y = 0.0              # No lateral movement
        z = 0.0              # No vertical movement
        roll = 0.0           # No roll
        pitch = 0.0          # No pitch  
        yaw = i * 0.02       # Slight rotation per frame (~1.1 degrees)
        
        odometry_pose = rtab.Transform(x, y, z, roll, pitch, yaw)
        
        # Process the frame
        start_time = time.time()
        added_to_map = slam.process(sensor_data, odometry_pose)
        process_time = (time.time() - start_time) * 1000  # Convert to ms
        
        # Get statistics
        stats = slam.getStatistics()
        
        # Check for loop closure
        loop_id = slam.getLoopClosureId()
        if loop_id > 0 and not loop_closure_detected:
            loop_closure_detected = True
            print(f"🎉 Loop closure detected! Linked to node {loop_id}")
        
        # Print frame results
        print(f"   Added to map: {added_to_map}")
        print(f"   Process time: {stats.getProcessTime():.2f}ms")
        print(f"   Features extracted: {stats.getFeaturesExtracted()}")
        print(f"   Working memory size: {stats.getWorkingMemorySize()}")
        print(f"   Loop closure ID: {loop_id}")
        
        # Small delay to simulate real-time processing
        time.sleep(0.05)
    
    # Step 6: Get final results
    print(f"\n=== Final Results ===")
    
    # Get optimized poses
    poses = slam.getOptimizedPoses()
    print(f"Total poses in map: {len(poses)}")
    
    # Get constraints/links
    constraints = slam.getConstraints()
    print(f"Total constraints: {len(constraints)}")
    
    # Get final statistics
    final_stats = slam.getStatistics()
    
    # Memory usage
    memory_used = slam.getMemoryUsed()
    print(f"\nMemory used: {memory_used / (1024*1024):.2f} MB")
    
    # Step 7: Close RTAB-Map
    print(f"\nStep 7: Closing RTAB-Map...")
    slam.close(database_saved=True)
    print("RTAB-Map closed and database saved successfully!")
    
    # Final summary
    print(f"\n=== Summary ===")
    print(f"✅ Processed {num_frames} frames successfully")
    print(f"✅ Built map with {len(poses)} poses")
    print(f"✅ Detected {len(constraints)} constraints")
    print(f"✅ Loop closure detected: {'Yes' if loop_closure_detected else 'No'}")
    print(f"✅ Database saved to: {database_path}")
    
    if loop_closure_detected:
        print("🎉 SLAM completed successfully with loop closure!")
    else:
        print("ℹ️  SLAM completed (no loop closures detected in this short sequence)")
    
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
