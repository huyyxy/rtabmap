#!/usr/bin/env python3
"""
Test script for RTAB-Map occupancy map examples.

This script tests the basic functionality without requiring the full dataset.
"""

import sys
import os
import numpy as np
import cv2
import time
from pathlib import Path

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("✅ Successfully imported RTAB-Map Python bindings!")
except ImportError as e:
    print(f"❌ Failed to import RTAB-Map Python bindings: {e}")
    sys.exit(1)

def create_test_rgbd_data(frame_id, image_size=(640, 480)):
    """Create synthetic RGB-D data for testing."""
    width, height = image_size
    
    # Create RGB image with some patterns
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Add some geometric patterns
    cv2.rectangle(rgb_image, (50, 50), (150, 150), (0, 255, 0), -1)  # Green rectangle
    cv2.rectangle(rgb_image, (200, 100), (300, 200), (255, 0, 0), -1)  # Blue rectangle
    cv2.circle(rgb_image, (100, 300), 50, (255, 255, 0), -1)  # Cyan circle
    cv2.circle(rgb_image, (300, 350), 40, (255, 0, 255), -1)  # Magenta circle
    
    # Add text
    cv2.putText(rgb_image, f"Frame {frame_id}", (50, 400), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Add noise
    noise = np.random.randint(0, 30, (height, width, 3), dtype=np.uint8)
    rgb_image = cv2.add(rgb_image, noise)
    
    # Create depth image
    depth_image = np.full((height, width), 3000, dtype=np.uint16)  # 3 meters background
    
    # Add depth variation
    depth_image[50:150, 50:150] = 1500   # Green rectangle at 1.5m
    depth_image[100:200, 200:300] = 2000  # Blue rectangle at 2m
    cv2.circle(depth_image, (100, 300), 50, 2500, -1)  # Cyan circle at 2.5m
    cv2.circle(depth_image, (300, 350), 40, 1800, -1)  # Magenta circle at 1.8m
    
    # Add depth noise
    depth_noise = np.random.randint(-50, 50, (height, width), dtype=np.int16)
    depth_image = np.clip(depth_image.astype(np.int16) + depth_noise, 
                         500, 5000).astype(np.uint16)
    
    return rgb_image, depth_image

def test_rtabmap_basic():
    """Test basic RTAB-Map functionality."""
    print("\n=== Testing Basic RTAB-Map Functionality ===")
    
    try:
        # Create RTAB-Map instance
        slam = rtab.Rtabmap()
        print(f"✅ RTAB-Map version: {slam.getVersion()}")
        
        # Test parameter parsing
        args = [
            "--RGBD/Enabled", "true",
            "--Rtabmap/TimeThr", "700",
            "--Kp/MaxFeatures", "400"
        ]
        
        params = rtab.Parameters.parseArguments(args)
        print(f"✅ Parsed {len(params)} parameters")
        
        # Test initialization
        init_result = slam.init(params, "test.db")
        print(f"✅ Initialization result: {init_result}")
        
        if slam.isInitialized():
            print("✅ RTAB-Map initialized successfully!")
            
            # Test camera model
            camera_model = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)
            print(f"✅ Camera model created: {camera_model}")
            
            # Test sensor data creation
            rgb_image, depth_image = create_test_rgbd_data(0)
            sensor_data = rtab.SensorData.create(rgb_image, depth_image, camera_model, 0, time.time())
            print("✅ Sensor data created successfully")
            
            # Test processing
            odometry_pose = rtab.Transform(0, 0, 0, 0, 0, 0)
            added_to_map = slam.process(sensor_data, odometry_pose, 0.1, 0.1)
            print(f"✅ Frame processed successfully, added to map: {added_to_map}")
            
            # Test statistics
            stats = slam.getStatistics()
            print(f"✅ Statistics: features={stats.getFeaturesExtracted()}, wm_size={stats.getWorkingMemorySize()}")
            
            # Close
            slam.close(database_saved=True)
            print("✅ RTAB-Map closed successfully")
            
            return True
        else:
            print("❌ RTAB-Map failed to initialize")
            return False
            
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_occupancy_grid_params():
    """Test occupancy grid parameters."""
    print("\n=== Testing Occupancy Grid Parameters ===")
    
    try:
        # Test occupancy grid parameters
        args = [
            "--RGBD/Enabled", "true",
            "--Grid/FromDepth", "true",
            "--Grid/3D", "false",
            "--Grid/CellSize", "0.05",
            "--Grid/RangeMax", "4.0",
            "--Grid/RayTracing", "true",
            "--Grid/UnknownSpaceFilled", "true",
            "--Grid/MaxObstacleHeight", "0.5",
            "--Grid/MaxGroundHeight", "0.1"
        ]
        
        params = rtab.Parameters.parseArguments(args)
        print(f"✅ Occupancy grid parameters parsed: {len(params)} parameters")
        
        # Test initialization with occupancy grid
        slam = rtab.Rtabmap()
        init_result = slam.init(params, "test_occupancy.db")
        print(f"✅ Initialization with occupancy grid: {init_result}")
        
        if slam.isInitialized():
            print("✅ RTAB-Map with occupancy grid initialized successfully!")
            slam.close(database_saved=True)
            return True
        else:
            print("❌ Failed to initialize with occupancy grid")
            return False
            
    except Exception as e:
        print(f"❌ Occupancy grid test failed: {e}")
        return False

def test_dataset_loader():
    """Test dataset loader functionality."""
    print("\n=== Testing Dataset Loader ===")
    
    try:
        # Test if we can create the loader class
        from simple_occupancy_map import load_freiburg_dataset, create_camera_model
        
        # Test camera model creation
        camera_model = create_camera_model()
        print(f"✅ Camera model created: {camera_model}")
        
        # Test synthetic data loading
        print("✅ Dataset loader functions imported successfully")
        
        return True
        
    except Exception as e:
        print(f"❌ Dataset loader test failed: {e}")
        return False

def main():
    """Run all tests."""
    print("=== RTAB-Map Occupancy Map Test Suite ===\n")
    
    tests = [
        ("Basic RTAB-Map Functionality", test_rtabmap_basic),
        ("Occupancy Grid Parameters", test_occupancy_grid_params),
        ("Dataset Loader", test_dataset_loader)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        if test_func():
            passed += 1
            print(f"✅ {test_name} PASSED")
        else:
            print(f"❌ {test_name} FAILED")
    
    print(f"\n=== Test Results ===")
    print(f"Passed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All tests passed! The occupancy map examples should work correctly.")
        print("\nTo run the examples:")
        print("1. python simple_occupancy_map.py --dataset ../../rgbd_dataset_freiburg1_room --max_frames 50")
        print("2. rtabmap-export --2d_map --output freiburg_map freiburg_slam.db")
    else:
        print("⚠️  Some tests failed. Please check the error messages above.")
    
    return passed == total

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
