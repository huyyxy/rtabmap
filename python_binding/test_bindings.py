#!/usr/bin/env python3
"""
Test script for RTAB-Map Python bindings.

This script tests the basic functionality of the Python bindings
to ensure they work correctly.
"""

import sys
import traceback

def test_import():
    """Test importing the module."""
    print("Testing import...")
    try:
        import rtabmap_python as rtab
        print("✅ Successfully imported rtabmap_python")
        print(f"   Module version: {getattr(rtab, '__version__', 'unknown')}")
        return rtab
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        print("\nPossible solutions:")
        print("1. Make sure RTAB-Map C++ library is installed")
        print("2. Build the Python bindings: ./build.sh")
        print("3. Install the package: pip install .")
        return None

def test_rtabmap_class(rtab):
    """Test Rtabmap class."""
    print("\nTesting Rtabmap class...")
    try:
        slam = rtab.Rtabmap()
        print("✅ Rtabmap instance created")
        
        # Test initialization status
        is_init = slam.isInitialized()
        print(f"   Initial state: initialized={is_init}")
        
        # Test string representation
        print(f"   String repr: {slam}")
        
        return True
    except Exception as e:
        print(f"❌ Rtabmap test failed: {e}")
        return False

def test_transform_class(rtab):
    """Test Transform class."""
    print("\nTesting Transform class...")
    try:
        # Test default constructor
        t1 = rtab.Transform()
        print("✅ Default Transform created")
        
        # Test parameterized constructor
        t2 = rtab.Transform(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        print("✅ Parameterized Transform created")
        
        # Test operations
        t3 = t1 * t2
        print("✅ Transform multiplication works")
        
        # Test accessors
        x, y, z = t2.x(), t2.y(), t2.z()
        print(f"   Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        
        # Test inverse
        t_inv = t2.inverse()
        print("✅ Transform inverse works")
        
        return True
    except Exception as e:
        print(f"❌ Transform test failed: {e}")
        return False

def test_camera_model_class(rtab):
    """Test CameraModel class."""
    print("\nTesting CameraModel class...")
    try:
        # Test constructor
        camera = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)
        print("✅ CameraModel created")
        
        # Test accessors
        fx, fy = camera.fx(), camera.fy()
        cx, cy = camera.cx(), camera.cy()
        print(f"   Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
        
        # Test validity
        is_valid = camera.isValid()
        print(f"   Valid: {is_valid}")
        
        return True
    except Exception as e:
        print(f"❌ CameraModel test failed: {e}")
        return False

def test_parameters_class(rtab):
    """Test Parameters class."""
    print("\nTesting Parameters class...")
    try:
        # Test ParametersMap
        params = rtab.ParametersMap()
        print("✅ ParametersMap created")
        
        # Test parameter setting
        params[rtab.Param.kRGBDEnabled] = "true"
        params[rtab.Param.kRtabmapTimeThr] = "700"
        print("✅ Parameters set successfully")
        
        # Test parameter access
        enabled = params[rtab.Param.kRGBDEnabled]
        print(f"   RGBD Enabled: {enabled}")
        
        # Test default parameters
        defaults = rtab.Parameters.getDefaultParameters()
        print(f"✅ Got {len(defaults)} default parameters")
        
        return True
    except Exception as e:
        print(f"❌ Parameters test failed: {e}")
        return False

def test_sensor_data_class(rtab):
    """Test SensorData class."""
    print("\nTesting SensorData class...")
    try:
        import numpy as np
        
        # Create dummy RGB-D data
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)
        
        # Create camera model
        camera = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)
        
        # Create sensor data using factory method
        sensor_data = rtab.SensorData.create(rgb, depth, camera, 1, 12345.0)
        print("✅ SensorData created from numpy arrays")
        
        # Test accessors
        data_id = sensor_data.id()
        stamp = sensor_data.stamp()
        print(f"   ID: {data_id}, Stamp: {stamp}")
        
        # Test validity checks
        has_image = sensor_data.hasImage()
        has_depth = sensor_data.hasDepth()
        is_valid = sensor_data.isValid()
        print(f"   Has image: {has_image}, Has depth: {has_depth}, Valid: {is_valid}")
        
        return True
    except Exception as e:
        print(f"❌ SensorData test failed: {e}")
        return False

def test_integration(rtab):
    """Test integration between classes."""
    print("\nTesting class integration...")
    try:
        import numpy as np
        
        # Create RTAB-Map instance
        slam = rtab.Rtabmap()
        
        # Create parameters
        params = rtab.ParametersMap()
        params[rtab.Param.kRGBDEnabled] = "true"
        params[rtab.Param.kRtabmapTimeThr] = "0"  # No time limit
        
        # Initialize SLAM
        success = slam.init(params, ":memory:")  # In-memory database
        if not success:
            print("❌ Failed to initialize SLAM")
            return False
        
        print("✅ SLAM initialized successfully")
        
        # Create test data
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.random.randint(500, 5000, (480, 640), dtype=np.uint16)
        camera = rtab.CameraModel(525.0, 525.0, 320.0, 240.0)
        sensor_data = rtab.SensorData.create(rgb, depth, camera, 1, 0.0)
        
        # Create odometry pose
        odom_pose = rtab.Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
        # Process frame
        result = slam.process(sensor_data, odom_pose)
        print(f"✅ Frame processed successfully: {result}")
        
        # Get statistics
        stats = slam.getStatistics()
        print(f"✅ Statistics retrieved: process_time={stats.getProcessTime():.2f}ms")
        
        # Close SLAM
        slam.close(database_saved=False)
        print("✅ SLAM closed successfully")
        
        return True
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        traceback.print_exc()
        return False

def main():
    """Main test function."""
    print("=== RTAB-Map Python Bindings Test Suite ===\n")
    
    # Test import
    rtab = test_import()
    if not rtab:
        return 1
    
    # Run individual tests
    tests = [
        ("Rtabmap", test_rtabmap_class),
        ("Transform", test_transform_class),
        ("CameraModel", test_camera_model_class),
        ("Parameters", test_parameters_class),
        ("SensorData", test_sensor_data_class),
        ("Integration", test_integration),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            if test_func(rtab):
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"❌ {test_name} test crashed: {e}")
            failed += 1
    
    # Print summary
    print(f"\n=== Test Summary ===")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Total:  {passed + failed}")
    
    if failed == 0:
        print("\n🎉 All tests passed! RTAB-Map Python bindings are working correctly.")
        return 0
    else:
        print(f"\n❌ {failed} test(s) failed. Please check the installation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
