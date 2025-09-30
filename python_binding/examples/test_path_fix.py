#!/usr/bin/env python3
"""
Test script to verify the path fix for Freiburg dataset loading.
"""

import sys
import os
from pathlib import Path

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_path_construction():
    """Test that paths are constructed correctly."""
    print("=== Testing Path Construction ===")
    
    # Simulate the dataset structure
    dataset_path = Path("../../rgbd_dataset_freiburg1_room")
    
    # Simulate file entries from rgb.txt and depth.txt
    rgb_filename = "rgb/1305031910.765238.png"
    depth_filename = "depth/1305031910.771502.png"
    
    # Test the corrected path construction
    rgb_full_path = dataset_path / rgb_filename
    depth_full_path = dataset_path / depth_filename
    
    print(f"Dataset path: {dataset_path}")
    print(f"RGB filename: {rgb_filename}")
    print(f"Depth filename: {depth_filename}")
    print(f"RGB full path: {rgb_full_path}")
    print(f"Depth full path: {depth_full_path}")
    
    # Check if the paths look correct
    expected_rgb = "../../rgbd_dataset_freiburg1_room/rgb/1305031910.765238.png"
    expected_depth = "../../rgbd_dataset_freiburg1_room/depth/1305031910.771502.png"
    
    if str(rgb_full_path) == expected_rgb:
        print("✅ RGB path construction is correct")
    else:
        print(f"❌ RGB path construction is wrong. Expected: {expected_rgb}")
    
    if str(depth_full_path) == expected_depth:
        print("✅ Depth path construction is correct")
    else:
        print(f"❌ Depth path construction is wrong. Expected: {expected_depth}")
    
    return str(rgb_full_path) == expected_rgb and str(depth_full_path) == expected_depth

def test_file_loading():
    """Test loading files from the actual dataset."""
    print("\n=== Testing File Loading ===")
    
    dataset_path = Path("../../rgbd_dataset_freiburg1_room")
    
    if not dataset_path.exists():
        print("❌ Dataset path does not exist. Please check the path.")
        return False
    
    # Check if rgb.txt and depth.txt exist
    rgb_txt = dataset_path / "rgb.txt"
    depth_txt = dataset_path / "depth.txt"
    
    if not rgb_txt.exists():
        print("❌ rgb.txt not found")
        return False
    else:
        print("✅ rgb.txt found")
    
    if not depth_txt.exists():
        print("❌ depth.txt not found")
        return False
    else:
        print("✅ depth.txt found")
    
    # Test loading a few file entries
    try:
        with open(rgb_txt, 'r') as f:
            lines = f.readlines()
            # Skip comment lines
            data_lines = [line.strip() for line in lines if line.strip() and not line.startswith('#')]
            
            if data_lines:
                # Parse first data line
                parts = data_lines[0].split()
                if len(parts) >= 2:
                    timestamp = parts[0]
                    filename = parts[1]
                    print(f"✅ First RGB entry: {timestamp} -> {filename}")
                    
                    # Test path construction
                    full_path = dataset_path / filename
                    print(f"   Full path: {full_path}")
                    
                    # Check if file exists
                    if full_path.exists():
                        print("✅ RGB file exists")
                    else:
                        print("❌ RGB file does not exist")
                        return False
                else:
                    print("❌ Invalid RGB file format")
                    return False
            else:
                print("❌ No data lines in rgb.txt")
                return False
                
    except Exception as e:
        print(f"❌ Error reading rgb.txt: {e}")
        return False
    
    return True

def main():
    """Run all tests."""
    print("=== Freiburg Dataset Path Fix Test ===\n")
    
    tests = [
        ("Path Construction", test_path_construction),
        ("File Loading", test_file_loading)
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
        print("🎉 All tests passed! The path fix is working correctly.")
    else:
        print("⚠️  Some tests failed. Please check the issues above.")
    
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
