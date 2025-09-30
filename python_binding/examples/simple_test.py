#!/usr/bin/env python3
"""
Simple test to debug the RTAB-Map initialization issue.
"""

import sys
import os

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("Successfully imported real RTAB-Map Python bindings!")
except ImportError as e:
    print(f"Failed to import RTAB-Map Python bindings: {e}")
    sys.exit(1)

def main():
    """Simple test function."""
    
    print("=== Simple RTAB-Map Test ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    
    # Step 2: Test different parameter initialization methods
    print("\nStep 2: Testing parameter initialization...")
    
    # Method 1: Empty parameters
    print("Testing with empty parameters...")
    try:
        success = slam.init(rtab.ParametersMap(), "test_empty.db")
        print(f"Empty parameters init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
    except Exception as e:
        print(f"Empty parameters init failed: {e}")
    
    # Method 2: Default parameters
    print("\nTesting with default parameters...")
    try:
        default_params = rtab.Parameters.getDefaultParameters()
        print(f"Got {len(default_params)} default parameters")
        
        # Try to modify a few key parameters
        default_params["RGBD/Enabled"] = "true"
        default_params["Rtabmap/TimeThr"] = "700"
        
        success = slam.init(default_params, "test_default.db")
        print(f"Default parameters init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
    except Exception as e:
        print(f"Default parameters init failed: {e}")
    
    # Method 3: Minimal parameters
    print("\nTesting with minimal parameters...")
    try:
        minimal_params = rtab.ParametersMap()
        minimal_params["RGBD/Enabled"] = "true"
        
        success = slam.init(minimal_params, "test_minimal.db")
        print(f"Minimal parameters init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
    except Exception as e:
        print(f"Minimal parameters init failed: {e}")
    
    print("\nTest completed!")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
