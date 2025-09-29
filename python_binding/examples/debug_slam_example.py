#!/usr/bin/env python3
"""
Debug version of RTAB-Map SLAM example to identify the recursion issue.
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

def main():
    """Main function for debugging."""
    
    print("=== Debug RTAB-Map Python Bindings ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    print(f"Initial state: {slam}")
    
    # Step 2: Test parameter creation
    print("\nStep 2: Creating ParametersMap...")
    params = rtab.ParametersMap()
    print(f"Empty parameters map created: {len(params)} parameters")
    
    # Step 3: Test parameter constants access
    print("\nStep 3: Testing parameter constants...")
    try:
        print(f"kRGBDEnabled = {rtab.Param.kRGBDEnabled}")
        print(f"kRtabmapTimeThr = {rtab.Param.kRtabmapTimeThr}")
        print(f"kKpMaxFeatures = {rtab.Param.kKpMaxFeatures}")
    except Exception as e:
        print(f"Error accessing parameter constants: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Step 4: Test parameter assignment
    print("\nStep 4: Testing parameter assignment...")
    try:
        # Try direct string assignment
        params["RGBD/Enabled"] = "true"
        print("Direct string assignment works")
        
        # Try using constant
        params[rtab.Param.kRGBDEnabled] = "true"
        print("Constant assignment works")
        
        print(f"Parameters after assignment: {len(params)} parameters")
        for key, value in params.items():
            print(f"  {key}: {value}")
            
    except Exception as e:
        print(f"Error in parameter assignment: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print("\nDebug test completed successfully!")
    return 0

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
