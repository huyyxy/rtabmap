#!/usr/bin/env python3
"""
Test RTAB-Map initialization using config file method.
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
    """Test function using config file initialization."""
    
    print("=== RTAB-Map Config File Test ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    
    # Step 2: Try initialization without parameters (using defaults)
    print("\nStep 2: Testing initialization without parameters...")
    try:
        success = slam.init("", "test_no_params.db")
        print(f"No parameters init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
    except Exception as e:
        print(f"No parameters init failed: {e}")
    
    # Step 3: Try initialization with empty config file
    print("\nStep 3: Testing initialization with empty config file...")
    try:
        success = slam.init("", "test_empty_config.db", False)
        print(f"Empty config init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
    except Exception as e:
        print(f"Empty config init failed: {e}")
    
    print("\nTest completed!")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
