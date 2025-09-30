#!/usr/bin/env python3
"""
Minimal test to debug RTAB-Map initialization step by step.
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
    """Minimal test function."""
    
    print("=== Minimal RTAB-Map Test ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    print(f"Initial state: {slam}")
    
    # Step 2: Test with minimal parameters using parseArguments
    print("\nStep 2: Testing with minimal parameters...")
    try:
        # Use only essential parameters
        args = [
            "--RGBD/Enabled", "true"
        ]
        
        print("Parsing minimal arguments...")
        params = rtab.Parameters.parseArguments(args)
        print(f"Parsed {len(params)} parameters")
        
        # Print some parameter details
        for key, value in list(params.items())[:5]:  # Show first 5 parameters
            print(f"  {key}: {value}")
        
        # Try initialization
        print("\nTesting initialization...")
        success = slam.init(params, "test_minimal.db")
        print(f"Minimal init: {'Success' if success else 'Failed'}")
        
        if success:
            print("RTAB-Map initialized successfully!")
            slam.close()
        else:
            print("Initialization failed. Let's try to get more info...")
            
            # Try to get error information
            try:
                print(f"RTAB-Map state: {slam}")
                print(f"Is initialized: {slam.isInitialized()}")
            except Exception as e:
                print(f"Could not get state info: {e}")
            
    except Exception as e:
        print(f"Minimal test failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nTest completed!")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
