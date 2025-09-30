#!/usr/bin/env python3
"""
Test RTAB-Map initialization using dictionary conversion.
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
    """Test function using dictionary conversion."""
    
    print("=== RTAB-Map Dictionary Test ===\n")
    
    # Step 1: Create RTAB-Map instance
    print("Step 1: Creating RTAB-Map instance...")
    slam = rtab.Rtabmap()
    print(f"RTAB-Map version: {slam.getVersion()}")
    
    # Step 2: Test dictionary conversion
    print("\nStep 2: Testing dictionary conversion...")
    try:
        # Create a regular Python dictionary
        param_dict = {
            "RGBD/Enabled": "true",
            "Rtabmap/TimeThr": "700",
            "Rtabmap/LoopThr": "0.11",
            "Rtabmap/MaxRetrieved": "2",
            "Kp/MaxFeatures": "400",
            "Kp/DetectorStrategy": "6",
            "RGBD/LinearUpdate": "0.1",
            "RGBD/AngularUpdate": "0.1",
            "Mem/RehearsalSimilarity": "0.6"
        }
        
        print(f"Created parameter dictionary with {len(param_dict)} parameters")
        
        # Try to convert to ParametersMap
        print("Converting dictionary to ParametersMap...")
        params = rtab.ParametersMap(param_dict)
        print("Dictionary conversion successful!")
        
        # Try initialization
        print("Testing initialization...")
        success = slam.init(params, "test_dict.db")
        print(f"Dictionary init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
            
    except Exception as e:
        print(f"Dictionary conversion failed: {e}")
        import traceback
        traceback.print_exc()
    
    # Step 3: Test with parseArguments
    print("\nStep 3: Testing parseArguments...")
    try:
        args = [
            "--RGBD/Enabled", "true",
            "--Rtabmap/TimeThr", "700",
            "--Rtabmap/LoopThr", "0.11",
            "--Rtabmap/MaxRetrieved", "2",
            "--Kp/MaxFeatures", "400",
            "--Kp/DetectorStrategy", "6",
            "--RGBD/LinearUpdate", "0.1",
            "--RGBD/AngularUpdate", "0.1",
            "--Mem/RehearsalSimilarity", "0.6"
        ]
        
        print("Parsing command line arguments...")
        params = rtab.Parameters.parseArguments(args)
        print(f"Parsed {len(params)} parameters")
        
        # Try initialization
        print("Testing initialization...")
        success = slam.init(params, "test_parse.db")
        print(f"ParseArguments init: {'Success' if success else 'Failed'}")
        if success:
            slam.close()
            
    except Exception as e:
        print(f"ParseArguments failed: {e}")
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
