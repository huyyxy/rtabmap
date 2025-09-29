#!/usr/bin/env python3
"""Test parameter creation."""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rtabmap_python as rtab

print("Testing Parameters.getDefaultParameters()...")
try:
    default_params = rtab.Parameters.getDefaultParameters()
    print(f"Got default parameters, type: {type(default_params)}")
    print("First few parameters:")
    count = 0
    for k, v in default_params.items():
        print(f"  {k}: {v}")
        count += 1
        if count >= 5:
            break
    
    print("Testing parameter modification...")
    # Try to modify an existing parameter
    if "RGBD/Enabled" in default_params:
        default_params["RGBD/Enabled"] = "true"
        print("Successfully modified RGBD/Enabled")
    else:
        print("RGBD/Enabled not found in default parameters")
        
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
