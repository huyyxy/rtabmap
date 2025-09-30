#!/usr/bin/env python3
"""
Export 2D map from RTAB-Map database using Python bindings.
This script works with the correct version of RTAB-Map Python bindings.
"""

import sys
import os
import numpy as np
import cv2

# Add the parent directory to Python path for importing rtabmap_python
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rtabmap_python as rtab
    print("Successfully imported RTAB-Map Python bindings!")
    print(f"RTAB-Map version: {rtab.RTABMAP_MAJOR_VERSION}.{rtab.RTABMAP_MINOR_VERSION}.{rtab.RTABMAP_PATCH_VERSION}")
except ImportError as e:
    print(f"Failed to import RTAB-Map Python bindings: {e}")
    sys.exit(1)

def export_2d_map(database_path, output_prefix="freiburg_map"):
    """
    Export 2D occupancy grid map from RTAB-Map database.
    """
    print(f"Opening database: {database_path}")
    
    # Create Rtabmap instance
    rtabmap = rtab.Rtabmap()
    
    # Get default parameters
    params = rtab.Parameters.getDefaultParameters()
    
    # Initialize with database
    if not rtabmap.init(params, database_path):
        print("Failed to initialize RTAB-Map with database!")
        return False
    
    print("Database loaded successfully!")
    
    # Get the optimized map
    print("Retrieving optimized map...")
    
    # Get map data from database
    map_data = rtabmap.getMap()
    if map_data is None:
        print("Failed to get map data!")
        return False
    
    # Get occupancy grid
    occupancy_grid = rtabmap.getOccupancyGrid()
    if occupancy_grid is None:
        print("Failed to get occupancy grid!")
        return False
    
    print(f"Occupancy grid size: {occupancy_grid.shape}")
    
    # Convert to proper format for visualization
    # RTAB-Map uses -1 for unknown, 0 for free, 100 for occupied
    # Convert to 0-255 range for image saving
    grid_vis = np.zeros_like(occupancy_grid, dtype=np.uint8)
    
    # Unknown areas (value = -1) -> gray (128)
    grid_vis[occupancy_grid == -1] = 128
    
    # Free areas (value = 0) -> white (255)
    grid_vis[occupancy_grid == 0] = 255
    
    # Occupied areas (value = 100) -> black (0)
    grid_vis[occupancy_grid == 100] = 0
    
    # Save the map as image
    map_image_path = f"{output_prefix}.png"
    cv2.imwrite(map_image_path, grid_vis)
    print(f"2D map saved as: {map_image_path}")
    
    # Also save as PGM format (standard for occupancy grids)
    pgm_path = f"{output_prefix}.pgm"
    
    # Create PGM header
    height, width = occupancy_grid.shape
    with open(pgm_path, 'w') as f:
        f.write("P2\n")
        f.write(f"# RTAB-Map occupancy grid\n")
        f.write(f"{width} {height}\n")
        f.write("100\n")
        
        # Write data
        for row in occupancy_grid:
            for val in row:
                if val == -1:  # Unknown
                    f.write("50 ")
                elif val == 0:  # Free
                    f.write("254 ")
                else:  # Occupied
                    f.write("0 ")
            f.write("\n")
    
    print(f"PGM map saved as: {pgm_path}")
    
    # Save map metadata
    yaml_path = f"{output_prefix}.yaml"
    with open(yaml_path, 'w') as f:
        f.write("image: freiburg_map.pgm\n")
        f.write("resolution: 0.05\n")  # Default resolution
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    
    print(f"Map metadata saved as: {yaml_path}")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 export_map.py <database_path> [output_prefix]")
        print("Example: python3 export_map.py freiburg_slam.db freiburg_map")
        sys.exit(1)
    
    database_path = sys.argv[1]
    output_prefix = sys.argv[2] if len(sys.argv) > 2 else "freiburg_map"
    
    if not os.path.exists(database_path):
        print(f"Database file not found: {database_path}")
        sys.exit(1)
    
    success = export_2d_map(database_path, output_prefix)
    if success:
        print("Map export completed successfully!")
    else:
        print("Map export failed!")
        sys.exit(1)
