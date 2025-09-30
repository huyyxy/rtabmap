#!/usr/bin/env python3
"""
Simple script to export 2D map from RTAB-Map database using direct SQLite access.
This avoids version compatibility issues.
"""

import sys
import os
import sqlite3
import numpy as np
import cv2
import struct

def export_map_from_db(database_path, output_prefix="freiburg_map"):
    """
    Export 2D occupancy grid map directly from SQLite database.
    """
    print(f"Opening database: {database_path}")
    
    try:
        conn = sqlite3.connect(database_path)
        cursor = conn.cursor()
        
        # Get map data from Admin table
        cursor.execute("SELECT opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution FROM Admin")
        result = cursor.fetchone()
        
        if not result or result[0] is None:
            print("No map data found in database!")
            return False
        
        map_blob, x_min, y_min, resolution = result
        print(f"Map bounds: x_min={x_min}, y_min={y_min}, resolution={resolution}")
        
        # Decompress the map data
        # RTAB-Map stores compressed data, we need to decompress it
        try:
            import zlib
            decompressed = zlib.decompress(map_blob)
            
            # Convert to numpy array
            # The data is stored as CV_8SC1 (8-bit signed char)
            map_data = np.frombuffer(decompressed, dtype=np.int8)
            
            # We need to determine the dimensions
            # This is tricky without knowing the exact format
            # Let's try to estimate based on common sizes
            possible_sizes = [
                (int(np.sqrt(len(map_data))), int(np.sqrt(len(map_data)))),
                (len(map_data), 1),
                (1, len(map_data))
            ]
            
            print(f"Decompressed data size: {len(map_data)} bytes")
            
            # Try to find a reasonable 2D shape
            best_shape = None
            for h, w in possible_sizes:
                if h * w == len(map_data) and h > 10 and w > 10:
                    best_shape = (h, w)
                    break
            
            if best_shape is None:
                # Try common aspect ratios
                for aspect_ratio in [1.0, 1.33, 1.5, 2.0]:
                    h = int(np.sqrt(len(map_data) / aspect_ratio))
                    w = int(len(map_data) / h)
                    if h * w == len(map_data) and h > 10 and w > 10:
                        best_shape = (h, w)
                        break
            
            if best_shape is None:
                print("Could not determine map dimensions!")
                return False
            
            print(f"Using map dimensions: {best_shape}")
            map_2d = map_data.reshape(best_shape)
            
        except Exception as e:
            print(f"Failed to decompress map data: {e}")
            # Try to read as raw data
            map_data = np.frombuffer(map_blob, dtype=np.uint8)
            print(f"Raw data size: {len(map_data)} bytes")
            
            # Try common dimensions
            for h in range(100, 2000, 50):
                if len(map_data) % h == 0:
                    w = len(map_data) // h
                    if w > 10:
                        print(f"Trying dimensions: {h}x{w}")
                        try:
                            map_2d = map_data.reshape(h, w)
                            break
                        except:
                            continue
            else:
                print("Could not reshape map data!")
                return False
        
        print(f"Map shape: {map_2d.shape}")
        print(f"Map value range: {map_2d.min()} to {map_2d.max()}")
        
        # Convert to visualization format
        # RTAB-Map uses: -1=unknown, 0=free, 100=occupied
        grid_vis = np.zeros_like(map_2d, dtype=np.uint8)
        
        # Unknown areas (value = -1) -> gray (128)
        grid_vis[map_2d == -1] = 128
        
        # Free areas (value = 0) -> white (255)
        grid_vis[map_2d == 0] = 255
        
        # Occupied areas (value = 100) -> black (0)
        grid_vis[map_2d == 100] = 0
        
        # Save as PNG
        png_path = f"{output_prefix}.png"
        cv2.imwrite(png_path, grid_vis)
        print(f"2D map saved as: {png_path}")
        
        # Save as PGM
        pgm_path = f"{output_prefix}.pgm"
        with open(pgm_path, 'w') as f:
            f.write("P2\n")
            f.write(f"# RTAB-Map occupancy grid\n")
            f.write(f"{map_2d.shape[1]} {map_2d.shape[0]}\n")
            f.write("100\n")
            
            for row in map_2d:
                for val in row:
                    if val == -1:  # Unknown
                        f.write("50 ")
                    elif val == 0:  # Free
                        f.write("254 ")
                    else:  # Occupied
                        f.write("0 ")
                f.write("\n")
        
        print(f"PGM map saved as: {pgm_path}")
        
        # Save metadata
        yaml_path = f"{output_prefix}.yaml"
        with open(yaml_path, 'w') as f:
            f.write("image: freiburg_map.pgm\n")
            f.write(f"resolution: {resolution if resolution else 0.05}\n")
            f.write(f"origin: [{x_min if x_min else 0.0}, {y_min if y_min else 0.0}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
        
        print(f"Map metadata saved as: {yaml_path}")
        
        conn.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 simple_export.py <database_path> [output_prefix]")
        print("Example: python3 simple_export.py freiburg_slam.db freiburg_map")
        sys.exit(1)
    
    database_path = sys.argv[1]
    output_prefix = sys.argv[2] if len(sys.argv) > 2 else "freiburg_map"
    
    if not os.path.exists(database_path):
        print(f"Database file not found: {database_path}")
        sys.exit(1)
    
    success = export_map_from_db(database_path, output_prefix)
    if success:
        print("Map export completed successfully!")
    else:
        print("Map export failed!")
        sys.exit(1)
