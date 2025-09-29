"""
Python wrapper for RTAB-Map Graph functionality.
"""

from typing import Dict, List, Tuple
from ..core.transform import Transform


class Graph:
    """
    Python wrapper for graph utilities.
    """
    
    @staticmethod
    def export_poses(filename: str, 
                    format_type: int,
                    poses: Dict[int, Transform],
                    constraints: Dict = None) -> bool:
        """Export poses to file."""
        try:
            with open(filename, 'w') as f:
                for node_id, pose in poses.items():
                    x, y, z = pose.x(), pose.y(), pose.z()
                    roll, pitch, yaw = pose.get_euler_angles()
                    f.write(f"{node_id} {x} {y} {z} {roll} {pitch} {yaw}\n")
            return True
        except:
            return False
