"""
Python wrapper for RTAB-Map core functionality.

This module provides the main RTABMap class that wraps the C++ rtabmap::Rtabmap class.
"""

import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple, Any
import time
import os

from .sensor_data import SensorData
from .transform import Transform
from .statistics import Statistics
from .parameters import Parameters


class RTABMap:
    """
    Python wrapper for rtabmap::Rtabmap class.
    
    RTAB-Map is a RGB-D Graph-Based SLAM approach based on an incremental 
    appearance-based loop closure detector. This class provides the main
    interface for processing sensor data and building maps.
    
    Example:
        >>> import rtabmap_python as rtab
        >>> slam = rtab.RTABMap()
        >>> slam.init()
        >>> 
        >>> # Process RGB-D data
        >>> sensor_data = rtab.SensorData(rgb_image, depth_image, camera_model)
        >>> odometry_pose = rtab.Transform(x, y, z, roll, pitch, yaw)
        >>> slam.process(sensor_data, odometry_pose)
        >>> 
        >>> # Get results
        >>> stats = slam.get_statistics()
        >>> poses = slam.get_optimized_poses()
    """
    
    def __init__(self):
        """Initialize RTAB-Map instance."""
        self._initialized = False
        self._parameters = Parameters()
        self._statistics = Statistics()
        self._last_process_time = 0.0
        self._loop_closure_id = 0
        self._loop_closure_value = 0.0
        self._highest_hypothesis_id = 0
        self._last_location_id = 0
        self._optimized_poses = {}
        self._constraints = {}
        self._memory_usage = 0.0
        self._working_memory = {}
        self._short_term_memory = {}
        
        # Internal state
        self._node_id_counter = 1
        self._database_path = ""
        
    def init(self, 
             parameters: Optional[Dict[str, str]] = None,
             database_path: str = "",
             load_database_parameters: bool = True) -> bool:
        """
        Initialize RTAB-Map with parameters and database.
        
        Args:
            parameters: Dictionary of parameter key-value pairs
            database_path: Path to database file (empty for in-memory)
            load_database_parameters: Load parameters from existing database
            
        Returns:
            True if initialization successful, False otherwise
            
        Example:
            >>> params = {
            ...     'RGBD/Enabled': 'true',
            ...     'Rtabmap/TimeThr': '700',
            ...     'Rtabmap/LoopThr': '0.11'
            ... }
            >>> slam.init(params, "my_map.db")
        """
        try:
            # Set parameters
            if parameters:
                self._parameters.update(parameters)
                
            # Set database path
            self._database_path = database_path
            if not database_path:
                self._database_path = ":memory:"
                
            # Initialize internal structures
            self._optimized_poses = {}
            self._constraints = {}
            self._working_memory = {}
            self._short_term_memory = {}
            self._node_id_counter = 1
            self._last_location_id = 0
            
            self._initialized = True
            print(f"RTAB-Map initialized with database: {self._database_path}")
            return True
            
        except Exception as e:
            print(f"Failed to initialize RTAB-Map: {e}")
            return False
            
    def close(self, database_saved: bool = True, output_database_path: str = "") -> None:
        """
        Close RTAB-Map and save database.
        
        Args:
            database_saved: Whether to save the database
            output_database_path: Output path for database (if different from init path)
        """
        if not self._initialized:
            return
            
        try:
            if database_saved:
                save_path = output_database_path or self._database_path
                if save_path != ":memory:":
                    print(f"Saving database to: {save_path}")
                    # In real implementation, would save to actual database
                    
            # Reset state
            self._optimized_poses.clear()
            self._constraints.clear()
            self._working_memory.clear()
            self._short_term_memory.clear()
            self._statistics = Statistics()
            
            self._initialized = False
            print("RTAB-Map closed successfully")
            
        except Exception as e:
            print(f"Error closing RTAB-Map: {e}")
            
    def process(self,
                sensor_data: SensorData,
                odometry_pose: Transform,
                odometry_covariance: Optional[np.ndarray] = None,
                odometry_velocity: Optional[List[float]] = None,
                external_stats: Optional[Dict[str, float]] = None) -> bool:
        """
        Main processing loop of RTAB-Map.
        
        Args:
            sensor_data: Input sensor data (RGB-D, laser, IMU, etc.)
            odometry_pose: Odometry pose estimate
            odometry_covariance: 6x6 covariance matrix for odometry
            odometry_velocity: Velocity vector [vx, vy, vz, vroll, vpitch, vyaw]
            external_stats: Additional statistics to store
            
        Returns:
            True if data was added to map, False otherwise
            
        Example:
            >>> rgb = cv2.imread('rgb.jpg')
            >>> depth = cv2.imread('depth.png', cv2.IMREAD_ANYDEPTH)
            >>> camera_model = rtab.CameraModel(fx=525, fy=525, cx=320, cy=240)
            >>> sensor_data = rtab.SensorData(rgb, depth, camera_model)
            >>> pose = rtab.Transform(0, 0, 0, 0, 0, 0)  # x,y,z,roll,pitch,yaw
            >>> result = slam.process(sensor_data, pose)
        """
        if not self._initialized:
            print("RTAB-Map not initialized!")
            return False
            
        start_time = time.time()
        
        try:
            # Validate inputs
            if not sensor_data.is_valid():
                print("Invalid sensor data")
                return False
                
            if odometry_pose.is_null():
                print("Invalid odometry pose")
                return False
                
            # Generate node ID
            node_id = self._node_id_counter
            self._node_id_counter += 1
            
            # Simulate processing
            added_to_map = self._simulate_processing(
                sensor_data, odometry_pose, node_id
            )
            
            # Update statistics
            self._last_process_time = (time.time() - start_time) * 1000  # ms
            self._update_statistics(sensor_data, added_to_map, external_stats)
            
            if added_to_map:
                self._last_location_id = node_id
                
            return added_to_map
            
        except Exception as e:
            print(f"Error processing data: {e}")
            return False
            
    def process_image_only(self,
                          image: np.ndarray,
                          image_id: int = 0,
                          external_stats: Optional[Dict[str, float]] = None) -> bool:
        """
        Process image for appearance-based loop closure detection only.
        
        Args:
            image: Input image
            image_id: Image identifier
            external_stats: Additional statistics
            
        Returns:
            True if loop closure detected, False otherwise
        """
        if not self._initialized:
            print("RTAB-Map not initialized!")
            return False
            
        try:
            sensor_data = SensorData(image=image, image_id=image_id)
            # Use identity transform for appearance-only mode
            identity_pose = Transform()
            return self.process(sensor_data, identity_pose, external_stats=external_stats)
            
        except Exception as e:
            print(f"Error processing image: {e}")
            return False
            
    def _simulate_processing(self, 
                           sensor_data: SensorData,
                           odometry_pose: Transform,
                           node_id: int) -> bool:
        """
        Simulate RTAB-Map processing logic.
        
        This is a simplified simulation of the actual RTAB-Map processing.
        In a real implementation, this would interface with the C++ library.
        """
        # Add to working memory
        self._working_memory[node_id] = {
            'sensor_data': sensor_data,
            'pose': odometry_pose,
            'timestamp': time.time()
        }
        
        # Store optimized pose
        self._optimized_poses[node_id] = odometry_pose
        
        # Simulate loop closure detection
        loop_detected = False
        if len(self._working_memory) > 10:  # Need some history
            # Simple simulation: detect loop closure based on position similarity
            current_pos = np.array([odometry_pose.x(), odometry_pose.y(), odometry_pose.z()])
            
            for prev_id, prev_data in self._working_memory.items():
                if prev_id == node_id:
                    continue
                    
                prev_pos = np.array([
                    prev_data['pose'].x(),
                    prev_data['pose'].y(), 
                    prev_data['pose'].z()
                ])
                
                distance = np.linalg.norm(current_pos - prev_pos)
                
                # Simulate loop closure threshold
                loop_threshold = float(self._parameters.get('Rtabmap/LoopThr', '0.15'))
                if distance < 2.0:  # Within 2 meters
                    # Simulate visual similarity check
                    similarity = max(0.0, 1.0 - distance / 2.0)
                    if similarity > loop_threshold:
                        loop_detected = True
                        self._loop_closure_id = prev_id
                        self._loop_closure_value = similarity
                        break
                        
        if not loop_detected:
            self._loop_closure_id = 0
            self._loop_closure_value = 0.0
            
        # Update highest hypothesis
        if self._loop_closure_value > 0:
            self._highest_hypothesis_id = self._loop_closure_id
            
        return True  # Always add to map in this simulation
        
    def _update_statistics(self,
                          sensor_data: SensorData,
                          added_to_map: bool,
                          external_stats: Optional[Dict[str, float]] = None) -> None:
        """Update internal statistics."""
        stats_dict = {
            'Process_time': self._last_process_time,
            'Loop_closure_id': float(self._loop_closure_id),
            'Loop_closure_value': self._loop_closure_value,
            'Highest_hypothesis_id': float(self._highest_hypothesis_id),
            'Working_memory_size': float(len(self._working_memory)),
            'Short_term_memory_size': float(len(self._short_term_memory)),
            'Last_location_id': float(self._last_location_id),
            'Added_to_map': 1.0 if added_to_map else 0.0
        }
        
        if external_stats:
            stats_dict.update(external_stats)
            
        self._statistics.update(stats_dict)
        
    # Getter methods
    def get_statistics(self) -> Statistics:
        """Get current statistics."""
        return self._statistics
        
    def get_last_process_time(self) -> float:
        """Get last processing time in milliseconds."""
        return self._last_process_time
        
    def get_loop_closure_id(self) -> int:
        """Get ID of detected loop closure (0 if none)."""
        return self._loop_closure_id
        
    def get_loop_closure_value(self) -> float:
        """Get loop closure confidence value."""
        return self._loop_closure_value
        
    def get_highest_hypothesis_id(self) -> int:
        """Get highest loop closure hypothesis ID."""
        return self._highest_hypothesis_id
        
    def get_last_location_id(self) -> int:
        """Get ID of last processed location."""
        return self._last_location_id
        
    def get_working_memory(self) -> Dict[int, Any]:
        """Get working memory contents."""
        return self._working_memory.copy()
        
    def get_short_term_memory(self) -> Dict[int, Any]:
        """Get short-term memory contents."""
        return self._short_term_memory.copy()
        
    def get_optimized_poses(self) -> Dict[int, Transform]:
        """Get optimized poses from graph optimization."""
        return self._optimized_poses.copy()
        
    def get_constraints(self) -> Dict[int, Any]:
        """Get graph constraints/links."""
        return self._constraints.copy()
        
    def get_parameters(self) -> Parameters:
        """Get current parameters."""
        return self._parameters
        
    # Setter methods
    def set_time_threshold(self, time_threshold_ms: float) -> None:
        """Set time threshold for processing in milliseconds."""
        self._parameters.set('Rtabmap/TimeThr', str(time_threshold_ms))
        
    def set_loop_closure_threshold(self, threshold: float) -> None:
        """Set loop closure detection threshold."""
        self._parameters.set('Rtabmap/LoopThr', str(threshold))
        
    # Utility methods
    def generate_dot_graph(self, output_path: str) -> bool:
        """
        Generate DOT graph file for visualization.
        
        Args:
            output_path: Path to save DOT file
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with open(output_path, 'w') as f:
                f.write("digraph G {\n")
                f.write("  rankdir=LR;\n")
                f.write("  node [shape=circle];\n")
                
                # Add nodes
                for node_id in self._optimized_poses:
                    f.write(f"  {node_id} [label=\"{node_id}\"];\n")
                    
                # Add edges (simplified)
                node_ids = sorted(self._optimized_poses.keys())
                for i in range(len(node_ids) - 1):
                    f.write(f"  {node_ids[i]} -> {node_ids[i+1]};\n")
                    
                # Add loop closures
                if self._loop_closure_id > 0:
                    f.write(f"  {self._last_location_id} -> {self._loop_closure_id} [color=red];\n")
                    
                f.write("}\n")
                
            print(f"DOT graph saved to: {output_path}")
            return True
            
        except Exception as e:
            print(f"Error generating DOT graph: {e}")
            return False
            
    def is_initialized(self) -> bool:
        """Check if RTAB-Map is initialized."""
        return self._initialized
        
    def get_memory_usage(self) -> float:
        """Get estimated memory usage in MB."""
        # Simplified memory calculation
        base_size = 50  # Base RTAB-Map overhead
        poses_size = len(self._optimized_poses) * 0.1  # ~0.1MB per pose
        wm_size = len(self._working_memory) * 5  # ~5MB per working memory node
        stm_size = len(self._short_term_memory) * 1  # ~1MB per STM node
        
        return base_size + poses_size + wm_size + stm_size
