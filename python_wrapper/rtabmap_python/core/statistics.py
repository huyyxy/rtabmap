"""
Python wrapper for RTAB-Map Statistics class.

This module provides statistics collection and analysis for RTAB-Map performance.
"""

import numpy as np
from typing import Dict, List, Optional, Any
import time


class Statistics:
    """
    Python wrapper for rtabmap::Statistics class.
    
    Collects and manages performance statistics and metrics from RTAB-Map processing,
    including timing information, memory usage, loop closure statistics, etc.
    
    Example:
        >>> stats = Statistics()
        >>> stats.add_statistic("Process_time", 150.5)
        >>> stats.add_statistic("Loop_closure_id", 42)
        >>> print(f"Process time: {stats.get_statistic('Process_time')} ms")
    """
    
    def __init__(self):
        """Initialize empty statistics."""
        self._statistics = {}
        self._timestamp = time.time()
        
        # Initialize common statistics with default values
        self._init_default_statistics()
        
    def _init_default_statistics(self) -> None:
        """Initialize common RTAB-Map statistics with default values."""
        # Timing statistics (in milliseconds)
        timing_stats = [
            "Process_time", "Memory_update", "Neighbor_link_refining",
            "Proximity_by_time", "Proximity_by_space_search", "Proximity_by_space_visual",
            "Proximity_by_space", "Cleaning_neighbors", "Reactivation",
            "Add_loop_closure_link", "Map_optimization", "Likelihood_computation",
            "Posterior_computation", "Hypotheses_creation", "Hypotheses_validation",
            "Statistics_creation"
        ]
        
        # Loop closure statistics
        loop_stats = [
            "Loop_closure_id", "Loop_closure_value", "Highest_hypothesis_id",
            "Virtual_place_hypothesis", "Rejected_hypothesis"
        ]
        
        # Memory statistics
        memory_stats = [
            "Working_memory_size", "Short_term_memory_size", "Database_memory_used",
            "Signatures_removed", "Immunized_globally", "Immunized_locally",
            "Immunized_locally_max", "Signatures_retrieved", "Images_buffered",
            "Local_graph_size", "Odom_cache_poses", "Odom_cache_links",
            "RAM_usage", "RAM_estimated"
        ]
        
        # Odometry and movement statistics
        odometry_stats = [
            "Small_movement", "Fast_movement", "Distance_travelled",
            "Odometry_variance_lin", "Odometry_variance_ang",
            "Closest_node_distance", "Closest_node_angle"
        ]
        
        # Feature and matching statistics
        feature_stats = [
            "Features_extracted", "Features_matched", "Inliers", "Outliers",
            "ICP_translation", "ICP_rotation", "ICP_complexity", "Variance"
        ]
        
        # Initialize all with default values
        all_stats = timing_stats + loop_stats + memory_stats + odometry_stats + feature_stats
        for stat in all_stats:
            self._statistics[stat] = 0.0
            
    def add_statistic(self, name: str, value: float) -> None:
        """
        Add or update a statistic.
        
        Args:
            name: Statistic name
            value: Statistic value
        """
        self._statistics[name] = float(value)
        
    def get_statistic(self, name: str, default_value: float = 0.0) -> float:
        """
        Get a statistic value.
        
        Args:
            name: Statistic name
            default_value: Default value if statistic doesn't exist
            
        Returns:
            Statistic value
        """
        return self._statistics.get(name, default_value)
        
    def has_statistic(self, name: str) -> bool:
        """Check if a statistic exists."""
        return name in self._statistics
        
    def remove_statistic(self, name: str) -> bool:
        """
        Remove a statistic.
        
        Args:
            name: Statistic name
            
        Returns:
            True if statistic was removed, False if it didn't exist
        """
        if name in self._statistics:
            del self._statistics[name]
            return True
        return False
        
    def clear(self) -> None:
        """Clear all statistics."""
        self._statistics.clear()
        self._timestamp = time.time()
        self._init_default_statistics()
        
    def update(self, statistics_dict: Dict[str, float]) -> None:
        """
        Update multiple statistics at once.
        
        Args:
            statistics_dict: Dictionary of statistic name-value pairs
        """
        for name, value in statistics_dict.items():
            self.add_statistic(name, value)
            
    def get_all_statistics(self) -> Dict[str, float]:
        """Get all statistics as a dictionary."""
        return self._statistics.copy()
        
    def get_statistic_names(self) -> List[str]:
        """Get list of all statistic names."""
        return list(self._statistics.keys())
        
    def get_timestamp(self) -> float:
        """Get timestamp when statistics were created."""
        return self._timestamp
        
    def set_timestamp(self, timestamp: float) -> None:
        """Set timestamp."""
        self._timestamp = timestamp
        
    # Convenience methods for common statistics
    def get_process_time(self) -> float:
        """Get total processing time in milliseconds."""
        return self.get_statistic("Process_time")
        
    def get_loop_closure_id(self) -> int:
        """Get loop closure ID (0 if no loop closure)."""
        return int(self.get_statistic("Loop_closure_id"))
        
    def get_loop_closure_value(self) -> float:
        """Get loop closure confidence value."""
        return self.get_statistic("Loop_closure_value")
        
    def get_working_memory_size(self) -> int:
        """Get working memory size."""
        return int(self.get_statistic("Working_memory_size"))
        
    def get_short_term_memory_size(self) -> int:
        """Get short-term memory size."""
        return int(self.get_statistic("Short_term_memory_size"))
        
    def get_ram_usage(self) -> float:
        """Get RAM usage in MB."""
        return self.get_statistic("RAM_usage")
        
    def get_distance_travelled(self) -> float:
        """Get total distance travelled in meters."""
        return self.get_statistic("Distance_travelled")
        
    def get_features_extracted(self) -> int:
        """Get number of features extracted."""
        return int(self.get_statistic("Features_extracted"))
        
    def get_inliers(self) -> int:
        """Get number of inlier matches."""
        return int(self.get_statistic("Inliers"))
        
    # Analysis methods
    def get_timing_statistics(self) -> Dict[str, float]:
        """Get all timing-related statistics."""
        timing_keys = [key for key in self._statistics.keys() 
                      if any(timing_word in key.lower() for timing_word in 
                            ['time', 'timing', 'ms'])]
        return {key: self._statistics[key] for key in timing_keys}
        
    def get_memory_statistics(self) -> Dict[str, float]:
        """Get all memory-related statistics."""
        memory_keys = [key for key in self._statistics.keys()
                      if any(memory_word in key.lower() for memory_word in
                            ['memory', 'ram', 'size', 'usage'])]
        return {key: self._statistics[key] for key in memory_keys}
        
    def get_loop_closure_statistics(self) -> Dict[str, float]:
        """Get all loop closure related statistics."""
        loop_keys = [key for key in self._statistics.keys()
                    if any(loop_word in key.lower() for loop_word in
                          ['loop', 'closure', 'hypothesis', 'virtual'])]
        return {key: self._statistics[key] for key in loop_keys}
        
    def is_loop_closure_detected(self) -> bool:
        """Check if a loop closure was detected."""
        return self.get_loop_closure_id() > 0
        
    def get_total_processing_time(self) -> float:
        """Get sum of all timing statistics."""
        timing_stats = self.get_timing_statistics()
        return sum(timing_stats.values())
        
    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get a summary of key performance metrics.
        
        Returns:
            Dictionary with performance summary
        """
        return {
            'timestamp': self._timestamp,
            'process_time_ms': self.get_process_time(),
            'total_time_ms': self.get_total_processing_time(),
            'loop_closure_detected': self.is_loop_closure_detected(),
            'loop_closure_id': self.get_loop_closure_id(),
            'loop_closure_confidence': self.get_loop_closure_value(),
            'working_memory_size': self.get_working_memory_size(),
            'stm_size': self.get_short_term_memory_size(),
            'ram_usage_mb': self.get_ram_usage(),
            'distance_travelled_m': self.get_distance_travelled(),
            'features_extracted': self.get_features_extracted(),
            'inliers': self.get_inliers()
        }
        
    # Export/Import methods
    def to_dict(self) -> Dict[str, Any]:
        """Export statistics to dictionary."""
        return {
            'timestamp': self._timestamp,
            'statistics': self._statistics.copy()
        }
        
    def from_dict(self, data: Dict[str, Any]) -> None:
        """Import statistics from dictionary."""
        self._timestamp = data.get('timestamp', time.time())
        self._statistics = data.get('statistics', {}).copy()
        
    def save_to_file(self, filename: str) -> None:
        """
        Save statistics to file.
        
        Args:
            filename: Output filename
        """
        import json
        
        data = self.to_dict()
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
    def load_from_file(self, filename: str) -> None:
        """
        Load statistics from file.
        
        Args:
            filename: Input filename
        """
        import json
        
        with open(filename, 'r') as f:
            data = json.load(f)
            
        self.from_dict(data)
        
    # String representations
    def __repr__(self) -> str:
        """String representation."""
        return f"Statistics(count={len(self._statistics)}, timestamp={self._timestamp:.3f})"
        
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = ["RTAB-Map Statistics:"]
        lines.append(f"  Timestamp: {self._timestamp:.3f}")
        lines.append(f"  Total statistics: {len(self._statistics)}")
        
        # Show performance summary
        summary = self.get_performance_summary()
        lines.append("  Key metrics:")
        lines.append(f"    Process time: {summary['process_time_ms']:.1f} ms")
        lines.append(f"    Loop closure: {'Yes' if summary['loop_closure_detected'] else 'No'}")
        if summary['loop_closure_detected']:
            lines.append(f"      ID: {summary['loop_closure_id']}")
            lines.append(f"      Confidence: {summary['loop_closure_confidence']:.3f}")
        lines.append(f"    Working memory: {summary['working_memory_size']} nodes")
        lines.append(f"    RAM usage: {summary['ram_usage_mb']:.1f} MB")
        lines.append(f"    Distance travelled: {summary['distance_travelled_m']:.2f} m")
        
        return "\n".join(lines)
        
    def print_all_statistics(self) -> None:
        """Print all statistics in a formatted way."""
        print("All RTAB-Map Statistics:")
        print(f"Timestamp: {self._timestamp:.3f}")
        print("-" * 50)
        
        # Group statistics by category
        categories = {
            'Timing': [k for k in self._statistics.keys() if 'time' in k.lower() or 'timing' in k.lower()],
            'Memory': [k for k in self._statistics.keys() if any(w in k.lower() for w in ['memory', 'ram', 'size'])],
            'Loop Closure': [k for k in self._statistics.keys() if any(w in k.lower() for w in ['loop', 'closure', 'hypothesis'])],
            'Features': [k for k in self._statistics.keys() if any(w in k.lower() for w in ['feature', 'inlier', 'outlier', 'match'])],
            'Movement': [k for k in self._statistics.keys() if any(w in k.lower() for w in ['movement', 'distance', 'odometry'])],
            'Other': []
        }
        
        # Add uncategorized statistics to 'Other'
        categorized = set()
        for cat_stats in categories.values():
            categorized.update(cat_stats)
        categories['Other'] = [k for k in self._statistics.keys() if k not in categorized]
        
        for category, stat_names in categories.items():
            if stat_names:
                print(f"\n{category}:")
                for name in sorted(stat_names):
                    value = self._statistics[name]
                    if 'time' in name.lower():
                        print(f"  {name}: {value:.2f} ms")
                    elif name.lower().endswith('_id'):
                        print(f"  {name}: {int(value)}")
                    elif any(w in name.lower() for w in ['size', 'count', 'extracted', 'matched']):
                        print(f"  {name}: {int(value)}")
                    else:
                        print(f"  {name}: {value:.3f}")
        
        print("-" * 50)
