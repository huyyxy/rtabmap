"""
Python wrapper for RTAB-Map Parameters class.

This module provides parameter management for RTAB-Map configuration.
"""

from typing import Dict, List, Optional, Any, Union
import os


class Parameters:
    """
    Python wrapper for rtabmap::Parameters class.
    
    Manages configuration parameters for RTAB-Map including default values,
    parameter validation, and parameter file I/O.
    
    Example:
        >>> params = Parameters()
        >>> params.set('RGBD/Enabled', 'true')
        >>> params.set('Rtabmap/TimeThr', '700')
        >>> enabled = params.get_bool('RGBD/Enabled')
        >>> time_thr = params.get_float('Rtabmap/TimeThr')
    """
    
    def __init__(self):
        """Initialize with default parameters."""
        self._parameters = {}
        self._init_default_parameters()
        
    def _init_default_parameters(self) -> None:
        """Initialize default RTAB-Map parameters."""
        # Core RTAB-Map parameters
        defaults = {
            # RTAB-Map core
            'Rtabmap/TimeThr': '700',
            'Rtabmap/MemoryThr': '0',
            'Rtabmap/LoopThr': '0.15',
            'Rtabmap/LoopRatio': '0.0',
            'Rtabmap/MaxRetrieved': '2',
            'Rtabmap/MaxRepublished': '2',
            'Rtabmap/StatisticLogged': 'false',
            'Rtabmap/StatisticLoggedHeaders': 'true',
            'Rtabmap/PublishStats': 'true',
            'Rtabmap/PublishLastSignature': 'true',
            'Rtabmap/PublishPdf': 'true',
            'Rtabmap/PublishLikelihood': 'true',
            'Rtabmap/PublishRAMUsage': 'false',
            'Rtabmap/ComputeRMSE': 'true',
            'Rtabmap/SaveWMState': 'false',
            'Rtabmap/WorkingDirectory': '',
            
            # RGB-D SLAM
            'RGBD/Enabled': 'true',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/LinearSpeedUpdate': '0.0',
            'RGBD/AngularSpeedUpdate': '0.0',
            'RGBD/NewMapOdomChangeDistance': '0',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/OptimizeMaxError': '3.0',
            'RGBD/SavedLocalizationIgnored': 'false',
            'RGBD/StartAtOrigin': 'false',
            'RGBD/GoalReachedRadius': '0.5',
            'RGBD/PlanStuckIterations': '0',
            'RGBD/PlanLinearVelocity': '0.0',
            'RGBD/PlanAngularVelocity': '0.0',
            'RGBD/GoalsSavedInUserData': 'false',
            'RGBD/MaxLocalRetrieved': '2',
            'RGBD/LocalRadius': '10',
            'RGBD/LocalImmunizationRatio': '0.25',
            'RGBD/ScanMatchingIdsSavedInLinks': 'true',
            'RGBD/NeighborLinkRefining': 'false',
            'RGBD/ProximityByTime': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '50',
            'RGBD/ProximityPathMaxNeighbors': '0',
            'RGBD/AggressiveLoopThr': '0.15',
            'RGBD/MaxLoopClosureDistance': '0.0',
            
            # Memory management
            'Mem/RehearsalSimilarity': '0.6',
            'Mem/ImageKept': 'false',
            'Mem/BinDataKept': 'true',
            'Mem/RawDescriptorsKept': 'true',
            'Mem/MapLabelsAdded': 'true',
            'Mem/RehearsalIdUpdatedToNewOne': 'false',
            'Mem/GenerateIds': 'true',
            'Mem/BadSignaturesIgnored': 'false',
            'Mem/InitWMWithAllNodes': 'false',
            'Mem/ReduceGraph': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/LocalSpaceLinksKeptInWM': 'true',
            'Mem/STMSize': '10',
            'Mem/LaserScanDownsampleStepSize': '1',
            'Mem/LaserScanVoxelSize': '0.0',
            'Mem/LaserScanNormalK': '0',
            'Mem/LaserScanNormalRadius': '0.0',
            'Mem/LaserScanGroundNormalsUp': '0.0',
            
            # Database
            'Db/TargetVersion': '',
            'DbSqlite3/InMemory': 'false',
            'DbSqlite3/CacheSize': '10000',
            'DbSqlite3/JournalMode': '3',
            'DbSqlite3/Synchronous': '0',
            'DbSqlite3/TempStore': '2',
            
            # Features
            'Kp/MaxFeatures': '400',
            'Kp/MaxDepth': '0',
            'Kp/MinDepth': '0',
            'Kp/RoiRatios': '0.0 0.0 0.0 0.0',
            'Kp/DictionaryPath': '',
            'Kp/NewDictionaryPath': '',
            'Kp/SubPixWinSize': '3',
            'Kp/SubPixIterations': '30',
            'Kp/SubPixEps': '0.02',
            'Kp/GridRows': '1',
            'Kp/GridCols': '1',
            'Kp/FlannRebalancingFactor': '2.0',
            'Kp/ByteToFloat': 'false',
            'Kp/Parallelized': 'true',
            'Kp/IncrementalDictionary': 'true',
            'Kp/IncrementalFlann': 'true',
            'Kp/FlannRebalancingFactor': '2.0',
            'Kp/NNStrategy': '1',
            'Kp/NndrRatio': '0.8',
            'Kp/TfIdfLikelihoodUsed': 'true',
            'Kp/DetectorStrategy': '6',  # GFTT/BRIEF
            'Kp/WordsPerImage': '0',
            
            # Visual odometry
            'Odom/Strategy': '1',  # F2M
            'Odom/ResetCountdown': '0',
            'Odom/Holonomic': 'true',
            'Odom/FillInfoData': 'true',
            'Odom/ImageBufferSize': '1',
            'Odom/FilteringStrategy': '0',
            'Odom/ParticleSize': '400',
            'Odom/ParticleNoiseT': '0.002',
            'Odom/ParticleNoiseR': '0.002',
            'Odom/ParticleLambdaT': '100',
            'Odom/ParticleLambdaR': '100',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/KalmanMeasurementNoise': '0.01',
            'Odom/GuessMotion': 'true',
            'Odom/KeyFrameThr': '0.3',
            'Odom/VisKeyFrameThr': '0.3',
            'Odom/ScanKeyFrameThr': '0.9',
            'Odom/ImageDecimation': '1',
            'Odom/AlignWithGround': 'false',
            
            # Graph optimization
            'Optimizer/Strategy': '2',  # g2o
            'Optimizer/Iterations': '20',
            'Optimizer/Epsilon': '0.0',
            'Optimizer/Robust': 'false',
            'Optimizer/VarianceIgnored': 'false',
            'Optimizer/LandmarksIgnored': 'false',
            'Optimizer/PriorsIgnored': 'true',
            'Optimizer/GravitySigma': '0.3',
            
            # ICP registration
            'Icp/MaxTranslation': '0.2',
            'Icp/MaxRotation': '0.78',
            'Icp/VoxelSize': '0.05',
            'Icp/DownsamplingStep': '1',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.0',
            'Icp/CorrespondenceRatio': '0.1',
            'Icp/Strategy': '1',  # Point to point
            'Icp/OutlierRatio': '0.85',
            'Icp/PointToPlane': 'false',
            'Icp/PointToPlaneK': '5',
            'Icp/PointToPlaneRadius': '0.0',
            'Icp/PointToPlaneGroundNormalsUp': '0.0',
            'Icp/PointToPlaneMinComplexity': '0.02',
            'Icp/RangeMin': '0.0',
            'Icp/RangeMax': '0.0',
            'Icp/FiltersEnabled': '1',
            'Icp/PMOutlierRatio': '0.95',
            'Icp/PMMatcherKnn': '1',
            'Icp/PMMatcherEpsilon': '0.0',
            'Icp/PMMatcherIntensity': 'false',
            'Icp/CCSamplingLimit': '50000',
            'Icp/CCFilteringRadius': '0.05',
            'Icp/CCMaxFinalRMS': '0.2',
            
            # Grid mapping
            'Grid/DepthDecimation': '4',
            'Grid/RangeMin': '0.0',
            'Grid/RangeMax': '5.0',
            'Grid/FootprintLength': '0.0',
            'Grid/FootprintWidth': '0.0',
            'Grid/FootprintHeight': '0.0',
            'Grid/Sensor': '1',
            'Grid/MapFrameProjection': 'false',
            'Grid/MaxObstacleHeight': '0.0',
            'Grid/MaxGroundHeight': '0.0',
            'Grid/MinGroundHeight': '0.0',
            'Grid/NormalsSegmentation': 'true',
            'Grid/ClusterRadius': '0.1',
            'Grid/MinClusterSize': '10',
            'Grid/FlatObstacleDetected': 'true',
            'Grid/3D': 'true',
            'Grid/GroundIsObstacle': 'false',
            'Grid/NoiseFilteringRadius': '0.0',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'Grid/Scan2dUnknownSpaceFilled': 'false',
            'Grid/RayTracing': 'false',
        }
        
        self._parameters = defaults.copy()
        
    def set(self, name: str, value: str) -> None:
        """
        Set parameter value.
        
        Args:
            name: Parameter name
            value: Parameter value as string
        """
        self._parameters[name] = str(value)
        
    def get(self, name: str, default_value: str = "") -> str:
        """
        Get parameter value as string.
        
        Args:
            name: Parameter name
            default_value: Default value if parameter doesn't exist
            
        Returns:
            Parameter value as string
        """
        return self._parameters.get(name, default_value)
        
    def get_bool(self, name: str, default_value: bool = False) -> bool:
        """
        Get parameter value as boolean.
        
        Args:
            name: Parameter name
            default_value: Default value if parameter doesn't exist
            
        Returns:
            Parameter value as boolean
        """
        value = self.get(name, str(default_value).lower())
        return value.lower() in ('true', '1', 'yes', 'on')
        
    def get_int(self, name: str, default_value: int = 0) -> int:
        """
        Get parameter value as integer.
        
        Args:
            name: Parameter name
            default_value: Default value if parameter doesn't exist
            
        Returns:
            Parameter value as integer
        """
        try:
            return int(float(self.get(name, str(default_value))))
        except ValueError:
            return default_value
            
    def get_float(self, name: str, default_value: float = 0.0) -> float:
        """
        Get parameter value as float.
        
        Args:
            name: Parameter name
            default_value: Default value if parameter doesn't exist
            
        Returns:
            Parameter value as float
        """
        try:
            return float(self.get(name, str(default_value)))
        except ValueError:
            return default_value
            
    def has_parameter(self, name: str) -> bool:
        """Check if parameter exists."""
        return name in self._parameters
        
    def remove_parameter(self, name: str) -> bool:
        """
        Remove parameter.
        
        Args:
            name: Parameter name
            
        Returns:
            True if parameter was removed, False if it didn't exist
        """
        if name in self._parameters:
            del self._parameters[name]
            return True
        return False
        
    def get_all_parameters(self) -> Dict[str, str]:
        """Get all parameters as dictionary."""
        return self._parameters.copy()
        
    def get_parameter_names(self) -> List[str]:
        """Get list of all parameter names."""
        return list(self._parameters.keys())
        
    def update(self, parameters: Dict[str, Union[str, int, float, bool]]) -> None:
        """
        Update multiple parameters at once.
        
        Args:
            parameters: Dictionary of parameter name-value pairs
        """
        for name, value in parameters.items():
            self.set(name, str(value))
            
    def clear(self) -> None:
        """Clear all parameters and reset to defaults."""
        self._parameters.clear()
        self._init_default_parameters()
        
    # File I/O methods
    def read_ini(self, filename: str) -> bool:
        """
        Read parameters from INI file.
        
        Args:
            filename: INI file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            import configparser
            
            config = configparser.ConfigParser()
            config.read(filename)
            
            for section in config.sections():
                for key, value in config.items(section):
                    param_name = f"{section}/{key}" if section != 'DEFAULT' else key
                    self.set(param_name, value)
                    
            return True
            
        except Exception as e:
            print(f"Error reading INI file {filename}: {e}")
            return False
            
    def write_ini(self, filename: str) -> bool:
        """
        Write parameters to INI file.
        
        Args:
            filename: Output INI file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            import configparser
            
            config = configparser.ConfigParser()
            
            # Group parameters by section
            sections = {}
            for param_name, value in self._parameters.items():
                if '/' in param_name:
                    section, key = param_name.split('/', 1)
                else:
                    section, key = 'DEFAULT', param_name
                    
                if section not in sections:
                    sections[section] = {}
                sections[section][key] = value
                
            # Add sections to config
            for section, params in sections.items():
                if section != 'DEFAULT':
                    config.add_section(section)
                for key, value in params.items():
                    config.set(section, key, value)
                    
            with open(filename, 'w') as f:
                config.write(f)
                
            return True
            
        except Exception as e:
            print(f"Error writing INI file {filename}: {e}")
            return False
            
    # Utility methods
    def get_default_database_name(self) -> str:
        """Get default database filename."""
        return "rtabmap.db"
        
    def create_default_working_directory(self) -> str:
        """Create and return default working directory path."""
        home_dir = os.path.expanduser("~")
        rtabmap_dir = os.path.join(home_dir, ".rtabmap")
        
        os.makedirs(rtabmap_dir, exist_ok=True)
        return rtabmap_dir
        
    def get_working_directory(self) -> str:
        """Get working directory path."""
        working_dir = self.get('Rtabmap/WorkingDirectory')
        if not working_dir:
            working_dir = self.create_default_working_directory()
            self.set('Rtabmap/WorkingDirectory', working_dir)
        return working_dir
        
    def is_rgb_d_enabled(self) -> bool:
        """Check if RGB-D SLAM is enabled."""
        return self.get_bool('RGBD/Enabled')
        
    def get_loop_closure_threshold(self) -> float:
        """Get loop closure detection threshold."""
        return self.get_float('Rtabmap/LoopThr')
        
    def get_time_threshold(self) -> float:
        """Get time threshold in milliseconds."""
        return self.get_float('Rtabmap/TimeThr')
        
    def get_max_features(self) -> int:
        """Get maximum number of features to extract."""
        return self.get_int('Kp/MaxFeatures')
        
    def get_detector_strategy(self) -> int:
        """Get feature detector strategy."""
        return self.get_int('Kp/DetectorStrategy')
        
    def get_optimizer_strategy(self) -> int:
        """Get graph optimizer strategy."""
        return self.get_int('Optimizer/Strategy')
        
    def get_icp_max_translation(self) -> float:
        """Get ICP maximum translation threshold."""
        return self.get_float('Icp/MaxTranslation')
        
    def get_icp_max_rotation(self) -> float:
        """Get ICP maximum rotation threshold."""
        return self.get_float('Icp/MaxRotation')
        
    # Validation methods
    def validate_parameters(self) -> List[str]:
        """
        Validate parameter values and return list of warnings.
        
        Returns:
            List of validation warning messages
        """
        warnings = []
        
        # Check time threshold
        time_thr = self.get_float('Rtabmap/TimeThr')
        if time_thr < 0:
            warnings.append("Rtabmap/TimeThr should be >= 0")
            
        # Check loop closure threshold
        loop_thr = self.get_float('Rtabmap/LoopThr')
        if not (0.0 <= loop_thr <= 1.0):
            warnings.append("Rtabmap/LoopThr should be between 0.0 and 1.0")
            
        # Check max features
        max_features = self.get_int('Kp/MaxFeatures')
        if max_features <= 0:
            warnings.append("Kp/MaxFeatures should be > 0")
            
        # Check optimizer iterations
        opt_iter = self.get_int('Optimizer/Iterations')
        if opt_iter <= 0:
            warnings.append("Optimizer/Iterations should be > 0")
            
        # Check ICP parameters
        icp_max_trans = self.get_float('Icp/MaxTranslation')
        if icp_max_trans < 0:
            warnings.append("Icp/MaxTranslation should be >= 0")
            
        icp_max_rot = self.get_float('Icp/MaxRotation')
        if icp_max_rot < 0:
            warnings.append("Icp/MaxRotation should be >= 0")
            
        return warnings
        
    def __repr__(self) -> str:
        """String representation."""
        return f"Parameters(count={len(self._parameters)})"
        
    def __str__(self) -> str:
        """Detailed string representation."""
        lines = ["RTAB-Map Parameters:"]
        lines.append(f"  Total parameters: {len(self._parameters)}")
        
        # Show key parameters
        key_params = [
            'RGBD/Enabled',
            'Rtabmap/TimeThr',
            'Rtabmap/LoopThr',
            'Kp/MaxFeatures',
            'Kp/DetectorStrategy',
            'Optimizer/Strategy'
        ]
        
        lines.append("  Key parameters:")
        for param in key_params:
            if self.has_parameter(param):
                value = self.get(param)
                lines.append(f"    {param}: {value}")
                
        return "\n".join(lines)
