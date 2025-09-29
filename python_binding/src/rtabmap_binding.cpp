/**
 * Python bindings for rtabmap::Rtabmap class (main SLAM class).
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/opencv.hpp>

namespace py = pybind11;

void init_rtabmap(py::module &m) {
    py::class_<rtabmap::Rtabmap>(m, "Rtabmap")
        .def(py::init<>(), "Default constructor")
        
        // Initialization and cleanup
        .def("init", py::overload_cast<const rtabmap::ParametersMap&, const std::string&, bool>(&rtabmap::Rtabmap::init),
             "Initialize RTAB-Map with parameters and database",
             py::arg("parameters") = rtabmap::ParametersMap(), 
             py::arg("database_path") = "",
             py::arg("load_database_parameters") = true)
        
        .def("init", py::overload_cast<const rtabmap::ParametersMap&, rtabmap::DBDriver*, bool>(&rtabmap::Rtabmap::init),
             "Initialize RTAB-Map with parameters and database driver",
             py::arg("parameters"), py::arg("db_driver"), py::arg("load_database_parameters") = true)
        
        .def("close", &rtabmap::Rtabmap::close,
             "Close RTAB-Map and save database",
             py::arg("database_saved") = true, py::arg("output_database_path") = "")
        
        // Main processing methods
        .def("process", py::overload_cast<const rtabmap::SensorData&, rtabmap::Transform, const cv::Mat&, const std::vector<float>&, const std::map<std::string, float>&>(&rtabmap::Rtabmap::process),
             "Main processing loop with full parameters",
             py::arg("data"), py::arg("odom_pose"), 
             py::arg("odom_covariance") = cv::Mat::eye(6,6,CV_64FC1),
             py::arg("odom_velocity") = std::vector<float>(),
             py::arg("external_stats") = std::map<std::string, float>())
        
        .def("process", py::overload_cast<const rtabmap::SensorData&, rtabmap::Transform, float, float, const std::vector<float>&, const std::map<std::string, float>&>(&rtabmap::Rtabmap::process),
             "Main processing loop with variance parameters",
             py::arg("data"), py::arg("odom_pose"),
             py::arg("odom_linear_variance"), py::arg("odom_angular_variance"),
             py::arg("odom_velocity") = std::vector<float>(),
             py::arg("external_stats") = std::map<std::string, float>())
        
        .def("process", py::overload_cast<const cv::Mat&, int, const std::map<std::string, float>&>(&rtabmap::Rtabmap::process),
             "Process image only (for loop closure detection)",
             py::arg("image"), py::arg("id") = 0, 
             py::arg("external_stats") = std::map<std::string, float>())
        
        // Convenience process method with numpy arrays
        .def("processRGBD", [](rtabmap::Rtabmap& self,
                              py::array_t<uint8_t> rgb_array,
                              py::array_t<uint16_t> depth_array,
                              const rtabmap::CameraModel& camera_model,
                              const rtabmap::Transform& odom_pose,
                              int id = 0,
                              double stamp = 0.0) -> bool {
            // Convert numpy arrays to cv::Mat
            py::buffer_info rgb_info = rgb_array.request();
            py::buffer_info depth_info = depth_array.request();
            
            cv::Mat rgb_mat(rgb_info.shape[0], rgb_info.shape[1], CV_8UC3, rgb_info.ptr);
            cv::Mat depth_mat(depth_info.shape[0], depth_info.shape[1], CV_16UC1, depth_info.ptr);
            
            // Create sensor data
            rtabmap::SensorData sensor_data(rgb_mat, depth_mat, camera_model, id, stamp);
            
            return self.process(sensor_data, odom_pose);
        }, "Process RGB-D data from numpy arrays",
           py::arg("rgb"), py::arg("depth"), py::arg("camera_model"), py::arg("odom_pose"),
           py::arg("id") = 0, py::arg("stamp") = 0.0)
        
        // State queries
        .def("getLastProcessTime", &rtabmap::Rtabmap::getLastProcessTime,
             "Get last processing time in milliseconds")
        .def("getLoopClosureId", &rtabmap::Rtabmap::getLoopClosureId,
             "Get loop closure ID (0 if no loop closure)")
        .def("getHighestHypothesisId", &rtabmap::Rtabmap::getHighestHypothesisId,
             "Get highest loop closure hypothesis ID")
        .def("getHighestHypothesisValue", &rtabmap::Rtabmap::getHighestHypothesisValue,
             "Get highest loop closure hypothesis value")
        .def("getLastLocationId", &rtabmap::Rtabmap::getLastLocationId,
             "Get last processed location ID")
        .def("getWMSize", &rtabmap::Rtabmap::getWMSize,
             "Get working memory size")
        .def("getSTMSize", &rtabmap::Rtabmap::getSTMSize,
             "Get short-term memory size")
        
        // Statistics and results
        .def("getStatistics", &rtabmap::Rtabmap::getStatistics,
             "Get current statistics",
             py::return_value_policy::copy)
        .def("getStatisticsData", &rtabmap::Rtabmap::getStatisticsData,
             "Get statistics data as dictionary",
             py::return_value_policy::copy)
        
        // Memory and map access
        .def("getMemory", &rtabmap::Rtabmap::getMemory,
             "Get memory object",
             py::return_value_policy::reference_internal)
        .def("getOptimizedPoses", &rtabmap::Rtabmap::getOptimizedPoses,
             "Get optimized poses from graph optimization",
             py::return_value_policy::copy)
        .def("getConstraints", &rtabmap::Rtabmap::getConstraints,
             "Get graph constraints/links",
             py::return_value_policy::copy)
        .def("getMapCorrection", &rtabmap::Rtabmap::getMapCorrection,
             "Get map correction transform",
             py::return_value_policy::copy)
        
        // Path planning
        .def("computePath", py::overload_cast<int, bool>(&rtabmap::Rtabmap::computePath),
             "Compute path to goal node",
             py::arg("target_node"), py::arg("global_path") = true)
        .def("computePath", py::overload_cast<const rtabmap::Transform&, bool>(&rtabmap::Rtabmap::computePath),
             "Compute path to goal pose",
             py::arg("target_pose"), py::arg("global_path") = true)
        .def("getPath", &rtabmap::Rtabmap::getPath,
             "Get current path",
             py::return_value_policy::copy)
        .def("clearPath", &rtabmap::Rtabmap::clearPath,
             "Clear current path")
        
        // Database operations
        .def("getMemoryUsed", &rtabmap::Rtabmap::getMemoryUsed,
             "Get memory usage in bytes")
        .def("exportPoses", &rtabmap::Rtabmap::exportPoses,
             "Export poses to file",
             py::arg("file_path"), py::arg("optimized") = true, py::arg("global") = true, py::arg("format") = 0)
        
        // Parameter management
        .def("parseParameters", &rtabmap::Rtabmap::parseParameters,
             "Parse and validate parameters",
             py::arg("parameters"))
        .def("getParameters", &rtabmap::Rtabmap::getParameters,
             "Get current parameters",
             py::return_value_policy::copy)
        .def("getParameter", &rtabmap::Rtabmap::getParameter,
             "Get a specific parameter value",
             py::arg("key"))
        
        // Advanced operations
        .def("generateDOTGraph", &rtabmap::Rtabmap::generateDOTGraph,
             "Generate DOT graph for visualization",
             py::arg("output_path"), py::arg("poses_id") = std::set<int>(), py::arg("neighbor_link_maps") = std::map<int, int>(),
             py::arg("global_loop_closure_maps") = std::map<int, int>(), py::arg("local_loop_closure_maps") = std::map<int, int>(),
             py::arg("user_link_maps") = std::map<int, int>(), py::arg("virtual_link_maps") = std::map<int, int>(),
             py::arg("neighbor_merged_maps") = std::map<int, int>(), py::arg("landmark_maps") = std::map<int, int>(),
             py::arg("inter_session_maps") = std::map<int, int>())
        
        .def("getNodesInRadius", py::overload_cast<const rtabmap::Transform&, float, int, int, bool>(&rtabmap::Rtabmap::getNodesInRadius, py::const_),
             "Get nodes within radius of a pose",
             py::arg("pose"), py::arg("radius"), py::arg("k") = 0, py::arg("ignore_id") = 0, py::arg("ignore_bad_signatures") = false)
        
        // Localization mode
        .def("setLocalizationMode", &rtabmap::Rtabmap::setLocalizationMode,
             "Set localization mode (true) or mapping mode (false)",
             py::arg("enabled"))
        .def("isLocalizationMode", [](const rtabmap::Rtabmap& self) -> bool {
            return self.getParameter("Mem/IncrementalMemory") == "false";
        }, "Check if in localization mode")
        
        // Recovery and cleanup
        .def("cleanupLocalGrids", &rtabmap::Rtabmap::cleanupLocalGrids,
             "Clean up local grids to save memory",
             py::arg("poses") = std::map<int, rtabmap::Transform>(),
             py::arg("radius") = 5.0f, py::arg("cleanup_laser_scan") = true)
        
        // Utility methods
        .def("isInitialized", [](const rtabmap::Rtabmap& self) -> bool {
            return self.getMemory() != nullptr;
        }, "Check if RTAB-Map is initialized")
        
        .def("getVersion", []() -> std::string {
            return "0.23.1"; // RTAB-Map version
        }, "Get RTAB-Map version")
        
        // String representation
        .def("__str__", [](const rtabmap::Rtabmap& self) {
            std::stringstream ss;
            ss << "Rtabmap(";
            if (self.getMemory()) {
                ss << "initialized=True, ";
                ss << "wm_size=" << self.getWMSize() << ", ";
                ss << "stm_size=" << self.getSTMSize() << ", ";
                ss << "last_id=" << self.getLastLocationId();
            } else {
                ss << "initialized=False";
            }
            ss << ")";
            return ss.str();
        })
        
        .def("__repr__", [](const rtabmap::Rtabmap& self) {
            return self.getMemory() ? "Rtabmap(initialized)" : "Rtabmap(not initialized)";
        });
    
    // Add VhStrategy enum
    py::enum_<rtabmap::Rtabmap::VhStrategy>(m, "VhStrategy", "Visual hypothesis strategy")
        .value("kVhNone", rtabmap::Rtabmap::kVhNone, "No visual hypothesis")
        .value("kVhEpipolar", rtabmap::Rtabmap::kVhEpipolar, "Epipolar constraint")
        .value("kVhUndef", rtabmap::Rtabmap::kVhUndef, "Undefined");
}
