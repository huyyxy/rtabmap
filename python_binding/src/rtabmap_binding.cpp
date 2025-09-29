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
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Memory.h>
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
             py::arg("load_database_parameters") = false)
        
        .def("init", py::overload_cast<const std::string&, const std::string&, bool>(&rtabmap::Rtabmap::init),
             "Initialize RTAB-Map with config file and database",
             py::arg("config_file") = "", py::arg("database_path") = "", py::arg("load_database_parameters") = false)
        
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
        
        // Memory and map access
        .def("getMemory", &rtabmap::Rtabmap::getMemory,
             "Get memory object",
             py::return_value_policy::reference_internal)
        .def("getLocalOptimizedPoses", &rtabmap::Rtabmap::getLocalOptimizedPoses,
             "Get local optimized poses from graph optimization",
             py::return_value_policy::copy)
        .def("getLocalConstraints", &rtabmap::Rtabmap::getLocalConstraints,
             "Get local graph constraints/links",
             py::return_value_policy::copy)
        .def("getMapCorrection", &rtabmap::Rtabmap::getMapCorrection,
             "Get map correction transform",
             py::return_value_policy::copy)
        
        // Path planning
        .def("computePath", py::overload_cast<int, bool>(&rtabmap::Rtabmap::computePath),
             "Compute path to goal node",
             py::arg("target_node"), py::arg("global_path") = true)
        .def("computePath", py::overload_cast<const rtabmap::Transform&, float>(&rtabmap::Rtabmap::computePath),
             "Compute path to goal pose",
             py::arg("target_pose"), py::arg("tolerance") = -1.0f)
        .def("getPath", &rtabmap::Rtabmap::getPath,
             "Get current path",
             py::return_value_policy::copy)
        .def("clearPath", [](rtabmap::Rtabmap& self, int status = 0) {
            self.clearPath(status);
        }, "Clear current path", py::arg("status") = 0)
        
        // Database operations
        .def("getTotalMemSize", &rtabmap::Rtabmap::getTotalMemSize,
             "Get total memory size in nodes")
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
        
        // Advanced operations
        .def("generateDOTGraph", &rtabmap::Rtabmap::generateDOTGraph,
             "Generate DOT graph for visualization",
             py::arg("path"), py::arg("id") = 0, py::arg("margin") = 5)
        
        .def("getNodesInRadius", [](rtabmap::Rtabmap& self, const rtabmap::Transform& pose, float radius, int k = 0) {
            return self.getNodesInRadius(pose, radius, k, nullptr);
        }, "Get nodes within radius of a pose",
           py::arg("pose"), py::arg("radius"), py::arg("k") = 0)
        
        // Localization mode
        .def("isLocalizationMode", [](const rtabmap::Rtabmap& self) -> bool {
            const rtabmap::ParametersMap& params = self.getParameters();
            auto it = params.find("Mem/IncrementalMemory");
            return it != params.end() && it->second == "false";
        }, "Check if in localization mode")
        
        // Recovery and cleanup
        .def("cleanupLocalGrids", [](rtabmap::Rtabmap& self, 
                                    const std::map<int, rtabmap::Transform>& mapPoses,
                                    const cv::Mat& map,
                                    float xMin, float yMin, float cellSize,
                                    int cropRadius = 1, bool filterScans = false) -> int {
            return self.cleanupLocalGrids(mapPoses, map, xMin, yMin, cellSize, cropRadius, filterScans);
        }, "Clean up local grids to save memory",
           py::arg("map_poses"), py::arg("map"), py::arg("x_min"), py::arg("y_min"), 
           py::arg("cell_size"), py::arg("crop_radius") = 1, py::arg("filter_scans") = false)
        
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
    
    // Add Memory class binding (minimal interface)
    py::class_<rtabmap::Memory>(m, "Memory")
        .def("getParameters", &rtabmap::Memory::getParameters,
             "Get memory parameters",
             py::return_value_policy::reference_internal)
        .def("getWorkingMem", &rtabmap::Memory::getWorkingMem,
             "Get working memory",
             py::return_value_policy::reference_internal)
        .def("getStMem", &rtabmap::Memory::getStMem,
             "Get short-term memory",
             py::return_value_policy::reference_internal)
        .def("getMaxStMemSize", &rtabmap::Memory::getMaxStMemSize,
             "Get maximum short-term memory size")
        .def("getLastSignatureId", &rtabmap::Memory::getLastSignatureId,
             "Get last signature ID")
        .def("getDatabaseMemoryUsed", &rtabmap::Memory::getDatabaseMemoryUsed,
             "Get database memory usage in bytes")
        .def("getDatabaseUrl", &rtabmap::Memory::getDatabaseUrl,
             "Get database URL")
        .def("getAllLabels", &rtabmap::Memory::getAllLabels,
             "Get all node labels",
             py::return_value_policy::reference_internal)
        .def("allNodesInWM", &rtabmap::Memory::allNodesInWM,
             "Check if all nodes are in working memory");

    // Add VhStrategy enum
    py::enum_<rtabmap::Rtabmap::VhStrategy>(m, "VhStrategy", "Visual hypothesis strategy")
        .value("kVhNone", rtabmap::Rtabmap::kVhNone, "No visual hypothesis")
        .value("kVhEpipolar", rtabmap::Rtabmap::kVhEpipolar, "Epipolar constraint")
        .value("kVhUndef", rtabmap::Rtabmap::kVhUndef, "Undefined");
}
