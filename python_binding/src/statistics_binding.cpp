/**
 * Python bindings for rtabmap::Statistics class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rtabmap/core/Statistics.h>

namespace py = pybind11;

void init_statistics(py::module &m) {
    py::class_<rtabmap::Statistics>(m, "Statistics")
        .def(py::init<>(), "Default constructor")
        
        // Basic accessors
        .def("stamp", &rtabmap::Statistics::stamp, "Get timestamp")
        .def("data", &rtabmap::Statistics::data, 
             "Get all statistics data as dictionary", 
             py::return_value_policy::reference_internal)
        
        // Individual statistic getters
        .def("extended", &rtabmap::Statistics::extended, 
             "Get extended statistics", 
             py::return_value_policy::reference_internal)
        .def("poses", &rtabmap::Statistics::poses,
             "Get poses dictionary",
             py::return_value_policy::reference_internal)
        .def("constraints", &rtabmap::Statistics::constraints,
             "Get constraints/links",
             py::return_value_policy::reference_internal)
        .def("mapCorrection", &rtabmap::Statistics::mapCorrection,
             "Get map correction transform",
             py::return_value_policy::reference_internal)
        .def("labels", &rtabmap::Statistics::labels,
             "Get node labels",
             py::return_value_policy::reference_internal)
        .def("posterior", &rtabmap::Statistics::posterior,
             "Get posterior values",
             py::return_value_policy::reference_internal)
        .def("likelihood", &rtabmap::Statistics::likelihood,
             "Get likelihood values", 
             py::return_value_policy::reference_internal)
        .def("rawLikelihood", &rtabmap::Statistics::rawLikelihood,
             "Get raw likelihood values",
             py::return_value_policy::reference_internal)
        .def("weights", &rtabmap::Statistics::weights,
             "Get node weights",
             py::return_value_policy::reference_internal)
        .def("localPath", &rtabmap::Statistics::localPath,
             "Get local path",
             py::return_value_policy::reference_internal)
        .def("currentGoalId", &rtabmap::Statistics::currentGoalId,
             "Get current goal ID")
        .def("loopClosureId", &rtabmap::Statistics::loopClosureId,
             "Get loop closure ID")
        .def("proximityDetectionId", &rtabmap::Statistics::proximityDetectionId,
             "Get proximity detection ID")
        .def("loopClosureTransform", &rtabmap::Statistics::loopClosureTransform,
             "Get loop closure transform",
             py::return_value_policy::reference_internal)
        
        // Convenience methods for common statistics
        .def("getStatistic", [](const rtabmap::Statistics& self, const std::string& name) -> py::object {
            const std::map<std::string, float>& data = self.data();
            auto it = data.find(name);
            if (it != data.end()) {
                return py::cast(it->second);
            }
            return py::none();
        }, "Get a specific statistic by name", py::arg("name"))
        
        .def("hasStatistic", [](const rtabmap::Statistics& self, const std::string& name) -> bool {
            const std::map<std::string, float>& data = self.data();
            return data.find(name) != data.end();
        }, "Check if a statistic exists", py::arg("name"))
        
        // Common statistics getters with defaults
        .def("getProcessTime", [](const rtabmap::Statistics& self) -> float {
            return self.data().count("Process/time/ms") ? 
                   self.data().at("Process/time/ms") : 0.0f;
        }, "Get processing time in milliseconds")
        
        .def("getLoopClosureValue", [](const rtabmap::Statistics& self) -> float {
            return self.data().count("Rtabmap/Loop_closure_hypothesis_value") ?
                   self.data().at("Rtabmap/Loop_closure_hypothesis_value") : 0.0f;
        }, "Get loop closure hypothesis value")
        
        .def("getWorkingMemorySize", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Memory/Working_memory_size") ?
                   self.data().at("Memory/Working_memory_size") : 0.0f);
        }, "Get working memory size")
        
        .def("getShortTermMemorySize", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Memory/Short_term_memory_size") ?
                   self.data().at("Memory/Short_term_memory_size") : 0.0f);
        }, "Get short-term memory size")
        
        .def("getLastLocationId", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Rtabmap/Last_location_id") ?
                   self.data().at("Rtabmap/Last_location_id") : 0.0f);
        }, "Get last location ID")
        
        .def("getFeaturesExtracted", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Keypoint/Features_extracted") ?
                   self.data().at("Keypoint/Features_extracted") : 0.0f);
        }, "Get number of features extracted")
        
        .def("getFeaturesMatched", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Keypoint/Features_matched") ?
                   self.data().at("Keypoint/Features_matched") : 0.0f);
        }, "Get number of features matched")
        
        .def("getInliers", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("Keypoint/Inliers") ?
                   self.data().at("Keypoint/Inliers") : 0.0f);
        }, "Get number of inliers")
        
        .def("getICPInliers", [](const rtabmap::Statistics& self) -> int {
            return static_cast<int>(self.data().count("ICP/Inliers") ?
                   self.data().at("ICP/Inliers") : 0.0f);
        }, "Get number of ICP inliers")
        
        .def("getICPError", [](const rtabmap::Statistics& self) -> float {
            return self.data().count("ICP/Fitness_score") ?
                   self.data().at("ICP/Fitness_score") : 0.0f;
        }, "Get ICP fitness score/error")
        
        // Performance summary
        .def("getPerformanceSummary", [](const rtabmap::Statistics& self) -> py::dict {
            py::dict summary;
            const std::map<std::string, float>& data = self.data();
            
            // Timing information
            if (data.count("Process/time/ms")) {
                summary["process_time_ms"] = data.at("Process/time/ms");
            }
            if (data.count("Process/Detection/time/ms")) {
                summary["detection_time_ms"] = data.at("Process/Detection/time/ms");
            }
            if (data.count("Process/Reactivation/time/ms")) {
                summary["reactivation_time_ms"] = data.at("Process/Reactivation/time/ms");
            }
            
            // Memory information
            if (data.count("Memory/Working_memory_size")) {
                summary["working_memory_size"] = static_cast<int>(data.at("Memory/Working_memory_size"));
            }
            if (data.count("Memory/Short_term_memory_size")) {
                summary["short_term_memory_size"] = static_cast<int>(data.at("Memory/Short_term_memory_size"));
            }
            
            // Loop closure information
            if (data.count("Rtabmap/Loop_closure_hypothesis_value")) {
                summary["loop_closure_value"] = data.at("Rtabmap/Loop_closure_hypothesis_value");
            }
            summary["loop_closure_id"] = self.loopClosureId();
            
            // Feature information
            if (data.count("Keypoint/Features_extracted")) {
                summary["features_extracted"] = static_cast<int>(data.at("Keypoint/Features_extracted"));
            }
            if (data.count("Keypoint/Features_matched")) {
                summary["features_matched"] = static_cast<int>(data.at("Keypoint/Features_matched"));
            }
            if (data.count("Keypoint/Inliers")) {
                summary["inliers"] = static_cast<int>(data.at("Keypoint/Inliers"));
            }
            
            return summary;
        }, "Get performance summary as dictionary")
        
        // Utility methods
        .def("addStatistic", [](rtabmap::Statistics& self, const std::string& name, float value) {
            // Note: This requires access to private members, so we simulate it
            // In practice, Statistics objects are typically created by RTAB-Map internally
            py::print("Warning: addStatistic is not directly supported on Statistics objects created from C++");
        }, "Add a custom statistic (warning: limited support)", py::arg("name"), py::arg("value"))
        
        // String representation
        .def("__str__", [](const rtabmap::Statistics& self) {
            std::stringstream ss;
            ss << "Statistics(stamp=" << self.stamp();
            ss << ", loop_closure_id=" << self.loopClosureId();
            const auto& data = self.data();
            if (data.count("Process/time/ms")) {
                ss << ", process_time=" << data.at("Process/time/ms") << "ms";
            }
            if (data.count("Memory/Working_memory_size")) {
                ss << ", wm_size=" << static_cast<int>(data.at("Memory/Working_memory_size"));
            }
            ss << ", total_stats=" << data.size() << ")";
            return ss.str();
        })
        
        .def("__repr__", [](const rtabmap::Statistics& self) {
            return "Statistics(stamp=" + std::to_string(self.stamp()) + ")";
        })
        
        // Dictionary-like access
        .def("__getitem__", [](const rtabmap::Statistics& self, const std::string& key) -> float {
            const std::map<std::string, float>& data = self.data();
            auto it = data.find(key);
            if (it != data.end()) {
                return it->second;
            }
            throw py::key_error("Statistic not found: " + key);
        })
        
        .def("__contains__", [](const rtabmap::Statistics& self, const std::string& key) -> bool {
            const std::map<std::string, float>& data = self.data();
            return data.find(key) != data.end();
        })
        
        .def("keys", [](const rtabmap::Statistics& self) {
            std::vector<std::string> keys;
            const std::map<std::string, float>& data = self.data();
            for (const auto& pair : data) {
                keys.push_back(pair.first);
            }
            return keys;
        }, "Get all statistic names")
        
        .def("values", [](const rtabmap::Statistics& self) {
            std::vector<float> values;
            const std::map<std::string, float>& data = self.data();
            for (const auto& pair : data) {
                values.push_back(pair.second);
            }
            return values;
        }, "Get all statistic values")
        
        .def("items", [](const rtabmap::Statistics& self) {
            std::vector<std::pair<std::string, float>> items;
            const std::map<std::string, float>& data = self.data();
            for (const auto& pair : data) {
                items.push_back(pair);
            }
            return items;
        }, "Get all statistic name-value pairs");
}
