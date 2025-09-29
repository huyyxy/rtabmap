/**
 * Python bindings for rtabmap::Parameters class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>

namespace py = pybind11;

void init_parameters(py::module &m) {
    // Note: Parameters class has private constructor, so we expose only static methods
    py::module params = m.def_submodule("Parameters", "RTAB-Map Parameters static methods");
    
    // Static methods for parameter access
    params.def("getDefaultParameters", &rtabmap::Parameters::getDefaultParameters,
               "Get all default parameters as a dictionary",
               py::return_value_policy::reference_internal);
    params.def("getType", &rtabmap::Parameters::getType,
               "Get parameter type", py::arg("key"));
    params.def("getDescription", &rtabmap::Parameters::getDescription,
               "Get parameter description", py::arg("key"));
    
    // Parameter validation
    params.def("isFeatureParameter", &rtabmap::Parameters::isFeatureParameter,
               "Check if parameter is a feature parameter", py::arg("param"));
    
    // Parameter parsing and validation
    params.def("parseArguments", &rtabmap::Parameters::parseArguments,
               "Parse command line arguments", py::arg("argc"), py::arg("argv"), py::arg("onlyParameters") = false);
    
    // Parameter groups for easier access
    params.def("getDefaultOdometryParameters", &rtabmap::Parameters::getDefaultOdometryParameters,
               "Get default odometry parameters", py::arg("stereo") = false, py::arg("vis") = true, py::arg("icp") = false);
    params.def("getDefaultParameters", py::overload_cast<const std::string&>(&rtabmap::Parameters::getDefaultParameters),
               "Get default parameters for a specific group", py::arg("group"));
    
    // Utility functions from UConversion (not Parameters class methods)
    py::module utils = m.def_submodule("Utils", "Utility conversion functions");
    utils.def("uStr2Bool", py::overload_cast<const std::string&>(&uStr2Bool),
              "Convert string to boolean", py::arg("str"));
    utils.def("uBool2Str", &uBool2Str,
              "Convert boolean to string", py::arg("boolean"));
    utils.def("uStr2Int", &uStr2Int,
              "Convert string to integer", py::arg("str"));
    utils.def("uNumber2Str", py::overload_cast<int>(&uNumber2Str),
              "Convert integer to string", py::arg("number"));
    utils.def("uStr2Float", &uStr2Float,
              "Convert string to float", py::arg("str"));
    utils.def("uNumber2Str", py::overload_cast<float, int, bool>(&uNumber2Str),
              "Convert float to string", py::arg("number"), py::arg("precision") = 6, py::arg("fixed") = false);
    
    // Add parameter constants as module attributes
    py::module param = m.def_submodule("Param", "RTAB-Map parameter constants");
    
    // Core RTAB-Map parameters (only include ones that actually exist)
    param.attr("kRtabmapTimeThr") = rtabmap::Parameters::kRtabmapTimeThr();
    param.attr("kRtabmapMemoryThr") = rtabmap::Parameters::kRtabmapMemoryThr();
    param.attr("kRtabmapDetectionRate") = rtabmap::Parameters::kRtabmapDetectionRate();
    param.attr("kRtabmapImageBufferSize") = rtabmap::Parameters::kRtabmapImageBufferSize();
    param.attr("kRtabmapMaxRetrieved") = rtabmap::Parameters::kRtabmapMaxRetrieved();
    param.attr("kRtabmapLoopThr") = rtabmap::Parameters::kRtabmapLoopThr();
    param.attr("kRtabmapLoopRatio") = rtabmap::Parameters::kRtabmapLoopRatio();
    param.attr("kRtabmapLoopGPS") = rtabmap::Parameters::kRtabmapLoopGPS();
    
    // Memory parameters
    param.attr("kMemRehearsalSimilarity") = rtabmap::Parameters::kMemRehearsalSimilarity();
    param.attr("kMemImageKept") = rtabmap::Parameters::kMemImageKept();
    param.attr("kMemBinDataKept") = rtabmap::Parameters::kMemBinDataKept();
    param.attr("kMemRawDescriptorsKept") = rtabmap::Parameters::kMemRawDescriptorsKept();
    param.attr("kMemSaveDepth16Format") = rtabmap::Parameters::kMemSaveDepth16Format();
    param.attr("kMemCompressionParallelized") = rtabmap::Parameters::kMemCompressionParallelized();
    
    // RGB-D parameters
    param.attr("kRGBDEnabled") = rtabmap::Parameters::kRGBDEnabled();
    param.attr("kRGBDLinearUpdate") = rtabmap::Parameters::kRGBDLinearUpdate();
    param.attr("kRGBDAngularUpdate") = rtabmap::Parameters::kRGBDAngularUpdate();
    param.attr("kRGBDLinearSpeedUpdate") = rtabmap::Parameters::kRGBDLinearSpeedUpdate();
    param.attr("kRGBDAngularSpeedUpdate") = rtabmap::Parameters::kRGBDAngularSpeedUpdate();
    param.attr("kRGBDNewMapOdomChangeDistance") = rtabmap::Parameters::kRGBDNewMapOdomChangeDistance();
    param.attr("kRGBDOptimizeFromGraphEnd") = rtabmap::Parameters::kRGBDOptimizeFromGraphEnd();
    param.attr("kRGBDOptimizeMaxError") = rtabmap::Parameters::kRGBDOptimizeMaxError();
    param.attr("kRGBDStartAtOrigin") = rtabmap::Parameters::kRGBDStartAtOrigin();
    param.attr("kRGBDGoalReachedRadius") = rtabmap::Parameters::kRGBDGoalReachedRadius();
    param.attr("kRGBDForceOdom3DoF") = rtabmap::Parameters::kRGBDForceOdom3DoF();
    
    // Feature detection parameters
    param.attr("kKpMaxFeatures") = rtabmap::Parameters::kKpMaxFeatures();
    param.attr("kKpDetectorStrategy") = rtabmap::Parameters::kKpDetectorStrategy();
    param.attr("kKpNNStrategy") = rtabmap::Parameters::kKpNNStrategy();
    param.attr("kKpNndrRatio") = rtabmap::Parameters::kKpNndrRatio();
    param.attr("kKpMaxDepth") = rtabmap::Parameters::kKpMaxDepth();
    param.attr("kKpMinDepth") = rtabmap::Parameters::kKpMinDepth();
    param.attr("kKpRoiRatios") = rtabmap::Parameters::kKpRoiRatios();
    param.attr("kKpSubPixWinSize") = rtabmap::Parameters::kKpSubPixWinSize();
    param.attr("kKpSubPixIterations") = rtabmap::Parameters::kKpSubPixIterations();
    param.attr("kKpSubPixEps") = rtabmap::Parameters::kKpSubPixEps();
    
    // Helper class for easier parameter management
    py::class_<rtabmap::ParametersMap>(m, "ParametersMap")
        .def(py::init<>())
        .def(py::init<const std::map<std::string, std::string>&>())
        .def("insert", [](rtabmap::ParametersMap& self, const std::string& key, const std::string& value) {
            self.insert(std::make_pair(key, value));
        }, "Insert a parameter")
        .def("find", [](const rtabmap::ParametersMap& self, const std::string& key) -> py::object {
            auto it = self.find(key);
            if (it != self.end()) {
                return py::cast(it->second);
            }
            return py::none();
        }, "Find a parameter value")
        .def("__getitem__", [](const rtabmap::ParametersMap& self, const std::string& key) {
            auto it = self.find(key);
            if (it != self.end()) {
                return it->second;
            }
            throw py::key_error("Parameter not found: " + key);
        })
        .def("__setitem__", [](rtabmap::ParametersMap& self, const std::string& key, const std::string& value) {
            self[key] = value;
        })
        .def("__contains__", [](const rtabmap::ParametersMap& self, const std::string& key) {
            return self.find(key) != self.end();
        })
        .def("keys", [](const rtabmap::ParametersMap& self) {
            std::vector<std::string> keys;
            for (const auto& pair : self) {
                keys.push_back(pair.first);
            }
            return keys;
        })
        .def("values", [](const rtabmap::ParametersMap& self) {
            std::vector<std::string> values;
            for (const auto& pair : self) {
                values.push_back(pair.second);
            }
            return values;
        })
        .def("items", [](const rtabmap::ParametersMap& self) {
            std::vector<std::pair<std::string, std::string>> items;
            for (const auto& pair : self) {
                items.push_back(pair);
            }
            return items;
        })
        .def("size", &rtabmap::ParametersMap::size)
        .def("empty", &rtabmap::ParametersMap::empty)
        .def("clear", &rtabmap::ParametersMap::clear)
        .def("__len__", &rtabmap::ParametersMap::size)
        .def("__iter__", [](const rtabmap::ParametersMap& self) {
            return py::make_iterator(self.begin(), self.end());
        }, py::keep_alive<0, 1>());
}
