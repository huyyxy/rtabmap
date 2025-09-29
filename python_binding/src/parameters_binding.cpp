/**
 * Python bindings for rtabmap::Parameters class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/cast.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>

namespace py = pybind11;

void init_parameters(py::module &m) {
    // Note: Parameters class has private constructor, so we expose only static methods
    py::module params = m.def_submodule("Parameters", "RTAB-Map Parameters static methods");
    
    // Static methods for parameter access
    params.def("getDefaultParameters", []() -> const rtabmap::ParametersMap& {
        return rtabmap::Parameters::getDefaultParameters();
    }, "Get all default parameters as a dictionary",
       py::return_value_policy::reference);
    params.def("getType", &rtabmap::Parameters::getType,
               "Get parameter type", py::arg("key"));
    params.def("getDescription", &rtabmap::Parameters::getDescription,
               "Get parameter description", py::arg("key"));
    
    // Parameter validation
    params.def("isFeatureParameter", &rtabmap::Parameters::isFeatureParameter,
               "Check if parameter is a feature parameter", py::arg("param"));
    
    // Parameter parsing and validation
    params.def("parseArguments", [](const py::list& args, bool onlyParameters = false) -> rtabmap::ParametersMap {
        std::vector<std::string> arg_strings;
        std::vector<char*> argv;
        
        // Convert Python list to C-style argv
        for (auto item : args) {
            arg_strings.push_back(py::cast<std::string>(item));
        }
        
        // Create char* array
        for (auto& arg : arg_strings) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        
        return rtabmap::Parameters::parseArguments(static_cast<int>(argv.size()), argv.data(), onlyParameters);
    }, "Parse command line arguments", py::arg("args"), py::arg("onlyParameters") = false);
    
    // Parameter groups for easier access
    params.def("getDefaultOdometryParameters", &rtabmap::Parameters::getDefaultOdometryParameters,
               "Get default odometry parameters", py::arg("stereo") = false, py::arg("vis") = true, py::arg("icp") = false);
    params.def("getDefaultParametersForGroup", [](const std::string& group) -> rtabmap::ParametersMap {
        return rtabmap::Parameters::getDefaultParameters(group);
    }, "Get default parameters for a specific group", py::arg("group"));
    
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
    
    // Use pybind11's built-in STL support for std::map
    py::bind_map<rtabmap::ParametersMap>(m, "ParametersMap");
}
