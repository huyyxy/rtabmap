/**
 * Python bindings for rtabmap::Parameters class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rtabmap/core/Parameters.h>

namespace py = pybind11;

void init_parameters(py::module &m) {
    py::class_<rtabmap::Parameters>(m, "Parameters")
        .def(py::init<>(), "Default constructor")
        
        // Static methods for parameter access
        .def_static("getDefaultParameters", &rtabmap::Parameters::getDefaultParameters,
                   "Get all default parameters as a dictionary")
        .def_static("getDefaultParameter", &rtabmap::Parameters::getDefaultParameter,
                   "Get default value for a parameter", py::arg("key"))
        .def_static("getType", &rtabmap::Parameters::getType,
                   "Get parameter type", py::arg("key"))
        .def_static("getDescription", &rtabmap::Parameters::getDescription,
                   "Get parameter description", py::arg("key"))
        
        // Parameter validation
        .def_static("isFeatureParameter", &rtabmap::Parameters::isFeatureParameter,
                   "Check if parameter is a feature parameter", py::arg("key"))
        .def_static("isBowParameter", &rtabmap::Parameters::isBowParameter,
                   "Check if parameter is a BoW parameter", py::arg("key"))
        .def_static("isIcpParameter", &rtabmap::Parameters::isIcpParameter,
                   "Check if parameter is an ICP parameter", py::arg("key"))
        .def_static("isStereoParameter", &rtabmap::Parameters::isStereoParameter,
                   "Check if parameter is a stereo parameter", py::arg("key"))
        
        // Parameter parsing and validation
        .def_static("parse", py::overload_cast<const std::map<std::string, std::string>&>(&rtabmap::Parameters::parse),
                   "Parse and validate parameters", py::arg("parameters"))
        .def_static("parseArguments", &rtabmap::Parameters::parseArguments,
                   "Parse command line arguments", py::arg("argc"), py::arg("argv"), py::arg("remove_processed") = true)
        
        // Utility methods for parameter conversion
        .def_static("uStr2Bool", &rtabmap::Parameters::uStr2Bool,
                   "Convert string to boolean", py::arg("str"))
        .def_static("uBool2Str", &rtabmap::Parameters::uBool2Str,
                   "Convert boolean to string", py::arg("boolean"))
        .def_static("uStr2Int", &rtabmap::Parameters::uStr2Int,
                   "Convert string to integer", py::arg("str"))
        .def_static("uInt2Str", &rtabmap::Parameters::uInt2Str,
                   "Convert integer to string", py::arg("integer"))
        .def_static("uStr2Float", &rtabmap::Parameters::uStr2Float,
                   "Convert string to float", py::arg("str"))
        .def_static("uFloat2Str", &rtabmap::Parameters::uFloat2Str,
                   "Convert float to string", py::arg("value"))
        
        // Parameter groups for easier access
        .def_static("getDefaultParametersRtabmap", &rtabmap::Parameters::getDefaultParametersRtabmap,
                   "Get default RTAB-Map core parameters")
        .def_static("getDefaultParametersCamera", &rtabmap::Parameters::getDefaultParametersCamera,
                   "Get default camera parameters")
        .def_static("getDefaultParametersOdometry", &rtabmap::Parameters::getDefaultParametersOdometry,
                   "Get default odometry parameters")
        .def_static("getDefaultParametersStereo", &rtabmap::Parameters::getDefaultParametersStereo,
                   "Get default stereo parameters")
        .def_static("getDefaultParametersRegistration", &rtabmap::Parameters::getDefaultParametersRegistration,
                   "Get default registration parameters");
    
    // Add parameter constants as module attributes
    py::module param = m.def_submodule("Param", "RTAB-Map parameter constants");
    
    // Core RTAB-Map parameters
    param.attr("kRtabmapPublishStats") = rtabmap::Parameters::kRtabmapPublishStats();
    param.attr("kRtabmapPublishLastSignature") = rtabmap::Parameters::kRtabmapPublishLastSignature();
    param.attr("kRtabmapPublishPdf") = rtabmap::Parameters::kRtabmapPublishPdf();
    param.attr("kRtabmapPublishLikelihood") = rtabmap::Parameters::kRtabmapPublishLikelihood();
    param.attr("kRtabmapTimeThr") = rtabmap::Parameters::kRtabmapTimeThr();
    param.attr("kRtabmapMemoryThr") = rtabmap::Parameters::kRtabmapMemoryThr();
    param.attr("kRtabmapDetectionRate") = rtabmap::Parameters::kRtabmapDetectionRate();
    param.attr("kRtabmapImageBufferSize") = rtabmap::Parameters::kRtabmapImageBufferSize();
    param.attr("kRtabmapMaxRetrieved") = rtabmap::Parameters::kRtabmapMaxRetrieved();
    param.attr("kRtabmapStatisticLogged") = rtabmap::Parameters::kRtabmapStatisticLogged();
    param.attr("kRtabmapStatisticLoggedHeaders") = rtabmap::Parameters::kRtabmapStatisticLoggedHeaders();
    
    // Memory parameters
    param.attr("kMemRehearsalSimilarity") = rtabmap::Parameters::kMemRehearsalSimilarity();
    param.attr("kMemImageKept") = rtabmap::Parameters::kMemImageKept();
    param.attr("kMemBinDataKept") = rtabmap::Parameters::kMemBinDataKept();
    param.attr("kMemRawDescriptorsKept") = rtabmap::Parameters::kMemRawDescriptorsKept();
    param.attr("kMemSaveDepth16Format") = rtabmap::Parameters::kMemSaveDepth16Format();
    param.attr("kMemCompressionParallelized") = rtabmap::Parameters::kMemCompressionParallelized();
    param.attr("kMemLaserScanDownsampleStepSize") = rtabmap::Parameters::kMemLaserScanDownsampleStepSize();
    param.attr("kMemLaserScanVoxelSize") = rtabmap::Parameters::kMemLaserScanVoxelSize();
    param.attr("kMemLaserScanNormalK") = rtabmap::Parameters::kMemLaserScanNormalK();
    param.attr("kMemLaserScanNormalRadius") = rtabmap::Parameters::kMemLaserScanNormalRadius();
    
    // RGB-D parameters
    param.attr("kRGBDEnabled") = rtabmap::Parameters::kRGBDEnabled();
    param.attr("kRGBDLinearUpdate") = rtabmap::Parameters::kRGBDLinearUpdate();
    param.attr("kRGBDAngularUpdate") = rtabmap::Parameters::kRGBDAngularUpdate();
    param.attr("kRGBDLinearSpeedUpdate") = rtabmap::Parameters::kRGBDLinearSpeedUpdate();
    param.attr("kRGBDAngularSpeedUpdate") = rtabmap::Parameters::kRGBDAngularSpeedUpdate();
    param.attr("kRGBDNewMapOdomChangeDistance") = rtabmap::Parameters::kRGBDNewMapOdomChangeDistance();
    param.attr("kRGBDOptimizeFromGraphEnd") = rtabmap::Parameters::kRGBDOptimizeFromGraphEnd();
    param.attr("kRGBDOptimizeMaxError") = rtabmap::Parameters::kRGBDOptimizeMaxError();
    param.attr("kRGBDSavedLocalizationIgnored") = rtabmap::Parameters::kRGBDSavedLocalizationIgnored();
    param.attr("kRGBDStartAtOrigin") = rtabmap::Parameters::kRGBDStartAtOrigin();
    param.attr("kRGBDGoalReachedRadius") = rtabmap::Parameters::kRGBDGoalReachedRadius();
    param.attr("kRGBDGoalsSavedInUserData") = rtabmap::Parameters::kRGBDGoalsSavedInUserData();
    param.attr("kRGBDMaxLocalRetrieved") = rtabmap::Parameters::kRGBDMaxLocalRetrieved();
    param.attr("kRGBDProximityByTime") = rtabmap::Parameters::kRGBDProximityByTime();
    param.attr("kRGBDProximityBySpace") = rtabmap::Parameters::kRGBDProximityBySpace();
    param.attr("kRGBDProximityMaxGraphDepth") = rtabmap::Parameters::kRGBDProximityMaxGraphDepth();
    param.attr("kRGBDProximityPathMaxNeighbors") = rtabmap::Parameters::kRGBDProximityPathMaxNeighbors();
    param.attr("kRGBDProximityPathFilteringRadius") = rtabmap::Parameters::kRGBDProximityPathFilteringRadius();
    param.attr("kRGBDProximityPathRawPosesUsed") = rtabmap::Parameters::kRGBDProximityPathRawPosesUsed();
    param.attr("kRGBDProximityAngle") = rtabmap::Parameters::kRGBDProximityAngle();
    param.attr("kRGBDScanMatchingIdsSavedInLinks") = rtabmap::Parameters::kRGBDScanMatchingIdsSavedInLinks();
    param.attr("kRGBDLocalRadius") = rtabmap::Parameters::kRGBDLocalRadius();
    param.attr("kRGBDLocalImmunizationRatio") = rtabmap::Parameters::kRGBDLocalImmunizationRatio();
    param.attr("kRGBDInvertedReg") = rtabmap::Parameters::kRGBDInvertedReg();
    param.attr("kRGBDForceOdom3DoF") = rtabmap::Parameters::kRGBDForceOdom3DoF();
    
    // Loop closure parameters
    param.attr("kRtabmapLoopThr") = rtabmap::Parameters::kRtabmapLoopThr();
    param.attr("kRtabmapLoopRatio") = rtabmap::Parameters::kRtabmapLoopRatio();
    param.attr("kRtabmapLoopGPS") = rtabmap::Parameters::kRtabmapLoopGPS();
    
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
