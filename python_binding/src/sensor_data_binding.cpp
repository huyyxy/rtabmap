/**
 * Python bindings for rtabmap::SensorData class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/GPS.h>
#include <opencv2/opencv.hpp>

namespace py = pybind11;

// Helper function to convert cv::Mat to numpy array
py::array_t<uint8_t> mat_to_numpy(const cv::Mat& mat) {
    if (mat.empty()) {
        return py::array_t<uint8_t>();
    }
    
    std::vector<ssize_t> shape;
    std::vector<ssize_t> strides;
    
    if (mat.channels() == 1) {
        shape = {mat.rows, mat.cols};
        strides = {sizeof(uint8_t) * mat.cols, sizeof(uint8_t)};
    } else {
        shape = {mat.rows, mat.cols, mat.channels()};
        strides = {sizeof(uint8_t) * mat.cols * mat.channels(), 
                   sizeof(uint8_t) * mat.channels(), sizeof(uint8_t)};
    }
    
    return py::array_t<uint8_t>(shape, strides, mat.data);
}

// Helper function to convert numpy array to cv::Mat
cv::Mat numpy_to_mat(py::array_t<uint8_t> input) {
    py::buffer_info buf_info = input.request();
    
    if (buf_info.ndim == 2) {
        return cv::Mat(buf_info.shape[0], buf_info.shape[1], CV_8UC1, buf_info.ptr);
    } else if (buf_info.ndim == 3) {
        return cv::Mat(buf_info.shape[0], buf_info.shape[1], 
                       CV_8UC(buf_info.shape[2]), buf_info.ptr);
    }
    
    throw std::runtime_error("Unsupported array dimensions");
}

void init_sensor_data(py::module &m) {
    py::class_<rtabmap::SensorData>(m, "SensorData")
        .def(py::init<>(), "Default constructor")
        .def(py::init<const cv::Mat&, int, double>(),
             "Construct with image",
             py::arg("image"), py::arg("id") = 0, py::arg("stamp") = 0.0)
        .def(py::init<const cv::Mat&, const cv::Mat&, const rtabmap::CameraModel&, int, double>(),
             "Construct with RGB-D data",
             py::arg("rgb"), py::arg("depth"), py::arg("camera_model"), 
             py::arg("id") = 0, py::arg("stamp") = 0.0)
        .def(py::init<const cv::Mat&, const cv::Mat&, const std::vector<rtabmap::CameraModel>&, int, double>(),
             "Construct with RGB-D data and multiple camera models",
             py::arg("rgb"), py::arg("depth"), py::arg("camera_models"),
             py::arg("id") = 0, py::arg("stamp") = 0.0)
        
        // Static factory methods
        .def_static("create", [](py::array_t<uint8_t> rgb_array, 
                                py::array_t<uint16_t> depth_array,
                                const rtabmap::CameraModel& camera_model,
                                int id = 0, double stamp = 0.0) {
            // Convert numpy arrays to cv::Mat
            py::buffer_info rgb_info = rgb_array.request();
            py::buffer_info depth_info = depth_array.request();
            
            cv::Mat rgb_mat(rgb_info.shape[0], rgb_info.shape[1], 
                           CV_8UC3, rgb_info.ptr);
            cv::Mat depth_mat(depth_info.shape[0], depth_info.shape[1], 
                             CV_16UC1, depth_info.ptr);
            
            return rtabmap::SensorData(rgb_mat, depth_mat, camera_model, id, stamp);
        }, "Create SensorData from numpy arrays",
           py::arg("rgb"), py::arg("depth"), py::arg("camera_model"),
           py::arg("id") = 0, py::arg("stamp") = 0.0)
        
        // Data access methods
        .def("id", &rtabmap::SensorData::id, "Get sensor data ID")
        .def("stamp", &rtabmap::SensorData::stamp, "Get timestamp")
        .def("setId", &rtabmap::SensorData::setId, "Set sensor data ID", py::arg("id"))
        .def("setStamp", &rtabmap::SensorData::setStamp, "Set timestamp", py::arg("stamp"))
        
        // Image data
        .def("imageRaw", [](const rtabmap::SensorData& self) -> py::array_t<uint8_t> {
            return mat_to_numpy(self.imageRaw());
        }, "Get raw RGB image as numpy array")
        
        .def("depthRaw", [](const rtabmap::SensorData& self) -> py::array_t<uint16_t> {
            cv::Mat depth = self.depthRaw();
            if (depth.empty()) {
                return py::array_t<uint16_t>();
            }
            return py::array_t<uint16_t>(
                {depth.rows, depth.cols},
                {sizeof(uint16_t) * depth.cols, sizeof(uint16_t)},
                (uint16_t*)depth.data
            );
        }, "Get raw depth image as numpy array")
        
        .def("imageCompressed", &rtabmap::SensorData::imageCompressed, 
             "Get compressed RGB image data")
        .def("depthCompressed", &rtabmap::SensorData::depthCompressed,
             "Get compressed depth image data")
        
        // Camera models
        .def("cameraModels", &rtabmap::SensorData::cameraModels,
             "Get camera models", py::return_value_policy::reference_internal)
        .def("stereoCameraModels", &rtabmap::SensorData::stereoCameraModels,
             "Get stereo camera models", py::return_value_policy::reference_internal)
        
        // Laser scan data
        .def("laserScanRaw", &rtabmap::SensorData::laserScanRaw,
             "Get raw laser scan data", py::return_value_policy::reference_internal)
        .def("laserScanCompressed", &rtabmap::SensorData::laserScanCompressed,
             "Get compressed laser scan data")
        
        // IMU data
        .def("imu", &rtabmap::SensorData::imu,
             "Get IMU data", py::return_value_policy::reference_internal)
        
        // GPS data  
        .def("gps", &rtabmap::SensorData::gps,
             "Get GPS data", py::return_value_policy::reference_internal)
        
        // Environment sensors
        .def("envSensors", &rtabmap::SensorData::envSensors,
             "Get environment sensor data", py::return_value_policy::reference_internal)
        
        // Utility methods
        .def("isValid", [](const rtabmap::SensorData& self) -> bool {
            return !self.imageRaw().empty() || !self.laserScanRaw().isEmpty();
        }, "Check if sensor data is valid")
        
        .def("isEmpty", [](const rtabmap::SensorData& self) -> bool {
            return self.imageRaw().empty() && self.depthRaw().empty() && 
                   self.laserScanRaw().isEmpty();
        }, "Check if sensor data is empty")
        
        .def("hasImage", [](const rtabmap::SensorData& self) -> bool {
            return !self.imageRaw().empty();
        }, "Check if has RGB image")
        
        .def("hasDepth", [](const rtabmap::SensorData& self) -> bool {
            return !self.depthRaw().empty();
        }, "Check if has depth data")
        
        .def("hasLaserScan", [](const rtabmap::SensorData& self) -> bool {
            return !self.laserScanRaw().isEmpty();
        }, "Check if has laser scan data")
        
        .def("hasIMU", [](const rtabmap::SensorData& self) -> bool {
            return !self.imu().empty();
        }, "Check if has IMU data")
        
        .def("hasGPS", [](const rtabmap::SensorData& self) -> bool {
            return !self.gps().stamp() == 0.0;
        }, "Check if has GPS data")
        
        // Data modification methods
        .def("setRGBDImage", [](rtabmap::SensorData& self, 
                               py::array_t<uint8_t> rgb_array,
                               py::array_t<uint16_t> depth_array,
                               const rtabmap::CameraModel& camera_model) {
            py::buffer_info rgb_info = rgb_array.request();
            py::buffer_info depth_info = depth_array.request();
            
            cv::Mat rgb_mat(rgb_info.shape[0], rgb_info.shape[1], 
                           CV_8UC3, rgb_info.ptr);
            cv::Mat depth_mat(depth_info.shape[0], depth_info.shape[1], 
                             CV_16UC1, depth_info.ptr);
            
            std::vector<rtabmap::CameraModel> models;
            models.push_back(camera_model);
            self.setRGBDImage(rgb_mat, depth_mat, models);
        }, "Set RGB-D image data from numpy arrays")
        
        .def("setLaserScan", &rtabmap::SensorData::setLaserScan,
             "Set laser scan data", py::arg("laser_scan"))
        
        .def("setIMU", &rtabmap::SensorData::setIMU,
             "Set IMU data", py::arg("imu"))
        
        .def("setGPS", &rtabmap::SensorData::setGPS,
             "Set GPS data", py::arg("gps"))
        
        .def("addEnvSensor", &rtabmap::SensorData::addEnvSensor,
             "Add environment sensor data", py::arg("env_sensor"))
        
        // Compression methods
        .def("uncompressData", py::overload_cast<>(&rtabmap::SensorData::uncompressData),
             "Uncompress all data")
        .def("uncompressData", py::overload_cast<cv::Mat*, cv::Mat*>(&rtabmap::SensorData::uncompressData),
             "Uncompress image and depth data", py::arg("rgb"), py::arg("depth"))
        
        // Clone method
        .def("clone", [](const rtabmap::SensorData& self) {
            return rtabmap::SensorData(self);
        }, "Create a copy of the sensor data")
        
        // String representation
        .def("__str__", [](const rtabmap::SensorData& self) {
            std::stringstream ss;
            ss << "SensorData(id=" << self.id() << ", stamp=" << self.stamp();
            if (!self.imageRaw().empty()) {
                ss << ", image=" << self.imageRaw().size();
            }
            if (!self.depthRaw().empty()) {
                ss << ", depth=" << self.depthRaw().size();
            }
            ss << ")";
            return ss.str();
        })
        
        .def("__repr__", [](const rtabmap::SensorData& self) {
            std::stringstream ss;
            ss << "SensorData(id=" << self.id() << ", stamp=" << self.stamp() << ")";
            return ss.str();
        });
}
