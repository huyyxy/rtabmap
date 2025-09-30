/**
 * Main pybind11 module definition for RTAB-Map Python bindings.
 * 
 * This file creates the main Python module and imports all the individual
 * class bindings from separate files.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <opencv2/opencv.hpp>
#include <rtabmap-0.21/rtabmap/core/Version.h>

// Forward declarations for binding functions
void init_rtabmap(pybind11::module &);
void init_transform(pybind11::module &);
void init_sensor_data(pybind11::module &);
void init_camera_model(pybind11::module &);
void init_parameters(pybind11::module &);
void init_statistics(pybind11::module &);

namespace py = pybind11;

PYBIND11_MODULE(rtabmap_python, m) {
    m.doc() = "Python bindings for RTAB-Map RGB-D SLAM library";
    
    // Register OpenCV basic types
    py::class_<cv::Size>(m, "Size")
        .def(py::init<>(), "Default constructor")
        .def(py::init<int, int>(), "Constructor with width and height", py::arg("width"), py::arg("height"))
        .def_readwrite("width", &cv::Size::width, "Width")
        .def_readwrite("height", &cv::Size::height, "Height")
        .def("__str__", [](const cv::Size& self) {
            return "Size(" + std::to_string(self.width) + ", " + std::to_string(self.height) + ")";
        })
        .def("__repr__", [](const cv::Size& self) {
            return "Size(" + std::to_string(self.width) + ", " + std::to_string(self.height) + ")";
        });
    
    py::class_<cv::Rect>(m, "Rect")
        .def(py::init<>(), "Default constructor")
        .def(py::init<int, int, int, int>(), "Constructor", py::arg("x"), py::arg("y"), py::arg("width"), py::arg("height"))
        .def_readwrite("x", &cv::Rect::x, "X coordinate")
        .def_readwrite("y", &cv::Rect::y, "Y coordinate") 
        .def_readwrite("width", &cv::Rect::width, "Width")
        .def_readwrite("height", &cv::Rect::height, "Height")
        .def("__str__", [](const cv::Rect& self) {
            return "Rect(" + std::to_string(self.x) + ", " + std::to_string(self.y) + ", " + 
                   std::to_string(self.width) + ", " + std::to_string(self.height) + ")";
        });
    
    // Register OpenCV Mat type
    py::class_<cv::Mat>(m, "Mat")
        .def(py::init<>(), "Default constructor")
        .def(py::init<int, int, int>(), "Constructor with rows, cols, type", 
             py::arg("rows"), py::arg("cols"), py::arg("type"))
        .def("rows", [](const cv::Mat& self) { return self.rows; }, "Get number of rows")
        .def("cols", [](const cv::Mat& self) { return self.cols; }, "Get number of cols")
        .def("channels", [](const cv::Mat& self) { return self.channels(); }, "Get number of channels")
        .def("empty", &cv::Mat::empty, "Check if matrix is empty")
        .def("__str__", [](const cv::Mat& self) {
            return "Mat(" + std::to_string(self.rows) + "x" + std::to_string(self.cols) + 
                   ", channels=" + std::to_string(self.channels()) + ")";
        });
    
    // Register cv::MatExpr type
    py::class_<cv::MatExpr>(m, "MatExpr")
        .def("__str__", [](const cv::MatExpr& self) {
            return "MatExpr()";
        });
    
    // Register OpenCV enums
    py::enum_<cv::InterpolationFlags>(m, "InterpolationFlags")
        .value("INTER_NEAREST", cv::INTER_NEAREST)
        .value("INTER_LINEAR", cv::INTER_LINEAR)
        .value("INTER_CUBIC", cv::INTER_CUBIC)
        .value("INTER_AREA", cv::INTER_AREA)
        .value("INTER_LANCZOS4", cv::INTER_LANCZOS4)
        .value("INTER_LINEAR_EXACT", cv::INTER_LINEAR_EXACT)
        .value("INTER_NEAREST_EXACT", cv::INTER_NEAREST_EXACT)
        .value("INTER_MAX", cv::INTER_MAX)
        .value("WARP_FILL_OUTLIERS", cv::WARP_FILL_OUTLIERS)
        .value("WARP_INVERSE_MAP", cv::WARP_INVERSE_MAP);
    
    // Initialize all class bindings
    init_transform(m);
    init_camera_model(m);
    init_sensor_data(m);
    init_parameters(m);
    init_statistics(m);
    init_rtabmap(m);
    
    // Module-level constants
    m.attr("__version__") = RTABMAP_VERSION;
    m.attr("RTABMAP_MAJOR_VERSION") = RTABMAP_VERSION_MAJOR;
    m.attr("RTABMAP_MINOR_VERSION") = RTABMAP_VERSION_MINOR;
    m.attr("RTABMAP_PATCH_VERSION") = RTABMAP_VERSION_PATCH;
}
