/**
 * Python bindings for rtabmap::Transform class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rtabmap/core/Transform.h>
#include <opencv2/opencv.hpp>

namespace py = pybind11;

void init_transform(py::module &m) {
    py::class_<rtabmap::Transform>(m, "Transform")
        .def(py::init<>(), "Default constructor (identity transform)")
        .def(py::init<float, float, float, float, float, float>(),
             "Construct from position and Euler angles (x,y,z,roll,pitch,yaw)",
             py::arg("x"), py::arg("y"), py::arg("z"), 
             py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
        .def(py::init<const cv::Mat&>(),
             "Construct from 4x4 transformation matrix",
             py::arg("matrix"))
        
        // Static factory methods
        .def_static("getIdentity", &rtabmap::Transform::getIdentity,
                   "Get identity transform")
        .def_static("fromEigen4d", [](const Eigen::Matrix4d& matrix) {
            cv::Mat mat(4, 4, CV_64FC1);
            for(int i = 0; i < 4; ++i) {
                for(int j = 0; j < 4; ++j) {
                    mat.at<double>(i, j) = matrix(i, j);
                }
            }
            return rtabmap::Transform(mat);
        }, "Create Transform from Eigen 4x4 matrix")
        
        // Access methods
        .def("x", &rtabmap::Transform::x, "Get x translation")
        .def("y", &rtabmap::Transform::y, "Get y translation") 
        .def("z", &rtabmap::Transform::z, "Get z translation")
        .def("roll", &rtabmap::Transform::roll, "Get roll rotation (radians)")
        .def("pitch", &rtabmap::Transform::pitch, "Get pitch rotation (radians)")
        .def("yaw", &rtabmap::Transform::yaw, "Get yaw rotation (radians)")
        
        // Matrix operations
        .def("data", [](const rtabmap::Transform& self) -> py::array_t<double> {
            cv::Mat mat = self.data();
            if(mat.empty()) {
                return py::array_t<double>();
            }
            return py::array_t<double>(
                {4, 4},
                {sizeof(double) * 4, sizeof(double)},
                (double*)mat.data
            );
        }, "Get 4x4 transformation matrix as numpy array")
        
        .def("dataMatrix", [](const rtabmap::Transform& self) -> Eigen::Matrix4d {
            cv::Mat mat = self.data();
            Eigen::Matrix4d eigen_mat;
            if(!mat.empty()) {
                for(int i = 0; i < 4; ++i) {
                    for(int j = 0; j < 4; ++j) {
                        eigen_mat(i, j) = mat.at<double>(i, j);
                    }
                }
            } else {
                eigen_mat = Eigen::Matrix4d::Identity();
            }
            return eigen_mat;
        }, "Get 4x4 transformation matrix as Eigen matrix")
        
        // Transformation operations
        .def("inverse", &rtabmap::Transform::inverse, "Get inverse transform")
        .def("rotation", &rtabmap::Transform::rotation, "Get rotation matrix")
        .def("translation", &rtabmap::Transform::translation, "Get translation vector")
        
        // Utility methods
        .def("isNull", &rtabmap::Transform::isNull, "Check if transform is null")
        .def("isIdentity", &rtabmap::Transform::isIdentity, "Check if transform is identity")
        .def("getNorm", &rtabmap::Transform::getNorm, "Get translation norm")
        .def("getNormSquared", &rtabmap::Transform::getNormSquared, "Get squared translation norm")
        .def("getDistance", &rtabmap::Transform::getDistance, 
             "Get distance to another transform", py::arg("other"))
        .def("getDistanceSquared", &rtabmap::Transform::getDistanceSquared,
             "Get squared distance to another transform", py::arg("other"))
        
        // Operators
        .def("__mul__", &rtabmap::Transform::operator*, "Transform multiplication")
        .def("__eq__", &rtabmap::Transform::operator==, "Transform equality")
        .def("__ne__", &rtabmap::Transform::operator!=, "Transform inequality")
        
        // String representation
        .def("__str__", [](const rtabmap::Transform& self) {
            return self.prettyPrint();
        })
        .def("__repr__", [](const rtabmap::Transform& self) {
            return "Transform(" + self.prettyPrint() + ")";
        })
        
        // Conversion methods
        .def("toEigen3d", [](const rtabmap::Transform& self) -> Eigen::Isometry3d {
            // Convert to Eigen::Isometry3d for compatibility with other libraries
            cv::Mat mat = self.data();
            Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
            if(!mat.empty()) {
                for(int i = 0; i < 3; ++i) {
                    for(int j = 0; j < 3; ++j) {
                        iso.linear()(i, j) = mat.at<double>(i, j);
                    }
                    iso.translation()(i) = mat.at<double>(i, 3);
                }
            }
            return iso;
        }, "Convert to Eigen::Isometry3d")
        
        .def("toEigen4f", [](const rtabmap::Transform& self) -> Eigen::Matrix4f {
            cv::Mat mat = self.data();
            Eigen::Matrix4f eigen_mat;
            if(!mat.empty()) {
                for(int i = 0; i < 4; ++i) {
                    for(int j = 0; j < 4; ++j) {
                        eigen_mat(i, j) = static_cast<float>(mat.at<double>(i, j));
                    }
                }
            } else {
                eigen_mat = Eigen::Matrix4f::Identity();
            }
            return eigen_mat;
        }, "Get 4x4 transformation matrix as Eigen Matrix4f")
        
        // Interpolation
        .def("interpolate", &rtabmap::Transform::interpolate,
             "Interpolate between two transforms",
             py::arg("t"), py::arg("other"))
        
        // Clone method
        .def("clone", [](const rtabmap::Transform& self) {
            return rtabmap::Transform(self);
        }, "Create a copy of the transform");
}
