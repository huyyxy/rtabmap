/**
 * Python bindings for rtabmap::Transform class.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rtabmap-0.21/rtabmap/core/Transform.h>
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
        .def_static("fromEigen4d", &rtabmap::Transform::fromEigen4d, "Create Transform from Eigen 4x4 matrix")
        
        // Access methods
        .def("x", [](const rtabmap::Transform& self) -> float {
            return self.x();
        }, "Get x translation")
        .def("y", [](const rtabmap::Transform& self) -> float {
            return self.y();
        }, "Get y translation") 
        .def("z", [](const rtabmap::Transform& self) -> float {
            return self.z();
        }, "Get z translation")
        .def("roll", [](const rtabmap::Transform& self) -> float {
            float roll, pitch, yaw;
            self.getEulerAngles(roll, pitch, yaw);
            return roll;
        }, "Get roll rotation (radians)")
        .def("pitch", [](const rtabmap::Transform& self) -> float {
            float roll, pitch, yaw;
            self.getEulerAngles(roll, pitch, yaw);
            return pitch;
        }, "Get pitch rotation (radians)")
        .def("yaw", [](const rtabmap::Transform& self) -> float {
            float roll, pitch, yaw;
            self.getEulerAngles(roll, pitch, yaw);
            return yaw;
        }, "Get yaw rotation (radians)")
        
        // Matrix operations
        .def("data", [](const rtabmap::Transform& self) -> py::array_t<float> {
            const float* data = self.data();
            if(self.isNull()) {
                return py::array_t<float>();
            }
            // Create a 4x4 matrix from the 3x4 data
            std::vector<float> matrix_data(16);
            for(int i = 0; i < 3; ++i) {
                for(int j = 0; j < 4; ++j) {
                    matrix_data[i*4 + j] = data[i*4 + j];
                }
            }
            // Add the last row [0, 0, 0, 1]
            matrix_data[12] = 0.0f; matrix_data[13] = 0.0f; matrix_data[14] = 0.0f; matrix_data[15] = 1.0f;
            
            return py::array_t<float>(
                {4, 4},
                {sizeof(float) * 4, sizeof(float)},
                matrix_data.data()
            );
        }, "Get 4x4 transformation matrix as numpy array")
        
        .def("dataMatrix", [](const rtabmap::Transform& self) -> Eigen::Matrix4d {
            return self.toEigen4d();
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
            Eigen::Affine3d affine = self.toEigen3d();
            Eigen::Isometry3d iso;
            iso.linear() = affine.linear();
            iso.translation() = affine.translation();
            return iso;
        }, "Convert to Eigen::Isometry3d")
        
        .def("toEigen4f", [](const rtabmap::Transform& self) -> Eigen::Matrix4f {
            return self.toEigen4f();
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
