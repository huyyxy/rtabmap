/**
 * Python bindings for rtabmap::CameraModel and rtabmap::StereoCameraModel classes.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rtabmap-0.21/rtabmap/core/CameraModel.h>
#include <rtabmap-0.21/rtabmap/core/StereoCameraModel.h>
#include <opencv2/opencv.hpp>

namespace py = pybind11;

void init_camera_model(py::module &m) {
    // CameraModel class
    py::class_<rtabmap::CameraModel>(m, "CameraModel")
        .def(py::init<>(), "Default constructor")
        .def(py::init<const std::string&, const cv::Size&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const rtabmap::Transform&>(),
             "Full constructor with calibration parameters",
             py::arg("name"), py::arg("image_size"), py::arg("K"), py::arg("D"), py::arg("R"), py::arg("P"),
             py::arg("local_transform") = rtabmap::CameraModel::opticalRotation())
        .def(py::init<double, double, double, double, const rtabmap::Transform&, double, const cv::Size&>(),
             "Minimal constructor with focal length and principal point",
             py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
             py::arg("local_transform") = rtabmap::CameraModel::opticalRotation(),
             py::arg("Tx") = 0.0, py::arg("image_size") = cv::Size(0,0))
        .def(py::init<const std::string&, double, double, double, double, const rtabmap::Transform&, double, const cv::Size&>(),
             "Named minimal constructor",
             py::arg("name"), py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
             py::arg("local_transform") = rtabmap::CameraModel::opticalRotation(),
             py::arg("Tx") = 0.0, py::arg("image_size") = cv::Size(0,0))
        
        // Factory methods
        .def_static("fromEigen", [](const Eigen::Matrix3d& K, 
                                   const Eigen::VectorXd& D,
                                   const cv::Size& image_size,
                                   const std::string& name = "") {
            cv::Mat K_mat(3, 3, CV_64FC1);
            cv::Mat D_mat(1, D.size(), CV_64FC1);
            
            for(int i = 0; i < 3; ++i) {
                for(int j = 0; j < 3; ++j) {
                    K_mat.at<double>(i, j) = K(i, j);
                }
            }
            
            for(int i = 0; i < D.size(); ++i) {
                D_mat.at<double>(0, i) = D(i);
            }
            
            // Create identity matrices for R and P
            cv::Mat R_mat = cv::Mat::eye(3, 3, CV_64FC1);
            cv::Mat P_mat = cv::Mat::eye(3, 4, CV_64FC1);
            K_mat.copyTo(P_mat.colRange(0, 3));
            
            return rtabmap::CameraModel(name, image_size, K_mat, D_mat, R_mat, P_mat);
        }, "Create CameraModel from Eigen matrices",
           py::arg("K"), py::arg("D"), py::arg("image_size"), py::arg("name") = "")
        
        // Accessors
        .def("name", &rtabmap::CameraModel::name, "Get camera name")
        .def("imageSize", &rtabmap::CameraModel::imageSize, "Get image size")
        .def("imageWidth", &rtabmap::CameraModel::imageWidth, "Get image width")
        .def("imageHeight", &rtabmap::CameraModel::imageHeight, "Get image height")
        
        .def("fx", &rtabmap::CameraModel::fx, "Get focal length x")
        .def("fy", &rtabmap::CameraModel::fy, "Get focal length y")
        .def("cx", &rtabmap::CameraModel::cx, "Get principal point x")
        .def("cy", &rtabmap::CameraModel::cy, "Get principal point y")
        
        .def("K_raw", [](const rtabmap::CameraModel& self) -> Eigen::Matrix3d {
            cv::Mat K = self.K_raw();
            Eigen::Matrix3d K_eigen;
            for(int i = 0; i < 3; ++i) {
                for(int j = 0; j < 3; ++j) {
                    K_eigen(i, j) = K.at<double>(i, j);
                }
            }
            return K_eigen;
        }, "Get intrinsic camera matrix K as Eigen matrix")
        
        .def("D_raw", [](const rtabmap::CameraModel& self) -> py::array_t<double> {
            cv::Mat D = self.D_raw();
            if(D.empty()) {
                return py::array_t<double>();
            }
            return py::array_t<double>(
                {D.rows, D.cols},
                {sizeof(double) * D.cols, sizeof(double)},
                (double*)D.data
            );
        }, "Get distortion coefficients as numpy array")
        
        .def("localTransform", &rtabmap::CameraModel::localTransform,
             "Get local transform", py::return_value_policy::reference_internal)
        
        // Utility methods
        .def("isValidForProjection", &rtabmap::CameraModel::isValidForProjection,
             "Check if valid for projection operations")
        .def("isValidForReprojection", &rtabmap::CameraModel::isValidForReprojection,
             "Check if valid for reprojection operations")
        .def("isValidForRectification", &rtabmap::CameraModel::isValidForRectification,
             "Check if rectification is needed")
        
        // Projection methods
        .def("project", [](const rtabmap::CameraModel& self, float u, float v, float depth) {
            float x, y, z;
            self.project(u, v, depth, x, y, z);
            return py::make_tuple(x, y, z);
        }, "Project 2D pixel to 3D point", py::arg("u"), py::arg("v"), py::arg("depth"))
        
        .def("reproject", [](const rtabmap::CameraModel& self, float x, float y, float z) {
            float u, v;
            self.reproject(x, y, z, u, v);
            return py::make_tuple(u, v);
        }, "Reproject 3D point to 2D pixel", py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def("reprojectToInt", [](const rtabmap::CameraModel& self, float x, float y, float z) {
            int u, v;
            self.reproject(x, y, z, u, v);
            return py::make_tuple(u, v);
        }, "Reproject 3D point to 2D pixel (integer coordinates)", py::arg("x"), py::arg("y"), py::arg("z"))
        
        // Image rectification
        .def("rectifyImage", [](const rtabmap::CameraModel& self, const cv::Mat& raw_image, int interpolation) {
            return self.rectifyImage(raw_image, interpolation);
        }, "Rectify image using calibration parameters", 
           py::arg("raw_image"), py::arg("interpolation") = cv::INTER_LINEAR)
        
        // Scaling
        .def("scaled", &rtabmap::CameraModel::scaled,
             "Get scaled camera model", py::arg("scale"))
        .def("roi", &rtabmap::CameraModel::roi,
             "Get camera model for ROI", py::arg("roi"))
        
        // Setters
        .def("setName", &rtabmap::CameraModel::setName, "Set camera name", py::arg("name"))
        .def("setImageSize", &rtabmap::CameraModel::setImageSize, "Set image size", py::arg("size"))
        .def("setLocalTransform", &rtabmap::CameraModel::setLocalTransform,
             "Set local transform", py::arg("transform"))
        
        // String representation
        .def("__str__", [](const rtabmap::CameraModel& self) {
            std::stringstream ss;
            ss << "CameraModel(name='" << self.name() << "', ";
            ss << "size=" << self.imageWidth() << "x" << self.imageHeight() << ", ";
            ss << "fx=" << self.fx() << ", fy=" << self.fy() << ", ";
            ss << "cx=" << self.cx() << ", cy=" << self.cy() << ")";
            return ss.str();
        })
        
        .def("__repr__", [](const rtabmap::CameraModel& self) {
            return "CameraModel('" + self.name() + "')";
        });
    
    // StereoCameraModel class
    py::class_<rtabmap::StereoCameraModel>(m, "StereoCameraModel")
        .def(py::init<>(), "Default constructor")
        .def(py::init<const std::string&, const cv::Size&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&,
                      const cv::Size&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&,
                      const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const rtabmap::Transform&>(),
             "Full constructor with stereo calibration",
             py::arg("name"), py::arg("image_size1"), 
             py::arg("K1"), py::arg("D1"), py::arg("R1"), py::arg("P1"),
             py::arg("image_size2"), py::arg("K2"), py::arg("D2"), py::arg("R2"), py::arg("P2"),
             py::arg("R"), py::arg("T"), py::arg("E"), py::arg("F"),
             py::arg("local_transform") = rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0))
        .def(py::init<double, double, double, double, double, const rtabmap::Transform&, const cv::Size&>(),
             "Simplified constructor",
             py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"), py::arg("baseline"),
             py::arg("local_transform") = rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
             py::arg("image_size") = cv::Size(0,0))
        .def(py::init<const std::string&, double, double, double, double, double, const rtabmap::Transform&, const cv::Size&>(),
             "Named simplified constructor",
             py::arg("name"), py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"), py::arg("baseline"),
             py::arg("local_transform") = rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
             py::arg("image_size") = cv::Size(0,0))
        .def(py::init<const std::string&, const rtabmap::CameraModel&, const rtabmap::CameraModel&, 
                      const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&>(),
             "Constructor with camera models and stereo matrices",
             py::arg("name"), py::arg("left_camera"), py::arg("right_camera"),
             py::arg("R") = cv::Mat(), py::arg("T") = cv::Mat(), py::arg("E") = cv::Mat(), py::arg("F") = cv::Mat())
        .def(py::init<const std::string&, const rtabmap::CameraModel&, const rtabmap::CameraModel&, const rtabmap::Transform&>(),
             "Constructor with camera models and extrinsics transform",
             py::arg("name"), py::arg("left_camera"), py::arg("right_camera"), py::arg("extrinsics"))
        
        // Accessors
        .def("left", &rtabmap::StereoCameraModel::left,
             "Get left camera model", py::return_value_policy::reference_internal)
        .def("right", &rtabmap::StereoCameraModel::right,
             "Get right camera model", py::return_value_policy::reference_internal)
        .def("baseline", &rtabmap::StereoCameraModel::baseline, "Get stereo baseline")
        
        .def("R", [](const rtabmap::StereoCameraModel& self) -> Eigen::Matrix3d {
            cv::Mat R = self.R();
            Eigen::Matrix3d R_eigen;
            for(int i = 0; i < 3; ++i) {
                for(int j = 0; j < 3; ++j) {
                    R_eigen(i, j) = R.at<double>(i, j);
                }
            }
            return R_eigen;
        }, "Get rotation matrix between cameras")
        
        .def("T", [](const rtabmap::StereoCameraModel& self) -> Eigen::Vector3d {
            cv::Mat T = self.T();
            Eigen::Vector3d T_eigen;
            for(int i = 0; i < 3; ++i) {
                T_eigen(i) = T.at<double>(i, 0);
            }
            return T_eigen;
        }, "Get translation vector between cameras")
        
        // Utility methods
        .def("isValidForProjection", &rtabmap::StereoCameraModel::isValidForProjection,
             "Check if valid for projection")
        .def("isValidForRectification", &rtabmap::StereoCameraModel::isValidForRectification,
             "Check if valid for rectification")
        
        // Stereo operations
        .def("computeDepth", [](const rtabmap::StereoCameraModel& self, float disparity) {
            return self.computeDepth(disparity);
        }, "Compute depth from disparity", py::arg("disparity"))
        
        .def("computeDisparity", [](const rtabmap::StereoCameraModel& self, float depth) {
            return self.computeDisparity(depth);
        }, "Compute disparity from depth", py::arg("depth"))
        
        // Scaling
        .def("scale", [](rtabmap::StereoCameraModel& self, double scale) {
            self.scale(scale);
        }, "Scale stereo camera model", py::arg("scale"))
        
        .def("roi", [](rtabmap::StereoCameraModel& self, const cv::Rect& roi) {
            self.roi(roi);
        }, "Apply ROI to stereo camera model", py::arg("roi"))
        
        // String representation
        .def("__str__", [](const rtabmap::StereoCameraModel& self) {
            std::stringstream ss;
            ss << "StereoCameraModel(name='" << self.left().name() << "', ";
            ss << "baseline=" << self.baseline() << ", ";
            ss << "size=" << self.left().imageWidth() << "x" << self.left().imageHeight() << ")";
            return ss.str();
        })
        
        .def("__repr__", [](const rtabmap::StereoCameraModel& self) {
            return "StereoCameraModel('" + self.left().name() + "')";
        });
}
