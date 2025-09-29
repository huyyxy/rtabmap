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
    
    // Initialize all class bindings
    init_transform(m);
    init_camera_model(m);
    init_sensor_data(m);
    init_parameters(m);
    init_statistics(m);
    init_rtabmap(m);
    
    // Module-level constants
    m.attr("__version__") = "0.23.1";
    m.attr("RTABMAP_MAJOR_VERSION") = 0;
    m.attr("RTABMAP_MINOR_VERSION") = 23;
    m.attr("RTABMAP_PATCH_VERSION") = 1;
}
