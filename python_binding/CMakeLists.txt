cmake_minimum_required(VERSION 3.14)
project(rtabmap_python)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(pybind11 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# Find RTAB-Map
find_package(RTABMap REQUIRED)
if(NOT RTABMap_FOUND)
    # Try to find it manually if not found via find_package
    set(RTABMap_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
    find_path(RTABMap_INCLUDE_DIRS
        NAMES rtabmap/core/Rtabmap.h
        PATHS ${RTABMap_ROOT_DIR}/corelib/include
        NO_DEFAULT_PATH
    )
    
    find_library(RTABMap_LIBRARIES
        NAMES rtabmap_core
        PATHS ${RTABMap_ROOT_DIR}/build/bin
        NO_DEFAULT_PATH
    )
    
    if(RTABMap_INCLUDE_DIRS AND RTABMap_LIBRARIES)
        set(RTABMap_FOUND TRUE)
        message(STATUS "Found RTAB-Map manually: ${RTABMap_LIBRARIES}")
    else()
        message(FATAL_ERROR "RTAB-Map not found! Please build RTAB-Map first.")
    endif()
endif()

# Include directories
include_directories(${RTABMap_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

# Add PCL compile definitions
add_definitions(${PCL_DEFINITIONS})

# Source files
set(SOURCES
    src/rtabmap_python.cpp
    src/rtabmap_binding.cpp
    src/transform_binding.cpp
    src/sensor_data_binding.cpp
    src/camera_model_binding.cpp
    src/parameters_binding.cpp
    src/statistics_binding.cpp
)

# Create pybind11 module
pybind11_add_module(rtabmap_python ${SOURCES})

# Link libraries
target_link_libraries(rtabmap_python PRIVATE 
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# Set properties
target_compile_definitions(rtabmap_python PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# Installation
install(TARGETS rtabmap_python DESTINATION .)
