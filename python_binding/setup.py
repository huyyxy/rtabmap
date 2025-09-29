#!/usr/bin/env python3
"""
Setup script for RTAB-Map Python bindings using pybind11.
"""

import os
import sys
import subprocess
import platform
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11
from setuptools import setup, Extension

# The main interface is through Pybind11Extension.
# * You can add cxx_std=14/17/20, but it may not be portable to old GCC.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)

# Get RTAB-Map installation paths
def find_rtabmap_paths():
    """Find RTAB-Map installation paths."""
    rtabmap_root = Path(__file__).parent.parent.absolute()
    
    # Try to find RTAB-Map installation
    include_dirs = []
    library_dirs = []
    libraries = []
    
    # Check if we're in the RTAB-Map source tree
    if (rtabmap_root / "corelib" / "include" / "rtabmap").exists():
        print("Found RTAB-Map source tree")
        include_dirs.append(str(rtabmap_root / "corelib" / "include"))
        include_dirs.append(str(rtabmap_root / "utilite" / "include"))
        
        # Look for build directory
        build_dir = rtabmap_root / "build"
        if build_dir.exists():
            library_dirs.append(str(build_dir / "bin"))
            # Common RTAB-Map libraries
            libraries.extend(['rtabmap_core', 'rtabmap_utilite'])
        else:
            print("Warning: RTAB-Map build directory not found. Please build RTAB-Map first.")
    else:
        # Try to find system-installed RTAB-Map
        print("Looking for system-installed RTAB-Map...")
        try:
            # Use pkg-config if available
            result = subprocess.run(['pkg-config', '--cflags', '--libs', 'rtabmap'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                # Parse pkg-config output
                flags = result.stdout.split()
                for flag in flags:
                    if flag.startswith('-I'):
                        include_dirs.append(flag[2:])
                    elif flag.startswith('-L'):
                        library_dirs.append(flag[2:])
                    elif flag.startswith('-l'):
                        libraries.append(flag[2:])
        except FileNotFoundError:
            print("pkg-config not found")
        
        # Fallback to common installation paths
        if not include_dirs:
            common_include_paths = [
                '/usr/local/include',
                '/usr/include',
                '/opt/ros/*/include',  # ROS installation
            ]
            for path_pattern in common_include_paths:
                if '*' in path_pattern:
                    # Handle glob patterns
                    import glob
                    for path in glob.glob(path_pattern):
                        if (Path(path) / 'rtabmap').exists():
                            include_dirs.append(path)
                            break
                else:
                    if (Path(path_pattern) / 'rtabmap').exists():
                        include_dirs.append(path_pattern)
                        break
        
        if not library_dirs:
            common_lib_paths = [
                '/usr/local/lib',
                '/usr/lib',
                '/usr/lib/x86_64-linux-gnu',  # Ubuntu
                '/opt/ros/*/lib',  # ROS installation
            ]
            for path_pattern in common_lib_paths:
                if '*' in path_pattern:
                    import glob
                    for path in glob.glob(path_pattern):
                        if Path(path).exists():
                            library_dirs.append(path)
                            break
                else:
                    if Path(path_pattern).exists():
                        library_dirs.append(path_pattern)
                        break
        
        if not libraries:
            libraries.extend(['rtabmap_core', 'rtabmap_utilite'])
    
    return include_dirs, library_dirs, libraries

def find_opencv_paths():
    """Find OpenCV installation paths."""
    include_dirs = []
    library_dirs = []
    libraries = []
    
    try:
        # Use pkg-config for OpenCV
        result = subprocess.run(['pkg-config', '--cflags', '--libs', 'opencv4'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            result = subprocess.run(['pkg-config', '--cflags', '--libs', 'opencv'], 
                                  capture_output=True, text=True)
        
        if result.returncode == 0:
            flags = result.stdout.split()
            for flag in flags:
                if flag.startswith('-I'):
                    include_dirs.append(flag[2:])
                elif flag.startswith('-L'):
                    library_dirs.append(flag[2:])
                elif flag.startswith('-l'):
                    libraries.append(flag[2:])
    except FileNotFoundError:
        print("pkg-config not found, using fallback OpenCV detection")
    
    # Fallback detection
    if not include_dirs:
        common_opencv_paths = [
            '/usr/local/include/opencv4',
            '/usr/include/opencv4',
            '/usr/local/include/opencv2',
            '/usr/include/opencv2',
        ]
        for path in common_opencv_paths:
            if Path(path).exists():
                include_dirs.append(path)
                break
    
    if not libraries:
        # Common OpenCV libraries
        libraries.extend(['opencv_core', 'opencv_imgproc', 'opencv_imgcodecs', 
                         'opencv_features2d', 'opencv_calib3d'])
    
    return include_dirs, library_dirs, libraries

# Get paths
rtabmap_includes, rtabmap_lib_dirs, rtabmap_libs = find_rtabmap_paths()
opencv_includes, opencv_lib_dirs, opencv_libs = find_opencv_paths()

# Combine all paths
all_include_dirs = rtabmap_includes + opencv_includes
all_library_dirs = rtabmap_lib_dirs + opencv_lib_dirs
all_libraries = rtabmap_libs + opencv_libs

print(f"Include directories: {all_include_dirs}")
print(f"Library directories: {all_library_dirs}")
print(f"Libraries: {all_libraries}")

# Source files
source_files = [
    "src/rtabmap_python.cpp",
    "src/rtabmap_binding.cpp", 
    "src/transform_binding.cpp",
    "src/sensor_data_binding.cpp",
    "src/camera_model_binding.cpp",
    "src/parameters_binding.cpp",
    "src/statistics_binding.cpp",
]

# Compiler flags
cxx_std = 14
compile_args = []
link_args = []

if platform.system() == "Darwin":  # macOS
    compile_args.extend(["-stdlib=libc++", "-mmacosx-version-min=10.9"])
    link_args.extend(["-stdlib=libc++", "-mmacosx-version-min=10.9"])
elif platform.system() == "Linux":
    compile_args.extend(["-fPIC"])

# Create extension
ext_modules = [
    Pybind11Extension(
        "rtabmap_python",
        source_files,
        include_dirs=all_include_dirs + [
            # Path to pybind11 headers
            pybind11.get_include(),
        ],
        library_dirs=all_library_dirs,
        libraries=all_libraries,
        language='c++',
        cxx_std=cxx_std,
        define_macros=[
            ("VERSION_INFO", '"dev"'),
            ("PYBIND11_DETAILED_ERROR_MESSAGES", None),
        ],
        extra_compile_args=compile_args,
        extra_link_args=link_args,
    ),
]

setup(
    name="rtabmap-python",
    version="0.23.1",
    author="RTAB-Map Python Bindings",
    author_email="",
    url="https://github.com/introlab/rtabmap",
    description="Python bindings for RTAB-Map RGB-D SLAM library",
    long_description=open("README.md").read() if os.path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "numpy>=1.19.0",
        "opencv-python>=4.5.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov",
            "black",
            "flake8",
            "mypy",
        ],
        "examples": [
            "matplotlib>=3.3.0",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Image Recognition",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords=[
        "slam", "rtabmap", "robotics", "computer-vision", 
        "rgb-d", "mapping", "localization", "opencv", "pybind11"
    ],
    project_urls={
        "Bug Reports": "https://github.com/introlab/rtabmap/issues",
        "Source": "https://github.com/introlab/rtabmap",
        "Documentation": "http://introlab.github.io/rtabmap",
    },
)
