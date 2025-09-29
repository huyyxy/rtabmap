#!/usr/bin/env python3
"""
Setup script for RTAB-Map Python Wrapper.
"""

from setuptools import setup, find_packages
import os

# Read README for long description
def read_readme():
    with open("README.md", "r", encoding="utf-8") as fh:
        return fh.read()

# Read requirements
def read_requirements():
    requirements = []
    if os.path.exists("requirements.txt"):
        with open("requirements.txt", "r") as fh:
            requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]
    return requirements

setup(
    name="rtabmap-python",
    version="0.1.0",
    author="RTAB-Map Python Wrapper",
    author_email="",
    description="Python wrapper for RTAB-Map RGB-D SLAM library",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/your-repo/rtabmap-python",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
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
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Image Recognition",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
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
    entry_points={
        "console_scripts": [
            "rtabmap-python-example=examples.basic_slam_example:main",
        ],
    },
    include_package_data=True,
    package_data={
        "rtabmap_python": ["*.md", "*.txt"],
    },
    keywords=[
        "slam", "rtabmap", "robotics", "computer-vision", 
        "rgb-d", "mapping", "localization", "opencv"
    ],
    project_urls={
        "Bug Reports": "https://github.com/your-repo/rtabmap-python/issues",
        "Source": "https://github.com/your-repo/rtabmap-python",
        "Documentation": "https://github.com/your-repo/rtabmap-python/blob/main/README.md",
        "Original RTAB-Map": "https://github.com/introlab/rtabmap",
    },
)
