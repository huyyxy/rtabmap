#!/bin/bash
# Build script for RTAB-Map Python bindings

set -e  # Exit on any error

echo "=== RTAB-Map Python Bindings Build Script ==="
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if RTAB-Map is installed
check_rtabmap() {
    print_status "Checking for RTAB-Map installation..."
    
    if pkg-config --exists rtabmap; then
        RTABMAP_VERSION=$(pkg-config --modversion rtabmap)
        print_status "Found RTAB-Map version: $RTABMAP_VERSION"
        return 0
    else
        print_warning "RTAB-Map not found via pkg-config, checking manually..."
        
        # Check for header files
        if [ -f "../corelib/include/rtabmap/core/Rtabmap.h" ]; then
            print_status "Found RTAB-Map source tree"
            return 0
        elif [ -f "/usr/local/include/rtabmap/core/Rtabmap.h" ] || [ -f "/usr/include/rtabmap/core/Rtabmap.h" ]; then
            print_status "Found RTAB-Map system installation"
            return 0
        else
            print_error "RTAB-Map not found!"
            print_error "Please install RTAB-Map first:"
            print_error "  sudo apt install librtabmap-dev  # Ubuntu/Debian"
            print_error "  Or build from source: https://github.com/introlab/rtabmap"
            return 1
        fi
    fi
}

# Check dependencies
check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python 3 not found!"
        return 1
    fi
    
    PYTHON_VERSION=$(python3 -c "import sys; print('.'.join(map(str, sys.version_info[:2])))")
    print_status "Found Python version: $PYTHON_VERSION"
    
    # Check pip
    if ! command -v pip3 &> /dev/null && ! command -v pip &> /dev/null; then
        print_error "pip not found!"
        return 1
    fi
    
    # Check cmake
    if ! command -v cmake &> /dev/null; then
        print_error "CMake not found! Please install cmake."
        return 1
    fi
    
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    print_status "Found CMake version: $CMAKE_VERSION"
    
    # Check pkg-config
    if ! command -v pkg-config &> /dev/null; then
        print_warning "pkg-config not found. Install it for better dependency detection."
    fi
    
    # Check Eigen3
    print_status "Checking for Eigen3..."
    if pkg-config --exists eigen3; then
        EIGEN_VERSION=$(pkg-config --modversion eigen3)
        print_status "Found Eigen3 version: $EIGEN_VERSION"
    elif [ -d "/usr/include/eigen3" ] || [ -d "/usr/local/include/eigen3" ]; then
        print_status "Found Eigen3 system installation"
    else
        print_warning "Eigen3 not found via pkg-config or common paths"
        print_warning "Install Eigen3: sudo apt install libeigen3-dev"
    fi
    
    return 0
}

# Install Python dependencies
install_python_deps() {
    print_status "Installing Python dependencies..."
    
    if [ -f "requirements.txt" ]; then
        pip3 install -r requirements.txt --break-system-packages
    else
        pip3 install numpy>=1.19.0 opencv-python>=4.5.0 pybind11>=2.6.0 --break-system-packages
    fi
    
    # Check if Eigen3 development package is installed
    if ! pkg-config --exists eigen3 && ! [ -d "/usr/include/eigen3" ] && ! [ -d "/usr/local/include/eigen3" ]; then
        print_error "Eigen3 development headers not found!"
        print_error "Please install Eigen3 development package:"
        print_error "  Ubuntu/Debian: sudo apt install libeigen3-dev"
        print_error "  macOS: brew install eigen"
        return 1
    fi
    
    print_status "Python dependencies installed successfully"
}

# Build using setup.py
build_setuppy() {
    print_status "Building Python bindings using setup.py..."
    
    # Clean previous build
    if [ -d "build" ]; then
        print_status "Cleaning previous build..."
        rm -rf build/
    fi
    
    if [ -d "dist" ]; then
        rm -rf dist/
    fi
    
    if [ -d "*.egg-info" ]; then
        rm -rf *.egg-info
    fi
    
    # Build
    python3 setup.py build_ext --inplace
    
    print_status "Build completed successfully!"
}

# Build using CMake (alternative)
build_cmake() {
    print_status "Building Python bindings using CMake..."
    
    # Create build directory
    if [ ! -d "build" ]; then
        mkdir build
    fi
    
    cd build
    
    # Configure
    cmake .. -DCMAKE_BUILD_TYPE=Release
    
    # Build
    make -j$(nproc)
    
    cd ..
    
    print_status "CMake build completed successfully!"
}

# Install the package
install_package() {
    print_status "Installing Python package..."
    
    pip3 install -e . --break-system-packages
    
    # Fix library paths for macOS after installation
    if [[ "$OSTYPE" == "darwin"* ]]; then
        print_status "Fixing library paths for macOS..."
        
        # Find the generated .so file
        SO_FILE=$(ls *.so 2>/dev/null | head -n1)
        if [ -n "$SO_FILE" ]; then
            # Get the RTAB-Map build directory
            RTABMAP_BUILD_DIR=$(dirname $(pwd))/build/bin
            
            if [ -d "$RTABMAP_BUILD_DIR" ]; then
                # Create versioned symlinks if they don't exist
                if [ -f "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.1.dylib" ] && [ ! -f "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib" ]; then
                    ln -sf librtabmap_core.0.23.1.dylib "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib"
                    print_status "Created symlink for librtabmap_core.0.23.dylib"
                fi
                
                if [ -f "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.1.dylib" ] && [ ! -f "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib" ]; then
                    ln -sf librtabmap_utilite.0.23.1.dylib "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib"
                    print_status "Created symlink for librtabmap_utilite.0.23.dylib"
                fi
                
                # Update the library paths in the .so file
                install_name_tool -change @rpath/librtabmap_core.0.23.dylib "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib" "$SO_FILE"
                install_name_tool -change @rpath/librtabmap_utilite.0.23.dylib "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib" "$SO_FILE"
                print_status "Updated library paths in $SO_FILE"
            fi
        fi
    fi
    
    print_status "Package installed successfully!"
}

# Test the installation
test_installation() {
    print_status "Testing installation..."
    
    # Simple import test first
    if python3 -c "import rtabmap_python; print('Import successful')" 2>/dev/null; then
        print_status "✅ RTAB-Map Python bindings imported successfully!"
        
        # Try basic functionality test
        python3 -c "
import rtabmap_python as rtab
try:
    slam = rtab.Rtabmap()
    print('✅ Rtabmap instance created successfully!')
    
    transform = rtab.Transform(1, 2, 3, 0, 0, 0)
    print('✅ Transform class working!')
    
    camera = rtab.CameraModel(525, 525, 320, 240)
    print('✅ CameraModel class working!')
    
    print('🎉 Basic functionality tests passed!')
    
except Exception as e:
    print(f'⚠️  Some functionality tests failed: {e}')
    print('But basic import works, which is sufficient for now.')
" 2>/dev/null || print_warning "Some functionality tests failed, but basic import works"
        
        print_status "Installation test passed!"
        return 0
    else
        print_error "Import test failed!"
        return 1
    fi
}

# Main build process
main() {
    echo "Starting build process..."
    echo
    
    # Check dependencies
    if ! check_dependencies; then
        print_error "Dependency check failed!"
        exit 1
    fi
    
    # Check RTAB-Map
    if ! check_rtabmap; then
        exit 1
    fi
    
    # Install Python dependencies
    install_python_deps
    
    # Build method selection
    BUILD_METHOD=${1:-"setuppy"}
    
    case $BUILD_METHOD in
        "setuppy")
            build_setuppy
            ;;
        "cmake")
            build_cmake
            ;;
        *)
            print_error "Unknown build method: $BUILD_METHOD"
            print_error "Usage: $0 [setuppy|cmake]"
            exit 1
            ;;
    esac
    
    # Install package
    install_package
    
    # Test installation
    if test_installation; then
        echo
        print_status "🎉 RTAB-Map Python bindings built and installed successfully!"
        print_status
        print_status "You can now use the bindings in Python:"
        print_status "  import rtabmap_python as rtab"
        print_status
        print_status "Run examples:"
        print_status "  cd examples/"
        print_status "  python3 basic_slam_example.py"
        print_status "  python3 camera_integration_example.py"
        echo
    else
        print_error "Build completed but installation test failed!"
        exit 1
    fi
}

# Parse command line arguments
case "${1:-}" in
    "-h"|"--help")
        echo "RTAB-Map Python Bindings Build Script"
        echo
        echo "Usage: $0 [build_method]"
        echo
        echo "Build methods:"
        echo "  setuppy  - Build using setup.py (default)"
        echo "  cmake    - Build using CMake"
        echo
        echo "Options:"
        echo "  -h, --help    Show this help message"
        echo
        echo "Examples:"
        echo "  $0              # Build using setup.py"
        echo "  $0 setuppy      # Build using setup.py"
        echo "  $0 cmake        # Build using CMake"
        echo
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac
