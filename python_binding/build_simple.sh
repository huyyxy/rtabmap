#!/bin/bash
# Simplified build script for RTAB-Map Python bindings
# Supports Linux (.so), macOS (.dylib), and Windows (.dll)

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Detect operating system
detect_os() {
    case "$(uname -s)" in
        Linux*)
            OS="linux"
            EXT="so"
            print_info "Detected OS: Linux"
            ;;
        Darwin*)
            OS="macos"
            EXT="dylib"
            print_info "Detected OS: macOS"
            ;;
        CYGWIN*|MINGW*|MSYS*)
            OS="windows"
            EXT="dll"
            print_info "Detected OS: Windows"
            ;;
        *)
            print_error "Unsupported operating system: $(uname -s)"
            exit 1
            ;;
    esac
}

# Check Python installation
check_python() {
    print_info "Checking Python installation..."
    
    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    elif command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        print_error "Python not found! Please install Python 3.7 or higher."
        exit 1
    fi
    
    PYTHON_VERSION=$($PYTHON_CMD -c "import sys; print('.'.join(map(str, sys.version_info[:2])))")
    print_info "Found Python version: $PYTHON_VERSION"
    
    # Check if Python version is >= 3.7
    PYTHON_MAJOR=$($PYTHON_CMD -c "import sys; print(sys.version_info[0])")
    PYTHON_MINOR=$($PYTHON_CMD -c "import sys; print(sys.version_info[1])")
    
    if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 7 ]); then
        print_error "Python 3.7 or higher is required. Found: $PYTHON_VERSION"
        exit 1
    fi
}

# Check required dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check pip
    if ! command -v pip3 &> /dev/null && ! command -v pip &> /dev/null; then
        print_error "pip not found! Please install pip."
        exit 1
    fi
    
    # Check if we're in the right directory
    if [ ! -f "setup.py" ]; then
        print_error "setup.py not found! Please run this script from the python_binding directory."
        exit 1
    fi
    
    # Check if source files exist
    if [ ! -d "src" ]; then
        print_error "src directory not found!"
        exit 1
    fi
    
    print_success "Dependencies check passed"
}

# Install Python dependencies
install_dependencies() {
    print_info "Installing Python dependencies..."
    
    # Install required packages
    $PYTHON_CMD -m pip install --upgrade pip
    $PYTHON_CMD -m pip install "numpy>=1.19.0" "opencv-python>=4.5.0" "pybind11>=2.6.0" setuptools wheel
    
    print_success "Python dependencies installed"
}

# Clean previous build
clean_build() {
    print_info "Cleaning previous build..."
    
    # Remove build directories
    if [ -d "build" ]; then
        rm -rf build/
        print_info "Removed build/ directory"
    fi
    
    if [ -d "dist" ]; then
        rm -rf dist/
        print_info "Removed dist/ directory"
    fi
    
    # Remove egg-info directories
    if ls *.egg-info 1> /dev/null 2>&1; then
        rm -rf *.egg-info
        print_info "Removed *.egg-info directories"
    fi
    
    # Remove compiled extensions
    if ls *.so 1> /dev/null 2>&1; then
        rm -f *.so
        print_info "Removed *.so files"
    fi
    
    if ls *.dylib 1> /dev/null 2>&1; then
        rm -f *.dylib
        print_info "Removed *.dylib files"
    fi
    
    if ls *.dll 1> /dev/null 2>&1; then
        rm -f *.dll
        print_info "Removed *.dll files"
    fi
    
    # Remove Python cache
    if [ -d "__pycache__" ]; then
        rm -rf __pycache__/
        print_info "Removed __pycache__/ directory"
    fi
    
    # Remove any files created by incorrect pip commands (e.g., =1.19.0 files)
    if ls =* 1> /dev/null 2>&1; then
        rm -f =*
        print_info "Removed files created by incorrect pip commands"
    fi
    
    print_success "Build cleaned"
}

# Build Python extension
build_extension() {
    print_info "Building Python extension..."
    
    # Build the extension
    $PYTHON_CMD setup.py build_ext --inplace
    
    # Fix library paths for macOS
    if [ "$OS" = "macos" ]; then
        print_info "Fixing library paths for macOS..."
        
        # Find the generated .so file
        SO_FILE=$(ls *.so 2>/dev/null | head -n1)
        if [ -n "$SO_FILE" ]; then
            # Get the RTAB-Map build directory
            RTABMAP_BUILD_DIR=$(dirname $(pwd))/build/bin
            
            if [ -d "$RTABMAP_BUILD_DIR" ]; then
                # Create versioned symlinks if they don't exist
                if [ -f "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.1.dylib" ] && [ ! -f "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib" ]; then
                    ln -sf librtabmap_core.0.23.1.dylib "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib"
                    print_info "Created symlink for librtabmap_core.0.23.dylib"
                fi
                
                if [ -f "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.1.dylib" ] && [ ! -f "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib" ]; then
                    ln -sf librtabmap_utilite.0.23.1.dylib "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib"
                    print_info "Created symlink for librtabmap_utilite.0.23.dylib"
                fi
                
                # Update the library paths in the .so file
                install_name_tool -change @rpath/librtabmap_core.0.23.dylib "$RTABMAP_BUILD_DIR/librtabmap_core.0.23.dylib" "$SO_FILE" 2>/dev/null || true
                install_name_tool -change @rpath/librtabmap_utilite.0.23.dylib "$RTABMAP_BUILD_DIR/librtabmap_utilite.0.23.dylib" "$SO_FILE" 2>/dev/null || true
                print_info "Updated library paths in $SO_FILE"
            fi
        fi
    fi
    
    print_success "Python extension built successfully"
}

# Generate egg-info
generate_egg_info() {
    print_info "Generating egg-info..."
    
    # Generate egg-info by running setup.py develop
    $PYTHON_CMD setup.py egg_info
    
    print_success "Egg-info generated"
}

# Verify build results
verify_build() {
    print_info "Verifying build results..."
    
    # Check for compiled extension
    EXTENSION_FOUND=false
    
    # On all platforms, Python extensions typically use .so extension
    # Check for .so files first (most common)
    if ls *.so 1> /dev/null 2>&1; then
        EXTENSION_FOUND=true
        EXTENSION_FILE=$(ls *.so | head -n1)
    # Fallback to platform-specific extensions
    elif [ "$OS" = "macos" ] && ls *.dylib 1> /dev/null 2>&1; then
        EXTENSION_FOUND=true
        EXTENSION_FILE=$(ls *.dylib | head -n1)
    elif [ "$OS" = "windows" ] && ls *.dll 1> /dev/null 2>&1; then
        EXTENSION_FOUND=true
        EXTENSION_FILE=$(ls *.dll | head -n1)
    fi
    
    if [ "$EXTENSION_FOUND" = true ]; then
        print_success "Found compiled extension: $EXTENSION_FILE"
    else
        print_error "No compiled extension found!"
        return 1
    fi
    
    # Check for egg-info
    if ls *.egg-info 1> /dev/null 2>&1; then
        EGG_INFO_DIR=$(ls *.egg-info | head -n1)
        print_success "Found egg-info directory: $EGG_INFO_DIR"
    else
        print_error "No egg-info directory found!"
        return 1
    fi
    
    # Test import
    print_info "Testing import..."
    if $PYTHON_CMD -c "import rtabmap_python; print('Import successful')" 2>/dev/null; then
        print_success "Python module can be imported successfully"
    else
        print_warning "Python module import test failed, but build files are present"
    fi
}

# Main build process
main() {
    echo "=========================================="
    echo "RTAB-Map Python Bindings - Simple Build"
    echo "=========================================="
    echo
    
    # Detect OS
    detect_os
    
    # Check Python
    check_python
    
    # Check dependencies
    check_dependencies
    
    # Install dependencies
    install_dependencies
    
    # Clean previous build
    clean_build
    
    # Build extension
    build_extension
    
    # Generate egg-info
    generate_egg_info
    
    # Verify build
    if verify_build; then
        echo
        print_success "🎉 Build completed successfully!"
        echo
        print_info "Generated files:"
        # Show all extension files regardless of platform
        ls -la *.so *.dylib *.dll 2>/dev/null || true
        ls -la *.egg-info 2>/dev/null || true
        echo
        print_info "You can now use the Python bindings:"
        print_info "  $PYTHON_CMD -c \"import rtabmap_python as rtab\""
    else
        print_error "Build verification failed!"
        exit 1
    fi
}

# Parse command line arguments
case "${1:-}" in
    "-h"|"--help")
        echo "RTAB-Map Python Bindings - Simple Build Script"
        echo
        echo "Usage: $0 [options]"
        echo
        echo "Options:"
        echo "  -h, --help     Show this help message"
        echo "  clean          Clean build files only"
        echo
        echo "This script will:"
        echo "  - Detect your operating system (Linux/macOS/Windows)"
        echo "  - Check Python installation (requires 3.7+)"
        echo "  - Install required dependencies"
        echo "  - Clean previous build files"
        echo "  - Build Python extension (.so/.dylib/.dll)"
        echo "  - Generate .egg-info directory"
        echo "  - Verify the build"
        echo
        exit 0
        ;;
    "clean")
        detect_os
        clean_build
        print_success "Clean completed"
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac