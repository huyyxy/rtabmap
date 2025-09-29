#!/bin/bash

# RTAB-Map Python Bindings Test Script
# This script sets up the correct environment and tests the Python bindings

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RTABMAP_BUILD_DIR="$(dirname "$SCRIPT_DIR")/build/bin"

echo "RTAB-Map Python Bindings Test"
echo "==============================="

# Check if build directory exists
if [ ! -d "$RTABMAP_BUILD_DIR" ]; then
    echo "❌ Error: RTAB-Map build directory not found: $RTABMAP_BUILD_DIR"
    echo "Please make sure RTAB-Map is built correctly."
    exit 1
fi

# Set library path
export DYLD_LIBRARY_PATH="$RTABMAP_BUILD_DIR:$DYLD_LIBRARY_PATH"
echo "✓ Set DYLD_LIBRARY_PATH to include: $RTABMAP_BUILD_DIR"

# Change to the python_binding directory
cd "$SCRIPT_DIR"

# Test the import
echo "Testing rtabmap_python import..."
python3 -c "
import rtabmap_python
print('✅ rtabmap_python imported successfully!')
print('Available classes:', [x for x in dir(rtabmap_python) if not x.startswith('_')])

# Test creating a basic camera model
print()
print('Testing basic functionality...')
try:
    # Create a simple camera model
    cam = rtabmap_python.CameraModel()
    print('✓ CameraModel created successfully')
    
    # Test Size creation
    size = rtabmap_python.Size(640, 480)
    print(f'✓ Size created: {size}')
    
    # Test Transform
    transform = rtabmap_python.Transform()
    print('✓ Transform created successfully')
    
    print()
    print('🎉 All tests passed! RTAB-Map Python bindings are working correctly.')
except Exception as e:
    print(f'❌ Error during functionality test: {e}')
    exit(1)
"

if [ $? -eq 0 ]; then
    echo
    echo "✅ Setup successful! You can now use rtabmap_python in your scripts."
    echo "To use in your own scripts, make sure to set the DYLD_LIBRARY_PATH:"
    echo "export DYLD_LIBRARY_PATH=\"$RTABMAP_BUILD_DIR:\$DYLD_LIBRARY_PATH\""
    echo
    echo "Or run your Python scripts from this directory: $SCRIPT_DIR"
else
    echo
    echo "❌ Setup failed. Please check the error messages above."
    exit 1
fi
