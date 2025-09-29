#!/usr/bin/env python3
"""
Wrapper script to run rtabmap_python with correct library paths.
This script sets up the necessary environment variables for the RTAB-Map Python bindings.
"""
import os
import sys
import subprocess

def setup_environment():
    """Set up the environment variables needed for rtabmap_python."""
    # Get the directory containing this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path to the RTAB-Map build directory
    rtabmap_build_dir = os.path.join(os.path.dirname(script_dir), "build", "bin")
    
    if not os.path.exists(rtabmap_build_dir):
        print(f"Error: RTAB-Map build directory not found: {rtabmap_build_dir}")
        print("Please make sure RTAB-Map is built correctly.")
        sys.exit(1)
    
    # Set the library path
    current_dyld_path = os.environ.get('DYLD_LIBRARY_PATH', '')
    if current_dyld_path:
        os.environ['DYLD_LIBRARY_PATH'] = f"{rtabmap_build_dir}:{current_dyld_path}"
    else:
        os.environ['DYLD_LIBRARY_PATH'] = rtabmap_build_dir
    
    print(f"✓ Set DYLD_LIBRARY_PATH to include: {rtabmap_build_dir}")

def test_import():
    """Test if rtabmap_python can be imported successfully."""
    try:
        # Change to the directory containing the module
        original_dir = os.getcwd()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        os.chdir(script_dir)
        
        import rtabmap_python
        print("✅ rtabmap_python imported successfully!")
        print(f"Available classes: {[x for x in dir(rtabmap_python) if not x.startswith('_')]}")
        
        # Change back to original directory
        os.chdir(original_dir)
        return True
    except ImportError as e:
        print(f"❌ Failed to import rtabmap_python: {e}")
        return False

def main():
    """Main function."""
    if len(sys.argv) == 1:
        # No arguments, just test the import
        print("RTAB-Map Python Bindings Test")
        print("=" * 40)
        setup_environment()
        success = test_import()
        if success:
            print("\n✅ Setup successful! You can now use rtabmap_python in your scripts.")
            print("To use in your own scripts, make sure to set the DYLD_LIBRARY_PATH:")
            print(f"export DYLD_LIBRARY_PATH={os.environ['DYLD_LIBRARY_PATH']}")
        else:
            print("\n❌ Setup failed. Please check the error messages above.")
            sys.exit(1)
    else:
        # Run the provided Python script with correct environment
        setup_environment()
        script_path = sys.argv[1]
        if not os.path.exists(script_path):
            print(f"Error: Script not found: {script_path}")
            sys.exit(1)
        
        # Run the script with the correct environment
        result = subprocess.run([sys.executable, script_path] + sys.argv[2:], 
                              env=os.environ)
        sys.exit(result.returncode)

if __name__ == "__main__":
    main()
