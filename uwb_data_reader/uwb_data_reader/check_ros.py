import subprocess
import sys

def check_ros2_installation():
    try:
        # Check ROS 2 environment
        result = subprocess.run(['ros2', '--help'], capture_output=True, text=True)
        if result.returncode == 0:
            print("ROS 2 is installed and working!")
            # Get ROS 2 version
            version = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
            print(f"ROS 2 version: {version.stdout.strip()}")
        else:
            print("ROS 2 command failed. Check if ROS 2 is properly installed.")
    except FileNotFoundError:
        print("ROS 2 is not installed or not in PATH.")
        print("Please install ROS 2 or source the setup file:")
        print("source /opt/ros/<distro>/setup.bash")

def main():
    check_ros2_installation()

if __name__ == '__main__':
    main()
