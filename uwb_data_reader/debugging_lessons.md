# ROS2 Workspace Debugging Lessons

## 1. Environment Conflicts
- **Issue**: Identified conflict between Conda and ROS2 Python environments
- **Solution**: Deactivate Conda when working with ROS2
- **Lesson**: Always check which Python environment is active when working with ROS2
- **Relevant Commands**:
  ```bash
  # Check if Conda is active
  echo $CONDA_PREFIX
  
  # Deactivate Conda
  conda deactivate
  
  # Check Python environment
  which python3
  python3 --version
  ```

## 2. Missing Dependencies
- **Issue**: Required tools were missing (rosdep, colcon)
- **Solution**: Install necessary build tools
- **Lesson**: Check for and install all build tools before attempting to build
- **Relevant Commands**:
  ```bash
  # Install rosdep
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  
  # Install colcon
  sudo apt install python3-colcon-common-extensions
  ```

## 3. Build Order Matters
- **Issue**: Dependencies need to be built before dependent packages
- **Solution**: Build packages in correct order
- **Lesson**: Understand package dependencies and build order
- **Relevant Commands**:
  ```bash
  # Build specific package
  colcon build --packages-select cf_msgs
  
  # Build package with dependencies
  colcon build --packages-up-to uwb_data_reader
  
  # Build entire workspace
  colcon build --symlink-install
  ```

## 4. Workspace Management
- **Issue**: Build artifacts can cause conflicts
- **Solution**: Clean workspace when needed
- **Lesson**: Maintain clean workspace state and proper sourcing
- **Relevant Commands**:
  ```bash
  # Clean workspace
  rm -rf build/ install/ log/
  
  # Source workspace
  source install/setup.bash
  
  # Source ROS2 environment
  source /opt/ros/jazzy/setup.bash
  ```

## 5. Environment Variables
- **Issue**: Multiple environment paths can interact
- **Solution**: Proper .bashrc configuration
- **Lesson**: Pay attention to environment variable order
- **Relevant Commands**:
  ```bash
  # Check ROS2 environment
  printenv | grep ROS
  
  # Check package paths
  echo $AMENT_PREFIX_PATH
  echo $CMAKE_PREFIX_PATH
  
  # List current environment
  env
  ```

## 6. Verification Steps
- **Issue**: Need to verify system state
- **Solution**: Use ROS2 verification commands
- **Lesson**: Regular verification prevents issues
- **Relevant Commands**:
  ```bash
  # Check ROS2 installation
  ros2 topic list
  
  # Verify package visibility
  ros2 pkg list | grep uwb_data_reader
  
  # Check dependencies
  rosdep install --from-path src --ignore-src -r -y
  
  # List available packages
  ros2 pkg list
  ```

## 7. Common Issues Identified
- Missing build tools
- Environment conflicts
- Incorrect sourcing order
- Workspace not built in correct order
- Python environment conflicts
- **Diagnostic Commands**:
  ```bash
  # Check ROS2 version
  ros2 --version
  
  # Check package dependencies
  rosdep check --from-path src --ignore-src
  
  # Check package path
  ros2 pkg prefix uwb_data_reader
  ```

## 8. Best Practices
- Clean workspace before rebuilding
- Build dependencies first
- Source workspace after building
- **Example Workflow**:
  ```bash
  # Clean workflow
  cd ~/ros2_ws
  rm -rf build/ install/ log/
  colcon build --symlink-install
  source install/setup.bash
  
  # Development workflow
  colcon build --symlink-install --packages-up-to uwb_data_reader
  ```

## 9. Debugging Tools Used
- **ROS2 Tools**:
  ```bash
  # Package management
  ros2 pkg list
  ros2 pkg executables
  
  # Build tools
  colcon build
  colcon test
  
  # Dependency management
  rosdep install
  rosdep check
  ```

## 10. Documentation Importance
- **Key Documentation Commands**:
  ```bash
  # Get package information
  ros2 pkg xml uwb_data_reader
  
  # List package contents
  ros2 pkg executables uwb_data_reader
  
  # Check package dependencies
  ros2 pkg dependencies uwb_data_reader
  ```

## Additional Tips
- Always check the ROS2 distribution you're using
- Keep track of environment changes
- Document custom configurations
- Use version control for workspace management

These lessons and commands help in maintaining a robust ROS2 development environment and quickly identifying and resolving common issues. 