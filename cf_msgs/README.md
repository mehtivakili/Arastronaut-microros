# cf_msgs (ROS 2 Package)

## Overview
`cf_msgs` is a ROS 2 package that provides custom message definitions for UWB (Ultra-Wideband) dataset handling. This package contains various message types for sensor data commonly used in UWB-based localization and navigation systems.

## Message Types
The package includes the following custom message types:

### Accel.msg
Acceleration data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `x`: X-axis acceleration (float32)
- `y`: Y-axis acceleration (float32)
- `z`: Z-axis acceleration (float32)

### Baro.msg
Barometer sensor data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `asl`: Above Sea Level measurement (float32)

### Flow.msg
Optical flow sensor data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `delta_x`: X-axis optical flow measurement (int32)
- `delta_y`: Y-axis optical flow measurement (int32)

### Gyro.msg
Gyroscope data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `x`: X-axis angular velocity (float32)
- `y`: Y-axis angular velocity (float32)
- `z`: Z-axis angular velocity (float32)

### Tdoa.msg
Time Difference of Arrival data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `id_a`: First anchor ID (int32)
- `id_b`: Second anchor ID (int32)
- `data`: Time difference measurement (float32)

### Tof.msg
Time of Flight data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `zrange`: Range measurement in z-axis (float32)

### Twr.msg
Two-Way Ranging data message containing:
- `header`: Standard ROS Header (timestamp and frame information)
- `id`: Anchor ID (int32)
- `data`: Range measurement (float32)

## Prerequisites
- ROS 2 (tested with ROS 2 Humble)
- `ament_cmake`
- `rosidl_default_generators`
- `std_msgs`

## Installation

1. Clone this package into your ROS 2 workspace's `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>/cf_msgs.git
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select cf_msgs
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Including in Your Package
To use these messages in your ROS 2 package:

1. Add the following dependencies to your `package.xml`:
   ```xml
   <depend>cf_msgs</depend>
   ```

2. Add the following to your `CMakeLists.txt`:
   ```cmake
   find_package(cf_msgs REQUIRED)
   ```

3. In your C++ code, include the desired message headers:
   ```cpp
   #include "cf_msgs/msg/accel.hpp"
   #include "cf_msgs/msg/baro.hpp"
   // ... other message types as needed
   ```

4. In your Python code, import the messages as:
   ```python
   from cf_msgs.msg import Accel
   from cf_msgs.msg import Baro
   # ... other message types as needed
   ```

