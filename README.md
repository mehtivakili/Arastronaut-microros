# UWB Data Reader

## Overview
The UWB Data Reader is a ROS2 package designed to process and visualize Ultra-Wideband (UWB) sensor data. It works in conjunction with the `cf_msgs` package to handle custom message types for various sensor data streams.

## Package Dependencies
- ROS2 (tested with Humble)
- cf_msgs (Custom message package)
- rclpy
- geometry_msgs
- sensor_msgs

## Integration with cf_msgs
The `cf_msgs` package provides custom message definitions that are used by the UWB Data Reader:

1. **Message Types Used**:
   - `cf_msgs/msg/Tdoa`: Time Difference of Arrival data
   - `cf_msgs/msg/Tof`: Time of Flight data
   - `cf_msgs/msg/Flow`: Optical flow measurements
   - `cf_msgs/msg/Gyro`: Gyroscope data
   - `cf_msgs/msg/Accel`: Acceleration data
   - `cf_msgs/msg/Baro`: Barometer data

2. **Integration Setup**:
   ```xml
   <!-- In package.xml -->
   <exec_depend>cf_msgs</exec_depend>
   ```

   ```python
   # In Python code
   from cf_msgs.msg import Tdoa, Tof, Flow, Gyro, Accel, Baro
   ```

## Working with ROS2 Bags

### Recording Data
To record UWB data into a ROS2 bag:
```bash
# Record all topics
ros2 bag record -a -o uwb_dataset

# Record specific topics
ros2 bag record /tdoa_data /tof_data /flow_data -o uwb_dataset
```

### Playing Back Data
To play back recorded data:
```bash
# Play the bag file
ros2 bag play uwb_dataset

# Play with options
ros2 bag play --rate 0.5 uwb_dataset  # Play at half speed
```

### Analyzing Bag Data
```bash
# View bag info
ros2 bag info uwb_dataset

# List topics in the bag
ros2 bag info -t uwb_dataset
```

## Installation

1. Clone both packages into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <url>/cf_msgs.git
   git clone <url>/uwb_data_reader.git
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the packages:
   ```bash
   colcon build --packages-select cf_msgs uwb_data_reader
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

1. Start the UWB Data Reader node:
   ```bash
   ros2 run uwb_data_reader data_reader
   ```

2. Play your dataset:
   ```bash
   ros2 bag play path/to/your/uwb_dataset
   ```

3. Monitor specific topics:
   ```bash
   # View TDOA data
   ros2 topic echo /tdoa_data

   # View TOF data
   ros2 topic echo /tof_data
   ```

## Topic Structure

The node subscribes to the following topics:
- `/tdoa_data` (cf_msgs/msg/Tdoa)
- `/tof_data` (cf_msgs/msg/Tof)
- `/flow_data` (cf_msgs/msg/Flow)
- `/gyro_data` (cf_msgs/msg/Gyro)
- `/accel_data` (cf_msgs/msg/Accel)
- `/baro_data` (cf_msgs/msg/Baro)
- `/pose_data` (geometry_msgs/msg/PoseWithCovarianceStamped)
- `/imu_data` (sensor_msgs/msg/Imu)

## Data Processing

The UWB Data Reader processes incoming messages and logs them with timestamps. Each message type is handled by a dedicated callback function that extracts and displays relevant information:

- TDOA data: Anchor IDs and time difference measurements
- TOF data: Range measurements
- Flow data: Optical flow measurements in x and y directions
- Gyro data: Angular velocities
- Accel data: Linear accelerations
- Baro data: Altitude measurements
- Pose data: Position and orientation with covariance
- IMU data: Combined acceleration and angular velocity

## Debugging

If you encounter issues:

1. Check ROS2 environment:
   ```bash
   ros2 doctor
   ```

2. Verify topic publishing:
   ```bash
   ros2 topic list
   ros2 topic info /tdoa_data
   ```

3. Check message flow:
   ```bash
   ros2 topic hz /tdoa_data
   ```

## Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License
[Your License Information] 