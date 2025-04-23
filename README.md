ndi

# Micro-ROS Implementation Guide

This guide explains how to set up and implement Micro-ROS using Docker for simplicity.

## Prerequisites
- Ubuntu OS
- Docker installed
- ROS2 environment
- USB connection to your microcontroller

## Installation Steps

### 1. Install ROS2 Key and Repository
```bash
# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install ROS2 Desktop
```bash
sudo apt install ros-foxy-desktop python3-argcomplete
source /opt/ros/foxy/setup.bash
```

### 3. Pull Micro-ROS Docker Image
```bash
sudo docker pull microros/micro-ros-agent:$ROS_DISTRO
```

## Usage

### Start Micro-ROS Agent

#### Using Serial Connection
```bash
sudo docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:$ROS_DISTRO \
    serial --dev /dev/ttyUSB0 -v6
```

#### Using UDP Connection
```bash
sudo docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:$ROS_DISTRO \
    udp4 --port 8888 -v6
```

### Monitor Topics
```bash
# List all available topics
ros2 topic list

# Monitor IMU data
ros2 topic echo /imu/data_raw
```

## Troubleshooting
- Ensure your device is properly connected and recognized at `/dev/ttyUSB0`
- Check Docker permissions if you encounter access issues
- Verify that ROS2 environment is properly sourced

## Contributing
Feel free to submit issues and enhancement requests!

## License
[Your License Here]

---
For more information, visit the [Micro-ROS Documentation](https://micro.ros.org/).



Do these steps to implement micro ros
We used docker for simplicity

sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo “deb [arch=$(dpkg –print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main” | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt install ros-foxy-desktop python3-argcomplete

source /opt/ros/foxy/setup.bash

sudo docker pull microros/micro-ros-agent:$ROS_DISTRO

sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6

sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888  -v6

ros2 topic list

ros2 topic echo /imu/data_raw
