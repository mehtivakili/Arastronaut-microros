cmake_minimum_required(VERSION 3.8)
project(cf_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Accel.msg"
  "msg/Baro.msg"
  "msg/Flow.msg"
  "msg/Gyro.msg"
  "msg/Tdoa.msg"
  "msg/Tof.msg"
  "msg/Twr.msg"
  DEPENDENCIES std_msgs
)

ament_package()
