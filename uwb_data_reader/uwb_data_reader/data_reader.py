#!/usr/bin/env python3
"""
Run Instructions:
1. In your workspace (e.g., cd ~/ros2_ws), run:
     colcon build
2. Source your workspace:
     source install/setup.bash
3. Run the node:
     ros2 run uwb_data_reader uwb_data_reader
"""

import rclpy
from rclpy.node import Node
from cf_msgs.msg import Tdoa, Tof, Flow, Gyro, Accel, Baro
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

class UWBDataReader(Node):
    def __init__(self):
        super().__init__('uwb_data_reader')
        
        # Create subscribers for each message type
        self.tdoa_sub = self.create_subscription(
            Tdoa, 'tdoa_data', self.tdoa_callback, 10)
        self.tof_sub = self.create_subscription(
            Tof, 'tof_data', self.tof_callback, 10)
        self.flow_sub = self.create_subscription(
            Flow, 'flow_data', self.flow_callback, 10)
        self.gyro_sub = self.create_subscription(
            Gyro, 'gyro_data', self.gyro_callback, 10)
        self.accel_sub = self.create_subscription(
            Accel, 'accel_data', self.accel_callback, 10)
        self.baro_sub = self.create_subscription(
            Baro, 'baro_data', self.baro_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'pose_data', self.pose_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu_data', self.imu_callback, 10)
        
        self.get_logger().info('UWB Data Reader Node has started')

    def tdoa_callback(self, msg):
        self.get_logger().info(f'TDOA: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'id_a={msg.id_a}, id_b={msg.id_b}, data={msg.data}')

    def tof_callback(self, msg):
        self.get_logger().info(f'TOF: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'range={msg.zrange}')

    def flow_callback(self, msg):
        self.get_logger().info(f'Flow: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'delta_x={msg.delta_x}, delta_y={msg.delta_y}')

    def gyro_callback(self, msg):
        self.get_logger().info(f'Gyro: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'x={msg.x}, y={msg.y}, z={msg.z}')

    def accel_callback(self, msg):
        self.get_logger().info(f'Accel: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'x={msg.x}, y={msg.y}, z={msg.z}')

    def baro_callback(self, msg):
        self.get_logger().info(f'Baro: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
                              f'asl={msg.asl}')

    def pose_callback(self, msg):
        self.get_logger().info(
            f'Pose: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n'
            f'Position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}, z={msg.pose.pose.position.z:.3f}\n'
            f'Orientation: x={msg.pose.pose.orientation.x:.3f}, y={msg.pose.pose.orientation.y:.3f}, '
            f'z={msg.pose.pose.orientation.z:.3f}, w={msg.pose.pose.orientation.w:.3f}\n'
            f'Covariance: {msg.pose.covariance}'
        )

    def imu_callback(self, msg):
        self.get_logger().info(
            f'IMU: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n'
            f'Angular velocity: x={msg.angular_velocity.x:.3f}, y={msg.angular_velocity.y:.3f}, z={msg.angular_velocity.z:.3f}\n'
            f'Linear acceleration: x={msg.linear_acceleration.x:.3f}, y={msg.linear_acceleration.y:.3f}, z={msg.linear_acceleration.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = UWBDataReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()