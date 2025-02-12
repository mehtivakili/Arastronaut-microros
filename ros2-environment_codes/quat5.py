import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from vqf import VQF
import math
from rclpy.qos import qos_profile_sensor_data


class IMUVQFNode(Node):
    def __init__(self):
        super().__init__('imu_vqf_node')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag_raw')
        self.declare_parameter('sampling_rate', 10.0)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        self.sampling_rate = self.get_parameter('sampling_rate').get_parameter_value().double_value

        # Subscriptions using qos_profile_sensor_data
        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(MagneticField, mag_topic, self.mag_callback, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic}")
        self.get_logger().info(f"Subscribed to Magnetometer topic: {mag_topic}")

        # VQF Filter
        self.vqf = VQF(1.0 / self.sampling_rate)
        self.mag_data = None  # Placeholder for magnetometer data
        self.orientation = np.array([1, 0, 0, 0])  # Initial quaternion

    def imu_callback(self, msg):
        """Callback function for IMU data."""
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # self.get_logger().info(f"IMU Data - Accel: {accel}, Gyro: {gyro}")

        # Update the VQF filter
        if self.mag_data is not None:
            self.vqf.update(gyro, accel, self.mag_data)
        else:
            self.vqf.update(gyro, accel)

        # Update orientation
        self.orientation = self.vqf.getQuat6D()
        # self.get_logger().info(f"Updated Orientation (quaternion): {self.orientation}")

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)

        # Convert radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Print the roll, pitch, and yaw angles
        self.get_logger().info(f"Roll: {roll_deg:.2f}, Pitch: {pitch_deg:.2f}, Yaw: {yaw_deg:.2f}")

    def mag_callback(self, msg):
        """Callback function for Magnetometer data."""
        mag_data = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        # self.get_logger().info(f"Magnetometer Data: {mag_data}")

        # Save magnetometer data
        self.mag_data = mag_data

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def run(self):
        """Keep the ROS node running."""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = IMUVQFNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
