import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
from rclpy.qos import qos_profile_sensor_data
import math

class EulerPrinter(Node):
    def __init__(self):
        super().__init__('euler_printer')
        self.get_logger().info("Initializing EulerPrinter Node")
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Replace with your IMU topic name
            self.listener_callback,
            qos_profile_sensor_data  # Use compatible QoS for sensor data
        )
        self.get_logger().info("Subscription to /imu/data_raw created")

    def listener_callback(self, msg):
        # Log that data is received
        self.get_logger().info("Received IMU data")

        # Extract quaternion
        q = msg.orientation
        quaternion = (q.w, q.x, q.y, q.z)  # Note: (w, x, y, z) for transforms3d

        # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
        roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')

        # Convert radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Log the Euler angles in degrees
        self.get_logger().info(
            f'Euler Angles (degrees): roll={roll_deg:.3f}, pitch={pitch_deg:.3f}, yaw={yaw_deg:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = EulerPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down EulerPrinter Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
