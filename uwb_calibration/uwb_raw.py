import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt

class UWBRangeProcessor(Node):
    def __init__(self):
        super().__init__('uwb_range_processor')
        self.get_logger().info("Initializing UWBRangeProcessor Node")

        # Lists to store timestamps and raw data
        self.timestamps = []
        self.raw_data = []

        # Subscription to UWB raw data
        self.subscription = self.create_subscription(
            Range,
            '/uwb/data_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /uwb/data_raw")

        # Initialize live plot
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def listener_callback(self, msg):
        # Extract the range measurement
        distance = msg.range

        # Store the timestamp and raw data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)
        self.raw_data.append(distance)

        # Keep only the last 100 points for better performance
        self.timestamps = self.timestamps[-100:]
        self.raw_data = self.raw_data[-100:]

        # Update live plot
        self.ax.clear()
        self.ax.plot(self.timestamps, self.raw_data, label='Raw Data', linestyle='--', marker='o')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Range (m)')
        self.ax.set_title('UWB Range Data: Raw')
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)

        # Log the raw measurements
        self.get_logger().info(
            f'Time: {timestamp:.2f}s, Raw Range: {distance:.3f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = UWBRangeProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UWBRangeProcessor Node")
    finally:
        plt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
