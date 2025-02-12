import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt

class UWBRangeProcessor(Node):
    def __init__(self):
        super().__init__('uwb_range_processor')
        self.get_logger().info("Initializing UWBRangeProcessor Node")

        # Lists to store timestamps, raw data, and filtered data
        self.timestamps = []
        self.raw_data = []
        self.filtered_data = []

        # Kalman filter initialization
        self.kalman_estimate = None
        self.kalman_error_cov = 1.0  # Initial error covariance
        self.kalman_process_var = 0.01  # Process variance
        self.kalman_measurement_var = 2.0  # Measurement variance

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

    def kalman_filter(self, measurement):
        """
        Applies a 1D Kalman filter to smooth the UWB range data.
        """
        if self.kalman_estimate is None:
            # Initialize the estimate with the first measurement
            self.kalman_estimate = measurement

        # Prediction step
        predicted_estimate = self.kalman_estimate
        predicted_error_cov = self.kalman_error_cov + self.kalman_process_var

        # Update step
        kalman_gain = predicted_error_cov / (predicted_error_cov + self.kalman_measurement_var)
        self.kalman_estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.kalman_error_cov = (1 - kalman_gain) * predicted_error_cov

        return self.kalman_estimate

    def listener_callback(self, msg):
        # Extract the range measurement
        distance = msg.range

        # Store the timestamp and raw data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)
        self.raw_data.append(distance)

        # Apply Kalman filter
        filtered_distance = self.kalman_filter(distance)
        self.filtered_data.append(filtered_distance)

        # Keep only the last 100 points for better performance
        self.timestamps = self.timestamps[-100:]
        self.raw_data = self.raw_data[-100:]
        self.filtered_data = self.filtered_data[-100:]

        # Update live plot
        self.ax.clear()
        self.ax.plot(self.timestamps, self.raw_data, label='Raw Data', linestyle='--', marker='o')
        self.ax.plot(self.timestamps, self.filtered_data, label='Filtered Data', linestyle='-', color='red')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Range (m)')
        self.ax.set_title('UWB Range Data: Raw vs Filtered')
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)

        # Log the raw and filtered measurements
        self.get_logger().info(
            f'Time: {timestamp:.2f}s, Raw Range: {distance:.3f}m, Filtered Range: {filtered_distance:.3f}m'
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
