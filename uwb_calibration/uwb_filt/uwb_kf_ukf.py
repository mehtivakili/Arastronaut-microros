import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter

class UWBRangeProcessor(Node):
    def __init__(self):
        super().__init__('uwb_range_processor')
        self.get_logger().info("Initializing UWBRangeProcessor Node")

        # Lists to store timestamps, raw data, KF data, and UKF data
        self.timestamps = []
        self.raw_data = []
        self.kf_data = []
        self.ukf_data = []

        # Kalman filter initialization
        self.kf_estimate = None
        self.kf_error_cov = 1.0  # Initial error covariance
        self.kf_process_var = 0.01  # Process variance for KF
        self.kf_measurement_var = 5.0  # Measurement variance for KF

        # Unscented Kalman filter initialization
        points = MerweScaledSigmaPoints(n=1, alpha=0.1, beta=1.0, kappa=0.0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=1, dim_z=1, fx=self.fx, hx=self.hx, dt=1.0, points=points
        )
        self.ukf.x = [0.0]  # Initial state
        self.ukf.P = 1.0  # Initial covariance
        self.ukf.Q = 0.01  # Process noise covariance
        self.ukf.R = 2.0  # Measurement noise covariance

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

    def fx(self, x, dt):
        """State transition function for UKF (identity for 1D constant data)."""
        return x

    def hx(self, x):
        """Measurement function for UKF (identity for 1D constant data)."""
        return x

    def kalman_filter(self, measurement):
        """
        Applies a 1D Kalman filter to smooth the UWB range data.
        """
        if self.kf_estimate is None:
            # Initialize the estimate with the first measurement
            self.kf_estimate = measurement

        # Prediction step
        predicted_estimate = self.kf_estimate
        predicted_error_cov = self.kf_error_cov + self.kf_process_var

        # Update step
        kalman_gain = predicted_error_cov / (predicted_error_cov + self.kf_measurement_var)
        self.kf_estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.kf_error_cov = (1 - kalman_gain) * predicted_error_cov

        return self.kf_estimate

    def listener_callback(self, msg):
        # Extract the range measurement
        distance = msg.range
        distance = (distance + 0.3)/0.0174

        # Store the timestamp and raw data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)
        self.raw_data.append(distance)

        # Apply Kalman filter
        kf_filtered_distance = self.kalman_filter(distance)
        self.kf_data.append(kf_filtered_distance)

        # Apply Unscented Kalman Filter
        self.ukf.predict()
        self.ukf.update(distance)
        ukf_filtered_distance = self.ukf.x[0]
        self.ukf_data.append(ukf_filtered_distance)

        # Keep only the last 100 points for better performance
        self.timestamps = self.timestamps[-100:]
        self.raw_data = self.raw_data[-100:]
        self.kf_data = self.kf_data[-100:]
        self.ukf_data = self.ukf_data[-100:]

        # Update live plot
        self.ax.clear()
        self.ax.plot(self.timestamps, self.raw_data, label='Raw Data', linestyle='--', marker='o')
        self.ax.plot(self.timestamps, self.kf_data, label='Kalman Filter', linestyle='-', color='red')
        self.ax.plot(self.timestamps, self.ukf_data, label='Unscented Kalman Filter', linestyle='-', color='blue')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Range (m)')
        self.ax.set_title('UWB Range Data: Raw, KF, and UKF')
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)

        # Log the raw, KF, and UKF measurements
        self.get_logger().info(
            f'Time: {timestamp:.2f}s, Raw: {distance:.3f}m, KF: {kf_filtered_distance:.3f}m, UKF: {ukf_filtered_distance:.3f}m'
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
