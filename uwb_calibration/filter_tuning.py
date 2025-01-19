import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class AdaptiveUKF:
    def __init__(self, dt):
        self.dt = dt
        self.dim_x = 2  # State dimension [position, velocity]
        self.dim_z = 1  # Measurement dimension [range]

        # Define sigma points
        self.points = MerweScaledSigmaPoints(
            n=self.dim_x, alpha=0.1, beta=2.0, kappa=0
        )

        # Initialize the UKF
        self.ukf = UKF(
            dim_x=self.dim_x,
            dim_z=self.dim_z,
            dt=self.dt,
            fx=self.state_transition_function,
            hx=self.measurement_function,
            points=self.points,
        )

        # Initial state
        self.ukf.x = np.array([0.0, 0.0])  # [position, velocity]

        # Initial covariances
        self.ukf.P = np.eye(self.dim_x) * 0.1
        self.ukf.Q = np.eye(self.dim_x) * 0.01
        self.ukf.R = np.eye(self.dim_z) * 0.1

        # Parameters for adaptive filtering
        self.alpha = 0.001  # Learning rate for process noise
        self.beta = 0.0001  # Learning rate for measurement noise

    def state_transition_function(self, state, dt):
        # State transition model: x = x + v*dt
        position = state[0] + state[1] * dt
        velocity = state[1]
        return np.array([position, velocity])

    def measurement_function(self, state):
        # Measurement model: direct observation of position
        return np.array([state[0]])

    def adapt_process_noise(self, innovation):
        # Adapt process noise covariance Q
        self.ukf.Q += self.alpha * (np.outer(innovation, innovation) - self.ukf.Q)

    def adapt_measurement_noise(self, innovation):
        # Adapt measurement noise covariance R
        self.ukf.R += self.beta * (innovation**2 - self.ukf.R)

    def process_measurement(self, measurement):
        # Predict step
        self.ukf.predict()

        # Measurement residual (innovation)
        innovation = measurement - self.ukf.x[0]

        # Update step
        self.ukf.update(np.array([measurement]))

        # Adapt noise covariances
        self.adapt_process_noise(innovation)
        self.adapt_measurement_noise(innovation)

        return self.ukf.x[0]  # Return the estimated position

class UWBRangeProcessor(Node):
    def __init__(self):
        super().__init__('uwb_range_processor')
        self.get_logger().info("Initializing UWBRangeProcessor Node")

        # Initialize the Adaptive UKF with a time step (dt)
        self.aukf = AdaptiveUKF(dt=0.1)

        # Lists to store timestamps, raw data, and filtered data
        self.timestamps = []
        self.raw_data = []
        self.filtered_data = []

        # Subscription to UWB raw data
        self.subscription = self.create_subscription(
            Range,
            '/uwb/data_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /uwb/data_raw")

    def listener_callback(self, msg):
        # Extract the range measurement
        distance = msg.range

        # Process the measurement through Adaptive UKF
        filtered_distance = self.aukf.process_measurement(distance)

        # Store the timestamp, raw, and filtered data
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)
        self.raw_data.append(distance)
        self.filtered_data.append(filtered_distance)

        # Log the raw and filtered measurements
        self.get_logger().info(
            f'Time: {timestamp:.2f}s, Raw Range: {distance:.3f}m, Filtered Range: {filtered_distance:.3f}m'
        )

    def plot_data(self):
        # Plot raw and filtered data
        plt.figure()
        plt.plot(self.timestamps, self.raw_data, label='Raw Data', linestyle='--', marker='o')
        plt.plot(self.timestamps, self.filtered_data, label='Filtered Data', linestyle='-', marker='x')
        plt.xlabel('Time (s)')
        plt.ylabel('Range (m)')
        plt.title('UWB Range Data: Raw vs. Filtered')
        plt.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = UWBRangeProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UWBRangeProcessor Node")
    finally:
        # Plot the data after shutting down
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
