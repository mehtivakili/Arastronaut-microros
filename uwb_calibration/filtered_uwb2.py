import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class VariableBayesianUKF:
    def __init__(self, dt, q_value=0.05, r_value=0.15):
        self.dt = dt
        self.dim_x = 2
        self.dim_z = 1

        self.points = MerweScaledSigmaPoints(n=self.dim_x, alpha=0.3, beta=2.0, kappa=0)

        self.ukf = UKF(
            dim_x=self.dim_x,
            dim_z=self.dim_z,
            dt=self.dt,
            fx=self.state_transition_function,
            hx=self.measurement_function,
            points=self.points,
        )

        self.ukf.x = np.array([0.0, 0.0])
        self.ukf.P = np.eye(self.dim_x) * 0.1
        self.ukf.Q = np.eye(self.dim_x) * q_value
        self.ukf.R = np.eye(self.dim_z) * r_value

    def state_transition_function(self, state, dt):
        position = state[0] + state[1] * dt
        velocity = state[1]
        return np.array([position, velocity])

    def measurement_function(self, state):
        return np.array([state[0]])

    def update_noise_covariances(self, measurement_residual):
        self.ukf.R = np.array([[np.var(measurement_residual)]])

    def process_measurement(self, measurement):
        self.ukf.predict()
        residual = measurement - self.ukf.x[0]
        self.update_noise_covariances(residual)
        self.ukf.update(np.array([measurement]))
        return self.ukf.x[0]


class UWBRangeProcessor(Node):
    def __init__(self):
        super().__init__('uwb_range_processor')
        self.get_logger().info("Initializing UWBRangeProcessor Node")

        self.vbukf = VariableBayesianUKF(dt=0.1)

        self.timestamps = []
        self.raw_data = []
        self.filtered_data = []

        self.subscription = self.create_subscription(
            Range,
            '/uwb/data_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /uwb/data_raw")

    def listener_callback(self, msg):
        distance = msg.range
        filtered_distance = self.vbukf.process_measurement(distance)

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)
        self.raw_data.append(distance)
        self.filtered_data.append(filtered_distance)

        self.timestamps = self.timestamps[-100:]
        self.raw_data = self.raw_data[-100:]
        self.filtered_data = self.filtered_data[-100:]

        self.get_logger().info(
            f'Time: {timestamp:.2f}s, Raw Range: {distance:.3f}m, Filtered Range: {filtered_distance:.3f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = UWBRangeProcessor()

    plt.ion()
    fig, ax = plt.subplots()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Update the plot
            if node.timestamps:
                ax.clear()
                ax.plot(node.timestamps, node.raw_data, label='Raw Data', linestyle='--', marker='o')
                ax.plot(node.timestamps, node.filtered_data, label='Filtered Data', linestyle='-', marker='x')
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Range (m)')
                ax.set_title('UWB Range Data: Raw vs. Filtered')
                ax.legend()
                ax.grid()
                plt.pause(0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UWBRangeProcessor Node")
    finally:
        plt.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
