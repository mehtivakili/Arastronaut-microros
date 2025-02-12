import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from vqf import VQF
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
import time
import threading
import queue


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

        # Matplotlib 3D Visualization
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.configure_plot()
        self.cube_vertices = self.get_cube_vertices()

        # Queue for multi-threaded data communication
        self.data_queue = queue.Queue(maxsize=10)

        # Start a thread to handle visualization updates
        self.visualization_thread = threading.Thread(target=self.visualization_loop)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()

        # Start a thread to handle data processing (VQF and calibration)
        self.data_thread = threading.Thread(target=self.data_processing_loop)
        self.data_thread.daemon = True
        self.data_thread.start()

        # Show the plot
        plt.ion()
        plt.show()

    def configure_plot(self):
        """Configure the static properties of the 3D plot."""
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def imu_callback(self, msg):
        """Callback function for IMU data."""
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        self.get_logger().info(f"IMU Data - Accel: {accel}, Gyro: {gyro}")

        # Put data into the queue for processing thread
        try:
            self.data_queue.put_nowait(('imu', accel, gyro))
        except queue.Full:
            pass  # Drop data if queue is full

    def mag_callback(self, msg):
        """Callback function for Magnetometer data."""
        mag_data = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        self.get_logger().info(f"Magnetometer Data: {mag_data}")

        # Put data into the queue for processing thread
        try:
            self.data_queue.put_nowait(('mag', mag_data))
        except queue.Full:
            pass  # Drop data if queue is full

    def data_processing_loop(self):
        """Loop to process IMU, magnetometer data, and update VQF filter."""
        while rclpy.ok():
            try:
                data_type, data = self.data_queue.get(timeout=1)  # Block if no data
                if data_type == 'imu':
                    accel, gyro = data
                    if self.mag_data is not None:
                        self.vqf.update(gyro, accel, self.mag_data)
                    else:
                        self.vqf.update(gyro, accel)

                    self.orientation = self.vqf.getQuat6D()

                elif data_type == 'mag':
                    self.mag_data = data

            except queue.Empty:
                continue  # Continue if the queue is empty

    def visualization_loop(self):
        """Loop to update visualization at the specified rate."""
        last_update_time = time.time()
        update_interval = 1.0 / self.sampling_rate  # Update rate

        while rclpy.ok():
            # Wait until the specified update interval
            current_time = time.time()
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time

                # Update the plot with new orientation data
                if self.orientation is not None:
                    rotation_matrix = R.from_quat(self.orientation).as_matrix()
                    self.update_cube(rotation_matrix)

                plt.draw()
                plt.pause(0.001)

            time.sleep(0.01)  # Small delay to prevent high CPU usage

    def get_cube_vertices(self):
        """Defines vertices of a unit cube centered at the origin."""
        return np.array([
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5]
        ])

    def update_cube(self, rotation_matrix):
        """Updates the cube visualization with a new rotation."""
        rotated_vertices = np.dot(rotation_matrix, self.cube_vertices.T).T
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
        ]

        self.ax.cla()  # Clear the plot area
        self.configure_plot()

        for edge in edges:
            p1, p2 = rotated_vertices[edge[0]], rotated_vertices[edge[1]]
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')

        plt.draw()
        plt.pause(0.001)

    def run(self):
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
