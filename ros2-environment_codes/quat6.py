import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from vqf import VQF
import math
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import csv

class IMUVQFVisualizer(Node):
    def __init__(self):
        super().__init__('imu_vqf_visualizer')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('sampling_rate', 10.0)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.sampling_rate = self.get_parameter('sampling_rate').get_parameter_value().double_value

        # Subscribe to IMU data
        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic}")

        # VQF Filter
        self.vqf = VQF(1.0 / self.sampling_rate)
        self.orientation = np.array([1, 0, 0, 0])  # Initial quaternion

        # Matplotlib 3D Visualization
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Initialize cube properties
        self.cube_vertices = self.get_cube_vertices()
        self.cube_faces = self.get_cube_faces(self.cube_vertices)
        self.colors = ['red', 'green', 'blue', 'yellow', 'cyan', 'magenta']
        self.poly3d = Poly3DCollection(self.cube_faces, facecolors=self.colors, linewidths=1, edgecolors='k', alpha=0.6)
        self.ax.add_collection3d(self.poly3d)

        # Logging setup
        self.csv_file = open('imu_orientation_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'])

        plt.ion()
        plt.show()

    def imu_callback(self, msg):
        # Extract accelerometer and gyroscope data
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Update VQF filter
        self.vqf.update(gyro, accel)

        # Update orientation
        self.orientation = self.vqf.getQuat6D()

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)

        # Convert radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Log roll, pitch, and yaw
        self.csv_writer.writerow([roll_deg, pitch_deg, yaw_deg])
        self.csv_file.flush()

        # Update visualization
        rotation_matrix = R.from_quat(self.orientation).as_matrix()
        self.update_cube(rotation_matrix)

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def get_cube_vertices(self):
        """Defines vertices of a unit cube centered at the origin."""
        return np.array([
            [-0.5, -0.5, -0.5],
            [ 0.5, -0.5, -0.5],
            [ 0.5,  0.5, -0.5],
            [-0.5,  0.5, -0.5],
            [-0.5, -0.5,  0.5],
            [ 0.5, -0.5,  0.5],
            [ 0.5,  0.5,  0.5],
            [-0.5,  0.5,  0.5]
        ])

    def get_cube_faces(self, vertices):
        """Defines faces of the cube based on vertices."""
        return [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left face
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right face
        ]

    def update_cube(self, rotation_matrix):
        """Updates the cube visualization with a new rotation."""
        rotated_vertices = np.dot(rotation_matrix, self.cube_vertices.T).T
        rotated_faces = self.get_cube_faces(rotated_vertices)

        self.poly3d.set_verts(rotated_faces)
        plt.draw()
        plt.pause(0.001)

    def run(self):
        """Run the ROS node."""
        rclpy.spin(self)

    def destroy_node(self):
        """Clean up resources on node shutdown."""
        super().destroy_node()
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = IMUVQFVisualizer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
