import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from transforms3d.euler import quat2euler
from rclpy.qos import qos_profile_sensor_data
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from vqf import VQF
import numpy as np
import csv
from scipy.spatial.transform import Rotation as R


class IMUVQFVisualizer(Node):
    def __init__(self):
        super().__init__('imu_vqf_visualizer')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag_raw')
        self.declare_parameter('sampling_rate', 10.0)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        self.sampling_rate = self.get_parameter('sampling_rate').get_parameter_value().double_value

        # Subscriptions
        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(MagneticField, mag_topic, self.mag_callback, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic}")
        self.get_logger().info(f"Subscribed to Magnetometer topic: {mag_topic}")

        # VQF Filter
        self.vqf = VQF(1.0 / self.sampling_rate)
        self.mag_data = None  # Placeholder for magnetometer data
        self.orientation = np.array([1, 0, 0, 0])  # Initial quaternion

        # CSV Logging
        # self.csv_file = open('imu_roll_pitch_yaw.csv', 'w', newline='')
        # self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow(['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'])

        # Visualization
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.cube_vertices = self.get_cube_vertices()
        self.cube_faces = self.get_cube_faces(self.cube_vertices)
        self.face_colors = ['red', 'green', 'blue', 'yellow', 'cyan', 'magenta']

        self.poly3d = Poly3DCollection(self.cube_faces, facecolors=self.face_colors, edgecolors='k', alpha=0.75)
        self.ax.add_collection3d(self.poly3d)

        plt.ion()
        plt.show()

    def imu_callback(self, msg):
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
        roll, pitch, yaw = quat2euler(self.orientation, axes='sxyz')
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Log angles to CSV
        # self.csv_writer.writerow([roll_deg, pitch_deg, yaw_deg])
        # self.csv_file.flush()

        # Update visualization
        rotation_matrix = self.quaternion_to_matrix(self.orientation)
        self.update_cube(rotation_matrix)

    def mag_callback(self, msg):
        self.mag_data = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        # self.get_logger().info(f"Magnetometer Data: {self.mag_data}")

    def get_cube_vertices(self):
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
        return [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left face
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right face
        ]

    def quaternion_to_matrix(self, quaternion):
        return R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]]).as_matrix()

    def update_cube(self, rotation_matrix):
        rotated_vertices = np.dot(rotation_matrix, self.cube_vertices.T).T
        rotated_faces = self.get_cube_faces(rotated_vertices)
        self.poly3d.set_verts(rotated_faces)
        plt.draw()
        plt.pause(0.001)

    def run(self):
        rclpy.spin(self)

    def destroy_node(self):
        super().destroy_node()
        # self.csv_file.close()


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
