import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
from rclpy.qos import qos_profile_sensor_data
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from transforms3d.quaternions import quat2mat
import numpy as np

class EulerVisualizer(Node):
    def __init__(self):
        super().__init__('euler_visualizer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Replace with your IMU topic name
            self.listener_callback,
            qos_profile_sensor_data  # Use compatible QoS for sensor data
        )
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        # Extract quaternion
        q = msg.orientation
        quaternion = (q.w, q.x, q.y, q.z)  # Note: (w, x, y, z) for transforms3d

        # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
        roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')

        # Convert radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Print Euler angles in degrees
        print(f'Roll: {roll_deg:.2f}, Pitch: {pitch_deg:.2f}, Yaw: {yaw_deg:.2f}')

        # Update cube orientation visualization
        self.update_cube(quaternion)

    def update_cube(self, quaternion):
        # Convert quaternion to rotation matrix
        rotation_matrix = quat2mat(quaternion)

        # Define cube vertices
        cube_vertices = np.array([
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5]
        ])

        # Apply rotation to the cube vertices
        rotated_vertices = np.dot(rotation_matrix, cube_vertices.T).T

        # Define cube edges
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        # Clear and update the 3D plot
        self.ax.clear()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        for edge in edges:
            p1, p2 = rotated_vertices[edge[0]], rotated_vertices[edge[1]]
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')

        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = EulerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
