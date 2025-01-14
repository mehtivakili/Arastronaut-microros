import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from vqf import VQF
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui
from scipy.spatial.transform import Rotation as R


class IMUVQFNode(Node):
    def __init__(self):
        super().__init__('imu_vqf_node')
        
        # Parameters
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag_raw')
        self.declare_parameter('sampling_rate', 10.0)
        
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        sampling_rate = self.get_parameter('sampling_rate').get_parameter_value().double_value
        
        # Subscriptions
        self.subscription_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        self.subscription_mag = self.create_subscription(
            MagneticField,
            mag_topic,
            self.mag_callback,
            10
        )
        
        # VQF Filter
        self.vqf = VQF(1.0 / sampling_rate)
        self.mag_data = None  # Placeholder for magnetometer data
        
        # Visualization Setup
        self.app = QtGui.QApplication([])
        self.view = gl.GLViewWidget()
        self.view.show()
        self.view.setWindowTitle('IMU Orientation Visualization')
        self.view.setCameraPosition(distance=5)
        self.cube = self.create_cube()
        self.view.addItem(self.cube)
        self.orientation = np.array([1, 0, 0, 0])  # Initial quaternion
        
        # Timer for Visualization Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(int(1000 / sampling_rate))  # Ensure integer interval

        # Test visualization without IMU data
        # Uncomment this line to verify visualization independently
        # self.test_visualization()

    def imu_callback(self, msg):
        # Debug: Received IMU data
        print("Received IMU data")
        
        # Extract accelerometer and gyroscope data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        print(f"Accel: {accel}, Gyro: {gyro}")

        # Update VQF filter
        if self.mag_data is not None:
            self.vqf.update(gyro, accel, self.mag_data)
        else:
            self.vqf.update(gyro, accel)
        
        # Update orientation
        self.orientation = self.vqf.getQuat6D()
        print(f"Orientation (quaternion): {self.orientation}")

    def mag_callback(self, msg):
        # Debug: Received Magnetometer data
        print("Received Magnetometer data")
        
        # Extract magnetometer data
        self.mag_data = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])
        print(f"Mag: {self.mag_data}")

    def create_cube(self):
        # Define vertices and faces for a unit cube
        verts = np.array([
            [1, 1, 1],
            [-1, 1, 1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, -1],
            [1, -1, -1]
        ])
        faces = np.array([
            [0, 1, 2],
            [0, 2, 3],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [1, 2, 6],
            [1, 6, 5],
            [0, 3, 7],
            [0, 7, 4]
        ])
        colors = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 0, 1, 1],
            [0, 1, 1, 1],
            [1, 0.5, 0, 1],
            [0.5, 0, 1, 1]
        ])
        meshdata = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
        return gl.GLMeshItem(meshdata=meshdata, smooth=False, shader='shaded', drawEdges=True, edgeColor=(1, 1, 1, 1))

    def update_visualization(self):
        # Debug: Updating visualization
        print(f"Updating visualization with orientation: {self.orientation}")
        
        # Convert quaternion to rotation matrix
        rotation = R.from_quat(self.orientation).as_matrix()
        
        # Apply the rotation to the cube
        self.cube.resetTransform()
        self.cube.rotate(90, 0, 0, 1)  # Align cube with the coordinate system
        self.cube.applyTransform(QtGui.QMatrix4x4(*rotation.flatten().tolist()))
        self.view.update()

    def test_visualization(self):
        """Simulates visualization without IMU data for testing."""
        from time import sleep
        for angle in range(0, 360, 10):
            self.orientation = R.from_euler('xyz', [angle, angle, angle], degrees=True).as_quat()
            self.update_visualization()
            sleep(0.1)

    def run(self):
        rclpy.spin(self)
        self.app.exec_()


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
