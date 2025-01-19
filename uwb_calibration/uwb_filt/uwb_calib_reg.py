import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class UWBRangeCalibrator(Node):
    def __init__(self):
        super().__init__('uwb_range_calibrator')

        self.get_logger().info("Initializing UWBRangeCalibrator Node")

        # Calibration data storage
        self.d_real = []  # Known real distances
        self.d_measured = []  # Corresponding measured distances
        self.current_real_distance = None  # Real distance currently being calibrated
        self.measurements = []  # Temporary storage for 50 measurements
        self.recording_limit = 50  # Number of measurements per real distance
        self.calibration_limit = 20  # Number of real distances to use for calibration

        # Calibration parameters
        self.bias = 0
        self.scale = 1

        # Subscription to UWB data
        self.subscription = self.create_subscription(
            Range,
            '/uwb/data_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /uwb/data_raw")

        self.recording_active = False  # To track if recording is active

    def start_recording(self, real_distance):
        """
        Start recording measured distances for a given real distance.
        """
        if len(self.d_real) >= self.calibration_limit:
            self.get_logger().warning("Calibration data collection complete. Run 'calibrate()' to compute parameters.")
            return

        self.current_real_distance = real_distance
        self.measurements = []
        self.recording_active = True
        self.get_logger().info(f"Started recording for real distance: {real_distance:.2f}m")

    def listener_callback(self, msg):
        """
        Callback to process UWB data and record measurements during calibration.
        """
        if not self.recording_active:
            return

        # Record the current measurement
        distance = msg.range  # Replace with filtered measurement if applicable
        self.measurements.append(distance)

        self.get_logger().info(f"Recording measurement {len(self.measurements)}/{self.recording_limit}: {distance:.3f}m")

        # Stop recording once 50 measurements are collected
        if len(self.measurements) >= self.recording_limit:
            # Average the 50 measurements
            average_distance = sum(self.measurements) / len(self.measurements)
            self.d_real.append(self.current_real_distance)
            self.d_measured.append(average_distance)

            self.get_logger().info(f"Finished recording for real distance: {self.current_real_distance:.2f}m, "
                                   f"Average measured distance: {average_distance:.3f}m")
            self.recording_active = False

    def calibrate(self):
        """
        Perform a linear fit to calculate the scale and bias.
        """
        if len(self.d_real) < 2:
            self.get_logger().error("Not enough data to calculate scale and bias. Add more calibration points.")
            return

        # Perform linear regression to find scale and bias
        A = np.vstack([self.d_real, np.ones(len(self.d_real))]).T
        self.scale, self.bias = np.linalg.lstsq(A, self.d_measured, rcond=None)[0]

        self.get_logger().info(f"Calibration complete: scale = {self.scale:.4f}, bias = {self.bias:.4f}")

    def save_calibration(self, filename="calibration_params.txt"):
        """
        Save the calibration parameters to a file.
        """
        with open(filename, "w") as f:
            f.write(f"Scale: {self.scale:.4f}\n")
            f.write(f"Bias: {self.bias:.4f}\n")
        self.get_logger().info(f"Calibration parameters saved to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = UWBRangeCalibrator()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            # Manual input for real distances
            if not node.recording_active and len(node.d_real) < node.calibration_limit:
                command = input("Enter a real distance (or type 'calibrate' to calculate parameters): ")

                if command.lower() == "calibrate":
                    node.calibrate()
                    node.save_calibration()
                else:
                    try:
                        real_distance = float(command)
                        node.start_recording(real_distance)
                    except ValueError:
                        print("Invalid input. Enter a numeric value for real distance or 'calibrate'.")

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UWBRangeCalibrator Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
