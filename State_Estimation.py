import gtsam
import numpy as np
import matplotlib.pyplot as plt
from gtsam.utils import plot

# Create an empty nonlinear factor graph
graph = gtsam.NonlinearFactorGraph()

# Create a prior noise model (diagonal sigmas: x, y, theta)
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

# Add a prior factor at the first pose (at origin)
first_pose = gtsam.Pose2(1, 0, 0)
graph.add(gtsam.PriorFactorPose2(0, first_pose, prior_noise))

# Create noise model for odometry
odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

# Add odometry factors between poses
odom_measurement1 = gtsam.Pose2(1.0, 0.0, 0.0)
graph.add(gtsam.BetweenFactorPose2(0, 1, odom_measurement1, odom_noise))

# Add odometry factors between poses
odom_measurement2 = gtsam.Pose2(1.0, 0.0, 3)
graph.add(gtsam.BetweenFactorPose2(1, 2, odom_measurement2, odom_noise))

# Create initial estimate
initial_estimate = gtsam.Values()
initial_estimate.insert(0, first_pose)
initial_estimate.insert(1, gtsam.Pose2(1.0, 0.0, 0.0))
initial_estimate.insert(2, gtsam.Pose2(2.0, 0.0, 0.0))

# Optimize using Levenberg-Marquardt optimization
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
result = optimizer.optimize()

# Calculate marginals for uncertainty ellipses
marginals = gtsam.Marginals(graph, result)

# Create figure for visualization
plt.figure(figsize=(10, 10))

# Plot the poses with uncertainty ellipses
for i in range(3):
    pose = result.atPose2(i)
    covariance = marginals.marginalCovariance(i)
    plot.plot_pose2_on_axes(plt.gca(), pose, axis_length=0.2, covariance=covariance)

# Plot connections between poses
for i in range(2):
    pose1 = result.atPose2(i)
    pose2 = result.atPose2(i+1)
    plt.plot([pose1.x(), pose2.x()], 
            [pose1.y(), pose2.y()], 'r--', label='Odometry Factor')

# Add labels and customize plot
plt.axis('equal')
plt.grid(True)
plt.title('Factor Graph Visualization with Uncertainty')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()

# Add pose labels
for i in range(2):
    pose = result.atPose2(i)
    plt.annotate(f'Pose {i}', 
                (pose.x(), pose.y()), 
                xytext=(10, 10), 
                textcoords='offset points')

plt.show()

# Print results
print("\nFinal Result:")
print(f"Pose 0: {result.atPose2(0)}")
print(f"Pose 1: {result.atPose2(1)}")