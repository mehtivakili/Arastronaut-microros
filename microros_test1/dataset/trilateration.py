#!/usr/bin/env python3

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.interpolate import interp1d

bag_path = '/home/mehdi/table_trj2'

# Anchors configuration with colors for plotting
anchors = {
    81.0: {'pos': np.array([0.5, 0.0, 4.5]), 'color': 'blue'},
    82.0: {'pos': np.array([4.1, 7.35, 4.5]), 'color': 'green'},
    83.0: {'pos': np.array([4.1, 0.0, 4.5]), 'color': 'red'},
    84.0: {'pos': np.array([0.5, 7.35, 4.5]), 'color': 'purple'}
}

# Read bag file
data = {aid: {'times': [], 'ranges': []} for aid in anchors}
with Reader(bag_path) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/uwb/data_raw':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            anchor_id = msg.min_range
            range_val = msg.range  # Test mm later if needed
            if anchor_id in anchors:
                data[anchor_id]['times'].append(time_sec)
                data[anchor_id]['ranges'].append(range_val)

if not any(data[aid]['ranges'] for aid in anchors):
    print("No valid data found.")
    exit()

# Normalize timestamps and print data stats
start_time = min(min(d['times']) for d in data.values() if d['times'])
for aid in data:
    data[aid]['times'] = [t - start_time for t in data[aid]['times']]
    print(f"Anchor {aid}: {len(data[aid]['times'])} measurements")

# Interpolation functions
interp_funcs = {}
for aid in anchors:
    times = data[aid]['times']
    ranges = data[aid]['ranges']
    if len(times) >= 2:
        interp_funcs[aid] = interp1d(times, ranges, bounds_error=False, fill_value="extrapolate")
    else:
        interp_funcs[aid] = None
        print(f"Anchor {aid} has insufficient data for interpolation ({len(times)} points)")

# Kalman Filter for smoothing
class KalmanFilter1D:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        self.x = 0
        self.P = 1
        self.Q = process_noise
        self.R = measurement_noise

    def update(self, z):
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x

kalman_filters = {aid: KalmanFilter1D() for aid in anchors}

# Trilateration
def trilateration(positions, distances):
    def residuals(x, positions, distances):
        return np.array([np.linalg.norm(x - p) - d for p, d in zip(positions, distances)])
    initial_guess = np.mean(positions, axis=0) * [1, 1, 0.5]  # Bias z downward
    result = least_squares(residuals, initial_guess, args=(positions, distances))
    return result.x, result.cost

# Set up interactive plotting
plt.ion()  # Turn on interactive mode
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot anchors once
for aid, info in anchors.items():
    pos = info['pos']
    ax.scatter(pos[0], pos[1], pos[2], color=info['color'], label=f'Anchor {int(aid)}', s=100)

# Initialize tag path plot
tag_x, tag_y, tag_z = [], [], []
line, = ax.plot([], [], [], color='black', label='Tag Path', marker='o')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D UWB Tag Position (Online Plotting)')
ax.legend()
ax.grid(True)

# Process all unique timestamps with online plotting
times = sorted(set(t for aid in data for t in data[aid]['times']))
print(f"Total unique timestamps: {len(times)}")

for t in times:
    current_ranges = []
    current_positions = []
    print(f"\nProcessing t={t:.2f}s:")
    for aid in anchors:
        if interp_funcs[aid] is not None:
            interp_range = interp_funcs[aid](t)
            smoothed_range = kalman_filters[aid].update(interp_range)
            current_ranges.append(smoothed_range)
            current_positions.append(anchors[aid]['pos'])
            print(f"  Anchor {aid}: interp_range={interp_range:.2f}, smoothed_range={smoothed_range:.2f}")
        else:
            print(f"  Anchor {aid}: No interpolation possible")
    
    if len(current_ranges) >= 3:
        tag_pos, residual = trilateration(np.array(current_positions), np.array(current_ranges))
        print(f"  t={t:.2f}s, x={tag_pos[0]:.2f}, y={tag_pos[1]:.2f}, z={tag_pos[2]:.2f}, residual={residual:.4f}")
        
        # Update tag path
        tag_x.append(tag_pos[0])
        tag_y.append(tag_pos[1])
        tag_z.append(tag_pos[2])
        
        # Update plot
        line.set_data_3d(tag_x, tag_y, tag_z)
        ax.relim()  # Recalculate limits
        ax.autoscale_view()  # Adjust view to new data
        plt.draw()
        plt.pause(0.1)  # Pause briefly to update display

# Finalize plot
print(f"\nPlotted points: {len(tag_x)} out of {len(times)} processed timestamps")
print(f"Tag Z-range: {min(tag_z):.2f} to {max(tag_z):.2f} meters")
plt.ioff()  # Turn off interactive mode
plt.show()

# Retry with mm to m if needed (optional)
if not tag_x:
    print("\nNo positions calculated. Trying ranges in millimeters...")
    data = {aid: {'times': [], 'ranges': []} for aid in anchors}
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/uwb/data_raw':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                anchor_id = msg.min_range
                range_val = msg.range / 1000
                if anchor_id in anchors:
                    data[anchor_id]['times'].append(time_sec)
                    data[anchor_id]['ranges'].append(range_val)
    
    start_time = min(min(d['times']) for d in data.values() if d['times'])
    for aid in data:
        data[aid]['times'] = [t - start_time for t in data[aid]['times']]
        print(f"Anchor {aid} (mm to m): {len(data[aid]['times'])} measurements")
    
    interp_funcs = {}
    for aid in anchors:
        times = data[aid]['times']
        ranges = data[aid]['ranges']
        if len(times) >= 2:
            interp_funcs[aid] = interp1d(times, ranges, bounds_error=False, fill_value="extrapolate")
        else:
            interp_funcs[aid] = None
            print(f"Anchor {aid} (mm to m): Insufficient data ({len(times)} points)")
    
    kalman_filters = {aid: KalmanFilter1D() for aid in anchors}
    times = sorted(set(t for aid in data for t in data[aid]['times']))
    print(f"Total unique timestamps (mm to m): {len(times)}")
    
    plt.ion()
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    for aid, info in anchors.items():
        pos = info['pos']
        ax.scatter(pos[0], pos[1], pos[2], color=info['color'], label=f'Anchor {int(aid)}', s=100)
    
    tag_x, tag_y, tag_z = [], [], []
    line, = ax.plot([], [], [], color='black', label='Tag Path', marker='o')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D UWB Tag Position (mm to m, Online Plotting)')
    ax.legend()
    ax.grid(True)
    
    for t in times:
        current_ranges = []
        current_positions = []
        print(f"\nProcessing t={t:.2f}s (mm to m):")
        for aid in anchors:
            if interp_funcs[aid] is not None:
                interp_range = interp_funcs[aid](t)
                smoothed_range = kalman_filters[aid].update(interp_range)
                current_ranges.append(smoothed_range)
                current_positions.append(anchors[aid]['pos'])
                print(f"  Anchor {aid}: interp_range={interp_range:.2f}, smoothed_range={smoothed_range:.2f}")
            else:
                print(f"  Anchor {aid}: No interpolation possible")
        
        if len(current_ranges) >= 3:
            tag_pos, residual = trilateration(np.array(current_positions), np.array(current_ranges))
            print(f"  t={t:.2f}s, x={tag_pos[0]:.2f}, y={tag_pos[1]:.2f}, z={tag_pos[2]:.2f}, residual={residual:.4f}")
            
            tag_x.append(tag_pos[0])
            tag_y.append(tag_pos[1])
            tag_z.append(tag_pos[2])
            
            line.set_data_3d(tag_x, tag_y, tag_z)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.1)
    
    print(f"\nPlotted points (mm to m): {len(tag_x)} out of {len(times)} processed timestamps")
    print(f"Tag Z-range (mm to m): {min(tag_z):.2f} to {max(tag_z):.2f} meters")
    plt.ioff()
    plt.show()