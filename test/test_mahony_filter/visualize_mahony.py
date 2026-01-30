#!/usr/bin/env python3
"""
Live Mahony Filter Visualization

This script reads CSV data from the Mahony filter test and plots:
1. Euler angles (roll, pitch, yaw) over time
2. 3D orientation visualization
3. Sensor data (accelerometer and gyroscope)

Usage:
    python visualize_mahony.py <data_file.csv>
    or run live:
    pio test -e native -f test_mahony_filter | python visualize_mahony.py --live
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import argparse
from collections import deque
import io

class MahonyVisualizer:
    def __init__(self, live_mode=False, buffer_size=200):
        self.live_mode = live_mode
        self.buffer_size = buffer_size

        # Data buffers
        self.time_buffer = deque(maxlen=buffer_size)
        self.roll_buffer = deque(maxlen=buffer_size)
        self.pitch_buffer = deque(maxlen=buffer_size)
        self.yaw_buffer = deque(maxlen=buffer_size)
        self.ax_buffer = deque(maxlen=buffer_size)
        self.ay_buffer = deque(maxlen=buffer_size)
        self.az_buffer = deque(maxlen=buffer_size)
        self.gx_buffer = deque(maxlen=buffer_size)
        self.gy_buffer = deque(maxlen=buffer_size)
        self.gz_buffer = deque(maxlen=buffer_size)
        self.mx_buffer = deque(maxlen=buffer_size)
        self.my_buffer = deque(maxlen=buffer_size)
        self.mz_buffer = deque(maxlen=buffer_size)
        self.qw_buffer = deque(maxlen=buffer_size)
        self.qx_buffer = deque(maxlen=buffer_size)
        self.qy_buffer = deque(maxlen=buffer_size)
        self.qz_buffer = deque(maxlen=buffer_size)

        # Setup figure
        self.setup_figure()

    def setup_figure(self):
        """Create the visualization figure with subplots"""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('Mahony Filter Live Visualization', fontsize=16, fontweight='bold')

        # Create grid for subplots
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # 1. Euler Angles (top row, full width)
        self.ax_euler = self.fig.add_subplot(gs[0, :])
        self.ax_euler.set_title('Orientation (Euler Angles)')
        self.ax_euler.set_xlabel('Time (s)')
        self.ax_euler.set_ylabel('Angle (degrees)')
        self.ax_euler.grid(True, alpha=0.3)
        self.line_roll, = self.ax_euler.plot([], [], 'r-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_euler.plot([], [], 'g-', label='Pitch', linewidth=2)
        self.line_yaw, = self.ax_euler.plot([], [], 'b-', label='Yaw', linewidth=2)
        self.ax_euler.legend(loc='upper right')

        # 2. 3D Orientation Visualization (middle left)
        self.ax_3d = self.fig.add_subplot(gs[1, 0], projection='3d')
        self.ax_3d.set_title('3D Orientation')
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_xlim([-1, 1])
        self.ax_3d.set_ylim([-1, 1])
        self.ax_3d.set_zlim([-1, 1])

        # Initialize 3D body axes
        self.arrow_x = None
        self.arrow_y = None
        self.arrow_z = None

        # 3. Accelerometer (middle center)
        self.ax_accel = self.fig.add_subplot(gs[1, 1])
        self.ax_accel.set_title('Accelerometer (m/s²)')
        self.ax_accel.set_xlabel('Time (s)')
        self.ax_accel.set_ylabel('Acceleration')
        self.ax_accel.grid(True, alpha=0.3)
        self.line_ax, = self.ax_accel.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_ay, = self.ax_accel.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_az, = self.ax_accel.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_accel.legend(loc='upper right')

        # 4. Gyroscope (middle right)
        self.ax_gyro = self.fig.add_subplot(gs[1, 2])
        self.ax_gyro.set_title('Gyroscope (rad/s)')
        self.ax_gyro.set_xlabel('Time (s)')
        self.ax_gyro.set_ylabel('Angular Velocity')
        self.ax_gyro.grid(True, alpha=0.3)
        self.line_gx, = self.ax_gyro.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_gy, = self.ax_gyro.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_gz, = self.ax_gyro.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_gyro.legend(loc='upper right')

        # 5. Magnetometer (bottom left)
        self.ax_mag = self.fig.add_subplot(gs[2, 0])
        self.ax_mag.set_title('Magnetometer (µT)')
        self.ax_mag.set_xlabel('Time (s)')
        self.ax_mag.set_ylabel('Magnetic Field')
        self.ax_mag.grid(True, alpha=0.3)
        self.line_mx, = self.ax_mag.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_my, = self.ax_mag.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_mz, = self.ax_mag.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_mag.legend(loc='upper right')

        # 6. Quaternion Components (bottom middle and right)
        self.ax_quat = self.fig.add_subplot(gs[2, 1:])
        self.ax_quat.set_title('Quaternion Components')
        self.ax_quat.set_xlabel('Time (s)')
        self.ax_quat.set_ylabel('Value')
        self.ax_quat.grid(True, alpha=0.3)
        self.line_qw, = self.ax_quat.plot([], [], 'k-', label='W', linewidth=2)
        self.line_qx, = self.ax_quat.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_qy, = self.ax_quat.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_qz, = self.ax_quat.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_quat.legend(loc='upper right')

    def quaternion_to_rotation_matrix(self, qw, qx, qy, qz):
        """Convert quaternion to rotation matrix"""
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        return R

    def update_3d_axes(self, qw, qx, qy, qz):
        """Update 3D body frame visualization"""
        # Clear previous arrows
        if self.arrow_x is not None:
            self.arrow_x.remove()
            self.arrow_y.remove()
            self.arrow_z.remove()

        # Get rotation matrix from quaternion
        R = self.quaternion_to_rotation_matrix(qw, qx, qy, qz)

        # Body frame axes in world coordinates
        x_axis = R[:, 0]
        y_axis = R[:, 1]
        z_axis = R[:, 2]

        # Draw body frame
        origin = [0, 0, 0]
        self.arrow_x = self.ax_3d.quiver(*origin, *x_axis, color='r',
                                          arrow_length_ratio=0.2, linewidth=2.5,
                                          label='X-axis')
        self.arrow_y = self.ax_3d.quiver(*origin, *y_axis, color='g',
                                          arrow_length_ratio=0.2, linewidth=2.5,
                                          label='Y-axis')
        self.arrow_z = self.ax_3d.quiver(*origin, *z_axis, color='b',
                                          arrow_length_ratio=0.2, linewidth=2.5,
                                          label='Z-axis')

    def add_data_point(self, data_line):
        """Parse and add a data point to buffers"""
        try:
            parts = data_line.strip().split(',')

            # Support both old format (14 columns) and new format (17 columns with mag)
            if len(parts) == 14:
                # Old format: time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,roll,pitch,yaw
                time = float(parts[0])
                ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                qw, qx, qy, qz = float(parts[7]), float(parts[8]), float(parts[9]), float(parts[10])
                roll, pitch, yaw = float(parts[11]), float(parts[12]), float(parts[13])
                mx, my, mz = 0.0, 0.0, 0.0  # No magnetometer data
            elif len(parts) == 17:
                # New format: time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw
                time = float(parts[0])
                ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                mx, my, mz = float(parts[7]), float(parts[8]), float(parts[9])
                qw, qx, qy, qz = float(parts[10]), float(parts[11]), float(parts[12]), float(parts[13])
                roll, pitch, yaw = float(parts[14]), float(parts[15]), float(parts[16])
            else:
                return False

            self.time_buffer.append(time)
            self.ax_buffer.append(ax)
            self.ay_buffer.append(ay)
            self.az_buffer.append(az)
            self.gx_buffer.append(gx)
            self.gy_buffer.append(gy)
            self.gz_buffer.append(gz)
            self.mx_buffer.append(mx)
            self.my_buffer.append(my)
            self.mz_buffer.append(mz)
            self.qw_buffer.append(qw)
            self.qx_buffer.append(qx)
            self.qy_buffer.append(qy)
            self.qz_buffer.append(qz)
            self.roll_buffer.append(roll)
            self.pitch_buffer.append(pitch)
            self.yaw_buffer.append(yaw)

            return True
        except (ValueError, IndexError):
            return False

    def update_plot(self, frame=None):
        """Update all plot elements"""
        if len(self.time_buffer) < 2:
            return

        times = list(self.time_buffer)

        # Update Euler angles
        self.line_roll.set_data(times, list(self.roll_buffer))
        self.line_pitch.set_data(times, list(self.pitch_buffer))
        self.line_yaw.set_data(times, list(self.yaw_buffer))
        self.ax_euler.relim()
        self.ax_euler.autoscale_view()

        # Update accelerometer
        self.line_ax.set_data(times, list(self.ax_buffer))
        self.line_ay.set_data(times, list(self.ay_buffer))
        self.line_az.set_data(times, list(self.az_buffer))
        self.ax_accel.relim()
        self.ax_accel.autoscale_view()

        # Update gyroscope
        self.line_gx.set_data(times, list(self.gx_buffer))
        self.line_gy.set_data(times, list(self.gy_buffer))
        self.line_gz.set_data(times, list(self.gz_buffer))
        self.ax_gyro.relim()
        self.ax_gyro.autoscale_view()

        # Update magnetometer
        self.line_mx.set_data(times, list(self.mx_buffer))
        self.line_my.set_data(times, list(self.my_buffer))
        self.line_mz.set_data(times, list(self.mz_buffer))
        self.ax_mag.relim()
        self.ax_mag.autoscale_view()

        # Update quaternion
        self.line_qw.set_data(times, list(self.qw_buffer))
        self.line_qx.set_data(times, list(self.qx_buffer))
        self.line_qy.set_data(times, list(self.qy_buffer))
        self.line_qz.set_data(times, list(self.qz_buffer))
        self.ax_quat.relim()
        self.ax_quat.autoscale_view()

        # Update 3D orientation (use latest quaternion)
        if len(self.qw_buffer) > 0:
            self.update_3d_axes(self.qw_buffer[-1], self.qx_buffer[-1],
                               self.qy_buffer[-1], self.qz_buffer[-1])

        self.fig.canvas.draw_idle()

    def load_from_file(self, filename):
        """Load data from CSV file"""
        print(f"Loading data from {filename}...")
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#') or line.startswith('time'):
                    continue
                self.add_data_point(line)
        print(f"Loaded {len(self.time_buffer)} data points")
        self.update_plot()
        plt.show()

    def run_live(self, input_stream):
        """Run live visualization from input stream"""
        print("Starting live visualization...")
        print("Reading data from stdin...")

        def data_generator():
            for line in input_stream:
                line = line.strip()
                if not line or line.startswith('#') or line.startswith('time'):
                    continue
                if self.add_data_point(line):
                    yield

        ani = FuncAnimation(self.fig, self.update_plot,
                           frames=data_generator,
                           interval=50, blit=False, repeat=False)
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Visualize Mahony Filter data')
    parser.add_argument('--live', action='store_true',
                       help='Read live data from stdin')
    parser.add_argument('--file', type=str,
                       help='Load data from CSV file')
    parser.add_argument('--buffer', type=int, default=200,
                       help='Buffer size for live mode (default: 200)')

    args = parser.parse_args()

    viz = MahonyVisualizer(live_mode=args.live, buffer_size=args.buffer)

    if args.file:
        viz.load_from_file(args.file)
    elif args.live:
        viz.run_live(sys.stdin)
    else:
        # Try to read from stdin
        print("No input specified. Reading from stdin...")
        print("Run with --help for usage information")
        viz.run_live(sys.stdin)

if __name__ == '__main__':
    main()
