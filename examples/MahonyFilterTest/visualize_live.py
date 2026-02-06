#!/usr/bin/env python3
"""
Direct serial port visualization for Mahony Filter

Usage:
    python visualize_live.py COM8
    python visualize_live.py /dev/ttyACM0
"""

import sys
import math
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)

def quat_normalize(q):
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm == 0:
        return q
    return q / norm

def quat_to_euler_deg(q):
    qw, qx, qy, qz = q
    # Match MahonyFilterTest.cpp formulas (roll, pitch, yaw)
    roll = math.atan2(2.0 * (qw * qx + qy * qz),
                      1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class LiveVisualizer:
    def __init__(self, port, baud=115200, buffer_size=200, relative=True):
        self.port = port
        self.baud = baud
        self.buffer_size = buffer_size
        self.relative = relative
        self.q_ref = None

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

        # Setup serial
        self.ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud")

        # Setup figure
        self.setup_figure()

    def setup_figure(self):
        self.fig = plt.figure(figsize=(16, 10))
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        mode = "Relative to Start" if self.relative else "Absolute"
        self.fig.suptitle(f'Mahony Filter Live ({mode}) - {self.port}', fontsize=16, fontweight='bold')

        # 3D Orientation (top-left, spanning 2 rows)
        self.ax_3d = self.fig.add_subplot(gs[0:2, 0], projection='3d')
        self.ax_3d.set_title('3D Orientation', fontweight='bold')
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])

        # Euler angles (top-middle)
        self.ax_euler = self.fig.add_subplot(gs[0, 1])
        self.ax_euler.set_title('Orientation (Euler Angles)')
        self.ax_euler.set_xlabel('Time (s)')
        self.ax_euler.set_ylabel('Angle (degrees)')
        self.ax_euler.grid(True, alpha=0.3)
        self.line_roll, = self.ax_euler.plot([], [], 'r-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_euler.plot([], [], 'g-', label='Pitch', linewidth=2)
        self.line_yaw, = self.ax_euler.plot([], [], 'b-', label='Yaw', linewidth=2)
        self.ax_euler.legend(loc='upper right', fontsize=8)

        # Accelerometer (top-right)
        self.ax_accel = self.fig.add_subplot(gs[0, 2])
        self.ax_accel.set_title('Accelerometer (m/s²)')
        self.ax_accel.set_xlabel('Time (s)')
        self.ax_accel.set_ylabel('Acceleration')
        self.ax_accel.grid(True, alpha=0.3)
        self.line_ax, = self.ax_accel.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_ay, = self.ax_accel.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_az, = self.ax_accel.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_accel.legend(loc='upper right', fontsize=8)

        # Gyroscope (middle-middle)
        self.ax_gyro = self.fig.add_subplot(gs[1, 1])
        self.ax_gyro.set_title('Gyroscope (rad/s)')
        self.ax_gyro.set_xlabel('Time (s)')
        self.ax_gyro.set_ylabel('Angular Velocity')
        self.ax_gyro.grid(True, alpha=0.3)
        self.line_gx, = self.ax_gyro.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_gy, = self.ax_gyro.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_gz, = self.ax_gyro.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_gyro.legend(loc='upper right', fontsize=8)

        # Magnetometer (middle-right)
        self.ax_mag = self.fig.add_subplot(gs[1, 2])
        self.ax_mag.set_title('Magnetometer (µT)')
        self.ax_mag.set_xlabel('Time (s)')
        self.ax_mag.set_ylabel('Magnetic Field')
        self.ax_mag.grid(True, alpha=0.3)
        self.line_mx, = self.ax_mag.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_my, = self.ax_mag.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_mz, = self.ax_mag.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_mag.legend(loc='upper right', fontsize=8)

        # Status text (bottom row)
        self.ax_status = self.fig.add_subplot(gs[2, :])
        self.ax_status.axis('off')
        self.status_text = self.ax_status.text(0.5, 0.5, 'Waiting for data...',
                                               fontsize=12, verticalalignment='center',
                                               horizontalalignment='center')

    def parse_line(self, line):
        try:
            parts = line.strip().split(',')
            if len(parts) == 17:
                time = float(parts[0])
                ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                mx, my, mz = float(parts[7]), float(parts[8]), float(parts[9])
                qw, qx, qy, qz = float(parts[10]), float(parts[11]), float(parts[12]), float(parts[13])
                q = np.array([qw, qx, qy, qz], dtype=float)

                if self.relative:
                    if self.q_ref is None:
                        self.q_ref = q
                    q = quat_mul(q, quat_conj(self.q_ref))
                    q = quat_normalize(q)

                roll, pitch, yaw = quat_to_euler_deg(q)

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
                self.qw_buffer.append(q[0])
                self.qx_buffer.append(q[1])
                self.qy_buffer.append(q[2])
                self.qz_buffer.append(q[3])
                self.roll_buffer.append(roll)
                self.pitch_buffer.append(pitch)
                self.yaw_buffer.append(yaw)
                return True
        except:
            pass
        return False

    def quaternion_to_rotation_matrix(self, qw, qx, qy, qz):
        """Convert quaternion to rotation matrix"""
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        return R

    def draw_3d_orientation(self):
        """Draw 3D orientation axes"""
        self.ax_3d.clear()
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])
        self.ax_3d.set_xlabel('X (East)', fontsize=9)
        self.ax_3d.set_ylabel('Y (North)', fontsize=9)
        self.ax_3d.set_zlabel('Z (Up)', fontsize=9)
        self.ax_3d.set_title('3D Orientation', fontweight='bold')

        if len(self.qw_buffer) == 0:
            return

        # Get latest quaternion
        qw = self.qw_buffer[-1]
        qx = self.qx_buffer[-1]
        qy = self.qy_buffer[-1]
        qz = self.qz_buffer[-1]

        # Convert to rotation matrix
        R = self.quaternion_to_rotation_matrix(qw, qx, qy, qz)

        # Define body frame axes (in body coordinates)
        origin = np.array([0, 0, 0])
        length = 1.0

        # Body axes (rotated by orientation)
        x_axis = R @ np.array([length, 0, 0])
        y_axis = R @ np.array([0, length, 0])
        z_axis = R @ np.array([0, 0, length])

        # Draw axes
        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                         x_axis[0], x_axis[1], x_axis[2],
                         color='red', arrow_length_ratio=0.2, linewidth=3, label='X (Roll)')
        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                         y_axis[0], y_axis[1], y_axis[2],
                         color='green', arrow_length_ratio=0.2, linewidth=3, label='Y (Pitch)')
        self.ax_3d.quiver(origin[0], origin[1], origin[2],
                         z_axis[0], z_axis[1], z_axis[2],
                         color='blue', arrow_length_ratio=0.2, linewidth=3, label='Z (Yaw)')

        # Draw reference frame (Earth frame)
        alpha = 0.3
        self.ax_3d.quiver(0, 0, 0, 0.5, 0, 0, color='red',
                         arrow_length_ratio=0.15, linewidth=1, alpha=alpha, linestyle='--')
        self.ax_3d.quiver(0, 0, 0, 0, 0.5, 0, color='green',
                         arrow_length_ratio=0.15, linewidth=1, alpha=alpha, linestyle='--')
        self.ax_3d.quiver(0, 0, 0, 0, 0, 0.5, color='blue',
                         arrow_length_ratio=0.15, linewidth=1, alpha=alpha, linestyle='--')

        self.ax_3d.legend(loc='upper left', fontsize=8)

    def update(self, frame):
        # Read available data
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                if line and not line.startswith('#'):
                    self.parse_line(line)
            except:
                pass

        if len(self.time_buffer) < 2:
            return

        times = list(self.time_buffer)

        # Update 3D orientation
        self.draw_3d_orientation()

        # Update plots
        self.line_roll.set_data(times, list(self.roll_buffer))
        self.line_pitch.set_data(times, list(self.pitch_buffer))
        self.line_yaw.set_data(times, list(self.yaw_buffer))
        self.ax_euler.relim()
        self.ax_euler.autoscale_view()

        self.line_ax.set_data(times, list(self.ax_buffer))
        self.line_ay.set_data(times, list(self.ay_buffer))
        self.line_az.set_data(times, list(self.az_buffer))
        self.ax_accel.relim()
        self.ax_accel.autoscale_view()

        self.line_gx.set_data(times, list(self.gx_buffer))
        self.line_gy.set_data(times, list(self.gy_buffer))
        self.line_gz.set_data(times, list(self.gz_buffer))
        self.ax_gyro.relim()
        self.ax_gyro.autoscale_view()

        self.line_mx.set_data(times, list(self.mx_buffer))
        self.line_my.set_data(times, list(self.my_buffer))
        self.line_mz.set_data(times, list(self.mz_buffer))
        self.ax_mag.relim()
        self.ax_mag.autoscale_view()

        # Update status
        if len(self.time_buffer) > 0:
            mag_magnitude = 0
            if len(self.mx_buffer) > 0:
                mag_magnitude = np.sqrt(self.mx_buffer[-1]**2 +
                                       self.my_buffer[-1]**2 +
                                       self.mz_buffer[-1]**2)

            self.status_text.set_text(
                f'Data Points: {len(self.time_buffer)}  |  '
                f'Time: {times[-1]:.1f}s  |  '
                f'Roll: {self.roll_buffer[-1]:.1f}°  |  '
                f'Pitch: {self.pitch_buffer[-1]:.1f}°  |  '
                f'Yaw: {self.yaw_buffer[-1]:.1f}°  |  '
                f'Mag: {mag_magnitude:.1f} µT'
            )

    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False)
        plt.show()
        self.ser.close()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Mahony filter live visualization")
    parser.add_argument("port", help="Serial port (e.g., COM8 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--relative", dest="relative", action="store_true",
                        help="Show orientation relative to first sample (default)")
    parser.add_argument("--absolute", dest="relative", action="store_false",
                        help="Show absolute orientation")
    parser.set_defaults(relative=True)
    args = parser.parse_args()

    viz = LiveVisualizer(args.port, baud=args.baud, relative=args.relative)
    viz.run()
