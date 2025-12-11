#!/usr/bin/env python3
"""
Desktop HITL Simulation for Astra-Rocket

This script simulates a simple rocket flight and sends sensor data to the
flight computer over USB Serial. It receives telemetry back and can be used
to iterate on flight software without deploying to hardware.

Requirements:
    pip install pyserial numpy matplotlib

Usage:
    python desktop_simulation.py /dev/ttyACM0  # Linux/Mac
    python desktop_simulation.py COM3          # Windows
"""

import serial
import time
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Dict

@dataclass
class SimState:
    """Simulation state variables"""
    time: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # [x, y, z] in meters
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # [vx, vy, vz] in m/s
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # [roll, pitch, yaw] in radians
    ang_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # [wx, wy, wz] in rad/s

class RocketSimulation:
    """Simple 3DOF rocket flight simulation"""

    def __init__(self, dt=0.02):
        self.dt = dt  # Timestep (50 Hz)
        self.state = SimState()
        self.g = 9.81  # Gravity (m/s^2)

        # Rocket parameters (typical high-power rocket)
        self.mass = 5.0  # kg (11 lbs)
        self.motor_thrust = 3000.0  # N (typical I-motor produces ~300N average)
        self.motor_burnout = 1.0  # seconds
        self.drag_coeff = 0.02  # N/(m/s)^2 (more realistic for streamlined rocket)

    def compute_forces(self, state: SimState) -> Tuple[np.ndarray, np.ndarray]:
        """Compute forces and torques on rocket"""
        # Thrust (only during motor burn)
        if state.time < self.motor_burnout:
            thrust = np.array([0, 0, self.motor_thrust])
        else:
            thrust = np.array([0, 0, 0])

        # Gravity
        gravity = np.array([0, 0, -self.mass * self.g])

        # Drag (simplified)
        v_mag = np.linalg.norm(state.velocity)
        if v_mag > 0:
            drag = -self.drag_coeff * v_mag**2 * (state.velocity / v_mag)
        else:
            drag = np.array([0, 0, 0])

        # Total force
        force = thrust + gravity + drag

        # No torques in this simple sim
        torque = np.array([0, 0, 0])

        return force, torque

    def step(self) -> SimState:
        """Advance simulation by one timestep"""
        # Compute forces
        force, torque = self.compute_forces(self.state)

        # Update velocity and position (Euler integration)
        acceleration = force / self.mass
        self.state.velocity += acceleration * self.dt
        self.state.position += self.state.velocity * self.dt

        # Ground contact
        if self.state.position[2] < 0:
            self.state.position[2] = 0
            self.state.velocity[2] = 0

        # Update time
        self.state.time += self.dt

        return self.state

    def get_sensor_data(self, state: SimState) -> dict:
        """Convert state to sensor readings"""
        # Acceleration in body frame (including gravity)
        force, _ = self.compute_forces(state)
        accel_body = force / self.mass + np.array([0, 0, self.g])

        # Pressure from altitude (barometric formula)
        # Calculate in Pa, then convert to hPa for the flight computer
        pressure_pa = 101325.0 * (1 - 0.0065 * state.position[2] / 288.15)**5.255
        pressure = pressure_pa / 100.0  # Convert Pa to hPa

        # GPS position (lat, lon, alt)
        # Assume launch site at 45.0°N, 122.0°W
        lat = 45.0 + (state.position[0] / 111320.0)  # 1 degree lat ≈ 111.32 km
        lon = -122.0 + (state.position[1] / (111320.0 * np.cos(np.radians(45.0))))
        alt = state.position[2]

        return {
            'timestamp': state.time,
            'accel': accel_body,
            'gyro': state.ang_velocity,
            'mag': np.array([20.0, 10.0, -45.0]),  # Constant magnetic field
            'pressure': pressure,
            'temperature': 25.0,  # Constant temp
            'gps_lat': lat,
            'gps_lon': lon,
            'gps_alt': alt,
            'gps_fix': 1 if state.time > 1.0 else 0,  # GPS fix after 1 second
            'gps_fix_quality': 8 if state.time > 1.0 else 0,
            'gps_heading': 0.0
        }

def format_hitl_packet(sensor_data: dict) -> str:
    """Format sensor data as HITL protocol message"""
    return (f"HITL/{sensor_data['timestamp']:.3f},"
            f"{sensor_data['accel'][0]:.6f},{sensor_data['accel'][1]:.6f},{sensor_data['accel'][2]:.6f},"
            f"{sensor_data['gyro'][0]:.6f},{sensor_data['gyro'][1]:.6f},{sensor_data['gyro'][2]:.6f},"
            f"{sensor_data['mag'][0]:.3f},{sensor_data['mag'][1]:.3f},{sensor_data['mag'][2]:.3f},"
            f"{sensor_data['pressure']:.2f},{sensor_data['temperature']:.2f},"
            f"{sensor_data['gps_lat']:.8f},{sensor_data['gps_lon']:.8f},{sensor_data['gps_alt']:.2f},"
            f"{sensor_data['gps_fix']},{sensor_data['gps_fix_quality']},{sensor_data['gps_heading']:.2f}\n")

def parse_telem_line(line: str, column_map: Optional[Dict[str, int]] = None) -> dict:
    """Parse TELEM/ line from flight computer"""
    # Simple parsing - you can extend this to extract specific fields
    if not line.startswith("TELEM/"):
        return {}

    # Strip prefix and split by comma
    data_str = line[6:].strip()
    values = data_str.split(',')

    result = {
        'raw': data_str,
        'values': values
    }

    # If we have a column map, extract specific fields
    if column_map:
        result['fields'] = {}
        for field_name, col_idx in column_map.items():
            if col_idx < len(values):
                result['fields'][field_name] = values[col_idx]

    return result

def parse_telem_header(header_line: str) -> dict:
    """Parse TELEM/ header line to create column mapping"""
    if not header_line.startswith("TELEM/"):
        return {}

    # Strip prefix and split by comma
    header_str = header_line[6:].strip()
    columns = header_str.split(',')

    # Create a mapping of interesting column names to their indices
    column_map = {}
    for idx, col_name in enumerate(columns):
        column_map[col_name.strip()] = idx

    return column_map

def main():
    if len(sys.argv) < 2:
        print("Usage: python desktop_simulation.py <serial_port>")
        print("Example: python desktop_simulation.py /dev/ttyACM0")
        sys.exit(1)

    port = sys.argv[1]
    baud = 115200

    print("===========================================")
    print("  Astra-Rocket HITL Desktop Simulation")
    print("===========================================")
    print(f"Connecting to {port} at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected!")
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port: {e}")
        sys.exit(1)

    # Column mapping from header (will be populated when we receive header)
    column_map: Optional[Dict[str, int]] = None
    header_columns: Optional[List[str]] = None

    # Clear any startup messages and look for header
    time.sleep(0.5)
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        # Check if this is the telemetry header
        if line.startswith("TELEM/") and ('State - Time (s)' in line or 'State - Flight Stage' in line):
            column_map = parse_telem_header(line)
            header_columns = line[6:].strip().split(',')  # Save header for CSV
            print(f"[FC] [HEADER] Parsed telemetry header with {len(column_map)} columns")
        else:
            print(f"[FC] {line}")

    print("\nStarting simulation...")
    print("-" * 60)

    # Create simulation
    sim = RocketSimulation(dt=0.02)  # 50 Hz
    max_time = 20.0  # 20 second flight

    # Track max altitude
    max_altitude = 0.0

    # Debug flag - set to True to see detailed packet data
    verbose = False
    packet_count = 0

    # Data collection for plotting
    time_data: List[float] = []
    sim_alt_data: List[float] = []
    fc_alt_data: List[float] = []

    # Track last stage for detecting changes
    last_stage = None

    # Open CSV file for logging telemetry
    csv_filename = 'hitl_telemetry_log.csv'
    csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
    csv_writer = None  # Will be initialized when we get the header

    try:
        while sim.state.time < max_time:
            # Step simulation
            state = sim.step()

            # Get sensor data
            sensor_data = sim.get_sensor_data(state)

            # Format and send HITL packet
            packet = format_hitl_packet(sensor_data)
            ser.write(packet.encode())
            packet_count += 1

            # Show detailed packet data for first few packets or if verbose
            if verbose or packet_count <= 3:
                print(f"\n>>> SENT TO FC (packet {packet_count}):")
                print(f"    Time: {sensor_data['timestamp']:.3f}s")
                print(f"    Accel: [{sensor_data['accel'][0]:7.2f}, {sensor_data['accel'][1]:7.2f}, {sensor_data['accel'][2]:7.2f}] m/s²")
                print(f"    Pressure: {sensor_data['pressure']:.2f} hPa")
                print(f"    Altitude: {state.position[2]:.2f} m")
                print(f"    Velocity: {state.velocity[2]:.2f} m/s")
                print(f"    Raw packet: {packet.strip()}")

            # Read telemetry response (with timeout)
            start_time = time.time()
            telem_received = False
            while time.time() - start_time < 0.1:  # 100ms timeout
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("TELEM/"):
                        # Check if this is the header line
                        if 'State - Time (s)' in line or 'State - Flight Stage' in line:
                            # Parse header to get column mapping
                            column_map = parse_telem_header(line)
                            print(f"\n[HEADER] Parsed telemetry header with {len(column_map)} columns")
                            if verbose:
                                print(f"  Column map: {column_map}")

                            # Initialize CSV writer with header
                            if csv_writer is None:
                                # Add simulation columns to the header
                                header_line = line[6:].strip()  # Remove "TELEM/" prefix
                                csv_writer = csv.writer(csv_file)
                                csv_writer.writerow(['SimTime', 'SimAlt', 'SimVel'] + header_line.split(','))
                            break

                        # Parse data line
                        telem = parse_telem_line(line, column_map)
                        telem_received = True

                        # Write to CSV
                        if csv_writer is not None:
                            csv_writer.writerow([sim.state.time, state.position[2], state.velocity[2]] + telem['values'])

                        # Extract fields using column mapping if available
                        if column_map and 'fields' in telem:
                            fields = telem['fields']
                            fc_time = fields.get('State - Time (s)', 'N/A')
                            fc_stage = fields.get('State - Flight Stage', 'N/A')
                            fc_pz = fields.get('State - PZ (m)', 'N/A')
                            fc_alt = fields.get('HITL_Barometer - Alt ASL (m)', 'N/A')

                            # Collect data for plotting
                            try:
                                time_data.append(sim.state.time)
                                sim_alt_data.append(state.position[2])
                                fc_alt_data.append(float(fc_pz) if fc_pz != 'N/A' else 0.0)
                            except (ValueError, IndexError):
                                pass

                            # Show response for first few packets or if verbose
                            if verbose or packet_count <= 3:
                                print(f"<<< RECEIVED FROM FC:")
                                print(f"    FC Time: {fc_time}s")
                                print(f"    Flight Stage: {fc_stage}")
                                print(f"    FC Altitude (Baro): {fc_alt} m")
                                print(f"    FC State PZ (KF): {fc_pz} m")

                            # Only show periodic status updates (every 1 second) or major events
                            # Check for stage changes
                            stage_names = ['PAD', 'BOOST', 'COAST', 'APOGEE', 'EXP_DROGUE',
                                         'DROGUE', 'EXP_MAIN', 'MAIN', 'LANDED']
                            try:
                                stage_name = stage_names[int(float(fc_stage))] if fc_stage != 'N/A' and int(float(fc_stage)) < len(stage_names) else fc_stage
                            except (ValueError, IndexError):
                                stage_name = fc_stage

                            # Print status every 50 packets (~1 second at 50Hz) or on stage change
                            stage_changed = (last_stage != fc_stage)
                            if (packet_count % 50 == 0) or stage_changed:
                                print(f"[{sim.state.time:6.2f}s] SimAlt: {state.position[2]:7.2f}m  "
                                      f"KF_PZ: {fc_pz:>7}m  "
                                      f"Vel: {state.velocity[2]:7.2f}m/s  "
                                      f"Stage: {stage_name}")
                                last_stage = fc_stage
                        else:
                            # Fallback if no column map available yet
                            if verbose or packet_count <= 3:
                                print(f"<<< RECEIVED TELEM (no column map yet): {len(telem['values'])} values")
                        break
                    else:
                        # Non-telemetry messages (events, logs)
                        print(f"[FC] {line}")

            if not telem_received and (verbose or packet_count <= 3):
                print(f"    WARNING: No telemetry response received!")

            # Track max altitude
            if state.position[2] > max_altitude:
                max_altitude = state.position[2]

            # Stop if landed and stationary
            if state.position[2] <= 0 and state.time > 10.0:
                print("\nRocket has landed. Ending simulation.")
                break

    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
    finally:
        # Close CSV file
        csv_file.close()
        print(f"\nTelemetry data saved to '{csv_filename}'")

    print("-" * 60)
    print(f"\nSimulation complete!")
    print(f"Max altitude: {max_altitude:.2f} m")
    print(f"Flight time: {sim.state.time:.2f} s")
    print(f"Data points collected: {len(time_data)}")

    ser.close()

    # Plot altitude comparison
    if len(time_data) > 0:
        print("\nGenerating altitude comparison plot...")

        plt.figure(figsize=(12, 6))
        plt.plot(time_data, sim_alt_data, 'b-', label='Simulation Truth', linewidth=2)
        plt.plot(time_data, fc_alt_data, 'r--', label='Kalman Filter (State PZ)', linewidth=2)

        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Altitude (m)', fontsize=12)
        plt.title('Altitude Comparison: Simulation vs Kalman Filter', fontsize=14, fontweight='bold')
        plt.legend(fontsize=11)
        plt.grid(True, alpha=0.3)

        # Add max altitude annotations
        max_sim_alt = max(sim_alt_data) if sim_alt_data else 0
        max_fc_alt = max(fc_alt_data) if fc_alt_data else 0
        plt.text(0.02, 0.98, f'Max Sim Alt: {max_sim_alt:.2f} m\nMax FC Alt: {max_fc_alt:.2f} m',
                 transform=plt.gca().transAxes, fontsize=10,
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plt.savefig('hitl_altitude_comparison.png', dpi=150)
        print("Plot saved as 'hitl_altitude_comparison.png'")
        plt.show()
    else:
        print("\nNo telemetry data received - cannot generate plot.")

if __name__ == "__main__":
    main()
