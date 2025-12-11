#!/usr/bin/env python3
"""
Desktop HITL Simulation for Astra-Rocket

This script simulates a simple rocket flight and sends sensor data to the
flight computer over USB Serial at a controlled rate. The FC processes the data
and sends back TELEM/ packets at its configured logging rate.

Protocol:
    1. Sim sends HITL/ packet at simulation rate (50Hz)
    2. FC processes and updates state
    3. FC sends TELEM/ at logging rate (configured in FC, typically 10-50Hz)
    4. Sim reads and logs all TELEM/ responses asynchronously

The sim paces itself to avoid overwhelming the FC's serial buffer while maintaining
accurate simulation timing.

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

    def __init__(self, dt=0.02, ignition_delay=3.0):
        self.dt = dt  # Timestep (50 Hz)
        self.state = SimState()
        self.g = 9.81  # Gravity (m/s^2)

        # Ignition delay - time to wait before motor ignites
        self.ignition_delay = ignition_delay

        # Rocket parameters (typical high-power rocket)
        self.mass = 5.0  # kg (11 lbs)
        self.motor_thrust = 3000.0  # N (typical I-motor produces ~300N average)
        self.motor_burnout = 1.0  # seconds
        self.drag_coeff = 0.02  # N/(m/s)^2 (more realistic for streamlined rocket)

    def compute_forces(self, state: SimState) -> Tuple[np.ndarray, np.ndarray]:
        """Compute forces and torques on rocket"""
        # Thrust (only after ignition delay and during motor burn)
        time_since_ignition = state.time - self.ignition_delay

        if 0 <= time_since_ignition < self.motor_burnout:
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

        # Ground contact (prevents falling through floor during ignition delay)
        if self.state.position[2] < 0:
            self.state.position[2] = 0
            # Only zero out velocity if falling down
            if self.state.velocity[2] < 0:
                self.state.velocity[2] = 0

        # Update time
        self.state.time += self.dt

        return self.state

    def get_sensor_data(self, state: SimState) -> dict:
        """Convert state to sensor readings"""
        # Accelerometers measure specific force (all forces EXCEPT gravity)
        # On the pad, they read +1g upward. In free fall, they read 0.
        force, _ = self.compute_forces(state)
        # Remove gravity from total force to get specific force
        specific_force = force - np.array([0, 0, -self.mass * self.g])
        accel_body = specific_force / self.mass

        # IMPORTANT: Mahony filter expects ENU frame (X=East, Y=North, Z=Up)
        # Our simulation is already in ENU, so no transformation needed
        # Just use accel_body directly

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
            'accel': accel_body,  # ENU frame (X=East, Y=North, Z=Up)
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

    # Simulation parameters
    dt = 0.02  # 50 Hz
    warmup_time = 1.0  # Hold at ground level for 1 second to let KF settle
    ignition_delay = 3.0  # Time on pad before motor ignites (additional settling time)
    max_time = 20.0  # 20 second flight (after warmup)

    # Track max altitude
    max_altitude = 0.0

    # Debug flag - set to True to see detailed packet data
    verbose = False
    packet_count = 0

    # Data collection for plotting
    time_data: List[float] = []
    sim_alt_data: List[float] = []
    fc_alt_data: List[float] = []

    # Track stage transitions for plotting
    stage_transitions: List[Tuple[float, str]] = []  # List of (time, stage_name)
    last_stage = None

    # Open CSV file for logging telemetry
    csv_filename = 'hitl_telemetry_log.csv'
    csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
    csv_writer = None  # Will be initialized when we get the header

    # Write CSV header now if we already have it from startup
    if header_columns is not None:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['SimTime', 'SimAlt', 'SimVel'] + header_columns)
        print(f"CSV file initialized with {len(header_columns) + 3} columns")

    # Warmup phase - send stationary data at steady rate while monitoring FC
    print("Warmup phase: Sending stationary data while FC initializes...")
    g = 9.81  # Gravity constant
    warmup_start = time.time()
    warmup_packet_count = 0
    fc_ready = False
    min_warmup_packets = int(warmup_time / dt)  # Minimum packets after FC ready
    packets_since_ready = 0

    while not fc_ready or packets_since_ready < min_warmup_packets:
        loop_start = time.time()

        # Send stationary sensor data
        sensor_data = {
            'timestamp': warmup_packet_count * dt,
            'accel': np.array([0.0, 0.0, g]),  # Just gravity
            'gyro': np.array([0.0, 0.0, 0.0]),
            'mag': np.array([20.0, 10.0, -45.0]),
            'pressure': 1013.25,  # Sea level pressure
            'temperature': 25.0,
            'gps_lat': 45.0,
            'gps_lon': -122.0,
            'gps_alt': 0.0,
            'gps_fix': 1,
            'gps_fix_quality': 8,
            'gps_heading': 0.0
        }

        packet = format_hitl_packet(sensor_data)
        ser.write(packet.encode())
        warmup_packet_count += 1

        # Read any available telemetry (non-blocking)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Check if FC reports it's ready
            if "Flight computer ready" in line or "Ready for simulation" in line:
                if not fc_ready:
                    print(f"  FC ready after {warmup_packet_count} packets! Settling KF for {warmup_time}s...")
                    fc_ready = True
                    packets_since_ready = 0

            # Look for telemetry
            if line.startswith("TELEM/"):
                if 'State - Time (s)' in line or 'State - Flight Stage' in line:
                    # Parse header if we haven't yet
                    if column_map is None:
                        column_map = parse_telem_header(line)
                        header_columns = line[6:].strip().split(',')
                        print(f"  Parsed telemetry header with {len(column_map)} columns")
                        # Initialize CSV with header
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(['SimTime', 'SimAlt', 'SimVel'] + header_columns)
                else:
                    # Got telemetry data
                    telem = parse_telem_line(line, column_map)

                    # Log telemetry to CSV during warmup
                    if csv_writer is not None:
                        sim_values = [
                            round((warmup_packet_count - 1) * dt, 2),  # Previous packet time
                            0.00,  # SimAlt = 0 during warmup
                            0.00   # SimVel = 0 during warmup
                        ]
                        csv_writer.writerow(sim_values + telem['values'])

                    # Mark FC as ready on first data telemetry
                    if not fc_ready:
                        print(f"  FC sending telemetry after {warmup_packet_count} packets! Settling KF for {warmup_time}s...")
                        fc_ready = True
                        packets_since_ready = 0
            else:
                # Non-telemetry messages
                if line:  # Only print non-empty lines
                    print(f"[FC] {line}")

        if fc_ready:
            packets_since_ready += 1

        # Maintain steady timing (50Hz = 20ms per loop)
        elapsed = time.time() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    warmup_duration = time.time() - warmup_start
    print(f"Warmup complete! Sent {warmup_packet_count} packets over {warmup_duration:.1f}s")

    # Reset simulation to start from zero after warmup
    sim = RocketSimulation(dt=dt, ignition_delay=ignition_delay)
    print(f"Simulation reset: time={sim.state.time}, altitude={sim.state.position[2]}, velocity={sim.state.velocity[2]}")
    print(f"Motor will ignite at t={ignition_delay:.1f}s")
    print(f"Starting flight simulation...\n")

    try:
        print("Starting main simulation loop...")

        while sim.state.time < max_time:
            loop_start = time.time()

            # Step simulation
            state = sim.step()
            sensor_data = sim.get_sensor_data(state)

            # Send HITL packet
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

            # Read all available telemetry (non-blocking)
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("TELEM/"):
                    # Check if this is the header line
                    if 'State - Time (s)' in line or 'State - Flight Stage' in line:
                        column_map = parse_telem_header(line)
                        print(f"\n[HEADER] Parsed telemetry header with {len(column_map)} columns")
                        if verbose:
                            print(f"  Column map: {column_map}")
                        # Initialize CSV writer with header if needed
                        if csv_writer is None:
                            header_line = line[6:].strip()
                            csv_writer = csv.writer(csv_file)
                            csv_writer.writerow(['SimTime', 'SimAlt', 'SimVel'] + header_line.split(','))
                    else:
                        # Got telemetry data
                        telem = parse_telem_line(line, column_map)

                        # Write to CSV
                        if csv_writer is not None:
                            sim_values = [
                                round(sim.state.time, 2),
                                round(state.position[2], 2),
                                round(state.velocity[2], 2)
                            ]
                            csv_writer.writerow(sim_values + telem['values'])

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

                            # Detect and record stage changes
                            stage_changed = (last_stage != fc_stage)
                            if stage_changed and last_stage is not None:
                                # Record transition for plotting
                                stage_transitions.append((sim.state.time, stage_name))

                            # Print status every 50 packets (~1 second at 50Hz) or on stage change
                            if (packet_count % 50 == 0) or stage_changed:
                                # Show pre-launch status differently
                                if sim.state.time < sim.ignition_delay:
                                    status = f"[PAD HOLD - T-{sim.ignition_delay - sim.state.time:.1f}s]"
                                else:
                                    status = f"[{sim.state.time:6.2f}s]"
                                print(f"{status} SimAlt: {state.position[2]:7.2f}m  "
                                      f"KF_PZ: {fc_pz:>7}m  "
                                      f"Vel: {state.velocity[2]:7.2f}m/s  "
                                      f"Stage: {stage_name}")
                                last_stage = fc_stage
                        else:
                            # Fallback if no column map available yet
                            if verbose or packet_count <= 3:
                                print(f"<<< RECEIVED TELEM (no column map yet): {len(telem['values'])} values")
                else:
                    # Non-telemetry messages (events, logs)
                    if line:  # Only print non-empty lines
                        print(f"[FC] {line}")

            # Track max altitude
            if state.position[2] > max_altitude:
                max_altitude = state.position[2]

            # Stop if landed and stationary
            if state.position[2] <= 0 and state.time > 10.0:
                print("\nRocket has landed. Ending simulation.")
                break

            # Maintain steady timing (50Hz = 20ms per loop)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

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

        # Add vertical lines for stage transitions
        stage_colors = {
            'PAD': 'gray',
            'BOOST': 'green',
            'COAST': 'orange',
            'APOGEE': 'red',
            'EXP_DROGUE': 'purple',
            'DROGUE': 'purple',
            'EXP_MAIN': 'blue',
            'MAIN': 'blue',
            'LANDED': 'black'
        }

        for trans_time, stage_name in stage_transitions:
            color = stage_colors.get(stage_name, 'gray')
            plt.axvline(x=trans_time, color=color, linestyle='--', alpha=0.6, linewidth=1.5)
            # Add text label for the stage
            plt.text(trans_time, plt.ylim()[1] * 0.95, stage_name,
                    rotation=90, verticalalignment='top', fontsize=9,
                    color=color, fontweight='bold')

        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Altitude (m)', fontsize=12)
        plt.title('Altitude Comparison: Simulation vs Kalman Filter', fontsize=14, fontweight='bold')
        plt.legend(fontsize=11, loc='upper left')
        plt.grid(True, alpha=0.3)

        # Add max altitude annotations
        max_sim_alt = max(sim_alt_data) if sim_alt_data else 0
        max_fc_alt = max(fc_alt_data) if fc_alt_data else 0
        stage_count = len(stage_transitions)
        plt.text(0.02, 0.85, f'Max Sim Alt: {max_sim_alt:.2f} m\nMax FC Alt: {max_fc_alt:.2f} m\nStage Transitions: {stage_count}',
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
