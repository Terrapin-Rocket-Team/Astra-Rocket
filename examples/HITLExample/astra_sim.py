import csv
import socket
import numpy as np
from dataclasses import dataclass

@dataclass
class PacketData:
    timestamp: float
    accel: np.ndarray  # [x, y, z] m/s^2
    gyro: np.ndarray   # [x, y, z] rad/s
    mag: np.ndarray    # [x, y, z] uT
    pressure: float    # hPa
    temp: float        # C
    lat: float
    lon: float
    alt: float
    fix: int
    sats: int
    heading: float

    def to_hitl_string(self) -> str:
        return (f"HITL/{self.timestamp:.3f},"
                f"{self.accel[0]:.4f},{self.accel[1]:.4f},{self.accel[2]:.4f},"
                f"{self.gyro[0]:.4f},{self.gyro[1]:.4f},{self.gyro[2]:.4f},"
                f"{self.mag[0]:.2f},{self.mag[1]:.2f},{self.mag[2]:.2f},"
                f"{self.pressure:.2f},{self.temp:.2f},"
                f"{self.lat:.7f},{self.lon:.7f},{self.alt:.2f},"
                f"{self.fix},{self.sats},{self.heading:.1f}\n")

class DataSource:
    def get_next_packet(self) -> PacketData: raise NotImplementedError
    def is_finished(self) -> bool: return False

class PhysicsSim(DataSource):
    def __init__(self):
        self.t = 0.0; self.alt = 0.0; self.vel = 0.0; self.dt = 0.02
        print("[Sim] Using Internal Physics Engine")
    def get_next_packet(self) -> PacketData:
        self.t += self.dt
        # Inertial acceleration (ENU frame, +Z is up)
        accel_z = 30.0 if 2.0 < self.t < 4.0 else (-9.81 if self.alt > 0 else 0.0)
        self.vel += accel_z * self.dt; self.alt += self.vel * self.dt
        if self.alt < 0: self.alt, self.vel, accel_z = 0, 0, 0

        # Accelerometer measures specific force = -(inertial_accel + gravity)
        # ENU: gravity = -9.81, so specific_force_z = -inertial_accel - (-9.81) = -accel_z + 9.81
        # At rest: -0 + 9.81 = +9.81 ❌ WAIT this is still wrong!
        # Actually: specific_force = -(inertial_accel) + gravity_force
        # gravity_force on accelerometer = -9.81 (downward)
        # At rest: inertial=0, specific = 0 - 9.81 = -9.81 ✓
        # Boost: inertial=30, specific = -30 - 9.81 = -39.81 ✓
        accel_measured = -accel_z - 9.81
        return PacketData(self.t, np.array([0., 0., accel_measured]), np.zeros(3), np.zeros(3),
                          1013.25 - (self.alt * 0.12), 25.0, 45.0, -122.0, self.alt, 1, 8, 0.0)

class CSVSim(DataSource):
    def __init__(self, filename):
        self.data = []
        self.index = 0
        self._load_csv(filename)
        print(f"[Sim] Ready. Duration: {self.data[-1].timestamp:.1f}s")

    def is_finished(self) -> bool:
        return self.index >= len(self.data)

    def get_next_packet(self) -> PacketData:
        if self.index < len(self.data):
            p = self.data[self.index]
            self.index += 1
            return p
        return self.data[-1]

    def _load_csv(self, filename):
        print(f"[Sim] Parsing {filename}...")
        with open(filename, 'r', encoding='utf-8-sig', errors='ignore') as f:
            lines = f.readlines()
        
        header_index = -1; header_map = {}; converters = {} 
        keys = {'time': ['time', 'timestamp'], 'alt': ['altitude', 'pos_z', 'height', 'alt asl'],
                'acc_z': ['acceleration z', 'accel z', 'az', 'vertical acc'], 'pres': ['pressure', 'baro'], 'temp': ['temperature', 'temp']}

        for i, line in enumerate(lines):
            clean_parts = [p.lower().replace('#','').replace('"','').strip() for p in line.split(',')]
            if len(clean_parts) < 2: continue
            found_cols = {}
            for col_idx, col_name in enumerate(clean_parts):
                for key_type, keywords in keys.items():
                    if any(k in col_name for k in keywords) and key_type not in found_cols:
                        found_cols[key_type] = col_idx
                        converters[key_type] = lambda x: x
                        if key_type == 'temp' and 'f' in col_name and 'c' not in col_name:
                            converters[key_type] = lambda x: (x - 32.0) * 5.0/9.0
                        elif key_type == 'alt' and ('ft' in col_name or 'feet' in col_name):
                            converters[key_type] = lambda x: x * 0.3048
                        elif key_type == 'acc_z' and ('g' in col_name and 'mag' not in col_name):
                            converters[key_type] = lambda x: x * 9.80665

            if 'time' in found_cols and 'alt' in found_cols:
                header_index = i; header_map = found_cols
                print(f"[Sim] Found Header at line {i+1}. Map: {header_map}")
                break
        
        if header_index == -1: raise ValueError("Headers not found.")

        count = 0
        for i in range(header_index + 1, len(lines)):
            line = lines[i].strip()
            if not line or line.startswith('#'): continue
            row = line.split(',')
            try:
                def get_val(k, d=0.0):
                    return converters[k](float(row[header_map[k]].strip())) if k in header_map and header_map[k] < len(row) else d
                sensor_z = get_val('acc_z')
                if count < 5 and abs(sensor_z) < 2.0: sensor_z += 9.81 
                self.data.append(PacketData(get_val('time'), np.array([0., 0., sensor_z]), np.zeros(3), np.zeros(3),
                                          get_val('pres', 1013.25), get_val('temp', 25.0), 45.0, -122.0, get_val('alt'), 1, 8, 0.0))
                count += 1
            except: continue
        print(f"[Sim] Successfully loaded {count} rows.")

class NetworkStreamSim(DataSource):
    def __init__(self, port=9000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.last_packet = PacketData(0, np.zeros(3), np.zeros(3), np.zeros(3), 1013, 25, 0,0,0, 1, 8, 0)
        print(f"[Sim] Listening for UDP data on port {port}")
    def get_next_packet(self) -> PacketData:
        try:
            data, _ = self.sock.recvfrom(4096)
            parts = data.decode().split(',')
            if len(parts) >= 5:
                self.last_packet.timestamp = float(parts[0])
                self.last_packet.accel = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                self.last_packet.alt = float(parts[4])
        except BlockingIOError: pass 
        return self.last_packet