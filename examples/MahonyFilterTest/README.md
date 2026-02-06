# Mahony Filter Hardware Test Example

This example tests the Mahony AHRS filter with real IMU hardware and provides live visualization of orientation data.

## Features

- Real-time orientation tracking using hardware IMU sensors
- Support for multiple IMU types (BNO055, BMI088+LIS3MDL)
- Automatic calibration on startup
- CSV output compatible with Python visualizer
- 50 Hz update rate for smooth visualization
- Works with Teensy 4.1 and STM32 boards

## Hardware Requirements

### Supported IMUs

**Option 1: BNO055 (9-DoF IMU)**
- Accelerometer + Gyroscope + Magnetometer (all-in-one)
- I2C Address: 0x28 or 0x29
- All-in-one sensor with built-in fusion
- Good for general applications

**Option 2: BMI088 + LIS3MDL (9-DoF IMU) ⭐ Recommended**
- BMI088: High-performance Accelerometer + Gyroscope
  - I2C Addresses: 0x18 (accel) + 0x68 (gyro)
  - Excellent for high-G environments (rocket launch)
- LIS3MDL: 3-axis Magnetometer
  - I2C Address: 0x1C (SA1=LOW) or 0x1E (SA1=HIGH)
  - Provides yaw stabilization
- Best performance for rocket applications
- This is the configuration you have!

### Wiring (I2C)

Connect your sensors to the board's I2C pins:

**Teensy 4.1:**
- SDA → Pin 18
- SCL → Pin 19
- VCC → 3.3V
- GND → GND

**STM32H723:**
- SDA → PB7 (or appropriate I2C pin)
- SCL → PB6 (or appropriate I2C pin)
- VCC → 3.3V
- GND → GND

**Note:** All sensors (BMI088 accel, BMI088 gyro, LIS3MDL mag) share the same I2C bus. Just connect them in parallel to SDA/SCL.

## Software Setup

### 1. Configure IMU Type

Edit [MahonyFilterTest.cpp](MahonyFilterTest.cpp) and uncomment your IMU:

```cpp
// Option 1: BNO055 9-DoF IMU (built-in magnetometer)
// #define USE_BNO055
// #define BNO055_ADDRESS 0x28  // or 0x29

// Option 2: BMI088 + LIS3MDL (9-DoF) ⭐ DEFAULT
#define USE_BMI088_LIS3MDL
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define LIS3MDL_SA1_STATE LIS3MDL::sa1_auto  // Auto-detect 0x1C or 0x1E
```

**The default configuration is already set for BMI088 + LIS3MDL, so you don't need to change anything!**

### 2. Install Python Dependencies

```bash
pip install numpy matplotlib pyserial
```

### 3. Build and Upload

**For Teensy 4.1:**
```bash
pio run -e teensy41 -t upload
```

**For STM32:**
```bash
pio run -e stm -t upload
```

## Running the Test

### Method 1: Live Visualization

1. **Upload the code** to your board
2. **Open serial monitor** to verify it's working
3. **Close serial monitor** (Python needs exclusive access)
4. **Run live visualization:**

```bash
# Windows (PowerShell)
python examples/MahonyFilterTest/stream_serial.py COM5 | python test/test_mahony_filter/visualize_mahony.py --live

# Linux/Mac
python examples/MahonyFilterTest/stream_serial.py /dev/ttyACM0 | python test/test_mahony_filter/visualize_mahony.py --live
```

Replace `COM5` or `/dev/ttyACM0` with your serial port.

### Method 2: Record and Replay

1. **Record data to file:**

```bash
# Windows
python examples/MahonyFilterTest/stream_serial.py COM5 > mahony_data.csv

# Linux/Mac
python examples/MahonyFilterTest/stream_serial.py /dev/ttyACM0 > mahony_data.csv
```

2. **Stop recording** with Ctrl+C after collecting data

3. **Visualize recorded data:**

```bash
python test/test_mahony_filter/visualize_mahony.py --file mahony_data.csv
```

## Serial Stream Helper

Here's a simple Python script to stream serial data (save as `stream_serial.py`):

```python
#!/usr/bin/env python3
import serial
import sys

if len(sys.argv) < 2:
    print("Usage: python stream_serial.py <serial_port>", file=sys.stderr)
    print("Example: python stream_serial.py COM5", file=sys.stderr)
    sys.exit(1)

port = sys.argv[1]
baud = 115200

try:
    ser = serial.Serial(port, baud, timeout=1)
    print(f"# Connected to {port} at {baud} baud", file=sys.stderr)

    while True:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='ignore'), end='')

except KeyboardInterrupt:
    print("\n# Stopped by user", file=sys.stderr)
except Exception as e:
    print(f"# Error: {e}", file=sys.stderr)
    sys.exit(1)
```

## Calibration Process

When you first start the program:

1. **Keep board STATIONARY** for 2 seconds
2. Board should be sitting **level** on a flat surface
3. **Z-axis should point UP** (standard orientation)
4. Watch serial output for "Calibration complete!"

During calibration:
- Gyroscope bias is measured and removed
- Accelerometer gravity vector is aligned
- Magnetometer calibration (BNO055 only) accumulates samples

**For best magnetometer calibration (BNO055):**
- Slowly rotate board in all axes during calibration
- Make figure-8 motions to sample magnetic field
- Avoid metal objects and electronics nearby

## Testing Procedure

### Test 1: Stationary Baseline
- Leave board sitting level
- Verify orientation shows ~0° roll, ~0° pitch
- Yaw may drift slowly (expected without mag)

### Test 2: Roll Test
- Rotate board around X-axis (forward direction)
- Roll angle should track rotation accurately
- Try ±45°, ±90° rotations

### Test 3: Pitch Test
- Tilt board forward/backward (Y-axis)
- Pitch angle should track accurately
- Watch for smooth transitions

### Test 4: Yaw Test (BNO055 only)
- Rotate board around Z-axis (spin)
- Yaw should track rotation
- BMI088 will show drift (no mag)

### Test 5: Combined Motion
- Perform arbitrary 3D rotations
- Watch 3D visualization - arrows should match physical orientation
- Check for smooth quaternion values (no jumps)

## Troubleshooting

### "Could not initialize IMU"
- Check I2C wiring (SDA, SCL, power, ground)
- Verify I2C address is correct
- Try I2C scanner sketch to detect devices
- Check for loose connections

### "No module named 'serial'"
```bash
pip install pyserial
```

### Orientation is flipped/wrong
- Check IMU mounting orientation
- Verify which axis points "up"
- May need to adjust coordinate transform in code

### Yaw drifts continuously (BMI088)
- **Expected behavior** - no magnetometer
- Use BNO055 for yaw stabilization
- If you need gyro-only handling during high-G, do it in the application layer

### Jittery/noisy output
- Increase Mahony `Kp` gain for faster response
- Decrease `Kp` for smoother filtering
- Check sensor sampling rate
- Verify I2C isn't glitching

### Quaternion flips sign suddenly
- **Normal behavior** - q and -q represent same rotation
- Visualization should handle this correctly
- Euler angles will stay continuous

### Serial port already in use
- Close Arduino Serial Monitor
- Close other terminal programs
- On Linux: check for ModemManager interference

### Slow/laggy visualization
- Reduce `buffer_size` in visualizer
- Lower update rate in firmware
- Close other matplotlib windows

## Understanding the Output

### During Calibration:
```
# Mahony Filter Hardware Test
# ========================================
# Initializing IMU... OK
# IMU: BNO055 9-DoF (with magnetometer)
# Calibrating... Keep the board STATIONARY!
```

### After Calibration:
```
# Calibration complete!
# CSV Data Format:
# time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,roll,pitch,yaw
time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,roll,pitch,yaw
0.000,0.123,0.045,-9.810,0.001,-0.002,0.000,1.0000,0.0000,0.0000,0.0000,0.00,0.00,0.00
0.020,0.125,0.043,-9.812,0.002,-0.001,0.001,0.9999,0.0001,0.0000,0.0001,0.12,0.05,0.02
...
```

### What Each Column Means:
- `time`: Seconds since calibration complete
- `ax,ay,az`: Acceleration in m/s² (includes gravity)
- `gx,gy,gz`: Angular velocity in rad/s
- `qw,qx,qy,qz`: Orientation quaternion components
- `roll,pitch,yaw`: Euler angles in degrees

## Filter Tuning

Edit these values in the code to tune filter behavior:

```cpp
const double MAHONY_KP = 0.5;      // Proportional gain
const double MAHONY_KI = 0.001;    // Integral gain
const double UPDATE_RATE = 50.0;   // Hz
```

**Kp (Proportional Gain):**
- **Higher** (1.0-2.0): Faster correction, less smooth, more responsive
- **Lower** (0.1-0.5): Smoother output, slower to converge, less noise
- **Default**: 0.5

**Ki (Integral Gain):**
- **Higher** (0.01): Faster bias correction, may oscillate
- **Lower** (0.0001): Slower bias correction, more stable
- **Default**: 0.001

**Update Rate:**
- **50-100 Hz**: Good balance for most applications
- **Higher**: More CPU load, better for fast motion
- **Lower**: Less data bandwidth, ok for slow motion

## Advanced Usage

### Get Earth Frame Acceleration

```cpp
Vector<3> accel = sensorManager.getAccel();
Vector<3> earthAccel = mahony.getEarthAcceleration(accel);
// earthAccel.z() is vertical acceleration (gravity removed)
```

### Check Filter Status

```cpp
if (mahony.isReady()) {
    // Orientation is valid
}

if (mahony.isMagCalibrated()) {
    // Magnetometer calibration is complete
}
```

## Next Steps

- Try different motion profiles
- Tune filter gains for your application
- Compare with onboard fusion (BNO055)
- Test during rocket flight simulation
- Integrate with full flight computer

## References

- [Mahony AHRS Paper](http://www.olliw.eu/2013/imu-data-fusing/)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
