# Mahony Filter Testing Guide

Complete guide for testing the Mahony AHRS filter with simulated and real sensor data.

## Overview

This repository includes two ways to test the Mahony orientation filter:

1. **Native Tests** - Run simulated sensor data through the filter (no hardware needed)
2. **Hardware Tests** - Run with real IMU sensors on Teensy/STM32 boards

Both methods output CSV data that can be visualized in real-time using Python.

## Quick Start

### Option 1: Native Testing (No Hardware)

Test the filter with simulated rocket flight data:

```bash
# Install Python dependencies
pip install numpy matplotlib

# Run test with live visualization
pio test -e native -f test_mahony_filter | python test/test_mahony_filter/visualize_mahony.py --live
```

This simulates a complete rocket flight (pad, liftoff, rotation, tumble, descent) and displays:
- Euler angles (roll/pitch/yaw) over time
- 3D orientation visualization
- Sensor data graphs
- Quaternion components

### Option 2: Hardware Testing (With Real Sensors)

Test with real IMU hardware connected to your board:

```bash
# 1. Configure your IMU type in examples/MahonyFilterTest/MahonyFilterTest.cpp

# 2. Upload to board
pio run -e teensy41 -t upload  # or -e stm for STM32

# 3. Stream and visualize (Windows)
python examples/MahonyFilterTest/stream_serial.py COM5 | python test/test_mahony_filter/visualize_mahony.py --live

# 3. Stream and visualize (Linux/Mac)
python examples/MahonyFilterTest/stream_serial.py /dev/ttyACM0 | python test/test_mahony_filter/visualize_mahony.py --live
```

## What Gets Tested

The test scenarios cover:

### Phase 1: Calibration (0-2s)
- Board stationary and level
- Gyroscope bias calculation
- Gravity vector alignment
- Magnetometer sampling (if available)

### Phase 2: Liftoff with Rotation (2-4s)
- High angular velocity (360°/s spin)
- Oscillating tilt (±30°)
- Tests gyro integration accuracy

### Phase 3: Apogee Tumble (4-6s)
- Random 3D rotation
- Complex multi-axis motion
- Tests filter stability

### Phase 4: Stable Descent (6-8s)
- Slow rotation under parachute
- Tests long-term stability
- Checks for drift

## Directory Structure

```
Astra-Rocket/
├── test/test_mahony_filter/
│   ├── test_mahony_filter.cpp    # Native C++ tests
│   ├── visualize_mahony.py       # Python visualization script
│   └── README.md                 # Testing documentation
│
├── examples/MahonyFilterTest/
│   ├── MahonyFilterTest.cpp      # Hardware test sketch
│   ├── stream_serial.py          # Serial data streamer
│   └── README.md                 # Hardware setup guide
│
└── .pio/libdeps/native/TRT-Astra/src/Filters/
    └── Mahony.h                   # Mahony filter implementation
```

## Detailed Documentation

- **[Native Testing Guide](test/test_mahony_filter/README.md)** - How to run native tests
- **[Hardware Testing Guide](examples/MahonyFilterTest/README.md)** - How to test with real sensors
- **[Mahony Filter Code](c:\Users\Drew\Documents\Astra-Rocket\.pio\libdeps\native\TRT-Astra\src\Filters\Mahony.h)** - Filter implementation

## Expected Results

### Good Filter Performance:
✅ Smooth orientation transitions
✅ Euler angles track expected motion
✅ No sudden quaternion jumps
✅ 3D visualization matches physical motion
✅ Minimal drift during stationary periods

### Potential Issues:
❌ **Quaternion flipping** - Signs flip but orientation looks correct (this is OK)
❌ **Gimbal lock** - Pitch near ±90° causes roll/yaw confusion
❌ **Drift** - Yaw slowly drifts without magnetometer (expected)
❌ **Divergence** - Orientation becomes incorrect over time (tune gains)
❌ **Noise** - Jittery output (lower Kp gain or filter sensor data)

## Tuning the Filter

Edit filter parameters in the code:

```cpp
// In test code or hardware example
MahonyAHRS mahony(Kp, Ki);

// Recommended values:
// Kp = 0.1 to 2.0 (proportional gain - correction speed)
// Ki = 0.0005 to 0.01 (integral gain - bias correction)
```

**High Kp (1.0-2.0):**
- Fast response to changes
- More susceptible to noise
- Good for rapid motion

**Low Kp (0.1-0.5):**
- Smooth, filtered output
- Slower convergence
- Better for slow motion

**High Ki (0.01):**
- Fast gyro bias correction
- May oscillate
- Use with clean sensors

**Low Ki (0.0001):**
- Slow bias correction
- Very stable
- Use with noisy sensors

## Troubleshooting

### Native Tests

**Issue**: Test fails to compile
```bash
# Make sure Unity framework is installed
pio lib install --platform native
```

**Issue**: Python visualization doesn't appear
```bash
# Update matplotlib
pip install --upgrade matplotlib
```

**Issue**: "No module named 'numpy'"
```bash
pip install numpy matplotlib
```

### Hardware Tests

**Issue**: IMU not detected
- Check I2C wiring (SDA, SCL, VCC, GND)
- Verify I2C address in code
- Run I2C scanner to find devices

**Issue**: Orientation is wrong
- Check IMU mounting orientation
- Verify which axis points "up"
- Board should be level during calibration

**Issue**: Serial port errors
```bash
# List available ports
python examples/MahonyFilterTest/stream_serial.py --list

# Check permissions (Linux)
sudo usermod -a -G dialout $USER
```

**Issue**: Data is choppy/laggy
- Lower update rate in firmware
- Reduce visualization buffer size
- Close other programs using CPU

## CSV Data Format

Both native and hardware tests output the same CSV format:

```csv
time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,roll,pitch,yaw
0.000,0.0,0.0,-9.81,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0
0.020,0.0,0.0,-9.81,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0
...
```

**Columns:**
- `time` - Seconds since start
- `ax,ay,az` - Acceleration (m/s²)
- `gx,gy,gz` - Angular velocity (rad/s)
- `qw,qx,qy,qz` - Quaternion orientation
- `roll,pitch,yaw` - Euler angles (degrees)

## Coordinate System

The filter uses **ENU (East-North-Up)** coordinates:
- **X-axis** → East (Red)
- **Y-axis** → North (Green)
- **Z-axis** → Up (Blue)

## Advanced Features

### Switch Filter Modes

```cpp
// During calibration (on pad)
mahony.setMode(MahonyMode::CALIBRATING);

// During flight (trust accel+gyro)
mahony.setMode(MahonyMode::CORRECTING);

// During high-G (gyro only)
mahony.setMode(MahonyMode::GYRO_ONLY);
```

### Magnetometer Calibration (9-DoF)

```cpp
// Collect samples during "mag dance"
mahony.collectMagCalibrationSample(mag);

// Finalize when done
mahony.finalizeCalibration();
```

### Get Earth-Frame Acceleration

```cpp
Vector<3> earthAccel = mahony.getEarthAcceleration(rawAccel);
// earthAccel.z() = vertical acceleration (gravity removed)
```

## Performance Metrics

**Native Tests:**
- Runtime: ~2-3 seconds for full simulation
- Memory: ~50KB stack usage
- CPU: Negligible (runs at full speed)

**Hardware Tests:**
- Update Rate: 50 Hz (20ms period)
- CPU Usage: <5% on Teensy 4.1
- Memory: <10KB RAM

## Next Steps

1. ✅ Run native tests to verify filter logic
2. ✅ Test with hardware sensors
3. ✅ Tune gains for your application
4. ✅ Compare with onboard sensor fusion (BNO055)
5. ✅ Integrate into full flight computer
6. ✅ Test during rocket ground tests
7. ✅ Validate during actual flight

## Contributing

Found a bug or have improvements? Please open an issue or PR!

## References

- [Mahony AHRS Original Paper](https://ieeexplore.ieee.org/document/4608934)
- [Complementary Filters Overview](http://www.olliw.eu/2013/imu-data-fusing/)
- [Quaternion Math Guide](https://www.3dgep.com/understanding-quaternions/)
- [Euler Angles and Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock)

## License

Same as Astra-Rocket repository.
