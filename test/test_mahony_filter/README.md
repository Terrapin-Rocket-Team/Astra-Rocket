# Mahony Filter Test & Visualization

This directory contains tests for the Mahony AHRS (Attitude and Heading Reference System) filter, including live visualization tools.

## Files

- `test_mahony_filter.cpp` - C++ unit tests for the Mahony filter
- `visualize_mahony.py` - Python script for live orientation visualization
- `README.md` - This file

## Test Description

The test program simulates a complete rocket flight scenario with four phases:

1. **Stationary on pad (0-2s)**: Filter calibration with gravity alignment
2. **Liftoff with rotation (2-4s)**: Spinning and tilting during ascent
3. **Apogee tumble (4-6s)**: Random tumbling motion at apogee
4. **Stable descent (6-8s)**: Slow spin under parachute

The test outputs CSV data with sensor readings and computed orientation that can be visualized in real-time.

## Running the Tests

### 1. Basic Test (Without Visualization)

```bash
# Run the test
pio test -e native -f test_mahony_filter
```

This runs the unit tests and prints CSV data to stdout.

### 2. Live Visualization

To see the orientation data plotted in real-time:

```bash
# Install Python dependencies first (one-time setup)
pip install numpy matplotlib

# Run test with live visualization
pio test -e native -f test_mahony_filter | python test/test_mahony_filter/visualize_mahony.py --live
```

### 3. Save and Replay Data

You can also save the test data and replay it:

```bash
# Save test output to file
pio test -e native -f test_mahony_filter > mahony_test_data.csv

# Visualize saved data
python test/test_mahony_filter/visualize_mahony.py --file mahony_test_data.csv
```

## Visualization Features

The Python visualization script shows:

1. **Euler Angles (Roll, Pitch, Yaw)**: Time-series plot showing orientation changes
2. **3D Orientation Viewer**: Real-time 3D visualization of the body frame
3. **Accelerometer Data**: Raw acceleration measurements in m/s²
4. **Gyroscope Data**: Angular velocity in rad/s
5. **Quaternion Components**: Raw quaternion values (w, x, y, z)

## Understanding the Output

### CSV Format
```
time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,roll,pitch,yaw
```

- `time`: Simulation time in seconds
- `ax,ay,az`: Accelerometer readings in m/s²
- `gx,gy,gz`: Gyroscope readings in rad/s
- `qw,qx,qy,qz`: Orientation quaternion
- `roll,pitch,yaw`: Euler angles in degrees

### Coordinate Frame

The filter uses an **ENU (East-North-Up)** coordinate system:
- **X-axis**: Points East (Red arrow in 3D view)
- **Y-axis**: Points North (Green arrow in 3D view)
- **Z-axis**: Points Up (Blue arrow in 3D view)

### What to Look For

**Successful Filter Behavior:**
- Smooth quaternion transitions (no jumps or discontinuities)
- Euler angles track expected motion
- Roll/pitch/yaw values are reasonable (-180° to +180°)
- 3D visualization shows smooth rotation

**Potential Issues:**
- Quaternion flipping (w,x,y,z suddenly flip sign)
- Gimbal lock (pitch near ±90°)
- Divergence (orientation drifts over time)
- Noise amplification (jittery output despite smooth input)

## Modifying the Test

To test different scenarios, edit `test_mahony_filter.cpp`:

### Change Filter Parameters
```cpp
mahony = new MahonyAHRS(Kp, Ki);  // Adjust Kp (proportional) and Ki (integral) gains
```

Typical values:
- `Kp = 0.1` to `2.0` (proportional gain, affects correction speed)
- `Ki = 0.0005` to `0.01` (integral gain, affects bias correction)

### Add Custom Motion Profiles
```cpp
// Example: Constant roll rate
gyro = Vector<3>(M_PI / 4.0, 0, 0);  // 45 deg/s roll
accel = Vector<3>(0, 0, -9.81);       // No linear acceleration

for (int i = 0; i < 100; i++) {
    mahony->update(accel, gyro, 0.02);  // 20ms updates
    // ... output data
}
```

### Test Magnetometer (9-DoF)
```cpp
// Use 9-DoF update with magnetometer
Vector<3> mag(0, 25, 0);  // Magnetic field in µT (pointing North)
mahony->update(accel, gyro, mag, dt);
```

## Troubleshooting

### "Python not found"
Make sure Python 3 is installed and in your PATH:
```bash
python --version  # Should show Python 3.x
```

### "No module named 'numpy'" or "'matplotlib'"
Install dependencies:
```bash
pip install numpy matplotlib
```

### "Test not found"
Make sure you're in the project root directory:
```bash
cd c:/Users/Drew/Documents/Astra-Rocket
```

### Visualization window doesn't appear
Try updating matplotlib:
```bash
pip install --upgrade matplotlib
```

## Further Reading

- [Mahony Filter Paper](http://www.olliw.eu/2013/imu-data-fusing/)
- [Quaternion Math](https://www.3dgep.com/understanding-quaternions/)
- [Euler Angles and Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock)
