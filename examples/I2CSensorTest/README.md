# I2C Sensor Test Example

This example tests all I2C sensors on the STM32H723VEH6 board:
- **DPS368** - Barometric Pressure Sensor
- **BMI088** - 6-axis IMU (Accelerometer + Gyroscope)
- **SAM-M10Q** - u-blox GPS Module

## Features

- I2C bus scanning to detect all connected devices
- Individual sensor initialization with status reporting
- Continuous sensor data reading every 500ms
- Detailed output showing all sensor readings

## Hardware Configuration

- **MCU**: STM32H723VEH6
- **I2C Pins**:
  - SDA: PB9
  - SCL: PB8
- **I2C Speed**: 400kHz

## Expected I2C Addresses

- DPS368: 0x77 (or 0x76)
- BMI088 Accelerometer: 0x18 (or 0x19)
- BMI088 Gyroscope: 0x68 (or 0x69)
- SAM-M10Q: 0x42

## Building

```bash
pio run -e stm32h723_sensor_test
```

## Uploading

```bash
pio run -e stm32h723_sensor_test -t upload
```

## Monitoring Serial Output

```bash
pio device monitor -b 115200
```

## Expected Output

The serial monitor will display:
1. I2C bus scan results showing detected devices
2. Sensor initialization status
3. Periodic sensor readings including:
   - Pressure, temperature, and altitude from DPS368
   - Acceleration and gyroscope data from BMI088
   - GPS position, speed, and satellite info from SAM-M10Q

## Troubleshooting

### No I2C devices found
- Check I2C pin connections (SDA: PB9, SCL: PB8)
- Verify sensors are powered (3.3V)
- Check pull-up resistors on I2C lines (typically 4.7kΩ)

### Sensor initialization failed
- Verify correct I2C address for your hardware variant
- Check for I2C bus conflicts
- Ensure sensor is properly connected and powered

### GPS shows "No GPS fix"
- GPS needs clear view of sky
- First fix can take 30+ seconds
- Check antenna connection if using external antenna

## Customization

If your I2C pins are different, modify these lines in the code:

```cpp
#define I2C_SDA PB9  // Change to your SDA pin
#define I2C_SCL PB8  // Change to your SCL pin
```
