/**
 * Mahony Filter Hardware Test Example
 *
 * This example demonstrates testing the Mahony AHRS filter with real IMU sensors.
 * It reads raw sensor data from an IMU and outputs orientation data over Serial
 * for live visualization with the Python script.
 *
 * Compatible with:
 * - Teensy 4.1 with BMI088 + LIS3MDL (9-DoF)
 * - Teensy 4.1 with BNO055 (9-DoF)
 * - STM32 with any supported IMU
 *
 * Output format (CSV):
 * time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw
 *
 * Usage:
 * 1. Upload this sketch to your board
 * 2. Connect via serial port
 * 3. Pipe output to Python visualizer:
 *    python stream_serial.py COM5 | python ../../test/test_mahony_filter/visualize_mahony.py --live
 *
 * Or use a serial monitor tool that can pipe to stdin
 */

#include <Arduino.h>
#include <Wire.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Sensors/HW/IMU/BNO055.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Filters/Mahony.h>
#include <Math/Vector.h>
#include "LIS3MDL_Wrapper.h"

using namespace astra;

// ============ HARDWARE CONFIGURATION ============
// Choose your IMU by uncommenting ONE of the following:

// Option 1: BNO055 9-DoF IMU (has built-in magnetometer)
// #define USE_BNO055
// #define BNO055_ADDRESS 0x28  // or 0x29

// Option 2: BMI088 6-DoF IMU + LIS3MDL Magnetometer = 9-DoF
#define USE_BMI088_LIS3MDL
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define LIS3MDL_SA1_STATE LIS3MDL::sa1_auto  // Auto-detect address (0x1C or 0x1E)

// ============ FILTER CONFIGURATION ============
const double MAHONY_KP = 0.5;      // Proportional gain (higher = faster correction)
const double MAHONY_KI = 0.001;    // Integral gain (gyro bias correction)
const double UPDATE_RATE = 50.0;   // Hz (50 Hz = 20ms update period)
const double DT = 1.0 / UPDATE_RATE;

// ============ CALIBRATION SETTINGS ============
const int CALIBRATION_SAMPLES = 100;  // Number of samples for initial calibration
const unsigned long CALIBRATION_TIME = 2000;  // Calibration time in milliseconds (stationary)
const unsigned long MAG_CALIBRATION_TIME = 10000;  // Magnetometer calibration time (moving)

// ============ GLOBAL OBJECTS ============
SensorManager sensorManager;
MahonyAHRS mahony(MAHONY_KP, MAHONY_KI);

#ifdef USE_BNO055
BNO055 bno055("BNO055", BNO055_ADDRESS, &Wire);
#endif

#ifdef USE_BMI088_LIS3MDL
BMI088 bmi088("BMI088", Wire, BMI088_ACCEL_ADDR, BMI088_GYRO_ADDR);
LIS3MDL_Wrapper magnetometer("LIS3MDL", LIS3MDL_SA1_STATE);
#endif

// Timing
unsigned long lastUpdate = 0;
unsigned long startTime = 0;
unsigned long magCalibStartTime = 0;
bool calibrationComplete = false;
bool magCalibrationComplete = false;
int calibrationCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    Serial.println("# ========================================");
    Serial.println("# Mahony Filter Hardware Test");
    Serial.println("# ========================================");
    Serial.println("#");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    delay(100);

    // Initialize IMU
    #ifdef USE_BNO055
    Serial.print("# Initializing BNO055... ");
    if (!bno055.init()) {
        Serial.println("FAILED!");
        Serial.println("# ERROR: Could not initialize BNO055");
        Serial.println("# Check I2C connections and addresses");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("OK");
    sensorManager.setPrimaryAccel(bno055.getAccelSensor());
    sensorManager.setPrimaryGyro(bno055.getGyroSensor());
    sensorManager.setPrimaryMag(bno055.getMagSensor());
    Serial.println("# IMU: BNO055 9-DoF (with built-in magnetometer)");
    #endif

    #ifdef USE_BMI088_LIS3MDL
    Serial.print("# Initializing BMI088... ");
    if (!bmi088.begin()) {
        Serial.println("FAILED!");
        Serial.println("# ERROR: Could not initialize BMI088");
        Serial.println("# Check I2C connections and addresses");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("OK");
    sensorManager.setPrimaryAccel(bmi088.getAccelSensor());
    sensorManager.setPrimaryGyro(bmi088.getGyroSensor());
    sensorManager.addMiscSensor(&bmi088);  // Add BMI088 as misc sensor for regular updates

    // Initialize magnetometer
    Serial.print("# Initializing LIS3MDL magnetometer... ");
    if (!magnetometer.begin()) {
        Serial.println("FAILED!");
        Serial.println("# ERROR: Could not initialize LIS3MDL");
        Serial.println("# Check I2C connections and address (should be 0x1C or 0x1E)");
        Serial.println("# Continuing without magnetometer (6-DoF mode)");
    } else {
        Serial.println("OK");
        sensorManager.setPrimaryMag(&magnetometer);
        Serial.println("# IMU: BMI088 + LIS3MDL = 9-DoF");
    }
    #endif

    sensorManager.begin();

    Serial.println("#");
    Serial.println("# Filter Configuration:");
    Serial.print("# - Kp (Proportional): ");
    Serial.println(MAHONY_KP, 4);
    Serial.print("# - Ki (Integral): ");
    Serial.println(MAHONY_KI, 6);
    Serial.print("# - Update Rate: ");
    Serial.print(UPDATE_RATE);
    Serial.println(" Hz");
    Serial.println("#");

    // Test sensor readings
    Serial.println("# Testing sensor readings...");
    sensorManager.update();
    Vector<3> testAccel = sensorManager.getAccel();
    Vector<3> testGyro = sensorManager.getGyro();
    Vector<3> testMag = sensorManager.getMag();

    Serial.print("# Test Accel: ");
    Serial.print(testAccel.x(), 3);
    Serial.print(", ");
    Serial.print(testAccel.y(), 3);
    Serial.print(", ");
    Serial.println(testAccel.z(), 3);

    Serial.print("# Test Gyro: ");
    Serial.print(testGyro.x(), 3);
    Serial.print(", ");
    Serial.print(testGyro.y(), 3);
    Serial.print(", ");
    Serial.println(testGyro.z(), 3);

    Serial.print("# Test Mag: ");
    Serial.print(testMag.x(), 3);
    Serial.print(", ");
    Serial.print(testMag.y(), 3);
    Serial.print(", ");
    Serial.println(testMag.z(), 3);

    Serial.println("#");
    Serial.println("# === CALIBRATION PHASE 1: STATIONARY ===");
    Serial.println("# Keep the board STATIONARY for gyro bias calibration!");
    Serial.println("#");

    mahony.setMode(MahonyMode::CALIBRATING);
    startTime = millis();
    lastUpdate = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // Update at fixed rate
    if (currentTime - lastUpdate < (unsigned long)(DT * 1000.0)) {
        return;
    }
    lastUpdate = currentTime;

    // Update sensors
    sensorManager.update();

    // Get sensor data
    Vector<3> accel = sensorManager.getAccel();
    Vector<3> gyro = sensorManager.getGyro();
    Vector<3> mag(0, 0, 0);
    bool hasMag = false;

    #if defined(USE_BNO055) || defined(USE_BMI088_LIS3MDL)
    mag = sensorManager.getMag();
    hasMag = true;
    #endif

    // Check if calibration is complete
    if (!calibrationComplete) {
        calibrationCount++;

        // Update filter in calibration mode (6-DoF for gyro bias)
        mahony.update(accel, gyro, DT);

        // Check if stationary calibration time has elapsed
        if (currentTime - startTime >= CALIBRATION_TIME) {
            mahony.finalizeCalibration();
            mahony.lockFrame();
            calibrationComplete = true;

            Serial.println("# Stationary calibration complete!");
            Serial.println("#");

            #if defined(USE_BNO055) || defined(USE_BMI088_LIS3MDL)
            if (hasMag) {
                Serial.println("# === CALIBRATION PHASE 2: MAGNETOMETER ===");
                Serial.println("# Rotate the board in ALL directions (the 'mag dance')!");
                Serial.println("# Move through as many orientations as possible...");
                Serial.println("#");
                magCalibStartTime = currentTime;
            } else {
                Serial.println("# No magnetometer detected - using 6-DoF mode");
                mahony.setMode(MahonyMode::CORRECTING);
                magCalibrationComplete = true;
            }
            #else
            Serial.println("# No magnetometer configured - using 6-DoF mode");
            mahony.setMode(MahonyMode::CORRECTING);
            magCalibrationComplete = true;
            #endif
        }
        return;
    }

    // Magnetometer calibration phase
    if (!magCalibrationComplete) {
        // Collect magnetometer samples while user rotates board
        if (hasMag) {
            mahony.collectMagCalibrationSample(mag);
        }

        // Check if mag calibration time has elapsed
        if (currentTime - magCalibStartTime >= MAG_CALIBRATION_TIME) {
            Serial.println("#");
            Serial.println("# Magnetometer calibration complete!");

            if (hasMag) {
                mahony.finalizeCalibration();  // This will compute mag calibration
                if (mahony.isMagCalibrated()) {
                    Serial.println("# Magnetometer successfully calibrated!");
                    Serial.println("# Using 9-DoF mode (accel + gyro + mag)");
                } else {
                    Serial.println("# Warning: Magnetometer calibration failed (not enough samples)");
                    Serial.println("# Falling back to 6-DoF mode");
                }
            }

            mahony.setMode(MahonyMode::CORRECTING);
            magCalibrationComplete = true;

            Serial.println("#");
            Serial.println("# CSV Data Format:");
            Serial.println("# time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw");
            Serial.println("#");
            Serial.println("# You can now move/rotate the board");
            Serial.println("# Pipe this output to visualize_live.py for live visualization");
            Serial.println("#");
            Serial.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw");
        }
        return;
    }

    // Update filter in flight mode with 9-DoF if magnetometer available
    if (hasMag) {
        mahony.update(accel, gyro, mag, DT);
    } else {
        mahony.update(accel, gyro, DT);
    }

    // Get orientation
    Quaternion q = mahony.getQuaternion();

    // Convert to Euler angles
    // Roll (X-axis rotation)
    double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                       1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (fabs(sinp) >= 1.0)
        pitch = copysign(M_PI / 2.0, sinp); // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (Z-axis rotation)
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    // Convert to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // Output CSV format
    double time = (currentTime - startTime) / 1000.0;  // Convert to seconds

    Serial.print(time, 3);
    Serial.print(",");
    Serial.print(accel.x(), 3);
    Serial.print(",");
    Serial.print(accel.y(), 3);
    Serial.print(",");
    Serial.print(accel.z(), 3);
    Serial.print(",");
    Serial.print(gyro.x(), 3);
    Serial.print(",");
    Serial.print(gyro.y(), 3);
    Serial.print(",");
    Serial.print(gyro.z(), 3);
    Serial.print(",");
    Serial.print(mag.x(), 3);
    Serial.print(",");
    Serial.print(mag.y(), 3);
    Serial.print(",");
    Serial.print(mag.z(), 3);
    Serial.print(",");
    Serial.print(q.w(), 4);
    Serial.print(",");
    Serial.print(q.x(), 4);
    Serial.print(",");
    Serial.print(q.y(), 4);
    Serial.print(",");
    Serial.print(q.z(), 4);
    Serial.print(",");
    Serial.print(roll, 3);
    Serial.print(",");
    Serial.print(pitch, 3);
    Serial.print(",");
    Serial.println(yaw, 3);
}
