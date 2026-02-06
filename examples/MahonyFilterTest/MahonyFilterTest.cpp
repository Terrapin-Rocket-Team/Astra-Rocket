/**
 * Mahony Filter Hardware Test Example
 *
 * This example demonstrates testing the Mahony AHRS filter with real IMU sensors.
 * It reads raw sensor data from an IMU and outputs orientation data over Serial
 * for live visualization with the Python script.
 *
 * Compatible with:
 * - Teensy 4.1 with BMI088 + LIS3MDL (9-DoF) - MAGNETOMETER REQUIRED
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
#include <Sensors/HW/IMU/BMI088.h>
#include <Filters/Mahony.h>
#include <Math/Vector.h>
#include "LIS3MDL_Wrapper.h"

using namespace astra;

// ============ HARDWARE CONFIGURATION ============
// BMI088 6-DoF IMU + LIS3MDL Magnetometer = 9-DoF (MAGNETOMETER REQUIRED)
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68
#define LIS3MDL_SA1_STATE LIS3MDL::sa1_auto  // Auto-detect address (0x1C or 0x1E)

// ============ FILTER CONFIGURATION ============
const double MAHONY_KP = 0.5;      // Proportional gain (higher = faster correction)
const double MAHONY_KI = 0.001;    // Integral gain (gyro bias correction)
const double UPDATE_RATE = 50.0;   // Hz (50 Hz = 20ms update period)
const double DT = 1.0 / UPDATE_RATE;

// ============ FLIGHT-LIKE BEHAVIOR ============
// In real flight, accelerometer measurements include thrust/drag, so using accel
// for attitude correction will "right" the filter toward the acceleration vector.
// Enable this to use gyro-only updates when accel magnitude deviates from 1 g.
const bool FLIGHT_LIKE_MODE = true;
const double ACCEL_TRUST_G = 9.81;     // m/s^2
const double ACCEL_TRUST_TOL = 1.5;    // m/s^2 tolerance around 1 g

// ============ CALIBRATION SETTINGS ============
const int CALIBRATION_SAMPLES = 100;  // Number of samples for initial calibration
const unsigned long CALIBRATION_TIME = 2000;  // Calibration time in milliseconds (stationary)
const unsigned long MAG_CALIBRATION_TIME = 10000;  // Magnetometer calibration time (moving)

// ============ GLOBAL OBJECTS ============
SensorManager sensorManager;
MahonyAHRS mahony(MAHONY_KP, MAHONY_KI);
BMI088 bmi088("BMI088", &Wire, BMI088_ACCEL_ADDR, BMI088_GYRO_ADDR);
LIS3MDL_Wrapper magnetometer("LIS3MDL", LIS3MDL_SA1_STATE);

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

    Serial.println("# Mahony Filter Hardware Test - 9-DoF");
    Serial.println("#");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    delay(100);

    // Initialize BMI088
    Serial.print("# Initializing BMI088... ");
    if (bmi088.init() != 0) {
        Serial.println("FAILED!");
        Serial.println("# ERROR: Could not initialize BMI088");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("OK");

    // Initialize magnetometer (REQUIRED)
    Serial.print("# Initializing LIS3MDL... ");
    if (magnetometer.init() != 0) {
        Serial.println("FAILED!");
        Serial.println("# ERROR: Magnetometer is REQUIRED for this test");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("OK");

    sensorManager.setAccelSource(bmi088.getAccelSensor());
    sensorManager.setGyroSource(bmi088.getGyroSensor());
    sensorManager.setMagSource(&magnetometer);
    sensorManager.addMiscSensor(&bmi088);
    sensorManager.begin();

    // Test sensor readings
    Serial.println("# Testing sensors...");
    sensorManager.update(0.0);
    Vector<3> testAccel = sensorManager.getAccelSource()->getAccel();
    Vector<3> testGyro = sensorManager.getGyroSource()->getAngVel();
    Vector<3> testMag = sensorManager.getMagSource()->getMag();

    Serial.print("# Accel: ");
    Serial.print(testAccel.x(), 2);
    Serial.print(", ");
    Serial.print(testAccel.y(), 2);
    Serial.print(", ");
    Serial.println(testAccel.z(), 2);

    Serial.print("# Gyro: ");
    Serial.print(testGyro.x(), 2);
    Serial.print(", ");
    Serial.print(testGyro.y(), 2);
    Serial.print(", ");
    Serial.println(testGyro.z(), 2);

    Serial.print("# Mag: ");
    Serial.print(testMag.x(), 2);
    Serial.print(", ");
    Serial.print(testMag.y(), 2);
    Serial.print(", ");
    Serial.println(testMag.z(), 2);

    // Check for NaN in initial readings
    if (isnan(testAccel.x()) || isnan(testAccel.y()) || isnan(testAccel.z()) ||
        isnan(testGyro.x()) || isnan(testGyro.y()) || isnan(testGyro.z()) ||
        isnan(testMag.x()) || isnan(testMag.y()) || isnan(testMag.z())) {
        Serial.println("# ERROR: NaN detected in sensor readings!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("#");
    Serial.println("# === CALIBRATION PHASE 1: STATIONARY ===");
    Serial.println("# Keep board STATIONARY for gyro bias calibration!");

    // Mahony filter no longer has modes; it is always ready
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
    sensorManager.update(currentTime / 1000.0);

    // Get sensor data
    Vector<3> accel = sensorManager.getAccelSource()->getAccel();
    Vector<3> gyro = sensorManager.getGyroSource()->getAngVel();
    Vector<3> mag = sensorManager.getMagSource()->getMag();

    // Check for NaN in sensor readings during operation
    if (isnan(accel.x()) || isnan(accel.y()) || isnan(accel.z()) ||
        isnan(gyro.x()) || isnan(gyro.y()) || isnan(gyro.z()) ||
        isnan(mag.x()) || isnan(mag.y()) || isnan(mag.z())) {
        Serial.println("# ERROR: NaN detected in sensor readings during operation!");
        Serial.print("# Accel: ");
        Serial.print(accel.x());
        Serial.print(", ");
        Serial.print(accel.y());
        Serial.print(", ");
        Serial.println(accel.z());
        Serial.print("# Gyro: ");
        Serial.print(gyro.x());
        Serial.print(", ");
        Serial.print(gyro.y());
        Serial.print(", ");
        Serial.println(gyro.z());
        Serial.print("# Mag: ");
        Serial.print(mag.x());
        Serial.print(", ");
        Serial.print(mag.y());
        Serial.print(", ");
        Serial.println(mag.z());
        return;
    }

    // Check if calibration is complete
    if (!calibrationComplete) {
        calibrationCount++;

        // Update filter in calibration mode (6-DoF for gyro bias)
        mahony.update(accel, gyro, DT);

        // Debug: Check quaternion during calibration
        if (calibrationCount % 50 == 0) {
            Quaternion q = mahony.getQuaternion();
            Serial.print("# Calib quat: ");
            Serial.print(q.w(), 4);
            Serial.print(", ");
            Serial.print(q.x(), 4);
            Serial.print(", ");
            Serial.print(q.y(), 4);
            Serial.print(", ");
            Serial.println(q.z(), 4);
        }

        // Check if stationary calibration time has elapsed
        if (currentTime - startTime >= CALIBRATION_TIME) {
            calibrationComplete = true;

            Serial.println("# Phase 1 complete!");
            Serial.println("#");
            Serial.println("# === CALIBRATION PHASE 2: MAGNETOMETER ===");
            Serial.println("# Rotate board in ALL directions!");
            magCalibStartTime = currentTime;
        }
        return;
    }

    // Magnetometer calibration phase
    if (!magCalibrationComplete) {
        // Collect magnetometer samples while user rotates board
        mahony.collectMagCalibrationSample(mag);

        // Check if mag calibration time has elapsed
        if (currentTime - magCalibStartTime >= MAG_CALIBRATION_TIME) {
            Quaternion q_before_finalize = mahony.getQuaternion();
            Serial.print("# Quat before finalize: ");
            Serial.print(q_before_finalize.w(), 4);
            Serial.print(", ");
            Serial.print(q_before_finalize.x(), 4);
            Serial.print(", ");
            Serial.print(q_before_finalize.y(), 4);
            Serial.print(", ");
            Serial.println(q_before_finalize.z(), 4);

            mahony.finalizeMagCalibration();  // This will compute mag calibration

            Quaternion q_after_finalize = mahony.getQuaternion();
            Serial.print("# Quat after finalize: ");
            Serial.print(q_after_finalize.w(), 4);
            Serial.print(", ");
            Serial.print(q_after_finalize.x(), 4);
            Serial.print(", ");
            Serial.print(q_after_finalize.y(), 4);
            Serial.print(", ");
            Serial.println(q_after_finalize.z(), 4);

            if (mahony.isMagCalibrated()) {
                Serial.println("# Phase 2 complete! Using 9-DoF mode");
            } else {
                Serial.println("# WARNING: Mag calibration failed!");
            }

            // Mahony filter runs in continuous correction mode

            Quaternion q_after_mode_change = mahony.getQuaternion();
            Serial.print("# Quat after mode change: ");
            Serial.print(q_after_mode_change.w(), 4);
            Serial.print(", ");
            Serial.print(q_after_mode_change.x(), 4);
            Serial.print(", ");
            Serial.print(q_after_mode_change.y(), 4);
            Serial.print(", ");
            Serial.println(q_after_mode_change.z(), 4);

            magCalibrationComplete = true;

            Serial.println("#");
            Serial.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw");
        }
        return;
    }

    // Update filter in flight mode
    Quaternion q_before_update = mahony.getQuaternion();

    double accelMag = accel.magnitude();
    bool accelTrusted = fabs(accelMag - ACCEL_TRUST_G) < ACCEL_TRUST_TOL;

    if (FLIGHT_LIKE_MODE && !accelTrusted) {
        // High dynamics (thrust/drag): trust gyro only
        mahony.update(gyro, DT);
    } else if (mahony.isMagCalibrated()) {
        // Normal operation: use 9-DoF if mag is calibrated
        mahony.update(accel, gyro, mag, DT);
    } else {
        // Fallback to 6-DoF if mag isn't ready
        mahony.update(accel, gyro, DT);
    }

    // Get orientation
    Quaternion q = mahony.getQuaternion();

    // Check for NaN in quaternion
    if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
        Serial.println("# ERROR: NaN detected in quaternion!");
        Serial.print("# Before update: ");
        Serial.print(q_before_update.w(), 4);
        Serial.print(", ");
        Serial.print(q_before_update.x(), 4);
        Serial.print(", ");
        Serial.print(q_before_update.y(), 4);
        Serial.print(", ");
        Serial.println(q_before_update.z(), 4);
        Serial.print("# After update: ");
        Serial.print(q.w());
        Serial.print(", ");
        Serial.print(q.x());
        Serial.print(", ");
        Serial.print(q.y());
        Serial.print(", ");
        Serial.println(q.z());
        Serial.print("# Inputs - Accel: ");
        Serial.print(accel.x(), 4);
        Serial.print(", ");
        Serial.print(accel.y(), 4);
        Serial.print(", ");
        Serial.println(accel.z(), 4);
        Serial.print("# Gyro: ");
        Serial.print(gyro.x(), 4);
        Serial.print(", ");
        Serial.print(gyro.y(), 4);
        Serial.print(", ");
        Serial.println(gyro.z(), 4);
        Serial.print("# Mag: ");
        Serial.print(mag.x(), 4);
        Serial.print(", ");
        Serial.print(mag.y(), 4);
        Serial.print(", ");
        Serial.println(mag.z(), 4);
        while (1) {
            delay(1000);
        }
    }

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
