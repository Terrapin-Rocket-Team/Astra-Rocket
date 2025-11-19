/**
 * I2C Sensor Test for STM32H723VEH6
 *
 * Tests the following sensors over I2C:
 * - DPS368 (Barometric Pressure Sensor)
 * - BMI088 (6-axis IMU - Accelerometer + Gyroscope)
 * - SAM-M10Q (GPS module)
 *
 * This example verifies sensor connectivity and reads basic data
 * from each sensor to ensure they're functioning correctly.
 */

#include <Wire.h>
#include <Astra.h>

using namespace Astra;

// Sensor objects
Sensors::DPS368Barometer *baro = nullptr;
Sensors::BMI088IMU *imu = nullptr;
Sensors::UBLOXGPS *gps = nullptr;

// I2C addresses
#define DPS368_I2C_ADDR 0x77  // Can also be 0x76
#define BMI088_ACCEL_ADDR 0x18  // Can also be 0x19
#define BMI088_GYRO_ADDR 0x68   // Can also be 0x69
#define SAM_M10Q_ADDR 0x42

// I2C pins for STM32H723VEH6 (adjust if needed)
#define I2C_SDA PB9
#define I2C_SCL PB8

void scanI2C();
void testDPS368();
void testBMI088();
void testSAMM10Q();

void setup() {
    Serial.begin(115200);
    delay(3000);  // Wait for serial connection

    Serial.println("========================================");
    Serial.println("  I2C Sensor Test - STM32H723VEH6");
    Serial.println("========================================");
    Serial.println();

    // Initialize I2C
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C speed

    delay(100);

    Serial.println("I2C Bus Initialized");
    Serial.println("SDA: PB9, SCL: PB8");
    Serial.println("Clock: 400kHz");
    Serial.println();

    // Scan I2C bus
    Serial.println("--- I2C Bus Scan ---");
    scanI2C();
    Serial.println();

    // Initialize sensors
    Serial.println("--- Initializing Sensors ---");

    // DPS368 Barometer
    Serial.print("DPS368 Barometer... ");
    baro = new Sensors::DPS368Barometer();
    if (baro->init()) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    // BMI088 IMU
    Serial.print("BMI088 IMU... ");
    imu = new Sensors::BMI088IMU();
    if (imu->init()) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    // SAM-M10Q GPS
    Serial.print("SAM-M10Q GPS... ");
    gps = new Sensors::UBLOXGPS();
    if (gps->init()) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    Serial.println();
    Serial.println("Setup complete!");
    Serial.println("========================================");
    Serial.println();
    delay(1000);
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    // Update sensors every 500ms
    if (now - lastUpdate >= 500) {
        lastUpdate = now;

        Serial.println("--- Sensor Readings ---");
        Serial.print("Time: ");
        Serial.print(now / 1000.0, 3);
        Serial.println(" s");
        Serial.println();

        // Test DPS368
        if (baro) {
            testDPS368();
        }
        Serial.println();

        // Test BMI088
        if (imu) {
            testBMI088();
        }
        Serial.println();

        // Test SAM-M10Q
        if (gps) {
            testSAMM10Q();
        }

        Serial.println("========================================");
        Serial.println();
    }
}

void scanI2C() {
    byte error, address;
    int deviceCount = 0;

    Serial.println("Scanning I2C bus (0x01 - 0x7F)...");

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  Device found at 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);

            // Identify known devices
            if (address == DPS368_I2C_ADDR || address == 0x76) {
                Serial.print(" (DPS368)");
            } else if (address == BMI088_ACCEL_ADDR || address == 0x19) {
                Serial.print(" (BMI088 Accelerometer)");
            } else if (address == BMI088_GYRO_ADDR || address == 0x69) {
                Serial.print(" (BMI088 Gyroscope)");
            } else if (address == SAM_M10Q_ADDR) {
                Serial.print(" (SAM-M10Q GPS)");
            }

            Serial.println();
            deviceCount++;
        } else if (error == 4) {
            Serial.print("  Unknown error at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    if (deviceCount == 0) {
        Serial.println("  No I2C devices found!");
    } else {
        Serial.print("  Total devices found: ");
        Serial.println(deviceCount);
    }
}

void testDPS368() {
    Serial.println("DPS368 Barometer:");

    if (!baro->update()) {
        Serial.println("  ERROR: Failed to read sensor");
        return;
    }

    float pressure = baro->getPressure();
    float temperature = baro->getTemperature();
    float altitude = baro->getAltitude();

    Serial.print("  Pressure:    ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    Serial.print("  Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("  Altitude:    ");
    Serial.print(altitude, 2);
    Serial.println(" m");
}

void testBMI088() {
    Serial.println("BMI088 IMU:");

    if (!imu->update()) {
        Serial.println("  ERROR: Failed to read sensor");
        return;
    }

    // Accelerometer data
    float accelX = imu->getAccelX();
    float accelY = imu->getAccelY();
    float accelZ = imu->getAccelZ();

    Serial.println("  Accelerometer (m/s²):");
    Serial.print("    X: ");
    Serial.print(accelX, 3);
    Serial.print("  Y: ");
    Serial.print(accelY, 3);
    Serial.print("  Z: ");
    Serial.println(accelZ, 3);

    // Gyroscope data
    float gyroX = imu->getGyroX();
    float gyroY = imu->getGyroY();
    float gyroZ = imu->getGyroZ();

    Serial.println("  Gyroscope (rad/s):");
    Serial.print("    X: ");
    Serial.print(gyroX, 3);
    Serial.print("  Y: ");
    Serial.print(gyroY, 3);
    Serial.print("  Z: ");
    Serial.println(gyroZ, 3);
}

void testSAMM10Q() {
    Serial.println("SAM-M10Q GPS:");

    if (!gps->update()) {
        Serial.println("  Waiting for GPS data...");
        return;
    }

    if (!gps->hasFix()) {
        Serial.println("  No GPS fix yet");
        Serial.print("  Satellites: ");
        Serial.println(gps->getSatellites());
        return;
    }

    Serial.print("  Fix Type:    ");
    Serial.println(gps->getFixType());

    Serial.print("  Satellites:  ");
    Serial.println(gps->getSatellites());

    Serial.print("  Latitude:    ");
    Serial.print(gps->getLatitude(), 7);
    Serial.println(" °");

    Serial.print("  Longitude:   ");
    Serial.print(gps->getLongitude(), 7);
    Serial.println(" °");

    Serial.print("  Altitude:    ");
    Serial.print(gps->getAltitude(), 2);
    Serial.println(" m");

    Serial.print("  Speed:       ");
    Serial.print(gps->getSpeed(), 2);
    Serial.println(" m/s");

    Serial.print("  HDOP:        ");
    Serial.println(gps->getHDOP(), 2);
}
