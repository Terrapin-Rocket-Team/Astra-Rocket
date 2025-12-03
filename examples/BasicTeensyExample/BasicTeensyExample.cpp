/**
 * Basic Teensy 4.1 Sensor Example using Astra Library
 *
 * Reads data from multiple sensors at 10Hz and logs to both Serial and SD card:
 * - DPS368 (Barometric Pressure Sensor)
 * - LIS3MDL (3-axis Magnetometer)
 * - BMI088 (6-axis IMU - Accelerometer + Gyroscope)
 * - MAX-M10S (GPS module)
 *
 * This example demonstrates how to use the Astra library to:
 * - Initialize and configure sensors
 * - Set up data logging to SD card and Serial
 * - Run the system at a specified update rate
 */

#include <Arduino.h>
#include "Sensors/GPS/MAX_M10S.h"
#include "Sensors/IMU/BMI088andLIS3MDL.h"
#include "Sensors/Baro/DPS368.h"
#include "State/State.h"
#include "Utils/Astra.h"
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;

// Define buzzer pin for status indication
const int BUZZER_PIN = 33;
const int LED_PIN = LED_BUILTIN;

// Create sensor instances
MAX_M10S gps;
BMI088andLIS3MDL imu;
DPS368 baro;

// Create sensor array for the State
Sensor *sensors[3] = {&gps, &imu, &baro};

// Create State with sensors (no filter for this simple example)
State state(sensors, 3, nullptr);

// Configure Astra system
AstraConfig config = AstraConfig()
                         .withState(&state)
                         .withUpdateRate(10)      // 10Hz sensor updates
                         .withLoggingRate(10)     // 10Hz logging rate
                         .withBBPin(LED_PIN)
                         .withBuzzerPin(BUZZER_PIN);

// Create Astra system instance
Astra sys(&config);

// Set up logging sinks
#ifdef ENV_TEENSY
USBLog serialLog(Serial, 115200, true);  // Log to USB Serial with prefix
#else
UARTLog serialLog(Serial1, 115200, true);
#endif

ILogSink *logs[] = {&serialLog};

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection

    Serial.println("========================================");
    Serial.println("  Basic Teensy 4.1 Sensor Example");
    Serial.println("  Using Astra Flight Software Library");
    Serial.println("========================================");
    Serial.println();

    // Configure event logger
    EventLogger::configure(logs, 1);

    // Initialize Astra system
    // This will:
    // - Initialize all sensors
    // - Set up SD card logging
    // - Configure the state estimator
    Serial.println("Initializing Astra system...");
    sys.init();

    Serial.println();
    Serial.println("Setup complete! Running at 10Hz...");
    Serial.println("Data will be logged to SD card and Serial");
    Serial.println("========================================");
    Serial.println();
}

void loop() {
    // Update the Astra system
    // This will:
    // - Read all sensors
    // - Update state estimation
    // - Log data to SD card and configured sinks
    // - Handle timing automatically
    sys.update();
}
