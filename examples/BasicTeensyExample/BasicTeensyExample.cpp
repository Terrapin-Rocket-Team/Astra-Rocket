/**
 * Basic Teensy 4.1 Sensor Example using AstraRocket Library
 *
 * This example demonstrates the simplest possible use of AstraRocket:
 * - Auto-detects available sensors (Barometer, GPS, IMU, high-G accelerometer)
 * - Automatically initializes all hardware
 * - Sets up logging to both Serial and SD card
 * - Tracks flight stages (pad idle, boost, coast, apogee, descent, landing)
 * - Adjusts logging rates based on flight phase
 * - Uses LED status indicators for sensor and GPS status
 *
 * AstraRocket handles all the complexity of:
 * - Sensor detection and initialization
 * - State estimation and filtering
 * - Flight stage detection
 * - Logging management
 * - Status indicator management
 *
 * LED Status Indicators:
 * - Pin 25 (Sensor Status): Solid ON = all sensors good, 2 blinks = sensor failure
 * - Pin 26 (GPS Status): Solid ON = GPS fix, 1 blink = GPS init but no fix, OFF = no GPS
 *
 * For more advanced usage with custom configuration, see the ConfigurableExample.
 */

#include <Arduino.h>
#include <AstraRocket.h>
#include <Sensors/HW/GPS/MAX_M10S.h>

using namespace astra_rocket;

// Create a custom configuration with LED status pins
AstraRocketConfig config = AstraRocketConfig()
                               .withGPS(new MAX_M10S)
                               .withSensorStatusLEDPin(32)
                               .withGPSStatusLEDPin(31);

// Create AstraRocket instance with custom configuration
// Note: For Teensy 4.1, the built-in SD card will be auto-configured
AstraRocket rocket(config);

void setup()
{
    // Initialize Serial for debug output
    Serial.begin(115200);
    delay(2000); // Wait for serial connection

    Serial.println("========================================");
    Serial.println("  Basic Teensy 4.1 Sensor Example");
    Serial.println("  Using AstraRocket Flight Computer");
    Serial.println("========================================");
    Serial.println();

    // Initialize AstraRocket
    // This will:
    // - Auto-detect and initialize all sensors
    // - Set up SD card logging
    // - Configure state estimation
    // - Establish ground level reference
    Serial.println("Initializing AstraRocket...");
    if (!rocket.init())
    {
        Serial.println("ERROR: AstraRocket initialization failed!");
        while (1)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("Setup complete!");
    Serial.println("Flight computer ready. Waiting for launch...");
    Serial.println("========================================");
    Serial.println();
}

void loop()
{
    // Update the rocket system
    // This will:
    // - Read all sensors
    // - Update state estimation
    // - Detect flight stage transitions
    // - Log data to SD card and Serial
    // - Update status indicators
    // - Handle timing automatically
    rocket.update();
}
