/**
 * Basic STM32H723 Sensor Example using AstraRocket Library
 *
 * This example demonstrates the simplest possible use of AstraRocket:
 * - Initializes sensors supplied through AstraRocketConfig
 * - Sets up logging to both Serial and SD card
 * - Tracks flight stages (pad idle, boost, coast, apogee, descent, landing)
 * - Exposes configurable logging rates for mission code
 * - Uses LED status indicators for sensor and GPS status
 *
 * AstraRocket handles all the complexity of:
 * - Sensor initialization and update orchestration
 * - State estimation and filtering
 * - Flight stage detection
 * - Logging management
 * - Status indicator management
 *
 * LED Status Indicators:
 * - Sensor Status LED: Solid ON = all sensors good, 2 blinks = sensor failure
 * - GPS Status LED: Solid ON = GPS fix, 1 blink = GPS init but no fix, OFF = no GPS
 *
 * For more advanced usage with custom configuration, see the ConfigurableExample.
 */

#include <Arduino.h>
#include <Wire.h>
#include <AstraRocket.h>

using namespace astra_rocket;
// Create a custom configuration with LED status pins
// Adjust the pin numbers based on your STM32H723 hardware setup
AstraRocketConfig config = AstraRocketConfig();
//    .withStatusLED(PC0)  // Adjust to your LED pin
//    .withGPSFixLED(PC1);     // Adjust to your LED pin

// Create AstraRocket instance with custom configuration
// Note: SD card configuration may need to be explicitly set for STM32
AstraRocket rocket(config);

void setup()
{
    // Configure I2C pins for STM32 (PB8 = SCL, PB9 = SDA)
    Wire.setSCL(PB8);
    Wire.setSDA(PB9);
    Wire.begin();

    delay(2000);
    rocket.init();
}

void loop()
{
    rocket.update();
}
