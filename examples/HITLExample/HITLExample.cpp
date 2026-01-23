/**
 * HITL (Hardware-In-The-Loop) Example for Astra-Rocket
 *
 * This example demonstrates how to use HITL mode for desktop simulation testing.
 * Instead of reading from physical sensors, the flight computer receives simulated
 * sensor data over USB Serial from a desktop simulation program.
 *
 * Protocol:
 * - Desktop sends: HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
 * - Flight computer responds: TELEM/... (automatic via DataLogger)
 *
 * Hardware: Teensy 4.1 or STM32
 * USB Baud: 115200
 */

#include <Arduino.h>
#include <AstraRocket.h>

using namespace astra_rocket;

// Create AstraRocket instance
AstraRocket* rocket = nullptr;

void setup() {
    // Initialize Serial for USB communication
    Serial.begin(115200);
    if(Serial.connectSITL("localhost", 5555)){
        Serial.println("Connected to SITL");
    }
    else{
        Serial.println("Failed to connect to SITL");
    }

    // Wait for Serial connection (optional, comment out for flight)
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    Serial.println("===========================================");
    Serial.println("  Astra-Rocket HITL Mode");
    Serial.println("  Hardware-In-The-Loop Simulation");
    Serial.println("===========================================");
    Serial.println();

    // Configure rocket for HITL mode
    // NOTE: config must be static because AstraRocket stores a reference to it
    static AstraRocketConfig config;
    config.withHITL(true)                      // Enable HITL mode
          .withUpdateRate(50.0)                 // 50 Hz update rate
          .withPreflightLogRate(50.0)           // Log at full rate in HITL
          .withFlightLogRate(50.0)
          .withPostflightLogRate(50.0);
          //.withBuzzerFeedback(true);            // Keep LEDs/buzzer active (it's HW-in-the-loop!)

    // Create rocket with HITL configuration
    rocket = new AstraRocket(config);

    Serial.println("Initializing HITL mode...");

    if (!rocket->init()) {
        Serial.println("ERROR: Rocket initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("HITL mode initialized successfully!");
    Serial.println();
    Serial.println("Ready for simulation. Waiting for HITL packets...");
    Serial.println("Expected format:");
    Serial.println("  HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading");
    Serial.println();
}

void loop() {
    // Update rocket - handles HITL parsing automatically
    rocket->update();

    // That's it! The HITL loop is handled internally:
    // 1. Waits for HITL/ packet on Serial
    // 2. Parses sensor data and updates HITLSensorBuffer
    // 3. Updates state estimation with simulation time
    // 4. Outputs TELEM/ packet automatically via DataLogger
}

#ifdef NATIVE
#include <NativeTestHelper.h>
#endif