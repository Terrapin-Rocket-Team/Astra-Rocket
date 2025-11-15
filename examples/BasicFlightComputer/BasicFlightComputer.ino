/**
 * Basic Flight Computer Example
 *
 * Demonstrates minimal code required for a functional rocket flight computer
 * using AstraRocket wrapper.
 *
 * Features:
 * - Automatic sensor detection
 * - Flight stage detection
 * - Automatic drogue deployment at apogee
 * - Automatic main deployment at 300m AGL
 * - Flight data logging
 */

#include <AstraRocket.h>

using namespace astra_rocket;

AstraRocket rocket;

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection

    Serial.println("=== AstraRocket Basic Flight Computer ===");
    Serial.println("Initializing...");

    if (!rocket.init()) {
        Serial.println("FATAL: Initialization failed!");
        while(1) {
            delay(1000);
        }
    }

    Serial.println("Initialization complete!");
    Serial.println("Flight computer ready. Waiting for liftoff...");
}

void loop() {
    rocket.update();

    // Optional: Print flight status periodically
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        FlightStage stage = rocket.getFlightStage();
        RocketState *state = rocket.getRocketState();

        Serial.print("Stage: ");
        Serial.print(flightStageToString(stage));
        Serial.print(" | Alt AGL: ");
        Serial.print(state->getAltitudeAGL(), 1);
        Serial.print(" m | Vel: ");
        Serial.print(state->getVerticalVelocity(), 1);
        Serial.print(" m/s | Accel: ");
        Serial.print(state->getVerticalAcceleration(), 2);
        Serial.println(" G");

        lastPrint = millis();
    }
}
