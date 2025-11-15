/**
 * Advanced Flight Computer Example
 *
 * Demonstrates custom configuration and sensor selection with AstraRocket.
 *
 * Features:
 * - Custom sensor selection
 * - Custom flight detection thresholds
 * - Custom deployment altitudes
 * - Custom logging rates
 * - Pyro arming with physical switch
 */

#include <AstraRocket.h>
#include <Sensors/Baro/DPS368.h>
#include <Sensors/GPS/MAX_M10S.h>
#include <Sensors/IMU/BMI088andLIS3MDL.h>
#include <Sensors/Accel/ADXL375.h>

using namespace astra;
using namespace astra_rocket;

// Instantiate specific sensors
DPS368 barometer;
MAX_M10S gps;
BMI088andLIS3MDL imu;
ADXL375 highGAccel;

// Configure flight computer
AstraRocketConfig config = AstraRocketConfig()
    // Use specific sensors
    .withBarometer(&barometer)
    .withGPS(&gps)
    .withIMU(&imu)
    .withHighGAccel(&highGAccel)

    // Custom flight detection thresholds
    .withLiftoffAccelThreshold(4.0)         // 4G threshold for high-thrust motor
    .withLiftoffDetectDuration(150)         // 150ms sustained acceleration
    .withApogeeVelocityThreshold(1.5)       // 1.5 m/s for apogee detection

    // Custom deployment configuration
    .withDrogueDeploymentAltitude(-1)       // Deploy at apogee
    .withMainDeploymentAltitude(500.0)      // Deploy main at 500m AGL
    .withDrogueBackupTime(20.0)             // Backup timer at 20s
    .withMainBackupTime(60.0)               // Backup timer at 60s

    // Pyro channel pins (Teensy 4.1)
    .withDroguePyroPin(23)
    .withMainPyroPin(24)
    .withPyroFireDuration(500)              // 500ms pyro pulse
    .withContinuityCheckPins(25, 26)        // Continuity check pins

    // Logging configuration
    .withSDCardCS(BUILTIN_SDCARD)
    .withPreflightLogRate(1.0)              // 1 Hz preflight
    .withFlightLogRate(100.0)               // 100 Hz during flight
    .withPostflightLogRate(1.0)             // 1 Hz postflight

    // Safety configuration
    .withArmingPin(27)                      // Physical arming switch
    .withBuzzerFeedback(true)
    .withLEDStatusPin(LED_BUILTIN)

    // Update rate
    .withUpdateRate(100.0);                 // 100 Hz sensor updates

AstraRocket rocket(config);

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("=== AstraRocket Advanced Flight Computer ===");
    Serial.println("Configuration:");
    Serial.println("  Liftoff threshold: 4.0 G");
    Serial.println("  Main deploy: 500m AGL");
    Serial.println("  Flight log rate: 100 Hz");
    Serial.println("  Arming: Physical switch on pin 27");
    Serial.println();

    Serial.println("Initializing...");

    if (!rocket.init()) {
        Serial.println("FATAL: Initialization failed!");
        while(1) {
            delay(1000);
        }
    }

    Serial.println("Initialization complete!");
    Serial.println();

    // Check sensor status
    Serial.println("Sensor Status:");
    Serial.print("  Barometer: ");
    Serial.println(rocket.getBarometer() ? "OK" : "MISSING");
    Serial.print("  GPS: ");
    Serial.println(rocket.getGPS() ? "OK" : "MISSING");
    Serial.print("  IMU: ");
    Serial.println(rocket.getIMU() ? "OK" : "MISSING");
    Serial.print("  High-G Accel: ");
    Serial.println(rocket.getHighGAccel() ? "OK" : "MISSING");
    Serial.println();

    Serial.print("Pyro armed: ");
    Serial.println(rocket.isPyroArmed() ? "YES" : "NO");
    Serial.println();

    Serial.println("Flight computer ready!");
}

void loop() {
    rocket.update();

    // Print detailed flight status
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {  // Update every 500ms
        RocketState *state = rocket.getRocketState();
        FlightStage stage = rocket.getFlightStage();

        Serial.println("========================================");
        Serial.print("Stage: ");
        Serial.print(flightStageToString(stage));
        Serial.print(" (");
        Serial.print(state->getTimeInStage(), 1);
        Serial.println(" s)");

        Serial.print("Altitude AGL: ");
        Serial.print(state->getAltitudeAGL(), 2);
        Serial.println(" m");

        Serial.print("Velocity (vert): ");
        Serial.print(state->getVerticalVelocity(), 2);
        Serial.println(" m/s");

        Serial.print("Acceleration (vert): ");
        Serial.print(state->getVerticalAcceleration(), 3);
        Serial.println(" G");

        if (stage == BOOST || stage == COAST) {
            Serial.print("Apogee estimate: ");
            Serial.print(state->getApogeeEstimate(), 1);
            Serial.print(" m MSL (");
            Serial.print(state->getTimeToApogee(), 1);
            Serial.println(" s)");
        }

        Serial.print("Max accel: ");
        Serial.print(state->getMaxAcceleration(), 2);
        Serial.println(" G");

        Serial.print("Max velocity: ");
        Serial.print(state->getMaxVelocity(), 1);
        Serial.println(" m/s");

        Serial.print("Off-vertical: ");
        Serial.print(state->getOffVerticalAngle(), 1);
        Serial.println(" deg");

        Serial.print("Pyro armed: ");
        Serial.println(rocket.isPyroArmed() ? "YES" : "NO");

        Serial.println();

        lastPrint = millis();
    }

    // Custom telemetry downlink during flight
    if (rocket.getFlightStage() >= BOOST && rocket.getFlightStage() <= MAIN_DESCENT) {
        // Send telemetry packet every 100ms
        static unsigned long lastTelemetry = 0;
        if (millis() - lastTelemetry > 100) {
            sendTelemetryPacket();
            lastTelemetry = millis();
        }
    }
}

void sendTelemetryPacket() {
    // Example: Simple packet format for telemetry downlink
    RocketState *state = rocket.getRocketState();

    // You would implement your radio protocol here
    // For demonstration, just print to serial
    Serial.print("TLM,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)rocket.getFlightStage());
    Serial.print(",");
    Serial.print(state->getAltitudeAGL(), 2);
    Serial.print(",");
    Serial.print(state->getVerticalVelocity(), 2);
    Serial.print(",");
    Serial.print(state->getVerticalAcceleration(), 3);
    Serial.println();
}
