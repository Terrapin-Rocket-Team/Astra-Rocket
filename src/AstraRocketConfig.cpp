#include "AstraRocketConfig.h"

namespace astra_rocket {

AstraRocketConfig::AstraRocketConfig()
    : barometer(nullptr),
      gps(nullptr),
      imu(nullptr),
      highGAccel(nullptr),
      liftoffAccelThreshold(3.0),
      liftoffDetectDuration(100),
      burnoutAccelThreshold(1.5),
      apogeeVelocityThreshold(2.0),
      landingVelocityThreshold(1.0),
      landingDetectDuration(3000),
      sdCardCS(-1),  // Will be set to BUILTIN_SDCARD or specific pin by user
      preflightLogRate(1.0),
      flightLogRate(50.0),
      postflightLogRate(1.0),
      flashBackup(false),
      buzzerFeedback(true),
      ledStatusPin(LED_BUILTIN)
{
    // Configure base AstraConfig with sensible defaults
    astraConfig.withUpdateRate(50.0);  // 50 Hz default update rate
}

// Sensor configuration
AstraRocketConfig& AstraRocketConfig::withBarometer(Barometer *baro) {
    barometer = baro;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withGPS(GPS *gpsPtr) {
    gps = gpsPtr;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withIMU(IMU *imuPtr) {
    imu = imuPtr;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withHighGAccel(Accel *accel) {
    highGAccel = accel;
    return *this;
}

// Flight detection thresholds
AstraRocketConfig& AstraRocketConfig::withLiftoffAccelThreshold(double accelG) {
    liftoffAccelThreshold = accelG;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLiftoffDetectDuration(unsigned long durationMs) {
    liftoffDetectDuration = durationMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withBurnoutAccelThreshold(double accelG) {
    burnoutAccelThreshold = accelG;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withApogeeVelocityThreshold(double velocityMs) {
    apogeeVelocityThreshold = velocityMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLandingVelocityThreshold(double velocityMs) {
    landingVelocityThreshold = velocityMs;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLandingDetectDuration(unsigned long durationMs) {
    landingDetectDuration = durationMs;
    return *this;
}

// Logging configuration
AstraRocketConfig& AstraRocketConfig::withSDCardCS(int csPin) {
    sdCardCS = csPin;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withPreflightLogRate(double rateHz) {
    preflightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withFlightLogRate(double rateHz) {
    flightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withPostflightLogRate(double rateHz) {
    postflightLogRate = rateHz;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withFlashBackup(bool enable) {
    flashBackup = enable;
    return *this;
}

// Status indicators configuration
AstraRocketConfig& AstraRocketConfig::withBuzzerFeedback(bool enable) {
    buzzerFeedback = enable;
    return *this;
}

AstraRocketConfig& AstraRocketConfig::withLEDStatusPin(int pin) {
    ledStatusPin = pin;
    return *this;
}

// Base Astra configuration pass-through
AstraRocketConfig& AstraRocketConfig::withUpdateRate(double rateHz) {
    astraConfig.withUpdateRate(rateHz);
    return *this;
}

} // namespace astra_rocket
