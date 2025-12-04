#ifndef ASTRA_ROCKET_H
#define ASTRA_ROCKET_H

#include <Utils/Astra.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/EventLogger.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <Sensors/IMU/IMU.h>

#include "AstraRocketConfig.h"
#include "RocketState.h"
#include "FlightStage.h"

using namespace astra;

namespace astra_rocket {

/**
 * AstraRocket: High-level rocketry flight computer wrapper
 *
 * Provides a simple interface for rocket flight computers built on TRT-Astra.
 * Features:
 * - Automatic sensor detection and initialization
 * - Flight-aware logging with automatic file management
 * - Flight stage detection and tracking
 * - Recovery event detection (based on descent rate changes)
 *
 * Basic usage:
 *   AstraRocket rocket;
 *   rocket.init();
 *   while(1) rocket.update();
 */
class AstraRocket {
public:
    /**
     * Default constructor
     * Uses auto-detected sensors and default configuration
     */
    AstraRocket();

    /**
     * Constructor with custom configuration
     * @param config AstraRocketConfig object with custom settings
     */
    AstraRocket(AstraRocketConfig &config);

    /**
     * Destructor
     */
    ~AstraRocket();

    /**
     * Initialize the flight computer
     * - Auto-detects sensors (if not specified in config)
     * - Initializes all sensors
     * - Sets up logging
     * - Performs pre-flight checks
     *
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /**
     * Update the flight computer state
     * Should be called in main loop()
     * - Updates all sensors
     * - Updates state estimation
     * - Handles flight stage transitions
     * - Manages logging
     */
    void update();

    /**
     * Get the current flight stage
     */
    FlightStage getFlightStage() const;

    /**
     * Get the rocket state object
     */
    RocketState* getRocketState() { return rocketState; }

    /**
     * Get the underlying Astra system object
     */
    Astra* getAstraSystem() { return astraSys; }

    /**
     * Get barometer sensor
     */
    Barometer* getBarometer() const { return barometer; }

    /**
     * Get GPS sensor
     */
    GPS* getGPS() const { return gps; }

    /**
     * Get IMU sensor
     */
    IMU* getIMU() const { return imu; }

    /**
     * Get high-G accelerometer
     */
    Accel* getHighGAccel() const { return highGAccel; }

private:
    // Configuration
    AstraRocketConfig config;

    // Core Astra system
    Astra *astraSys;
    RocketState *rocketState;

    // Sensors
    Barometer *barometer;
    GPS *gps;
    IMU *imu;  // Keep for backwards compatibility, but prefer individual sensors
    Accel *accel;
    Gyro *gyro;
    Mag *mag;
    Accel *highGAccel;
    Sensor **sensorArray;
    int numSensors;

    // Logging
    ILogSink **dataSinks;
    ILogSink **eventSinks;
    int numDataSinks;
    int numEventSinks;

    // Flight tracking
    unsigned long liftoffTime;
    FlightStage previousStage;
    double groundLevelAltitude;

    // Helper methods
    bool autoDetectSensors();
    void setupLogging();
    void handleStageTransition(FlightStage newStage);
    void updateStatusIndicators();

    // Sensor auto-detection helpers
    Barometer* detectBarometer();
    GPS* detectGPS();
    IMU* detectIMU();
    Accel* detectAccel();
    Gyro* detectGyro();
    Mag* detectMag();
    Accel* detectHighGAccel();

    // Constants
    static constexpr int MAX_SENSORS = 10;
    static constexpr int MAX_LOG_SINKS = 5;
};

} // namespace astra_rocket

#endif // ASTRA_ROCKET_H
