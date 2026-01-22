#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <State/State.h>
#include <Filters/Filter.h>
#include <Sensors/Sensor.h>
#include "FlightStage.h"

using namespace astra;

namespace astra_rocket {

/**
 * RocketState: Rocket-specific state estimator
 *
 * Extends TRT-Astra's State class with rocket flight-specific variables:
 * - Flight stage detection and tracking
 * - Apogee estimation and prediction
 * - Vertical acceleration and velocity tracking
 * - Off-vertical angle for stability analysis
 */
class RocketState : public State {
public:
    /**
     * Constructor
     * @param filter Optional Kalman filter for state estimation
     * @param orientationFilter Optional Mahony AHRS filter for orientation estimation
     */
    RocketState(Filter *filter = nullptr, MahonyAHRS *orientationFilter = nullptr);

    /**
     * Initialize state estimation
     * @return true if initialization successful
     */
    bool begin() override;

    /**
     * Get current flight stage
     */
    FlightStage getFlightStage() const { return currentStage; }

    /**
     * Get estimated apogee altitude (meters ASL)
     */
    double getApogeeEstimate() const { return apogeeEstimate; }

    /**
     * Get time to apogee (seconds, negative if past apogee)
     */
    double getTimeToApogee() const { return timeToApogee; }

    /**
     * Get maximum acceleration experienced (G's)
     */
    double getMaxAcceleration() const { return maxAcceleration; }

    /**
     * Get maximum velocity achieved (m/s)
     */
    double getMaxVelocity() const { return maxVelocity; }

    /**
     * Get vertical acceleration (G's, positive = up)
     */
    double getVerticalAcceleration() const { return verticalAccel; }

    /**
     * Get off-vertical angle (degrees from vertical)
     */
    double getOffVerticalAngle() const { return offVerticalAngle; }

    /**
     * Get altitude above ground level (meters)
     */
    double getAltitudeAGL() const { return altitudeAGL; }

    /**
     * Get vertical velocity (m/s, positive = up)
     */
    double getVerticalVelocity() const { return verticalVelocity; }

    /**
     * Set ground level altitude (for AGL calculations)
     * Typically called during pad idle to establish reference
     */
    void setGroundLevel(double altitudeMSL);

    /**
     * Manually set flight stage (for testing or overrides)
     */
    void setFlightStage(FlightStage stage);

    /**
     * Get time in current stage (seconds)
     */
    double getTimeInStage() const { return timeInCurrentStage; }

    // Override split update methods to add rocket-specific logic
    void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt) override;
    void updateMeasurements(const Vector<3> &gpsPos, double baroAlt, bool hasGPS, bool hasBaro, double currentTime = -1) override;

private:
    // Flight stage tracking
    FlightStage currentStage;
    FlightStage previousStage;
    double timeInCurrentStage;
    unsigned long stageStartTime;

    // Altitude tracking
    double groundLevelAltitude;  // MSL altitude of launch pad
    double baroOffset;           // Offset to subtract from baro before feeding to KF (for zeroing)
    double altitudeAGL;          // Altitude above ground level
    double maxAltitudeAGL;       // Maximum altitude achieved
    double previousAltitudeAGL;  // Previous altitude for velocity calculation

    // Velocity tracking
    double verticalVelocity;     // Vertical component of velocity
    double maxVelocity;          // Maximum total velocity

    // Acceleration tracking
    double verticalAccel;        // Vertical acceleration in G's
    double maxAcceleration;      // Maximum acceleration in G's

    // Apogee estimation
    double apogeeEstimate;       // Estimated apogee altitude MSL
    double timeToApogee;         // Estimated time to apogee

    // Orientation tracking
    double offVerticalAngle;     // Angle from vertical axis

    // Helper methods
    void detectFlightStage();
    void updateApogeeEstimate();
    void updateMaxValues();
    void calculateVerticalComponents();

    // Stage detection thresholds (can be made configurable)
    static constexpr double LIFTOFF_ACCEL_THRESHOLD = 3.0 * 9.81;      // m/s's
    static constexpr unsigned long LIFTOFF_DURATION = 100;      // ms
    static constexpr double BURNOUT_ACCEL_THRESHOLD = 2.0 * 9.81;      // m/s's
    static constexpr unsigned long BURNOUT_DURATION = 200;      // ms
    static constexpr double APOGEE_VELOCITY_THRESHOLD = 2.0;    // m/s
    static constexpr double LANDING_VELOCITY_THRESHOLD = 1.0;   // m/s
    static constexpr unsigned long LANDING_DURATION = 3000;     // ms

    // Descent rate detection thresholds
    static constexpr double DROGUE_DESCENT_MIN = 5.0;           // m/s minimum under drogue
    static constexpr double DROGUE_DESCENT_MAX = 40.0;          // m/s maximum under drogue
    static constexpr double MAIN_DESCENT_MIN = 2.0;             // m/s minimum under main
    static constexpr double MAIN_DESCENT_MAX = 10.0;            // m/s maximum under main
    static constexpr double MAIN_DEPLOY_ALTITUDE = 400.0;       // m AGL to start expecting main

    // Tracking for sustained condition detection
    unsigned long highAccelStartTime;
    unsigned long lowAccelStartTime;
    unsigned long lowVelocityStartTime;
    bool highAccelDetected;
    bool lowAccelDetected;
    bool lowVelocityDetected;
};

} // namespace astra_rocket

#endif // ROCKET_STATE_H
