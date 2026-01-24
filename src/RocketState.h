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
    RocketState(Filter *filter, MahonyAHRS *orientationFilter);

    /**
     * Get current flight stage
     */
    FlightStage getFlightStage() const { return currentStage; }

    /**
     * Get off-vertical angle (degrees from vertical)
     */
    double getOffVerticalAngle() const { return offVerticalAngle; }

    /**
     * Get altitude above ground level (meters)
     */
    double getAltitudeAGL() const { return position.z(); }

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

    void update(double currentTimeSec = -1) override;

private:
    // Flight stage tracking
    FlightStage currentStage;
    FlightStage previousStage;
    double timeInCurrentStage;
    unsigned long stageStartTime;

    // Orientation tracking
    double offVerticalAngle;     // Angle from vertical axis

    // Helper methods
    void detectFlightStage();
    void calculateTilt();

    // Stage detection thresholds (can be made configurable)
    static constexpr double LIFTOFF_ACCEL_THRESHOLD = 3.0 * 9.81;      // m/s² (3.0G)
    static constexpr unsigned long LIFTOFF_DURATION = 100;      // ms
    static constexpr double BURNOUT_ACCEL_THRESHOLD = 1.5 * 9.81;      // m/s² (1.5G)
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
