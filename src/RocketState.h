#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <State/State.h>
#include <Filters/LinearKalmanFilter.h>
#include <Sensors/Sensor.h>
#include <Sensors/SensorManager/SensorManager.h>
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
    RocketState(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter);

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

    /**
     * Update orientation estimate from gyro and accel data
     * Allows RocketState to enforce gyro-only mode after liftoff
     */
    void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt) override;

    /**
     * Update orientation estimate from gyro, accel, and mag data (9-DOF)
     */
    void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, const Vector<3> &mag, double dt) override;

    int update(double currentTimeSec = -1) override;

    // Test/simulation helper: pull KF state into RocketState without full Astra loop
    void predictState(double currentTimeSec = -1);

    // Test helper: accepts a SensorManager to match existing tests
    RocketState &withSensorManager(SensorManager *sm);

    // Test helper: force frame lock without waiting for liftoff detection
    void lockFrameForTest();

private:
    enum UpAxis
    {
        POS_X,
        NEG_X,
        POS_Y,
        NEG_Y,
        POS_Z,
        NEG_Z
    };

    // Flight stage tracking
    FlightStage currentStage;
    FlightStage previousStage;
    double timeInCurrentStage;
    unsigned long stageStartTime;

    // Orientation tracking
    double offVerticalAngle;     // Angle from vertical axis
    Quaternion mountQuat_rb;     // Rocket->Body mapping
    UpAxis currentUpAxis;
    UpAxis pendingUpAxis;
    int axisStableCount;
    bool frameLocked;
    bool forceGyroOnly;
    SensorManager *sensorManager;

    // Helper methods
    void detectFlightStage();
    void calculateTilt();
    void updateMountingAlignment();
    void computeMountingQuaternion(UpAxis axis);
    UpAxis chooseUpAxis(double &bestDot) const;
    Quaternion getRocketOrientation() const;

    // Alignment tuning
    static constexpr double AXIS_SWITCH_DOT_THRESHOLD = 0.90;
    static constexpr int AXIS_SWITCH_STABLE_COUNT = 10;

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
    static constexpr unsigned long DROGUE_DETECT_DURATION = 2000; // ms to confirm drogue
    static constexpr unsigned long MAIN_DETECT_DURATION = 2000;   // ms to confirm main

    // Tracking for sustained condition detection
    unsigned long highAccelStartTime;
    unsigned long lowAccelStartTime;
    unsigned long lowVelocityStartTime;
    unsigned long drogueDetectStartTime;
    unsigned long mainDetectStartTime;
    bool highAccelDetected;
    bool lowAccelDetected;
    bool lowVelocityDetected;
    bool drogueRateDetected;
    bool mainRateDetected;
};

} // namespace astra_rocket

#endif // ROCKET_STATE_H
