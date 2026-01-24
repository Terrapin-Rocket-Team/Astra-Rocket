#include "RocketState.h"
#include <RecordData/Logging/EventLogger.h>
#include <Math/Vector.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/SensorManager/SensorManager.h>

using namespace astra;

namespace astra_rocket {

RocketState::RocketState(Filter *filter, MahonyAHRS *orientationFilter)
    : State(filter, orientationFilter),
      currentStage(PAD_IDLE),
      previousStage(PAD_IDLE),
      timeInCurrentStage(0),
      stageStartTime(0),
      offVerticalAngle(0),
      highAccelStartTime(0),
      lowAccelStartTime(0),
      lowVelocityStartTime(0),
      highAccelDetected(false),
      lowAccelDetected(false),
      lowVelocityDetected(false)
{
    // Add rocket-specific columns to DataReporter
    insertColumn(0, "%d", &currentStage, "Flight Stage");
    addColumn("%0.3f", &offVerticalAngle, "Off-Vert Angle (deg)");
    addColumn("%0.3f", &timeInCurrentStage, "Time in Stage (s)");
}

void RocketState::setGroundLevel(double altitudeMSL) {
    origin.z() = altitudeMSL;
    LOGI("Ground level set to %0.2f m MSL (baro offset applied)", origin.z());
}

void RocketState::setFlightStage(FlightStage stage) {
    if (stage != currentStage) {
        previousStage = currentStage;
        currentStage = stage;
        stageStartTime = (unsigned long)(currentTime * 1000.0);
        timeInCurrentStage = 0;
        LOGI("Flight stage transition: %s -> %s",
             flightStageToString(previousStage),
             flightStageToString(currentStage));
    }
}

void RocketState::update(double ms) {
    // Call parent implementation first - handles orientation filter updates
    State::update(ms);

    // Update time in current stage
    unsigned long currentMillis = (unsigned long)(currentTime * 1000.0);
    timeInCurrentStage = (currentMillis - stageStartTime) / 1000.0;

    // Calculate rocket-specific derived values that depend on orientation
    calculateTilt();
    detectFlightStage();
}

void RocketState::calculateTilt() {
    // Calculate off-vertical angle using orientation quaternion
    // This is more accurate than using acceleration alone
    MahonyAHRS *ahrs = getOrientationFilter();
    if (ahrs && ahrs->isInitialized()) {
        // Get the current orientation quaternion
        Quaternion q = orientation;

        // Rocket body Z-axis in body frame (pointing along rocket)
        Vector<3> bodyZ(0, 0, 1);

        // Rotate body Z into earth frame to see which way rocket is pointing
        Vector<3> rocketDir = q.rotateVector(bodyZ);

        // Earth vertical reference (up in NED is negative Z)
        Vector<3> verticalRef(0, 0, -1);

        // Calculate angle between rocket direction and vertical
        double cosAngle = rocketDir.dot(verticalRef);
        offVerticalAngle = acos(fmax(-1.0, fmin(1.0, cosAngle))) * 180.0 / M_PI;
    } 
}

void RocketState::detectFlightStage() {
    unsigned long now = (unsigned long)(currentTime * 1000.0);
    FlightStage newStage = currentStage;

    switch (currentStage) {
        case PAD_IDLE:
            // Detect liftoff: sustained high vertical acceleration
            if (sensorManager->getAccel().magnitude() > LIFTOFF_ACCEL_THRESHOLD) {
                if (!highAccelDetected) {
                    highAccelStartTime = now;
                    highAccelDetected = true;
                } else if (now - highAccelStartTime > LIFTOFF_DURATION) {
                    newStage = BOOST;
                    LOGI("LIFTOFF DETECTED! Vertical accel: %0.2f G", acceleration.z());

                    // Initialize orientation filter at launch - locks in launch orientation
                    MahonyAHRS *ahrs = getOrientationFilter();
                    if (ahrs && !ahrs->isInitialized()) {
                        ahrs->initialize();
                        LOGI("Orientation filter initialized at launch");
                    }
                }
            } else {
                highAccelDetected = false;
            }
            break;

        case BOOST:
            // Detect motor burnout: acceleration drops to or below threshold
            if (sensorManager->getAccel().magnitude() <= BURNOUT_ACCEL_THRESHOLD)
            {
                if (!lowAccelDetected) {
                    lowAccelStartTime = now;
                    lowAccelDetected = true;
                } else if (now - lowAccelStartTime > BURNOUT_DURATION) {
                    newStage = COAST;
                    LOGI("MOTOR BURNOUT DETECTED at %0.2f m AGL", position.z());
                }
            }
            else
            {
                lowAccelDetected = false;
            }
            break;

        case COAST:
            // Detect apogee: vertical velocity approaches zero
            if (fabs(velocity.z()) < APOGEE_VELOCITY_THRESHOLD || timeInCurrentStage > 25) {
                newStage = APOGEE;
                LOGI("APOGEE DETECTED at %0.2f m AGL", position.z());
            }
            break;

        case APOGEE:
            // Transition to expecting drogue - starting descent
            if (velocity.z() < -2.0) {  // Descending at > 2 m/s
                newStage = EXPECTING_DROGUE;
                LOGI("EXPECTING DROGUE DEPLOYMENT at %0.2f m AGL, descent rate: %0.2f m/s",
                     position.z(), -velocity.z());
            }
            break;

        case EXPECTING_DROGUE:
            // Detect drogue deployment by change in descent rate
            // Under drogue, descent rate should be moderate (DROGUE_DESCENT_MIN to MAX)
            // Without drogue (ballistic), descent rate would be much higher
            if (velocity.z() < -DROGUE_DESCENT_MIN && velocity.z() > -DROGUE_DESCENT_MAX) {
                // Moderate descent rate suggests drogue is deployed
                newStage = UNDER_DROGUE;
                LOGI("UNDER DROGUE detected at %0.2f m AGL, descent rate: %0.2f m/s",
                     position.z(), -velocity.z());
            }
            break;

        case UNDER_DROGUE:
            // At typical main deployment altitude, expect main deployment
            // Check if we're below the main deployment altitude threshold
            if (position.z() < MAIN_DEPLOY_ALTITUDE) {
                newStage = EXPECTING_MAIN;
                LOGI("EXPECTING MAIN DEPLOYMENT at %0.2f m AGL", position.z());
            }
            break;

        case EXPECTING_MAIN:
            // Detect main deployment by significant reduction in descent rate
            // Under main, descent rate should be slow (MAIN_DESCENT_MIN to MAX)
            if (velocity.z() < -MAIN_DESCENT_MIN && velocity.z() > -MAIN_DESCENT_MAX) {
                newStage = UNDER_MAIN;
                LOGI("UNDER MAIN detected at %0.2f m AGL, descent rate: %0.2f m/s",
                     position.z(), -velocity.z());
            }
            break;

        case UNDER_MAIN:
            // Detect landing: low velocity sustained
            if (fabs(velocity.z()) < LANDING_VELOCITY_THRESHOLD) {
                if (!lowVelocityDetected) {
                    lowVelocityStartTime = now;
                    lowVelocityDetected = true;
                } else if (now - lowVelocityStartTime > LANDING_DURATION) {
                    newStage = LANDED;
                    LOGI("LANDING DETECTED at %0.2f m AGL", position.z());
                }
            } else {
                lowVelocityDetected = false;
            }
            break;

        case LANDED:
            // Terminal state
            break;
    }

    // Update stage if changed
    if (newStage != currentStage) {
        setFlightStage(newStage);
    }
}

} // namespace astra_rocket
