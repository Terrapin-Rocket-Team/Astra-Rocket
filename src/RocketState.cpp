#include "RocketState.h"
#include <RecordData/Logging/EventLogger.h>
#include <Math/Vector.h>

using namespace astra;

namespace astra_rocket {

RocketState::RocketState(Sensor **sensors, int numSensors, Filter *filter)
    : State(sensors, numSensors, filter),
      currentStage(PAD_IDLE),
      previousStage(PAD_IDLE),
      timeInCurrentStage(0),
      stageStartTime(0),
      groundLevelAltitude(0),
      altitudeAGL(0),
      maxAltitudeAGL(0),
      previousAltitudeAGL(0),
      verticalVelocity(0),
      maxVelocity(0),
      verticalAccel(0),
      maxAcceleration(0),
      apogeeEstimate(0),
      timeToApogee(0),
      offVerticalAngle(0),
      highAccelStartTime(0),
      lowAccelStartTime(0),
      lowVelocityStartTime(0),
      highAccelDetected(false),
      lowAccelDetected(false),
      lowVelocityDetected(false)
{
    // Add rocket-specific columns to DataReporter
    addColumn("%d", &currentStage, "Flight Stage");
    addColumn("%0.2f", &altitudeAGL, "Alt AGL (m)");
    addColumn("%0.2f", &verticalVelocity, "Vert Vel (m/s)");
    addColumn("%0.3f", &verticalAccel, "Vert Accel (G)");
    addColumn("%0.2f", &apogeeEstimate, "Apogee Est (m)");
    addColumn("%0.2f", &timeToApogee, "Time to Apogee (s)");
    addColumn("%0.3f", &maxAcceleration, "Max Accel (G)");
    addColumn("%0.2f", &maxVelocity, "Max Vel (m/s)");
    addColumn("%0.2f", &offVerticalAngle, "Off-Vert Angle (deg)");
    addColumn("%0.2f", &timeInCurrentStage, "Time in Stage (s)");
}

void RocketState::setGroundLevel(double altitudeMSL) {
    groundLevelAltitude = altitudeMSL;
    LOGI("Ground level set to %0.2f m MSL", groundLevelAltitude);
}

void RocketState::setFlightStage(FlightStage stage) {
    if (stage != currentStage) {
        previousStage = currentStage;
        currentStage = stage;
        stageStartTime = millis();
        timeInCurrentStage = 0;
        LOGI("Flight stage transition: %s -> %s",
             flightStageToString(previousStage),
             flightStageToString(currentStage));
    }
}

void RocketState::updateVariables() {
    // Call parent class update (sensors, filter, base state variables)
    State::updateVariables();

    // Update time in current stage
    unsigned long currentMillis = millis();
    timeInCurrentStage = (currentMillis - stageStartTime) / 1000.0;

    // Calculate rocket-specific derived values
    calculateVerticalComponents();
    updateMaxValues();
    detectFlightStage();
    updateApogeeEstimate();
}

void RocketState::calculateVerticalComponents() {
    // Calculate altitude AGL
    Barometer *baro = static_cast<Barometer*>(getSensor("Barometer"_i));
    if (baro && sensorOK(baro)) {
        double altitudeMSL = baro->getASLAltM();
        altitudeAGL = altitudeMSL - groundLevelAltitude;
    }

    // Calculate vertical velocity
    // First check if State class is calculating velocity (from filter/GPS)
    double stateVertVel = -velocity.z();

    // If State velocity is essentially zero (not being calculated),
    // calculate from altitude changes
    if (fabs(stateVertVel) < 0.01 && currentTime > lastTime) {
        double dt = currentTime - lastTime;
        if (dt > 0.001) {  // Avoid division by very small numbers
            verticalVelocity = (altitudeAGL - previousAltitudeAGL) / dt;
        }
    } else {
        // Use State class velocity
        verticalVelocity = stateVertVel;
    }

    // Store current altitude for next iteration
    previousAltitudeAGL = altitudeAGL;

    // Get acceleration from IMU directly
    IMU *imu = static_cast<IMU*>(getSensor("IMU"_i));
    Vector<3> accelNED(0, 0, 0);
    if (imu && sensorOK(imu)) {
        accelNED = imu->getAcceleration();
    }

    // Calculate vertical acceleration
    // Total acceleration magnitude in G's
    double totalAccelMagnitude = accelNED.magnitude() / 9.81;

    // Vertical component (z-axis in NED, negate for up-positive)
    verticalAccel = -accelNED.z() / 9.81;

    // Calculate off-vertical angle
    // Angle between rocket's longitudinal axis and vertical
    // Using acceleration direction as a proxy for rocket orientation
    Vector<3> verticalRef(0, 0, -1);  // Up in NED frame (negative Z)

    // For now, use a simple approximation based on acceleration direction
    if (totalAccelMagnitude > 0.1) {
        Vector<3> accelDir = accelNED;
        accelDir.normalize();
        double cosAngle = accelDir.dot(verticalRef);
        offVerticalAngle = acos(fmax(-1.0, fmin(1.0, cosAngle))) * 180.0 / M_PI;
    } else {
        offVerticalAngle = 0;
    }
}

void RocketState::updateMaxValues() {
    // Update maximum acceleration - get from IMU directly
    IMU *imu = static_cast<IMU*>(getSensor("IMU"_i));
    if (imu && sensorOK(imu)) {
        Vector<3> accel = imu->getAcceleration();
        double currentAccelG = accel.magnitude() / 9.81;
        if (currentAccelG > maxAcceleration) {
            maxAcceleration = currentAccelG;
        }
    }

    // Update maximum velocity
    double currentVel = velocity.magnitude();
    if (currentVel > maxVelocity) {
        maxVelocity = currentVel;
    }

    // Update maximum altitude
    if (altitudeAGL > maxAltitudeAGL) {
        maxAltitudeAGL = altitudeAGL;
    }
}

void RocketState::detectFlightStage() {
    unsigned long now = millis();
    FlightStage newStage = currentStage;

    switch (currentStage) {
        case PAD_IDLE:
            // Detect liftoff: sustained high vertical acceleration
            if (verticalAccel > LIFTOFF_ACCEL_THRESHOLD) {
                if (!highAccelDetected) {
                    highAccelStartTime = now;
                    highAccelDetected = true;
                } else if (now - highAccelStartTime > LIFTOFF_DURATION) {
                    newStage = BOOST;
                    LOGI("LIFTOFF DETECTED! Vertical accel: %0.2f G", verticalAccel);
                }
            } else {
                highAccelDetected = false;
            }
            break;

        case BOOST:
            // Detect motor burnout: acceleration drops to or below threshold
            if (verticalAccel <= BURNOUT_ACCEL_THRESHOLD) {
                if (!lowAccelDetected) {
                    lowAccelStartTime = now;
                    lowAccelDetected = true;
                } else if (now - lowAccelStartTime > BURNOUT_DURATION) {
                    newStage = COAST;
                    LOGI("MOTOR BURNOUT DETECTED at %0.2f m AGL", altitudeAGL);
                }
            } else {
                lowAccelDetected = false;
            }
            break;

        case COAST:
            // Detect apogee: vertical velocity approaches zero
            if (fabs(verticalVelocity) < APOGEE_VELOCITY_THRESHOLD) {
                newStage = APOGEE;
                LOGI("APOGEE DETECTED at %0.2f m AGL", altitudeAGL);
            }
            break;

        case APOGEE:
            // Transition to expecting drogue - starting descent
            if (verticalVelocity < -2.0) {  // Descending at > 2 m/s
                newStage = EXPECTING_DROGUE;
                LOGI("EXPECTING DROGUE DEPLOYMENT at %0.2f m AGL, descent rate: %0.2f m/s",
                     altitudeAGL, -verticalVelocity);
            }
            break;

        case EXPECTING_DROGUE:
            // Detect drogue deployment by change in descent rate
            // Under drogue, descent rate should be moderate (DROGUE_DESCENT_MIN to MAX)
            // Without drogue (ballistic), descent rate would be much higher
            if (verticalVelocity < -DROGUE_DESCENT_MIN && verticalVelocity > -DROGUE_DESCENT_MAX) {
                // Moderate descent rate suggests drogue is deployed
                newStage = UNDER_DROGUE;
                LOGI("UNDER DROGUE detected at %0.2f m AGL, descent rate: %0.2f m/s",
                     altitudeAGL, -verticalVelocity);
            }
            break;

        case UNDER_DROGUE:
            // At typical main deployment altitude, expect main deployment
            // Check if we're below the main deployment altitude threshold
            if (altitudeAGL < MAIN_DEPLOY_ALTITUDE) {
                newStage = EXPECTING_MAIN;
                LOGI("EXPECTING MAIN DEPLOYMENT at %0.2f m AGL", altitudeAGL);
            }
            break;

        case EXPECTING_MAIN:
            // Detect main deployment by significant reduction in descent rate
            // Under main, descent rate should be slow (MAIN_DESCENT_MIN to MAX)
            if (verticalVelocity < -MAIN_DESCENT_MIN && verticalVelocity > -MAIN_DESCENT_MAX) {
                newStage = UNDER_MAIN;
                LOGI("UNDER MAIN detected at %0.2f m AGL, descent rate: %0.2f m/s",
                     altitudeAGL, -verticalVelocity);
            }
            break;

        case UNDER_MAIN:
            // Detect landing: low velocity sustained
            if (fabs(verticalVelocity) < LANDING_VELOCITY_THRESHOLD) {
                if (!lowVelocityDetected) {
                    lowVelocityStartTime = now;
                    lowVelocityDetected = true;
                } else if (now - lowVelocityStartTime > LANDING_DURATION) {
                    newStage = LANDED;
                    LOGI("LANDING DETECTED at %0.2f m AGL", altitudeAGL);
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

void RocketState::updateApogeeEstimate() {
    if (currentStage == BOOST || currentStage == COAST) {
        // Simple kinematic apogee estimation
        // Assumes constant deceleration due to drag and gravity
        // More sophisticated models could use drag coefficient estimation

        if (verticalVelocity > 0) {
            // Time to apogee: t = v / a (assuming constant deceleration)
            // Use current vertical deceleration as estimate
            double currentDecel = fabs(verticalAccel - 1.0);  // Subtract 1G for gravity
            if (currentDecel > 0.1) {
                timeToApogee = verticalVelocity / (currentDecel * 9.81);

                // Apogee altitude: h = h0 + v*t - 0.5*a*t^2
                apogeeEstimate = altitudeAGL + groundLevelAltitude +
                                verticalVelocity * timeToApogee -
                                0.5 * currentDecel * 9.81 * timeToApogee * timeToApogee;
            } else {
                // Fallback: assume free fall deceleration (1G)
                timeToApogee = verticalVelocity / 9.81;
                apogeeEstimate = altitudeAGL + groundLevelAltitude +
                                0.5 * verticalVelocity * timeToApogee;
            }
        } else {
            timeToApogee = 0;
            apogeeEstimate = altitudeAGL + groundLevelAltitude;
        }
    } else if (currentStage == APOGEE || currentStage == EXPECTING_DROGUE ||
               currentStage == UNDER_DROGUE || currentStage == EXPECTING_MAIN ||
               currentStage == UNDER_MAIN || currentStage == LANDED) {
        // Past apogee - use maximum altitude achieved
        apogeeEstimate = maxAltitudeAGL + groundLevelAltitude;
        timeToApogee = 0;
    } else {
        // Pad idle - no estimate
        apogeeEstimate = 0;
        timeToApogee = 0;
    }
}

} // namespace astra_rocket
