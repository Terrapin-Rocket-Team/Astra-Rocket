#include "RocketState.h"
#include <RecordData/Logging/EventLogger.h>
#include <Math/Vector.h>
#include <Sensors/Accel/Accel.h>

using namespace astra;

namespace astra_rocket {

RocketState::RocketState(Sensor **sensors, int numSensors, Filter *filter, MahonyAHRS *orientationFilter)
    : State(sensors, numSensors, filter, orientationFilter),
      currentStage(PAD_IDLE),
      previousStage(PAD_IDLE),
      timeInCurrentStage(0),
      stageStartTime(0),
      groundLevelAltitude(0),
      baroOffset(0),
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
    insertColumn(0, "%d", &currentStage, "Flight Stage");
    addColumn("%0.3f", &apogeeEstimate, "Apogee Est (m)");
    addColumn("%0.3f", &timeToApogee, "Time to Apogee (s)");
    addColumn("%0.3f", &offVerticalAngle, "Off-Vert Angle (deg)");
    addColumn("%0.3f", &timeInCurrentStage, "Time in Stage (s)");
}

void RocketState::setGroundLevel(double altitudeMSL) {
    groundLevelAltitude = altitudeMSL;
    baroOffset = altitudeMSL;  // Zero the KF by subtracting this offset from baro readings
    LOGI("Ground level set to %0.2f m MSL (baro offset applied)", groundLevelAltitude);
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
    // We need to handle the KF update ourselves to apply the baro offset
    // So we partially replicate State::updateVariables() logic

    GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
    // IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
    Accel *accel_sensor = reinterpret_cast<Accel *>(getSensor("Accelerometer"_i));
    Gyro *gyro_sensor = reinterpret_cast<Gyro *>(getSensor("Gyroscope"_i));
    Barometer *baro = reinterpret_cast<Barometer *>(getSensor("Barometer"_i));

    // Determine which sensors are available for orientation
    bool hasIMU = false;
    bool hasAccelGyro = sensorOK(accel_sensor) && sensorOK(gyro_sensor);

    // Update orientation filter if available
    if (orientationFilter && (hasIMU || hasAccelGyro))
    {
        double dt = currentTime - lastTime;
        Vector<3> accel, gyro;

        // Get accel and gyro data from IMU or separate sensors
        if (hasIMU)
        {
            // accel = imu->getAcceleration();
            // gyro = imu->getAngularVelocity();
        }
        else
        {
            accel = accel_sensor->getAccel();
            gyro = gyro_sensor->getAngVel();
        }

        // Automatic mode switching based on accelerometer magnitude
        if (orientationFilter->isInitialized())
        {
            double accelMag = accel.magnitude();
            double accelError = abs(accelMag - 9.81);

            // Threshold: if accel error < 1 m/s^2, trust the accelerometer
            if (accelError < 1.0)
            {
                if (orientationFilter->getMode() != MahonyMode::CORRECTING)
                {
                    orientationFilter->setMode(MahonyMode::CORRECTING);
                }
            }
            else
            {
                if (orientationFilter->getMode() != MahonyMode::GYRO_ONLY)
                {
                    orientationFilter->setMode(MahonyMode::GYRO_ONLY);
                }
            }
        }

        orientationFilter->update(accel, gyro, dt);

        // Update orientation and earth-frame acceleration from filter
        if (orientationFilter->isInitialized())
        {
            orientation = orientationFilter->getQuaternion();
            // Get earth-frame acceleration (with gravity subtracted)
            acceleration = orientationFilter->getEarthAcceleration(accel);
        }
    }

    if (filter)
    {
        double *measurements = new double[filter->getMeasurementSize()];
        double *inputs = new double[filter->getInputSize()];

        // gps x y barometer z (WITH OFFSET APPLIED FOR ZEROING)
        measurements[0] = sensorOK(gps) ? coordinates.x() - origin.x() : 0;
        measurements[1] = sensorOK(gps) ? coordinates.y() - origin.y() : 0;
        measurements[2] = baro->getASLAltM() - baroOffset;  // Apply offset here!

        // Earth-frame acceleration inputs (gravity already subtracted by orientation filter)
        if (orientationFilter && orientationFilter->isInitialized())
        {
            inputs[0] = acceleration.x();
            inputs[1] = acceleration.y();
            inputs[2] = acceleration.z();
        }
        else
        {
            inputs[0] = 0.0;
            inputs[1] = 0.0;
            inputs[2] = 0.0;
        }

        stateVars[0] = position.x();
        stateVars[1] = position.y();
        stateVars[2] = position.z();
        stateVars[3] = velocity.x();
        stateVars[4] = velocity.y();
        stateVars[5] = velocity.z();

        filter->iterate(currentTime - lastTime, stateVars, measurements, inputs);
        // pos x, y, z, vel x, y, z
        position.x() = stateVars[0];
        position.y() = stateVars[1];
        position.z() = stateVars[2];
        velocity.x() = stateVars[3];
        velocity.y() = stateVars[4];
        velocity.z() = stateVars[5];
    }

    if (sensorOK(gps))
    {
        coordinates = gps->getHasFix() ? Vector<2>(gps->getPos().x(), gps->getPos().y()) : Vector<2>(0, 0);
        heading = gps->getHeading();
    }
    else
    {
        coordinates = Vector<2>(0, 0);
        heading = 0;
    }

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
    // Use State class velocity from orientation filter if available
    // The State class now provides earth-frame velocity with orientation filter
    double stateVertVel = -velocity.z();  // NED frame: negative Z is up

    // If State velocity is essentially zero (not being calculated),
    // calculate from altitude changes as fallback
    if (fabs(stateVertVel) < 0.01 && currentTime > lastTime) {
        double dt = currentTime - lastTime;
        if (dt > 0.001) {  // Avoid division by very small numbers
            verticalVelocity = (altitudeAGL - previousAltitudeAGL) / dt;
        }
    } else {
        // Use State class velocity (now includes orientation filter data)
        verticalVelocity = stateVertVel;
    }

    // Store current altitude for next iteration
    previousAltitudeAGL = altitudeAGL;

    // Get earth-frame acceleration from State (which uses orientation filter)
    // This is already gravity-compensated and in earth frame (ENU: X=East, Y=North, Z=Up)
    Vector<3> earthAccel = acceleration;

    // Vertical component (Z-axis in earth frame, positive for up in ENU)
    verticalAccel = earthAccel.z() / 9.81;  // Convert to G's

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
    } else {
        // Fallback: use acceleration direction (less accurate during high-G)
        // IMU *imu = static_cast<IMU*>(getSensor("IMU"_i));
        Accel *accel_sensor = static_cast<Accel*>(getSensor("Accelerometer"_i));
        Vector<3> accelBody(0, 0, 0);

        // if (imu && sensorOK(imu)) {
        //     accelBody = imu->getAcceleration();
        // } else 
        if (accel_sensor && sensorOK(accel_sensor)) {
            accelBody = accel_sensor->getAccel();
        }

        double totalAccelMagnitude = accelBody.magnitude();
        if (totalAccelMagnitude > 1.0) {  // Only if acceleration is significant
            Vector<3> accelDir = accelBody;
            accelDir.normalize();
            Vector<3> verticalRef(0, 0, -1);  // Up in NED frame
            double cosAngle = accelDir.dot(verticalRef);
            offVerticalAngle = acos(fmax(-1.0, fmin(1.0, cosAngle))) * 180.0 / M_PI;
        } else {
            offVerticalAngle = 0;
        }
    }
}

void RocketState::updateMaxValues() {
    // Update maximum acceleration - get from IMU directly
    // IMU *imu = static_cast<IMU*>(getSensor("IMU"_i));
    // if (imu && sensorOK(imu)) {
    //     Vector<3> accel = imu->getAcceleration();
    //     double currentAccelG = accel.magnitude() / 9.81;
    //     if (currentAccelG > maxAcceleration) {
    //         maxAcceleration = currentAccelG;
    //     }
    // }

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
    Accel *accel_sensor = static_cast<Accel*>(getSensor("Accelerometer"_i));

    switch (currentStage) {
        case PAD_IDLE:
            // Detect liftoff: sustained high vertical acceleration
            if (accel_sensor->getAccel().magnitude() > LIFTOFF_ACCEL_THRESHOLD) {
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
            if (accel_sensor->getAccel().magnitude() <= BURNOUT_ACCEL_THRESHOLD) {
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
            if (fabs(verticalVelocity) < APOGEE_VELOCITY_THRESHOLD || timeInCurrentStage > 25) {
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
