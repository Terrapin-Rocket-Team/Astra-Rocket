#include "RocketState.h"
#include <RecordData/Logging/EventLogger.h>
#include <Math/Vector.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/SensorManager/SensorManager.h>

using namespace astra;

namespace astra_rocket
{

    RocketState::RocketState(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter)
        : State(filter, orientationFilter),
          currentStage(PAD_IDLE),
          previousStage(PAD_IDLE),
          timeInCurrentStage(0),
          stageStartTime(0),
          offVerticalAngle(0),
          highAccelStartTime(0),
          lowAccelStartTime(0),
          lowVelocityStartTime(0),
          drogueDetectStartTime(0),
          mainDetectStartTime(0),
          highAccelDetected(false),
          lowAccelDetected(false),
          lowVelocityDetected(false),
          drogueRateDetected(false),
          mainRateDetected(false)
    {
        // Add rocket-specific columns to DataReporter
        insertColumn(0, "%d", &currentStage, "Flight Stage");
        addColumn("%0.3f", &offVerticalAngle, "Off-Vert Angle (deg)");
        addColumn("%0.3f", &timeInCurrentStage, "Time in Stage (s)");
    }

    void RocketState::setGroundLevel(double altitudeMSL)
    {
        origin.z() = altitudeMSL;
        LOGI("Ground level set to %0.2f m MSL (baro offset applied)", origin.z());
    }

    void RocketState::setFlightStage(FlightStage stage)
    {
        if (stage != currentStage)
        {
            previousStage = currentStage;
            currentStage = stage;
            stageStartTime = (unsigned long)(currentTime * 1000.0);
            timeInCurrentStage = 0;
            LOGI("Flight stage transition: %s -> %s",
                 flightStageToString(previousStage),
                 flightStageToString(currentStage));
        }
    }

    bool RocketState::update(double currentTimeSec)
    {
        // Call parent implementation first - handles measurement update
        bool success = State::update(currentTimeSec);

        // Update time in current stage
        unsigned long currentMillis = (unsigned long)(currentTime * 1000.0);
        timeInCurrentStage = (currentMillis - stageStartTime) / 1000.0;

        // Calculate rocket-specific derived values that depend on orientation
        calculateTilt();
        detectFlightStage();

        return success;
    }

    void RocketState::calculateTilt()
    {
        // Calculate off-vertical angle using orientation quaternion
        // This is more accurate than using acceleration alone
        MahonyAHRS *ahrs = getOrientationFilter();

        // Updated to use isReady() (was isInitialized)
        if (ahrs && ahrs->isReady())
        {
            // Get the current orientation quaternion
            Quaternion q = orientation;

            // Rocket body Z-axis in body frame (pointing along rocket, up in ENU)
            Vector<3> bodyZ(0, 0, 1);

            // Rotate body Z into earth frame to see which way rocket is pointing
            Vector<3> rocketDir = q.rotateVector(bodyZ);

            // Earth vertical reference (up in ENU is +Z)
            Vector<3> verticalRef(0, 0, 1);

            // Calculate angle between rocket direction and vertical
            double cosAngle = rocketDir.dot(verticalRef);
            offVerticalAngle = acos(fmax(-1.0, fmin(1.0, cosAngle))) * 180.0 / M_PI;
        }
    }

    void RocketState::detectFlightStage()
    {
        unsigned long now = (unsigned long)(currentTime * 1000.0);
        FlightStage newStage = currentStage;

        switch (currentStage)
        {
        case PAD_IDLE:
        {
            // Detect liftoff: sustained high vertical acceleration from KF state
            double accelMag = acceleration.magnitude();
            // Optional debug spam reduction
            // fprintf(stderr, "DEBUG RocketState: PAD_IDLE - accelMag=%0.3f\n", accelMag);

            if (accelMag > LIFTOFF_ACCEL_THRESHOLD)
            {
                if (!highAccelDetected)
                {
                    highAccelStartTime = now;
                    highAccelDetected = true;
                    fprintf(stderr, "DEBUG RocketState: High accel detected! Starting timer...\n");
                }
                else if (now - highAccelStartTime > LIFTOFF_DURATION)
                {
                    newStage = BOOST;
                    LOGI("LIFTOFF DETECTED! Vertical accel: %0.2f G", acceleration.z());
                    fprintf(stderr, "DEBUG RocketState: LIFTOFF DETECTED! Locking orientation frame...\n");

                    // --- CRITICAL ORIENTATION UPDATE ---
                    MahonyAHRS *ahrs = getOrientationFilter();
                    if (ahrs)
                    {
                        // 1. Lock the reference frame.
                        // This takes the current "Up" (which we've been snapping to on the pad)
                        // and locks it as the permanent Earth Z-axis reference.
                        if (!ahrs->isFrameLocked())
                        {
                            ahrs->lockFrame();
                            LOGI("Orientation filter frame locked at launch");
                        }

                        // 2. Switch Mode to CORRECTING.
                        // We are currently in CALIBRATING mode (Pad Snapping).
                        // We must switch to CORRECTING to enable Gyro integration and allow
                        // the State::updateOrientation logic to manage High-G handling.
                        ahrs->setMode(MahonyMode::CORRECTING);
                        LOGI("Orientation filter switched to CORRECTING mode for flight.");
                    }
                }
            }
            else
            {
                highAccelDetected = false;
            }
            break;
        }

        case BOOST:
            // Detect motor burnout: earth-frame acceleration magnitude drops to or below threshold
            // Use earth-frame acceleration (from State), not raw accelerometer reading
            {
                double earthAccelMag = acceleration.magnitude();

                if (earthAccelMag <= BURNOUT_ACCEL_THRESHOLD)
                {
                    if (!lowAccelDetected)
                    {
                        lowAccelStartTime = now;
                        lowAccelDetected = true;
                    }
                    else if (now - lowAccelStartTime > BURNOUT_DURATION)
                    {
                        newStage = COAST;
                        LOGI("MOTOR BURNOUT DETECTED at %0.2f m AGL, earth accel: %0.2f m/s²",
                             position.z(), earthAccelMag);
                    }
                }
                else
                {
                    lowAccelDetected = false;
                }
            }
            break;

        case COAST:
            // Detect apogee: vertical velocity approaches zero
            if (fabs(velocity.z()) < APOGEE_VELOCITY_THRESHOLD || timeInCurrentStage > 25)
            {
                newStage = APOGEE;
                LOGI("APOGEE DETECTED at %0.2f m AGL", position.z());
            }
            break;

        case APOGEE:
            // Transition to expecting drogue - starting descent
            if (velocity.z() < -2.0)
            { // Descending at > 2 m/s
                newStage = EXPECTING_DROGUE;
                LOGI("EXPECTING DROGUE DEPLOYMENT at %0.2f m AGL, descent rate: %0.2f m/s",
                     position.z(), -velocity.z());
            }
            break;

        case EXPECTING_DROGUE:
            // Detect drogue deployment by sustained descent rate in valid range
            if (velocity.z() < -DROGUE_DESCENT_MIN && velocity.z() > -DROGUE_DESCENT_MAX)
            {
                if (!drogueRateDetected)
                {
                    drogueDetectStartTime = now;
                    drogueRateDetected = true;
                }
                else if (now - drogueDetectStartTime > DROGUE_DETECT_DURATION)
                {
                    newStage = UNDER_DROGUE;
                    LOGI("UNDER DROGUE detected at %0.2f m AGL, descent rate: %0.2f m/s",
                         position.z(), -velocity.z());
                }
            }
            else
            {
                drogueRateDetected = false;
            }
            break;

        case UNDER_DROGUE:
            // Check if we're below the main deployment altitude threshold
            if (position.z() < MAIN_DEPLOY_ALTITUDE)
            {
                newStage = EXPECTING_MAIN;
                LOGI("EXPECTING MAIN DEPLOYMENT at %0.2f m AGL", position.z());
            }
            break;

        case EXPECTING_MAIN:
            // Detect main deployment by sustained descent rate in valid range
            if (velocity.z() < -MAIN_DESCENT_MIN && velocity.z() > -MAIN_DESCENT_MAX)
            {
                if (!mainRateDetected)
                {
                    mainDetectStartTime = now;
                    mainRateDetected = true;
                }
                else if (now - mainDetectStartTime > MAIN_DETECT_DURATION)
                {
                    newStage = UNDER_MAIN;
                    LOGI("UNDER MAIN detected at %0.2f m AGL, descent rate: %0.2f m/s",
                         position.z(), -velocity.z());
                }
            }
            else
            {
                mainRateDetected = false;
            }
            break;

        case UNDER_MAIN:
            // Detect landing: low velocity sustained
            if (fabs(velocity.z()) < LANDING_VELOCITY_THRESHOLD)
            {
                if (!lowVelocityDetected)
                {
                    lowVelocityStartTime = now;
                    lowVelocityDetected = true;
                }
                else if (now - lowVelocityStartTime > LANDING_DURATION)
                {
                    newStage = LANDED;
                    LOGI("LANDING DETECTED at %0.2f m AGL", position.z());
                }
            }
            else
            {
                lowVelocityDetected = false;
            }
            break;

        case LANDED:
            // Terminal state
            break;
        }

        // Update stage if changed
        if (newStage != currentStage)
        {
            setFlightStage(newStage);
        }
    }

} // namespace astra_rocket