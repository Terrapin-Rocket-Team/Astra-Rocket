#include "AstraRocket.h"
#include "Sensors/SensorManager/SensorManager.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/DataLogger.h>
#include <BlinkBuzz/BlinkBuzz.h>
#include "RadioLog.h"
#include <Sensors/HITL/HITL.h>

#ifndef ASTRA_ROCKET_VERSION
#define ASTRA_ROCKET_VERSION "UNKNOWN"
#endif

namespace astra_rocket
{
    AstraRocket::AstraRocket(AstraRocketConfig &cfg)
        : config(cfg),
          astraSys(nullptr),
          rocketState(nullptr),
          kalmanFilter(nullptr),
          orientationFilter(nullptr),
          dataSinks(nullptr),
          eventSinks(nullptr),
          numDataSinks(0),
          numEventSinks(0),
          groundLevelAltitude(0),
          hitlGroundLevelSet(false)
    {
    }

    AstraRocket::~AstraRocket()
    {
        if (astraSys)
            delete astraSys;
        if (rocketState)
            delete rocketState;
        if (kalmanFilter)
            delete kalmanFilter;
        if (orientationFilter)
            delete orientationFilter;
        if (dataSinks)
            delete[] dataSinks;
        if (eventSinks)
            delete[] eventSinks;
    }

    bool AstraRocket::init()
    {
        LOGI("Initializing Astra-Rocket version %s", ASTRA_ROCKET_VERSION);

        // Setup logging first so we can log initialization progress
        setupLogging();

        // Note: Sensors will be initialized by Astra::init() -> State::begin()
        // No need to initialize them manually here

        // Create Kalman filter for state estimation
        kalmanFilter = new RocketKF();
        LOGI("RocketKF Kalman filter created");

        // Create orientation filter for AHRS
        // Using default gains: Kp=0.1, Ki=0.0005
        orientationFilter = new MahonyAHRS(0.1, 0.0005);
        LOGI("MahonyAHRS orientation filter created");

        // Create rocket state with Kalman filter and orientation filter
        // Sensors are managed by Astra's SensorManager, not passed to RocketState
        rocketState = new RocketState(kalmanFilter, orientationFilter);
        LOGI("RocketState created with Kalman filter and orientation filter");
        config.withState(rocketState);

        // Configure base Astra system
        config.withLoggingRate(config.getPreflightLogRate());
        config.withDataLogs(dataSinks, numDataSinks);

        auto sm = config.getSensorManager();
        if (sm)
        {
            sm->setPrimaryAccel(new HITLAccel());
            sm->setPrimaryGyro(new HITLGyro());
            sm->setPrimaryBaro(new HITLBarometer());
            sm->setPrimaryGPS(new HITLGPS());
            sm->setPrimaryMag(new HITLMag());
        }
        else
        {
            LOGE("Sensor Manager is not working properly. HITL/SITL cannot continue.");
            return false;
        }

        // Create Astra system
        astraSys = new Astra(&config);
        astraSys->init();
        LOGI("Astra system initialized successfully. Reading sensors to establish baseline.");

        // Establish ground level reference
        if (config.getHITLEnabled())
        {
            // In HITL mode, skip initial ground level setup
            // It will be established on first valid HITL packet
            LOGI("HITL mode: Ground level will be set from first simulation packet");
        }
        else
        {
            // Hardware mode: Wait for barometer to stabilize
            delay(500);
            astraSys->update();
            if (config.getSensorManager() && config.getSensorManager()->getPrimaryBaro()->isInitialized())
            {
                groundLevelAltitude = Barometer::calcAltitude(config.getSensorManager()->getPressure());
                rocketState->setGroundLevel(groundLevelAltitude);
                LOGI("Ground level established: %0.2f m MSL", groundLevelAltitude);
            }
            else
            {
                LOGE("Barometer not available!");
                return false;
            }
        }

        // Initialization complete
        LOGI("AstraRocket initialization complete!");
        LOGI("Flight computer ready. Flight stage: %s", flightStageToString(PAD_IDLE));

        return ready = true;
    }

    void AstraRocket::update()
    {

        if (!ready)
        {
            LOGE("AstraRocket not initialized!");
            return;
        }
        // Check if HITL mode is enabled
        if (config.getHITLEnabled())
        {

            // HITL mode: wait for incoming sensor data from simulation
            if (Serial.available())
            {
                String line = Serial.readStringUntil('\n');

                if (line.startsWith("HITL/"))
                {
                    // Parse incoming HITL packet

                    double simTime;
                    if (HITLParser::parseAndInject(line.c_str(), simTime))
                    {

                        // Update with simulation time (in seconds)
                        // simTime is already in seconds from the HITL packet
                        bool updateResult = astraSys->update(simTime);

                        // Set ground level from first valid packet (after update)
                        if (!hitlGroundLevelSet)
                        {
                            auto barometer = config.getSensorManager()->getPrimaryBaro();

                            // Debug: Check what pressure value we're reading
                            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

                            groundLevelAltitude = barometer->getASLAltM();
                            rocketState->setGroundLevel(groundLevelAltitude);
                            hitlGroundLevelSet = true;
                        }
                    }
                    else
                    {
                        LOGE("AstraRocket::update() - HITL parse FAILED");
                    }
                }
                else
                {
                }
            }
            else
            {
                // No data available - this is normal and expected most of the time
            }
        }
        else
        {

            // Normal hardware mode: update with real time
            astraSys->update();
        }
    }

    // ===== Private Helper Methods =====

    void AstraRocket::setupLogging()
    {
        // Create log sinks
        dataSinks = new ILogSink *[ASTRA_ROCKET_MAX_LOG_SINKS];
        eventSinks = new ILogSink *[ASTRA_ROCKET_MAX_LOG_SINKS];

        // USB log for debugging - use PrintLog which works with any Print object
        Serial.begin(115200);
        PrintLog *usbLog = new PrintLog(Serial, true);
        dataSinks[numDataSinks++] = usbLog;
        eventSinks[numEventSinks++] = usbLog;

        // RadioLog *rad = new RadioLog(*config.getRadioSerial());
        // dataSinks[numDataSinks++] = rad;

        // SD card log for data recording
        // FileLogSink *sdEventLog = new FileLogSink("events.log", config.getStorageBackend(), false);
        // FileLogSink *sdDataLog = new FileLogSink("data.csv", config.getStorageBackend(), false);

        // eventSinks[numEventSinks++] = sdEventLog;
        // dataSinks[numDataSinks++] = sdDataLog;

        // Configure EventLogger
        EventLogger::configure(eventSinks, numEventSinks);

        // Now that EventLogger is configured, log the summary
        for (int i = 0; i < numEventSinks; i++)
        {
            if (eventSinks[i]->ok())
            {
                LOGI("Data sink %d ok.", i);
            }
            else
            {
                LOGW("Data sink %d FAILED", i);
            }
        }
    }
} // namespace astra_rocket
