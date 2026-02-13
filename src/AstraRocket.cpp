#include "AstraRocket.h"
#include "Sensors/SensorManager/SensorManager.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/DataLogger.h>
#include <BlinkBuzz/BlinkBuzz.h>
#include "RecordData/Logging/LoggingBackend/RadioLog.h"

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
          groundLevelAltitude(0)
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
        kalmanFilter = new DefaultKalmanFilter();
        LOGI("DefaultKalmanFilter created");

        // Create orientation filter for AHRS
        // Using default gains: Kp=0.1, Ki=0.0005
        orientationFilter = new MahonyAHRS(0.1, 0.0005);
        LOGI("MahonyAHRS orientation filter created");

        // Create rocket state with Kalman filter and orientation filter
        // Sensors are managed by Astra's SensorManager, not passed to RocketState
        rocketState = new RocketState(kalmanFilter, orientationFilter);
        LOGI("RocketState created with Kalman filter and orientation filter");
        config.withState(rocketState);

        // Configure mode-dependent runtime behavior (HITL vs hardware)
        configureRuntimeMode();

        // Create Astra system
        astraSys = new Astra(&config);
        int initErrors = astraSys->init();
        if (initErrors > 0)
        {
            LOGW("Astra initialized with %d sensor error(s).", initErrors);
        }
        LOGI("Astra system initialized successfully. Reading sensors to establish baseline.");

        // Establish ground level reference
        if (config.getHITLEnabled())
        {
            // Astra core now owns HITL routing/parsing and baseline setup.
            LOGI("HITL mode enabled. Astra core will process HITL packets.");
        }
        else
        {
            // Hardware mode: Wait for barometer to stabilize
            delay(500);
            astraSys->update();
            if (config.getSensorManager() && config.getSensorManager()->getBaroSource() && config.getSensorManager()->getBaroSource()->isInitialized())
            {
                groundLevelAltitude = config.getSensorManager()->getBaroSource()->getASLAltM();
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

    void AstraRocket::configureRuntimeMode()
    {
        config.withDataLogs(dataSinks, numDataSinks);
        config.withEventLogs(eventSinks, numEventSinks);

        if (config.getHITLEnabled())
        {
            configureHITLMode();
        }
        else
        {
            // Hardware mode uses preflight log rate until liftoff logic changes it.
            config.withLoggingRate(config.getPreflightLogRate());
        }
    }

    void AstraRocket::configureHITLMode()
    {
        // HITL policy is handled in Astra core via AstraConfig::withHITL().
        config.withHITL(true);
    }

    void AstraRocket::update()
    {

        if (!ready)
        {
            LOGE("AstraRocket not initialized!");
            return;
        }

        // Astra::update() handles both hardware and HITL modes.
        astraSys->update();
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
