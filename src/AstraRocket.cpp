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
    AstraRocket *AstraRocket::s_activeInstance = nullptr;

    AstraRocket::AstraRocket(AstraRocketConfig &cfg)
        : config(cfg),
          astraSys(nullptr),
          rocketState(nullptr),
          kalmanFilter(nullptr),
          orientationFilter(nullptr),
          hitlAccel(nullptr),
          hitlGyro(nullptr),
          hitlMag(nullptr),
          hitlBaro(nullptr),
          hitlGps(nullptr),
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
        if (s_activeInstance == this)
            s_activeInstance = nullptr;
        if (astraSys)
            delete astraSys;
        if (rocketState)
            delete rocketState;
        if (kalmanFilter)
            delete kalmanFilter;
        if (orientationFilter)
            delete orientationFilter;
        if (hitlAccel)
            delete hitlAccel;
        if (hitlGyro)
            delete hitlGyro;
        if (hitlMag)
            delete hitlMag;
        if (hitlBaro)
            delete hitlBaro;
        if (hitlGps)
            delete hitlGps;
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

        // Configure base Astra system
        config.withLoggingRate(config.getPreflightLogRate());
        config.withDataLogs(dataSinks, numDataSinks);
        config.withEventLogs(eventSinks, numEventSinks);

        if (config.getHITLEnabled())
        {
            hitlAccel = new HITLAccel();
            hitlGyro = new HITLGyro();
            hitlMag = new HITLMag();
            hitlBaro = new HITLBarometer();
            hitlGps = new HITLGPS();

            config.withAccel(hitlAccel)
                .withGyro(hitlGyro)
                .withMag(hitlMag)
                .withBaro(hitlBaro)
                .withGPS(hitlGps);
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

            // Register HITL handler with Astra's SerialMessageRouter
            s_activeInstance = this;
            SerialMessageRouter *router = astraSys->getMessageRouter();
            if (router)
            {
                router->withListener("HITL/", AstraRocket::handleHITLMessage);
            }
            else
            {
                LOGE("Astra message router not available - HITL cannot continue.");
                return false;
            }
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
            // HITL mode: router handles all incoming HITL/ messages
            SerialMessageRouter *router = astraSys->getMessageRouter();
            if (router)
                router->update();
            return;
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

    void AstraRocket::handleHITLMessage(const char *message, const char *prefix, Stream *source)
    {
        (void)prefix;
        (void)source;

        if (!s_activeInstance || !message)
            return;

        double simTime = 0.0;
        if (!HITLParser::parse(message, simTime))
        {
            LOGE("AstraRocket::handleHITLMessage() - HITL parse FAILED");
            return;
        }

        s_activeInstance->astraSys->update(simTime);

        if (!s_activeInstance->hitlGroundLevelSet)
        {
            SensorManager *sm = s_activeInstance->config.getSensorManager();
            if (sm && sm->getBaroSource() && sm->getBaroSource()->isInitialized())
            {
                s_activeInstance->groundLevelAltitude = sm->getBaroSource()->getASLAltM();
                s_activeInstance->rocketState->setGroundLevel(s_activeInstance->groundLevelAltitude);
                s_activeInstance->hitlGroundLevelSet = true;
                LOGI("HITL ground level established: %0.2f m MSL", s_activeInstance->groundLevelAltitude);
            }
        }
    }
} // namespace astra_rocket
