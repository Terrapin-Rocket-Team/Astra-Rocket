#include "AstraRocket.h"
#include <RecordData/Logging/EventLogger.h>

// Include all possible sensor implementations for auto-detection
#include <Sensors/Baro/DPS368.h>
#include <Sensors/Baro/BMP390.h>
#include <Sensors/GPS/MAX_M10S.h>
#include <Sensors/GPS/SAM_M8Q.h>
#include <Sensors/IMU/BMI088andLIS3MDL.h>
#include <Sensors/IMU/BNO055.h>
#include <Sensors/Accel/ADXL375.h>
#include <Sensors/Accel/H3LIS331DL.h>

namespace astra_rocket {

AstraRocket::AstraRocket()
    : astraSys(nullptr),
      rocketState(nullptr),
      barometer(nullptr),
      gps(nullptr),
      imu(nullptr),
      highGAccel(nullptr),
      sensorArray(nullptr),
      numSensors(0),
      dataSinks(nullptr),
      eventSinks(nullptr),
      numDataSinks(0),
      numEventSinks(0),
      liftoffTime(0),
      previousStage(PAD_IDLE),
      groundLevelAltitude(0)
{
    // Default configuration already set in AstraRocketConfig constructor
}

AstraRocket::AstraRocket(AstraRocketConfig &cfg)
    : config(cfg),
      astraSys(nullptr),
      rocketState(nullptr),
      barometer(nullptr),
      gps(nullptr),
      imu(nullptr),
      highGAccel(nullptr),
      sensorArray(nullptr),
      numSensors(0),
      dataSinks(nullptr),
      eventSinks(nullptr),
      numDataSinks(0),
      numEventSinks(0),
      liftoffTime(0),
      previousStage(PAD_IDLE),
      groundLevelAltitude(0)
{
}

AstraRocket::~AstraRocket() {
    if (astraSys) delete astraSys;
    if (rocketState) delete rocketState;
    if (sensorArray) delete[] sensorArray;
    if (dataSinks) delete[] dataSinks;
    if (eventSinks) delete[] eventSinks;
}

bool AstraRocket::init() {
    LOGI("AstraRocket initialization starting...");

    // Setup logging first so we can log initialization progress
    setupLogging();

    // Auto-detect or use configured sensors
    if (!autoDetectSensors()) {
        LOGE("Sensor detection/initialization failed!");
        return false;
    }

    // Initialize all sensors
    if (!initializeSensors()) {
        LOGE("Sensor initialization failed!");
        return false;
    }

    // Create rocket state with detected sensors
    rocketState = new RocketState(sensorArray, numSensors, nullptr);
    LOGI("RocketState created with %d sensors", numSensors);

    // Configure base Astra system
    config.getAstraConfig()->withState(rocketState);
    config.getAstraConfig()->withUpdateRate(50.0);  // 50 Hz default
    config.getAstraConfig()->withLoggingRate(config.getPreflightLogRate());

    // Configure BlinkBuzz for status indicators
    if (config.getBuzzerFeedback()) {
        config.getAstraConfig()->withBBAsync(true);
        config.getAstraConfig()->withBBPin(config.getLEDStatusPin());
    }

    // Create Astra system
    astraSys = new Astra(config.getAstraConfig());
    astraSys->init();
    LOGI("Astra system initialized successfully");

    // Establish ground level reference
    // Wait a moment for barometer to stabilize
    delay(500);
    astraSys->update();
    if (barometer && barometer->isInitialized()) {
        groundLevelAltitude = barometer->getASLAltM();
        rocketState->setGroundLevel(groundLevelAltitude);
        LOGI("Ground level established: %0.2f m MSL", groundLevelAltitude);
    } else {
        LOGW("Barometer not available - AGL calculations disabled");
    }

    // Initialization complete
    LOGI("AstraRocket initialization complete!");
    LOGI("Flight computer ready. Flight stage: %s", flightStageToString(PAD_IDLE));

    return true;
}

void AstraRocket::update() {
    // Update Astra system (sensors, state estimation, logging)
    astraSys->update();

    // Check for flight stage transitions
    FlightStage currentStage = rocketState->getFlightStage();
    if (currentStage != previousStage) {
        handleStageTransition(currentStage);
        previousStage = currentStage;
    }

    // Update status indicators
    updateStatusIndicators();
}

FlightStage AstraRocket::getFlightStage() const {
    if (rocketState) {
        return rocketState->getFlightStage();
    }
    return PAD_IDLE;
}

// ===== Private Helper Methods =====

bool AstraRocket::autoDetectSensors() {
    LOGI("Starting sensor auto-detection...");

    // Get sensors from config or auto-detect
    barometer = config.getBarometer();
    if (!barometer) {
        barometer = detectBarometer();
    }

    gps = config.getGPS();
    if (!gps) {
        gps = detectGPS();
    }

    imu = config.getIMU();
    if (!imu) {
        imu = detectIMU();
    }

    highGAccel = config.getHighGAccel();
    if (!highGAccel) {
        highGAccel = detectHighGAccel();
    }

    // Build sensor array
    sensorArray = new Sensor*[MAX_SENSORS];
    numSensors = 0;

    if (barometer) {
        sensorArray[numSensors++] = barometer;
        LOGI("Using barometer: %s", barometer->getName());
    } else {
        LOGW("No barometer detected!");
    }

    if (gps) {
        sensorArray[numSensors++] = gps;
        LOGI("Using GPS: %s", gps->getName());
    } else {
        LOGI("No GPS configured (optional)");
    }

    if (imu) {
        sensorArray[numSensors++] = imu;
        LOGI("Using IMU: %s", imu->getName());
    } else {
        LOGW("No IMU detected!");
    }

    if (highGAccel) {
        sensorArray[numSensors++] = highGAccel;
        LOGI("Using high-G accelerometer: %s", highGAccel->getName());
    } else {
        LOGW("No high-G accelerometer detected!");
    }

    // Require at least barometer for basic functionality
    if (!barometer) {
        LOGE("Cannot operate without barometer!");
        return false;
    }

    return true;
}

bool AstraRocket::initializeSensors() {
    LOGI("Initializing %d sensors...", numSensors);

    bool allSuccess = true;
    for (int i = 0; i < numSensors; i++) {
        if (!sensorArray[i]->begin()) {
            LOGE("Failed to initialize sensor: %s", sensorArray[i]->getName());
            allSuccess = false;
        } else {
            LOGI("Sensor initialized: %s", sensorArray[i]->getName());
        }
    }

    return allSuccess;
}

void AstraRocket::setupLogging() {
    // Create log sinks
    dataSinks = new ILogSink*[MAX_LOG_SINKS];
    eventSinks = new ILogSink*[MAX_LOG_SINKS];

    // USB log for debugging
    USBLog *usbLog = new USBLog(Serial, 115200, true);
    dataSinks[numDataSinks++] = usbLog;
    eventSinks[numEventSinks++] = usbLog;

    // Configure EventLogger
    EventLogger::configure(eventSinks, numEventSinks);

    // DataLogger will be configured after sensors are created
    LOGI("Logging configured with %d sinks", numDataSinks);
}

void AstraRocket::handleStageTransition(FlightStage newStage) {
    LOGI("Stage transition: %s -> %s",
         flightStageToString(previousStage),
         flightStageToString(newStage));

    // Handle specific transitions
    switch (newStage) {
        case BOOST:
            liftoffTime = millis();
            // Switch to high-rate logging
            config.getAstraConfig()->withLoggingRate(config.getFlightLogRate());
            LOGI("Switched to flight logging rate: %0.1f Hz", config.getFlightLogRate());
            break;

        case LANDED:
            // Switch to low-rate logging
            config.getAstraConfig()->withLoggingRate(config.getPostflightLogRate());
            LOGI("Switched to postflight logging rate: %0.1f Hz", config.getPostflightLogRate());
            break;

        default:
            break;
    }
}

void AstraRocket::updateStatusIndicators() {
    // Simple status LED pattern based on flight stage
    // This could be expanded with BlinkBuzz patterns
    FlightStage stage = rocketState->getFlightStage();

    // For now, just keep LED on during flight
    if (stage != PAD_IDLE && stage != LANDED) {
        digitalWrite(config.getLEDStatusPin(), HIGH);
    } else {
        // Blink slowly on pad/landed
        digitalWrite(config.getLEDStatusPin(), (millis() / 1000) % 2);
    }
}

// ===== Sensor Auto-Detection =====

Barometer* AstraRocket::detectBarometer() {
    LOGI("Auto-detecting barometer...");

    // Try DPS368 first (I2C 0x77)
    DPS368 *dps = new DPS368();
    if (dps->begin()) {
        LOGI("Detected DPS368 barometer");
        return dps;
    }
    delete dps;

    // Try BMP390 (I2C 0x76 or 0x77)
    BMP390 *bmp = new BMP390();
    if (bmp->begin()) {
        LOGI("Detected BMP390 barometer");
        return bmp;
    }
    delete bmp;

    LOGW("No barometer detected");
    return nullptr;
}

GPS* AstraRocket::detectGPS() {
    LOGI("Auto-detecting GPS...");

    // Try MAX-M10S
    MAX_M10S *maxm10 = new MAX_M10S();
    if (maxm10->begin()) {
        LOGI("Detected MAX-M10S GPS");
        return maxm10;
    }
    delete maxm10;

    // Try SAM-M8Q (in mmfs namespace)
    mmfs::SAM_M8Q *samm8 = new mmfs::SAM_M8Q();
    if (samm8->begin()) {
        LOGI("Detected SAM-M8Q GPS");
        return samm8;
    }
    delete samm8;

    LOGI("No GPS detected (optional)");
    return nullptr;
}

IMU* AstraRocket::detectIMU() {
    LOGI("Auto-detecting IMU...");

    // Try BMI088 + LIS3MDL
    BMI088andLIS3MDL *bmi088 = new BMI088andLIS3MDL();
    if (bmi088->begin()) {
        LOGI("Detected BMI088+LIS3MDL IMU");
        return bmi088;
    }
    delete bmi088;

    // Try BNO055
    BNO055 *bno = new BNO055();
    if (bno->begin()) {
        LOGI("Detected BNO055 IMU");
        return bno;
    }
    delete bno;

    LOGW("No IMU detected");
    return nullptr;
}

Accel* AstraRocket::detectHighGAccel() {
    LOGI("Auto-detecting high-G accelerometer...");

    // Try ADXL375 (I2C 0x53)
    ADXL375 *adxl = new ADXL375();
    if (adxl->begin()) {
        LOGI("Detected ADXL375 high-G accelerometer");
        return adxl;
    }
    delete adxl;

    // Try H3LIS331DL (I2C 0x18 or 0x19)
    H3LIS331DL *h3lis = new H3LIS331DL();
    if (h3lis->begin()) {
        LOGI("Detected H3LIS331DL high-G accelerometer");
        return h3lis;
    }
    delete h3lis;

    LOGW("No high-G accelerometer detected");
    return nullptr;
}

} // namespace astra_rocket
