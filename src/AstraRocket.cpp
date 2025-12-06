#include "AstraRocket.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/DataLogger.h>
#include <BlinkBuzz/BlinkBuzz.h>

// Include all possible sensor implementations for auto-detection
#include <Sensors/Baro/DPS368.h>
#include <Sensors/Baro/BMP390.h>
#include <Sensors/Baro/MS5611F.h>
#include <Sensors/GPS/MAX_M10S.h>
#include <Sensors/GPS/SAM_M8Q.h>
#include <Sensors/IMU/BMI088andLIS3MDL.h>
#include <Sensors/IMU/BNO055.h>
#include <Sensors/Accel/BMI088Accel.h>
#include <Sensors/Accel/ADXL375.h>
#include <Sensors/Accel/H3LIS331DL.h>
#include <Sensors/Gyro/BMI088Gyro.h>
#include <Sensors/Mag/LIS3MDL.h>

namespace astra_rocket {

AstraRocket::AstraRocket()
    : astraSys(nullptr),
      rocketState(nullptr),
      barometer(nullptr),
      gps(nullptr),
      imu(nullptr),
      accel(nullptr),
      gyro(nullptr),
      mag(nullptr),
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
      accel(nullptr),
      gyro(nullptr),
      mag(nullptr),
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

    // Note: Sensors will be initialized by Astra::init() -> State::begin()
    // No need to initialize them manually here

    // Create rocket state with detected sensors
    rocketState = new RocketState(sensorArray, numSensors, nullptr);
    LOGI("RocketState created with %d sensors", numSensors);

    // Configure base Astra system
    config.getAstraConfig()->withState(rocketState);
    config.getAstraConfig()->withUpdateRate(50.0);  // 50 Hz default
    config.getAstraConfig()->withLoggingRate(1.0);  // 1 Hz for testing/debugging
    config.getAstraConfig()->withDataLogs(dataSinks, numDataSinks);

    // Configure BlinkBuzz for status indicators
    if (config.getBuzzerFeedback()) {
        config.getAstraConfig()->withBBAsync(true);
        config.getAstraConfig()->withBBPin(config.getLEDStatusPin());
    }

    // Add sensor status and GPS status LEDs to BlinkBuzz if configured
    if (config.getSensorStatusLEDPin() >= 0) {
        config.getAstraConfig()->withBBPin(config.getSensorStatusLEDPin());
    }
    if (config.getGPSStatusLEDPin() >= 0) {
        config.getAstraConfig()->withBBPin(config.getGPSStatusLEDPin());
    }

    // Create Astra system
    astraSys = new Astra(config.getAstraConfig());
    astraSys->init();
    LOGI("Astra system initialized successfully. Reading sensors to establish baseline.");

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
    // Astra handles telemetry logging automatically at the configured rate
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

    // Try to use individual sensors instead of IMU (IMU is broken)
    imu = config.getIMU();
    if (!imu) {
        // Auto-detect individual sensors
        accel = detectAccel();
        gyro = detectGyro();
        mag = detectMag();
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
    } else {
        LOGE("Cannot operate without barometer!");
        return false;
    }

    if (gps) {
        sensorArray[numSensors++] = gps;
    }

    // Use individual sensors instead of IMU
    if (imu) {
        sensorArray[numSensors++] = imu;
    } else {
        if (accel) {
            sensorArray[numSensors++] = accel;
        }
        if (gyro) {
            sensorArray[numSensors++] = gyro;
        }
        if (mag) {
            sensorArray[numSensors++] = mag;
        }
    }

    if (highGAccel) {
        sensorArray[numSensors++] = highGAccel;
    }

    LOGI("Sensor configuration complete: %d sensors ready", numSensors);
    return true;
}

void AstraRocket::setupLogging() {
    // Create log sinks
    dataSinks = new ILogSink*[MAX_LOG_SINKS];
    eventSinks = new ILogSink*[MAX_LOG_SINKS];

    // USB log for debugging
    USBLog *usbLog = new USBLog(Serial, 115200, true);
    dataSinks[numDataSinks++] = usbLog;
    eventSinks[numEventSinks++] = usbLog;

    // SD card log for data recording
    FileLogSink *sdEventLog = new FileLogSink("events.log", config.getStorageBackend(), false);
    FileLogSink *sdDataLog = new FileLogSink("data.csv", config.getStorageBackend(), false);


    eventSinks[numEventSinks++] = sdEventLog;
    dataSinks[numDataSinks++] = sdDataLog;

    // Configure EventLogger
    EventLogger::configure(eventSinks, numEventSinks);

    // Now that EventLogger is configured, log the summary
    LOGI("Logging system initialized with %d sink(s)", numDataSinks);

    // Write test messages to verify SD card is working
    LOGI("SD card test: If you can read this in events.log, SD logging is working!");
    LOGI("Timestamp test: %lu ms", millis());
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
    FlightStage stage = rocketState->getFlightStage();

    // For now, just keep LED on during flight
    if (stage != PAD_IDLE && stage != LANDED) {
        digitalWrite(config.getLEDStatusPin(), HIGH);
    } else {
        // Blink slowly on pad/landed
        digitalWrite(config.getLEDStatusPin(), (millis() / 1000) % 2);
    }

    // === Sensor Status LED ===
    // Shows good status if all sensors (except GPS) are initialized
    if (config.getSensorStatusLEDPin() >= 0) {
        bool allSensorsGood = true;

        // Check all sensors except GPS
        if (barometer && !barometer->isInitialized()) allSensorsGood = false;
        if (imu && !imu->isInitialized()) allSensorsGood = false;
        if (accel && !accel->isInitialized()) allSensorsGood = false;
        if (gyro && !gyro->isInitialized()) allSensorsGood = false;
        if (mag && !mag->isInitialized()) allSensorsGood = false;
        if (highGAccel && !highGAccel->isInitialized()) allSensorsGood = false;

        if (allSensorsGood) {
            // All sensors good - solid green (on)
            bb.on(config.getSensorStatusLEDPin());
        } else {
            // Some sensor failed - blink pattern (2 quick blinks)
            static unsigned long lastSensorUpdate = 0;
            if (millis() - lastSensorUpdate > 2000) {
                bb.aonoff(config.getSensorStatusLEDPin(), 100, 2, 100);
                lastSensorUpdate = millis();
            }
        }
    }

    // === GPS Status LED ===
    // Shows good if GPS init, extra good if GPS has fix
    if (config.getGPSStatusLEDPin() >= 0) {
        if (gps) {
            if (gps->isInitialized()) {
                if (gps->getHasFix()) {
                    // GPS has fix - solid on (extra good)
                    bb.on(config.getGPSStatusLEDPin());
                } else {
                    // GPS initialized but no fix - slow blink pattern
                    static unsigned long lastGPSUpdate = 0;
                    if (millis() - lastGPSUpdate > 1000) {
                        bb.aonoff(config.getGPSStatusLEDPin(), 200);
                        lastGPSUpdate = millis();
                    }
                }
            } else {
                // GPS not initialized - off
                bb.off(config.getGPSStatusLEDPin());
            }
        } else {
            // No GPS present - off
            bb.off(config.getGPSStatusLEDPin());
        }
    }
}

// ===== Sensor Auto-Detection =====

Barometer* AstraRocket::detectBarometer() {
    // Try DPS368 first (I2C 0x77)
    DPS368 *dps = new DPS368();
    if (dps->begin()) {
        LOGI("Detected DPS368 barometer");
        return dps;
    }
    delete dps;
    
    // Try MS5611
    astra::MS5611 *ms5 = new astra::MS5611(0x77);
    if (ms5->begin()) {
        LOGI("Detected MS5611 barometer");
        return ms5;
    }
    delete dps;

    // Try BMP390 (I2C 0x76 or 0x77)
    BMP390 *bmp = new BMP390();
    if (bmp->begin()) {
        LOGI("Detected BMP390 barometer");
        return bmp;
    }
    delete bmp;

    LOGE("No barometer detected - required for operation");
    return nullptr;
}

GPS* AstraRocket::detectGPS() {
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

    return nullptr;
}

IMU* AstraRocket::detectIMU() {
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

    return nullptr;
}

Accel* AstraRocket::detectAccel() {
    // Try BMI088 accelerometer
    BMI088Accel *bmi088accel = new BMI088Accel();
    if (bmi088accel->begin()) {
        LOGI("Detected BMI088 accelerometer");
        return bmi088accel;
    }
    delete bmi088accel;

    return nullptr;
}

Gyro* AstraRocket::detectGyro() {
    // Try BMI088 gyroscope
    BMI088Gyro *bmi088gyro = new BMI088Gyro();
    if (bmi088gyro->begin()) {
        LOGI("Detected BMI088 gyroscope");
        return bmi088gyro;
    }
    delete bmi088gyro;

    return nullptr;
}

Mag* AstraRocket::detectMag() {
    // Try LIS3MDL magnetometer
    astra::LIS3MDL *lis3mdl = new astra::LIS3MDL();
    if (lis3mdl->begin()) {
        LOGI("Detected LIS3MDL magnetometer");
        return lis3mdl;
    }
    delete lis3mdl;

    return nullptr;
}

Accel* AstraRocket::detectHighGAccel() {
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

    return nullptr;
}

} // namespace astra_rocket
