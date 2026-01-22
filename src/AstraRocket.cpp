#include "AstraRocket.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/DataLogger.h>
#include <BlinkBuzz/BlinkBuzz.h>
#include "RadioLog.h"

// Include all possible sensor implementations for auto-detection
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/Baro/BMP390.h>
#include <Sensors/HW/Baro/MS5611.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/IMU/BNO055.h>
#include <Sensors/HW/Accel/ADXL375.h>
#include <Sensors/HW/Accel/H3LIS331DL.h>
// Note: BMI088Accel, BNO055Accel, BMI088Gyro, BNO055Gyro, BNO055Mag
// are component classes within the IMU - accessed via getAccelSensor(), getGyroSensor(), getMagSensor()

#ifndef ASTRA_ROCKET_VERSION
#define ASTRA_ROCKET_VERSION "UNKNOWN"
#endif

namespace astra_rocket {

// Static default configuration instance
AstraRocketConfig AstraRocket::defaultConfig;

AstraRocket::AstraRocket()
    : config(defaultConfig),
      astraSys(nullptr),
      rocketState(nullptr),
      kalmanFilter(nullptr),
      orientationFilter(nullptr),
      rocketSensorManager(nullptr),
      barometer(nullptr),
      gps(nullptr),
      imu6(nullptr),
      imu9(nullptr),
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
      groundLevelAltitude(0),
      hitlGroundLevelSet(false),
      i2c_device_count(0),
      i2c_scanned(false),
      i2c_claimed_count(0)
{
    // Default configuration already set in AstraRocketConfig constructor
}

AstraRocket::AstraRocket(AstraRocketConfig &cfg)
    : config(cfg),
      astraSys(nullptr),
      rocketState(nullptr),
      kalmanFilter(nullptr),
      orientationFilter(nullptr),
      rocketSensorManager(nullptr),
      barometer(nullptr),
      gps(nullptr),
      imu6(nullptr),
      imu9(nullptr),
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
      groundLevelAltitude(0),
      hitlGroundLevelSet(false),
      i2c_device_count(0),
      i2c_scanned(false),
      i2c_claimed_count(0)
{
}

AstraRocket::~AstraRocket() {
    if (astraSys) delete astraSys;
    if (rocketState) delete rocketState;
    if (kalmanFilter) delete kalmanFilter;
    if (orientationFilter) delete orientationFilter;
    if (rocketSensorManager) delete rocketSensorManager;

    // Delete composite IMUs (these own the sensor data)
    // Note: accel, gyro, mag pointers reference IMU components - do NOT delete them
    if (imu6) delete imu6;
    if (imu9) delete imu9;

    // Delete standalone sensors
    if (barometer) delete barometer;
    if (gps) delete gps;
    if (highGAccel) delete highGAccel;

    if (sensorArray) delete[] sensorArray;
    if (dataSinks) delete[] dataSinks;
    if (eventSinks) delete[] eventSinks;
}

bool AstraRocket::init() {
    LOGI("Initializing Astra-Rocket version %s", ASTRA_ROCKET_VERSION);

    // Setup logging first so we can log initialization progress
    setupLogging();

    // Auto-detect or use configured sensors
    if (!autoDetectSensors()) {
        LOGE("Sensor detection/initialization failed!");
        return false;
    }

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

    // Create and configure RocketSensorManager
    rocketSensorManager = new RocketSensorManager();
    if (accel) {
        rocketSensorManager->withLowGAccel(accel, MountingTransform(MountingOrientation::IDENTITY));
        LOGI("RocketSensorManager: Low-G accelerometer registered");
    }
    if (highGAccel) {
        rocketSensorManager->withHighGAccel(highGAccel, MountingTransform(MountingOrientation::IDENTITY));
        LOGI("RocketSensorManager: High-G accelerometer registered");
    }
    if (gyro) {
        rocketSensorManager->withGyro(gyro, MountingTransform(MountingOrientation::IDENTITY));
        LOGI("RocketSensorManager: Gyroscope registered");
    }
    if (mag) {
        rocketSensorManager->withMag(mag, MountingTransform(MountingOrientation::IDENTITY));
        LOGI("RocketSensorManager: Magnetometer registered");
    }
    if (barometer) {
        rocketSensorManager->withBaro(barometer);
        LOGI("RocketSensorManager: Barometer registered");
    }

    // Initialize the sensor manager
    if (!rocketSensorManager->begin()) {
        LOGE("RocketSensorManager initialization failed!");
        return false;
    }
    LOGI("RocketSensorManager initialized successfully");

    // Auto-detect sensor mounting orientations
    // Assumes rocket is vertical (nose up) during initialization
    LOGI("Auto-detecting sensor mounting orientations...");
    if (rocketSensorManager->autoDetectMountingOrientations()) {
        LOGI("Sensor mounting orientations detected automatically");
    } else {
        LOGW("Auto-detection failed or no sensors available - using default orientations");
    }

    // Configure base Astra system
    config.getAstraConfig()->withState(rocketState);
    config.getAstraConfig()->withSensorManager(rocketSensorManager);  // Use RocketSensorManager for sensor fusion
    config.getAstraConfig()->withSensors(sensorArray, numSensors);  // Still pass sensors for logging/access
    config.getAstraConfig()->withUpdateRate(50.0);  // 50 Hz default (can be overridden by user config)
    config.getAstraConfig()->withLoggingRate(config.getPreflightLogRate());
    config.getAstraConfig()->withDataLogs(dataSinks, numDataSinks);

    // Configure BlinkBuzz for status indicators
    if (config.getBuzzerFeedback()) {
        config.getAstraConfig()->withBBAsync(true);
        config.getAstraConfig()->withBuzzerPin(config.getLEDStatusPin());

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
    if (config.getHITLEnabled()) {
        // In HITL mode, skip initial ground level setup
        // It will be established on first valid HITL packet
        LOGI("HITL mode: Ground level will be set from first simulation packet");
    } else {
        // Hardware mode: Wait for barometer to stabilize
        delay(500);
        astraSys->update();
        if (barometer && barometer->isInitialized()) {
            groundLevelAltitude = barometer->getASLAltM();
            rocketState->setGroundLevel(groundLevelAltitude);
            LOGI("Ground level established: %0.2f m MSL", groundLevelAltitude);
        } else {
            LOGW("Barometer not available - AGL calculations disabled");
        }
    }

    // Initialization complete
    LOGI("AstraRocket initialization complete!");
    LOGI("Flight computer ready. Flight stage: %s", flightStageToString(PAD_IDLE));

    return true;
}

void AstraRocket::update() {
    // Check if HITL mode is enabled
    if (config.getHITLEnabled()) {
        // HITL mode: wait for incoming sensor data from simulation
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');

            if (line.startsWith("HITL/")) {
                // Parse incoming HITL packet
                double simTime;
                if (HITLParser::parseAndInject(line.c_str(), simTime)) {
                    // Update with simulation time (NOT millis()!)
                    // Convert simTime from seconds to milliseconds for Astra
                    double simTimeMs = simTime * 1000.0;
                    // This will read sensors from the HITL buffer
                    astraSys->update(simTimeMs);

                    // Set ground level from first valid packet (after update)
                    if (!hitlGroundLevelSet && barometer && barometer->isInitialized()) {
                        // Debug: Check what pressure value we're reading
                        HITLSensorBuffer& buffer = HITLSensorBuffer::instance();
                        LOGI("HITL Debug: Buffer pressure = %0.2f hPa", buffer.data.pressure);
                        LOGI("HITL Debug: Barometer pressure = %0.2f hPa", barometer->getPressure());

                        groundLevelAltitude = barometer->getASLAltM();
                        rocketState->setGroundLevel(groundLevelAltitude);
                        hitlGroundLevelSet = true;
                        LOGI("HITL: Ground level established at %0.2f m MSL from first packet", groundLevelAltitude);
                    }

                    // DataLogger automatically outputs "TELEM/" data to Serial
                } else {
                    LOGE("HITL: Failed to parse packet");
                }
            }
        }
    } else {
        // Normal hardware mode: update with real time
        astraSys->update();
    }

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

    // Check if HITL mode is enabled
    if (config.getHITLEnabled()) {
        LOGI("HITL mode enabled - creating simulated sensors");

        // Create HITL sensors instead of hardware sensors
        barometer = new HITLBarometer();
        gps = new HITLGPS();
        accel = new HITLAccel();
        gyro = new HITLGyro();
        mag = new HITLMag();

        // No high-G accelerometer in HITL mode (use normal accel)
        highGAccel = nullptr;

        LOGI("HITL sensors created successfully");
    } else {
        // Normal hardware mode - get sensors from config or auto-detect
        barometer = config.getBarometer();
        if (!barometer) {
            barometer = detectBarometer();
        }

        gps = config.getGPS();
        if (!gps) {
            gps = detectGPS();
        }

        // Try to use individual sensors instead of IMU (IMU is broken)
        // imu = config.getIMU();
        // if (!imu) {
            // Check for configured individual sensors or auto-detect
            accel = config.getAccel();
            if (!accel) {
                accel = detectAccel();
            }

            gyro = config.getGyro();
            if (!gyro) {
                gyro = detectGyro();
            }

            mag = config.getMag();
            if (!mag) {
                mag = detectMag();
            }
        // }

        highGAccel = config.getHighGAccel();
        if (!highGAccel) {
            highGAccel = detectHighGAccel();
        }
    }

    // Build sensor array
    sensorArray = new Sensor*[ASTRA_ROCKET_MAX_SENSORS];
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
    // if (imu) {
    //     sensorArray[numSensors++] = imu;
    // } else {
        if (accel) {
            sensorArray[numSensors++] = accel;
        }
        if (gyro) {
            sensorArray[numSensors++] = gyro;
        }
        if (mag) {
            sensorArray[numSensors++] = mag;
        }
    // }

    if (highGAccel) {
        sensorArray[numSensors++] = highGAccel;
    }

    LOGI("Sensor configuration complete: %d sensors ready", numSensors);
    return true;
}

void AstraRocket::setupLogging() {
    // Create log sinks
    dataSinks = new ILogSink*[ASTRA_ROCKET_MAX_LOG_SINKS];
    eventSinks = new ILogSink*[ASTRA_ROCKET_MAX_LOG_SINKS];

    // USB log for debugging - use PrintLog which works with any Print object
    Serial.begin(115200);
    PrintLog *usbLog = new PrintLog(Serial, true);
    dataSinks[numDataSinks++] = usbLog;
    eventSinks[numEventSinks++] = usbLog;

    RadioLog *rad = new RadioLog(*config.getRadioSerial());
    dataSinks[numDataSinks++] = rad;

    // SD card log for data recording
    FileLogSink *sdEventLog = new FileLogSink("events.log", config.getStorageBackend(), false);
    FileLogSink *sdDataLog = new FileLogSink("data.csv", config.getStorageBackend(), false);


    eventSinks[numEventSinks++] = sdEventLog;
    dataSinks[numDataSinks++] = sdDataLog;

    // Configure EventLogger
    EventLogger::configure(eventSinks, numEventSinks);
    
    // Now that EventLogger is configured, log the summary
    for(int i = 0; i < numEventSinks; i++){
        if(eventSinks[i]->ok()){
            LOGI("Data sink %d ok.", i);
        }
        else{
            LOGW("Data sink %d FAILED", i);
        }
    }
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

            // During boost, high acceleration interferes with accelerometer-based orientation
            // Switch to gyro-only mode to prevent orientation drift
            if (orientationFilter) {
                rocketState->setOrientationFilterMode(MahonyMode::GYRO_ONLY);
                LOGI("Orientation filter: switched to GYRO_ONLY mode for boost phase");
            }
            break;

        case COAST:
            // After motor burnout, we're in lower G conditions
            // Re-enable accelerometer correction for better orientation accuracy
            if (orientationFilter) {
                rocketState->setOrientationFilterMode(MahonyMode::CORRECTING);
                LOGI("Orientation filter: switched to CORRECTING mode for coast phase");
            }
            break;

        case APOGEE:
            // Near apogee, acceleration is very low (near 0G)
            // Keep gyro-only to avoid noise from low-gravity conditions
            if (orientationFilter) {
                rocketState->setOrientationFilterMode(MahonyMode::GYRO_ONLY);
                LOGI("Orientation filter: switched to GYRO_ONLY mode for apogee phase");
            }
            break;

        case EXPECTING_DROGUE:
        case UNDER_DROGUE:
        case EXPECTING_MAIN:
        case UNDER_MAIN:
            // During descent under parachute, acceleration should be relatively stable
            // Re-enable accelerometer correction
            if (orientationFilter) {
                rocketState->setOrientationFilterMode(MahonyMode::CORRECTING);
                LOGI("Orientation filter: switched to CORRECTING mode for descent phase");
            }
            break;

        case LANDED:
            // Switch to low-rate logging
            config.getAstraConfig()->withLoggingRate(config.getPostflightLogRate());
            LOGI("Switched to postflight logging rate: %0.1f Hz", config.getPostflightLogRate());

            // After landing, re-enable accelerometer correction for final orientation
            if (orientationFilter) {
                rocketState->setOrientationFilterMode(MahonyMode::CORRECTING);
                LOGI("Orientation filter: switched to CORRECTING mode for landed state");
            }
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
        // if (imu && !imu->isInitialized()) allSensorsGood = false;
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

// Helper function to check if an address has been claimed
bool AstraRocket::isAddressClaimed(uint8_t addr) {
    for (uint8_t i = 0; i < i2c_claimed_count; i++) {
        if (i2c_claimed_addresses[i] == addr) {
            return true;
        }
    }
    return false;
}

// Helper function to mark an address as claimed
void AstraRocket::claimAddress(uint8_t addr) {
    if (i2c_claimed_count < 20 && !isAddressClaimed(addr)) {
        i2c_claimed_addresses[i2c_claimed_count++] = addr;
    }
}

// Helper function to scan I2C bus and return list of active addresses
void AstraRocket::scanI2CBus(uint8_t* addresses, uint8_t& count, uint8_t maxCount) {
    // Use cached scan results if available
    if (i2c_scanned) {
        count = i2c_device_count;
        for (uint8_t i = 0; i < count && i < maxCount; i++) {
            addresses[i] = i2c_addresses[i];
        }
        return;
    }

    // Perform scan
    count = 0;
    LOGI("Scanning I2C bus...");

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            LOGI("  Found device at 0x%02X", addr);
            if (count < maxCount) {
                addresses[count++] = addr;
            }
        }
    }

    LOGI("I2C scan complete: %d devices found", count);

    // Cache the results
    i2c_device_count = count;
    for (uint8_t i = 0; i < count && i < 20; i++) {
        i2c_addresses[i] = addresses[i];
    }
    i2c_scanned = true;

    // Give sensors time to recover from the scan before initialization
    // BMI088 sensors are particularly sensitive to bus activity
    // Longer delay helps with register write/readback verification issues
    delay(200);

    // Set I2C clock speed to a conservative 100 kHz for more reliable communication
    // The BMI088 library will try to change this to 400 kHz, but starting slower may help
    Wire.setClock(100000);
}

Barometer* AstraRocket::detectBarometer() {
    LOGI("Auto-detecting barometer...");

    // Skip I2C scan - try direct initialization at expected addresses
    const uint8_t baro_addresses[] = {0x77, 0x76};

    for (uint8_t addr : baro_addresses) {
        // Skip if already claimed
        if (isAddressClaimed(addr)) {
            continue;
        }

        LOGI("Trying barometer sensors at address 0x%02X...", addr);

        // Try DPS368
        DPS368 *dps = new DPS368(addr);
        if (dps->begin()) {
            LOGI("Detected DPS368 barometer at 0x%02X", addr);
            claimAddress(addr);
            return dps;
        }
        delete dps;

        // Try MS5611
        astra::MS5611 *ms5 = new astra::MS5611(addr);
        if (ms5->begin()) {
            LOGI("Detected MS5611 barometer at 0x%02X", addr);
            claimAddress(addr);
            return ms5;
        }
        delete ms5;

        // Try BMP390
        BMP390 *bmp = new BMP390(addr);
        if (bmp->begin()) {
            LOGI("Detected BMP390 barometer at 0x%02X", addr);
            claimAddress(addr);
            return bmp;
        }
        delete bmp;
    }

    LOGE("No barometer detected - required for operation");
    return nullptr;
}

GPS* AstraRocket::detectGPS() {
    // Try SAM_M10Q
    SAM_M10Q *sam = new SAM_M10Q("SAM_M10Q");
    if (sam->begin()) {
        LOGI("Detected SAM_M10Q GPS");
        return sam;
    }
    delete sam;

    return nullptr;
}

// IMU* AstraRocket::detectIMU() {
//     // Try BNO055
//     BNO055 *bno = new BNO055();
//     if (bno->begin()) {
//         LOGI("Detected BNO055 IMU");
//         return bno;
//     }
//     delete bno;

//     return nullptr;
// }

Accel* AstraRocket::detectAccel() {
    LOGI("Auto-detecting accelerometer (via composite IMU)...");

    // Try BMI088 (6DoF IMU) at typical addresses
    const uint8_t bmi088_accel_addresses[] = {0x18, 0x19};
    const uint8_t bmi088_gyro_addresses[] = {0x68, 0x69};

    for (uint8_t accel_addr : bmi088_accel_addresses) {
        // Skip if already claimed
        if (isAddressClaimed(accel_addr)) {
            continue;
        }

        for (uint8_t gyro_addr : bmi088_gyro_addresses) {
            if (isAddressClaimed(gyro_addr)) {
                continue;
            }

            LOGI("Trying BMI088 at accel=0x%02X gyro=0x%02X...", accel_addr, gyro_addr);

            BMI088 *bmi = new BMI088(Wire, accel_addr, gyro_addr);
            if (bmi->begin()) {
                LOGI("Detected BMI088 IMU at accel=0x%02X gyro=0x%02X", accel_addr, gyro_addr);
                claimAddress(accel_addr);
                claimAddress(gyro_addr);
                imu6 = bmi;  // Store composite IMU
                return bmi->getAccelSensor();  // Return component
            }
            delete bmi;
        }
    }

    // Try BNO055 (9DoF IMU) at typical addresses
    const uint8_t bno055_addresses[] = {0x28, 0x29};

    for (uint8_t addr : bno055_addresses) {
        // Skip if already claimed
        if (isAddressClaimed(addr)) {
            continue;
        }

        LOGI("Trying BNO055 at address 0x%02X...", addr);

        BNO055 *bno = new BNO055(addr);
        if (bno->begin()) {
            LOGI("Detected BNO055 IMU at 0x%02X", addr);
            claimAddress(addr);
            imu9 = bno;  // Store composite IMU
            return bno->getAccelSensor();  // Return component
        }
        delete bno;
    }

    LOGW("No accelerometer/IMU detected");
    return nullptr;
}

Gyro* AstraRocket::detectGyro() {
    LOGI("Auto-detecting gyroscope...");

    // Check if IMU was already created by detectAccel()
    if (imu6) {
        LOGI("Using gyroscope from existing BMI088 IMU");
        return imu6->getGyroSensor();
    }
    if (imu9) {
        LOGI("Using gyroscope from existing BNO055 IMU");
        return imu9->getGyroSensor();
    }

    // No existing IMU - try to create one
    // Give extra time for I2C bus to settle
    delay(50);

    // Try BMI088 (6DoF IMU) at typical addresses
    const uint8_t bmi088_accel_addresses[] = {0x18, 0x19};
    const uint8_t bmi088_gyro_addresses[] = {0x68, 0x69};

    for (uint8_t accel_addr : bmi088_accel_addresses) {
        if (isAddressClaimed(accel_addr)) {
            continue;
        }

        for (uint8_t gyro_addr : bmi088_gyro_addresses) {
            if (isAddressClaimed(gyro_addr)) {
                continue;
            }

            LOGI("Trying BMI088 at accel=0x%02X gyro=0x%02X...", accel_addr, gyro_addr);

            BMI088 *bmi = new BMI088(Wire, accel_addr, gyro_addr);
            if (bmi->begin()) {
                LOGI("Detected BMI088 IMU at accel=0x%02X gyro=0x%02X", accel_addr, gyro_addr);
                claimAddress(accel_addr);
                claimAddress(gyro_addr);
                imu6 = bmi;
                return bmi->getGyroSensor();
            }
            delete bmi;
        }
    }

    // Try BNO055 (9DoF IMU) at typical addresses
    const uint8_t bno055_addresses[] = {0x28, 0x29};

    for (uint8_t addr : bno055_addresses) {
        if (isAddressClaimed(addr)) {
            continue;
        }

        LOGI("Trying BNO055 at address 0x%02X...", addr);

        BNO055 *bno = new BNO055(addr);
        if (bno->begin()) {
            LOGI("Detected BNO055 IMU at 0x%02X", addr);
            claimAddress(addr);
            imu9 = bno;
            return bno->getGyroSensor();
        }
        delete bno;
    }

    LOGW("No gyroscope/IMU detected");
    return nullptr;
}

Mag* AstraRocket::detectMag() {
    LOGI("Auto-detecting magnetometer...");

    // Check if BNO055 (9DoF IMU) was already created by detectAccel() or detectGyro()
    if (imu9) {
        LOGI("Using magnetometer from existing BNO055 IMU");
        return imu9->getMagSensor();
    }

    // No existing 9DoF IMU - try to create BNO055
    // Note: BMI088 is only 6DoF (no magnetometer)
    const uint8_t bno055_addresses[] = {0x28, 0x29};

    for (uint8_t addr : bno055_addresses) {
        // Skip if already claimed
        if (isAddressClaimed(addr)) {
            continue;
        }

        LOGI("Trying BNO055 at address 0x%02X...", addr);

        BNO055 *bno = new BNO055(addr);
        if (bno->begin()) {
            LOGI("Detected BNO055 IMU at 0x%02X", addr);
            claimAddress(addr);
            imu9 = bno;
            return bno->getMagSensor();
        }
        delete bno;
    }

    LOGW("No magnetometer/9DoF IMU detected");
    return nullptr;
}

Accel* AstraRocket::detectHighGAccel() {
    LOGI("Auto-detecting high-G accelerometer...");

    // Skip I2C scan - try direct initialization at expected addresses
    const uint8_t high_g_addresses[] = {0x1D, 0x53, 0x18, 0x19};

    for (uint8_t addr : high_g_addresses) {
        // Skip if already claimed
        if (isAddressClaimed(addr)) {
            continue;
        }

        LOGI("Trying high-G accelerometers at address 0x%02X...", addr);

        // Try ADXL375
        ADXL375 *adxl = new ADXL375("ADXL375", Wire, addr);
        if (adxl->begin()) {
            LOGI("Detected ADXL375 high-G accelerometer at 0x%02X", addr);
            claimAddress(addr);
            return adxl;
        }
        delete adxl;

        // Try H3LIS331DL
        astra::H3LIS331DL *h3lis = new astra::H3LIS331DL("H3LIS331DL", Wire, addr);
        if (h3lis->begin()) {
            LOGI("Detected H3LIS331DL high-G accelerometer at 0x%02X", addr);
            claimAddress(addr);
            return h3lis;
        }
        delete h3lis;
    }

    LOGW("No high-G accelerometer detected");
    return nullptr;
}

} // namespace astra_rocket
