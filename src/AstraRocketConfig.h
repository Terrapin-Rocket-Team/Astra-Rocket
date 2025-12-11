#ifndef ASTRA_ROCKET_CONFIG_H
#define ASTRA_ROCKET_CONFIG_H

#include <Utils/AstraConfig.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/IMU/IMU.h>
#include <Sensors/Accel/Accel.h>
#include <RecordData/Storage/IStorage.h>

using namespace astra;

namespace astra_rocket {

/**
 * AstraRocketConfig: Configuration builder for rocket flight computers
 *
 * Extends AstraConfig with rocket-specific configuration options:
 * - Sensor selection (with auto-detection fallback)
 * - Flight detection thresholds
 * - Pyro channel configuration
 * - Deployment logic parameters
 * - Logging rates for different flight phases
 */
class AstraRocketConfig {
public:
    AstraRocketConfig();

    // ===== Sensor Configuration =====

    /**
     * Set barometer sensor (nullptr = auto-detect)
     */
    AstraRocketConfig& withBarometer(Barometer *baro);

    /**
     * Set GPS sensor (nullptr = auto-detect or disable)
     */
    AstraRocketConfig& withGPS(GPS *gps);

    /**
     * Set IMU sensor (nullptr = auto-detect)
     */
    AstraRocketConfig& withIMU(IMU *imu);

    /**
     * Set high-G accelerometer (nullptr = auto-detect)
     */
    AstraRocketConfig& withHighGAccel(Accel *accel);

    // ===== Flight Detection Thresholds =====

    /**
     * Set liftoff acceleration threshold (G's)
     * Default: 3.0 G
     */
    AstraRocketConfig& withLiftoffAccelThreshold(double accelG);

    /**
     * Set liftoff detection duration (milliseconds)
     * Acceleration must exceed threshold for this duration
     * Default: 100 ms
     */
    AstraRocketConfig& withLiftoffDetectDuration(unsigned long durationMs);

    /**
     * Set motor burnout acceleration threshold (G's)
     * Default: 1.5 G
     */
    AstraRocketConfig& withBurnoutAccelThreshold(double accelG);

    /**
     * Set apogee velocity threshold (m/s)
     * Vertical velocity must be below this to detect apogee
     * Default: 2.0 m/s
     */
    AstraRocketConfig& withApogeeVelocityThreshold(double velocityMs);

    /**
     * Set landing velocity threshold (m/s)
     * Default: 1.0 m/s
     */
    AstraRocketConfig& withLandingVelocityThreshold(double velocityMs);

    /**
     * Set landing detection duration (milliseconds)
     * Low velocity must be sustained for this duration
     * Default: 3000 ms
     */
    AstraRocketConfig& withLandingDetectDuration(unsigned long durationMs);

    // ===== Storage Configuration =====

    /**
     * Set storage backend type
     * Platform defaults:
     * - STM32: EMMC
     * - Teensy: SD_CARD (SDIO via SdFat)
     * - ESP32: SD_CARD (SDMMC 4-bit)
     */
    AstraRocketConfig& withStorageBackend(StorageBackend backend);

    // ===== Logging Configuration =====

    /**
     * Set SD card chip select pin
     * Default: BUILTIN_SDCARD (for Teensy)
     */
    AstraRocketConfig& withSDCardCS(int csPin);

    /**
     * Set preflight logging rate (Hz)
     * Default: 1.0 Hz
     */
    AstraRocketConfig& withPreflightLogRate(double rateHz);

    /**
     * Set in-flight logging rate (Hz)
     * Default: 50.0 Hz
     */
    AstraRocketConfig& withFlightLogRate(double rateHz);

    /**
     * Set postflight logging rate (Hz)
     * Default: 1.0 Hz
     */
    AstraRocketConfig& withPostflightLogRate(double rateHz);

    /**
     * Enable onboard flash memory as backup logging
     * Default: false
     */
    AstraRocketConfig& withFlashBackup(bool enable);

    // ===== Status Indicators Configuration =====

    /**
     * Enable buzzer feedback for flight events
     * Default: true
     */
    AstraRocketConfig& withBuzzerFeedback(bool enable);

    /**
     * Set status LED pin
     * Default: LED_BUILTIN
     */
    AstraRocketConfig& withLEDStatusPin(int pin);

    /**
     * Set sensor status LED pin
     * Shows status of all sensors except GPS
     * Default: -1 (disabled)
     */
    AstraRocketConfig& withSensorStatusLEDPin(int pin);

    /**
     * Set GPS status LED pin
     * Shows GPS initialization and fix status
     * Default: -1 (disabled)
     */
    AstraRocketConfig& withGPSStatusLEDPin(int pin);

    // ===== HITL (Hardware-In-The-Loop) Configuration =====

    /**
     * Enable HITL simulation mode
     * When enabled, uses simulated sensors instead of hardware
     * Default: false
     */
    AstraRocketConfig& withHITL(bool enable);

    // ===== Base Astra Configuration Pass-through =====

    /**
     * Set main update rate (Hz)
     * Default: 50.0 Hz
     */
    AstraRocketConfig& withUpdateRate(double rateHz);

    /**
     * Get the underlying AstraConfig object
     */
    AstraConfig* getAstraConfig() { return &astraConfig; }

    // Getters for all configuration parameters
    Barometer* getBarometer() const { return barometer; }
    GPS* getGPS() const { return gps; }
    IMU* getIMU() const { return imu; }
    Accel* getHighGAccel() const { return highGAccel; }

    double getLiftoffAccelThreshold() const { return liftoffAccelThreshold; }
    unsigned long getLiftoffDetectDuration() const { return liftoffDetectDuration; }
    double getBurnoutAccelThreshold() const { return burnoutAccelThreshold; }
    double getApogeeVelocityThreshold() const { return apogeeVelocityThreshold; }
    double getLandingVelocityThreshold() const { return landingVelocityThreshold; }
    unsigned long getLandingDetectDuration() const { return landingDetectDuration; }

    StorageBackend getStorageBackend() const { return storageBackend; }
    int getSDCardCS() const { return sdCardCS; }
    double getPreflightLogRate() const { return preflightLogRate; }
    double getFlightLogRate() const { return flightLogRate; }
    double getPostflightLogRate() const { return postflightLogRate; }
    bool getFlashBackup() const { return flashBackup; }

    bool getBuzzerFeedback() const { return buzzerFeedback; }
    int getLEDStatusPin() const { return ledStatusPin; }
    int getSensorStatusLEDPin() const { return sensorStatusLEDPin; }
    int getGPSStatusLEDPin() const { return gpsStatusLEDPin; }

    bool getHITLEnabled() const { return hitlEnabled; }

private:
    // Base Astra configuration
    AstraConfig astraConfig;

    // Sensor instances
    Barometer *barometer;
    GPS *gps;
    IMU *imu;
    Accel *highGAccel;

    // Flight detection thresholds
    double liftoffAccelThreshold;
    unsigned long liftoffDetectDuration;
    double burnoutAccelThreshold;
    double apogeeVelocityThreshold;
    double landingVelocityThreshold;
    unsigned long landingDetectDuration;

    // Storage configuration
    StorageBackend storageBackend;

    // Logging configuration
    int sdCardCS;
    double preflightLogRate;
    double flightLogRate;
    double postflightLogRate;
    bool flashBackup;

    // Status indicators configuration
    bool buzzerFeedback;
    int ledStatusPin;
    int sensorStatusLEDPin;
    int gpsStatusLEDPin;

    // HITL configuration
    bool hitlEnabled;
};

} // namespace astra_rocket

#endif // ASTRA_ROCKET_CONFIG_H
