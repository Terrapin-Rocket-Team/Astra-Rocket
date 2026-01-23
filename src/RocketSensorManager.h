#ifndef ROCKET_SENSOR_MANAGER_H
#define ROCKET_SENSOR_MANAGER_H

#include <Sensors/SensorManager/ISensorManager.h>
#include <Sensors/SensorManager/BodyFrameData.h>
#include <Sensors/SensorManager/MountingTransform.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/Sensor.h>

using namespace astra;

/**
 * RocketSensorManager - Rocket-specific sensor management with dual accelerometers
 *
 * Extends Astra's ISensorManager with intelligent high-G/low-G accelerometer switching:
 * - Uses low-G accelerometer (e.g., BMI088) for normal flight: 0-15g
 * - Automatically switches to high-G (e.g., ADXL375) above 15g threshold
 * - Includes hysteresis (12-15g) to prevent oscillation
 * - Fuses all sensor data into body frame
 * - Monitors health per sensor with automatic fallback
 *
 * Body frame convention (standard aerospace):
 *   X: Forward (through nosecone)
 *   Y: Right (starboard)
 *   Z: Down (towards fins)
 *
 * Usage:
 *   RocketSensorManager sensorMgr;
 *   sensorMgr.withLowGAccel(accel, MountingOrientation::IDENTITY);
 *   sensorMgr.withHighGAccel(highGAccel, MountingOrientation::IDENTITY);
 *   sensorMgr.withGyro(gyro);
 *   sensorMgr.withMag(mag);
 *   sensorMgr.withBaro(baro);
 *   sensorMgr.begin();
 *
 *   // In update loop:
 *   sensorMgr.update();
 *   BodyFrameData data = sensorMgr.getBodyFrameData();
 */
class RocketSensorManager : public ISensorManager
{
public:
    RocketSensorManager();
    virtual ~RocketSensorManager();

    // ========================= Configuration =========================

    /**
     * Register low-G accelerometer (e.g., BMI088, BNO055)
     * Used for 0-15g acceleration range
     */
    void withLowGAccel(Accel* accel, const MountingTransform& mount = MountingTransform());

    /**
     * Register high-G accelerometer (e.g., ADXL375, H3LIS331DL)
     * Used above 15g acceleration
     */
    void withHighGAccel(Accel* accel, const MountingTransform& mount = MountingTransform());

    /**
     * Register gyroscope
     */
    void withGyro(Gyro* gyro, const MountingTransform& mount = MountingTransform());

    /**
     * Register magnetometer
     */
    void withMag(Mag* mag, const MountingTransform& mount = MountingTransform());

    /**
     * Register barometer
     */
    void withBaro(Barometer* baro);

    /**
     * Set high-G threshold in g's (default: 15.0)
     * Switches from low-G to high-G when |accel| > threshold
     */
    void setAccelHighThreshold(double g) { accelThresholdHigh = g * 9.81; }

    /**
     * Set low-G threshold in g's (default: 12.0)
     * Switches from high-G back to low-G when |accel| < threshold
     * Must be less than high threshold to provide hysteresis
     */
    void setAccelLowThreshold(double g) { accelThresholdLow = g * 9.81; }

    /**
     * Set maximum consecutive failures before marking sensor as failed
     * Default: 10
     */
    void setMaxFailures(uint32_t maxFailures) { this->maxFailures = maxFailures; }

    /**
     * Set debounce time for mode switching in milliseconds
     * Prevents rapid oscillation between modes (default: 100ms)
     */
    void setModeSwitchDebounce(uint32_t debounceMs) { modeSwitchDebounceMs = debounceMs; }

    /**
     * Auto-detect mounting orientations by measuring gravity
     * Assumes rocket is vertical (nose up, body X pointing up) during startup
     * Reads accelerometers and determines which axis sees ~1g, then sets appropriate transforms
     *
     * Call this after sensors are initialized but before flying
     * For IMUs, the detected transform is applied to accel, gyro, and mag
     * For standalone accels, only the accel gets the transform
     *
     * @return true if orientation detected successfully for at least one accelerometer
     */
    bool autoDetectMountingOrientations();

    // ========================= ISensorManager Implementation =========================

    bool begin() override;
    bool update() override;

    const BodyFrameData& getBodyFrameData() const override { return bodyData; }
    Vector<3> getAccel() const override { return bodyData.accel; }
    Vector<3> getGyro() const override { return bodyData.gyro; }
    Vector<3> getMag() const override { return bodyData.mag; }
    double getPressure() const override { return bodyData.pressure; }
    double getBaroAltitude() const override { return bodyData.baroAltASL; }

    bool isReady() const override;
    SensorHealth getAccelHealth() const override;
    SensorHealth getGyroHealth() const override { return gyroHealth.status; }
    SensorHealth getMagHealth() const override { return magHealth.status; }
    SensorHealth getBaroHealth() const override { return baroHealth.status; }
    const SensorHealthInfo& getHealthInfo(const char* type) const override;

    void setAccelMount(const MountingTransform& mount) override { lowGAccelMount = mount; }
    void setGyroMount(const MountingTransform& mount) override { gyroMount = mount; }
    void setMagMount(const MountingTransform& mount) override { magMount = mount; }

    /**
     * Set mounting transform specifically for high-G accelerometer
     */
    void setHighGAccelMount(const MountingTransform& mount) { highGAccelMount = mount; }

    Accel* getActiveAccel() const override { return activeAccel; }
    Gyro* getActiveGyro() const override { return gyro; }
    Mag* getActiveMag() const override { return mag; }
    Barometer* getActiveBaro() const override { return baro; }

    // ========================= Rocket-Specific Accessors =========================

    /**
     * Get the current accelerometer mode
     */
    enum AccelMode
    {
        MODE_NONE,      // No accelerometer available
        MODE_LOW_G,     // Using low-G accelerometer
        MODE_HIGH_G,    // Using high-G accelerometer
        MODE_FUSED      // Crossover zone - could fuse both (future)
    };

    AccelMode getAccelMode() const { return accelMode; }

    /**
     * Get the last measured acceleration magnitude (m/s²)
     */
    double getAccelMagnitude() const { return lastAccelMagnitude; }

    /**
     * Set simulation time for HITL mode
     */
    void setSimTime(uint32_t timeMs) { simTimeMs = timeMs; useSimTime = true; }

private:
    // ========================= Sensor Instances =========================

    // Dual accelerometers
    Accel* lowGAccel = nullptr;
    Accel* highGAccel = nullptr;
    Accel* activeAccel = nullptr;

    // Single instances of other sensors
    Gyro* gyro = nullptr;
    Mag* mag = nullptr;
    Barometer* baro = nullptr;

    // ========================= Mounting Transforms =========================

    MountingTransform lowGAccelMount;
    MountingTransform highGAccelMount;
    MountingTransform gyroMount;
    MountingTransform magMount;

    // ========================= Health Tracking =========================

    SensorHealthInfo lowGAccelHealth;
    SensorHealthInfo highGAccelHealth;
    SensorHealthInfo gyroHealth;
    SensorHealthInfo magHealth;
    SensorHealthInfo baroHealth;

    // ========================= Accelerometer Selection State =========================

    AccelMode accelMode = MODE_NONE;
    double lastAccelMagnitude = 0.0;
    uint32_t lastModeSwitchTime = 0;

    // ========================= Configuration =========================

    double accelThresholdHigh = 15.0 * 9.81;  // m/s² (15g)
    double accelThresholdLow = 12.0 * 9.81;   // m/s² (12g) - hysteresis
    uint32_t maxFailures = 10;
    uint32_t modeSwitchDebounceMs = 100;

    // ========================= Output Data =========================

    BodyFrameData bodyData;
    bool initialized = false;
    bool useSimTime = false;
    uint32_t simTimeMs = 0;

    // ========================= Helper Methods =========================

    /**
     * Update accelerometer data and handle mode switching
     */
    bool updateAccel(uint32_t currentTimeMs);

    /**
     * Update gyroscope data
     */
    bool updateGyro(uint32_t currentTimeMs);

    /**
     * Update magnetometer data
     */
    bool updateMag(uint32_t currentTimeMs);

    /**
     * Update barometer data
     */
    bool updateBaro(uint32_t currentTimeMs);

    /**
     * Check sensor health and update health info
     * @return true if sensor is healthy and data is valid
     */
    bool checkSensorHealth(Sensor* sensor, SensorHealthInfo& health, uint32_t currentTimeMs);

    /**
     * Validate that vector data is finite and within reasonable range
     */
    bool isValidVector(const Vector<3>& vec, double maxMagnitude = 1000.0) const;

    /**
     * Validate that scalar data is finite
     */
    bool isValidScalar(double value) const;

    /**
     * Detect mounting orientation from accelerometer gravity reading
     * Assumes rocket is vertical with body Z pointing up
     * Returns IDENTITY if detection fails or reading is ambiguous
     *
     * @param accel Accelerometer to read
     * @return Detected mounting orientation
     */
    MountingTransform detectOrientationFromGravity(Accel* accel);

    // Dummy health info for unknown sensor types
    static SensorHealthInfo unknownHealth;
};

#endif // ROCKET_SENSOR_MANAGER_H
