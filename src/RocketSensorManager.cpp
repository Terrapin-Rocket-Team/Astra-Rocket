#include "RocketSensorManager.h"
#include <Arduino.h>
#include <cmath>

// Static member initialization
SensorHealthInfo RocketSensorManager::unknownHealth;

RocketSensorManager::RocketSensorManager()
    : lowGAccel(nullptr)
    , highGAccel(nullptr)
    , activeAccel(nullptr)
    , gyro(nullptr)
    , mag(nullptr)
    , baro(nullptr)
    , accelMode(MODE_NONE)
    , lastAccelMagnitude(0.0)
    , lastModeSwitchTime(0)
    , initialized(false)
    , useSimTime(false)
    , simTimeMs(0)
{
}

RocketSensorManager::~RocketSensorManager()
{
    // Sensors are owned by AstraRocket, not by us
    // Don't delete them here
}

// ========================= Configuration Methods =========================

void RocketSensorManager::withLowGAccel(Accel* accel, const MountingTransform& mount)
{
    lowGAccel = accel;
    lowGAccelMount = mount;
}

void RocketSensorManager::withHighGAccel(Accel* accel, const MountingTransform& mount)
{
    highGAccel = accel;
    highGAccelMount = mount;
}

void RocketSensorManager::withGyro(Gyro* gyro, const MountingTransform& mount)
{
    this->gyro = gyro;
    this->gyroMount = mount;
}

void RocketSensorManager::withMag(Mag* mag, const MountingTransform& mount)
{
    this->mag = mag;
    this->magMount = mount;
}

void RocketSensorManager::withBaro(Barometer* baro)
{
    this->baro = baro;
}

// ========================= ISensorManager Implementation =========================

bool RocketSensorManager::begin()
{
    // Validate we have at least one accelerometer
    if (!lowGAccel && !highGAccel)
    {
        Serial.println("[RocketSensorManager] ERROR: No accelerometers configured");
        return false;
    }

    // Initialize health tracking
    lowGAccelHealth = SensorHealthInfo();
    highGAccelHealth = SensorHealthInfo();
    gyroHealth = SensorHealthInfo();
    magHealth = SensorHealthInfo();
    baroHealth = SensorHealthInfo();

    // Set initial mode based on available sensors
    if (lowGAccel)
    {
        accelMode = MODE_LOW_G;
        activeAccel = lowGAccel;
        lowGAccelHealth.status = SensorHealth::HEALTHY;
        Serial.println("[RocketSensorManager] Starting with low-G accelerometer");
    }
    else if (highGAccel)
    {
        accelMode = MODE_HIGH_G;
        activeAccel = highGAccel;
        highGAccelHealth.status = SensorHealth::HEALTHY;
        Serial.println("[RocketSensorManager] Starting with high-G accelerometer (no low-G available)");
    }

    if (gyro)
    {
        gyroHealth.status = SensorHealth::HEALTHY;
    }
    if (mag)
    {
        magHealth.status = SensorHealth::HEALTHY;
    }
    if (baro)
    {
        baroHealth.status = SensorHealth::HEALTHY;
    }

    initialized = true;
    lastModeSwitchTime = millis();

    Serial.print("[RocketSensorManager] Initialized - High threshold: ");
    Serial.print(accelThresholdHigh / 9.81, 1);
    Serial.print("g, Low threshold: ");
    Serial.print(accelThresholdLow / 9.81, 1);
    Serial.println("g");

    return true;
}

bool RocketSensorManager::update()
{
    if (!initialized)
    {
        return false;
    }

    uint32_t currentTime = useSimTime ? simTimeMs : millis();
    bodyData.timestamp = currentTime / 1000.0;

    // Update all sensors
    bool hasValidData = false;

    if (updateAccel(currentTime))
    {
        hasValidData = true;
    }

    if (updateGyro(currentTime))
    {
        hasValidData = true;
    }

    if (updateMag(currentTime))
    {
        hasValidData = true;
    }

    if (updateBaro(currentTime))
    {
        hasValidData = true;
    }

    return hasValidData;
}

bool RocketSensorManager::isReady() const
{
    return initialized && (bodyData.hasAccel || bodyData.hasGyro);
}

SensorHealth RocketSensorManager::getAccelHealth() const
{
    // Return health of currently active accelerometer
    if (accelMode == MODE_LOW_G)
    {
        return lowGAccelHealth.status;
    }
    else if (accelMode == MODE_HIGH_G)
    {
        return highGAccelHealth.status;
    }
    return SensorHealth::UNKNOWN;
}

const SensorHealthInfo& RocketSensorManager::getHealthInfo(const char* type) const
{
    if (strcmp(type, "accel_low") == 0)
    {
        return lowGAccelHealth;
    }
    else if (strcmp(type, "accel_high") == 0)
    {
        return highGAccelHealth;
    }
    else if (strcmp(type, "accel") == 0)
    {
        // Return currently active accel health
        if (accelMode == MODE_LOW_G)
        {
            return lowGAccelHealth;
        }
        else if (accelMode == MODE_HIGH_G)
        {
            return highGAccelHealth;
        }
    }
    else if (strcmp(type, "gyro") == 0)
    {
        return gyroHealth;
    }
    else if (strcmp(type, "mag") == 0)
    {
        return magHealth;
    }
    else if (strcmp(type, "baro") == 0)
    {
        return baroHealth;
    }

    return unknownHealth;
}

// ========================= Sensor Update Methods =========================

bool RocketSensorManager::updateAccel(uint32_t currentTimeMs)
{
    // Read both accelerometers if available
    Vector<3> lowGBodyAccel, highGBodyAccel;
    bool hasLowG = false, hasHighG = false;

    // Read and transform low-G accelerometer
    if (lowGAccel && checkSensorHealth(lowGAccel, lowGAccelHealth, currentTimeMs))
    {
        Vector<3> rawLowG = lowGAccel->getAccel();
        lowGBodyAccel = lowGAccelMount.transform(rawLowG);
        hasLowG = true;
    }

    // Read and transform high-G accelerometer
    if (highGAccel && checkSensorHealth(highGAccel, highGAccelHealth, currentTimeMs))
    {
        Vector<3> rawHighG = highGAccel->getAccel();
        highGBodyAccel = highGAccelMount.transform(rawHighG);
        hasHighG = true;
    }

    // If we have no valid data, mark as unavailable
    if (!hasLowG && !hasHighG)
    {
        bodyData.hasAccel = false;
        return false;
    }

    // Determine which accelerometer to use based on acceleration magnitude and current mode
    // Use debouncing to prevent rapid switching
    bool canSwitch = (currentTimeMs - lastModeSwitchTime) >= modeSwitchDebounceMs;

    // Calculate magnitude from currently preferred sensor for threshold comparison
    double accelMag = 0.0;
    if (accelMode == MODE_LOW_G && hasLowG)
    {
        accelMag = lowGBodyAccel.magnitude();
    }
    else if (accelMode == MODE_HIGH_G && hasHighG)
    {
        accelMag = highGBodyAccel.magnitude();
    }
    else if (hasLowG)
    {
        accelMag = lowGBodyAccel.magnitude();
    }
    else if (hasHighG)
    {
        accelMag = highGBodyAccel.magnitude();
    }

    lastAccelMagnitude = accelMag;

    // State machine for mode switching with hysteresis
    AccelMode newMode = accelMode;

    if (canSwitch)
    {
        switch (accelMode)
        {
        case MODE_NONE:
            // Initialize to low-G if available, else high-G
            if (hasLowG)
            {
                newMode = MODE_LOW_G;
            }
            else if (hasHighG)
            {
                newMode = MODE_HIGH_G;
            }
            break;

        case MODE_LOW_G:
            // Switch to high-G if magnitude exceeds high threshold
            if (accelMag > accelThresholdHigh && hasHighG)
            {
                newMode = MODE_HIGH_G;
                Serial.print("[RocketSensorManager] Switching to HIGH-G (");
                Serial.print(accelMag / 9.81, 1);
                Serial.println("g)");
            }
            else if (!hasLowG && hasHighG)
            {
                // Fallback if low-G failed
                newMode = MODE_HIGH_G;
                Serial.println("[RocketSensorManager] Low-G failed, switching to HIGH-G");
            }
            break;

        case MODE_HIGH_G:
            // Switch back to low-G if magnitude drops below low threshold (hysteresis)
            if (accelMag < accelThresholdLow && hasLowG)
            {
                newMode = MODE_LOW_G;
                Serial.print("[RocketSensorManager] Switching to LOW-G (");
                Serial.print(accelMag / 9.81, 1);
                Serial.println("g)");
            }
            else if (!hasHighG && hasLowG)
            {
                // Fallback if high-G failed
                newMode = MODE_LOW_G;
                Serial.println("[RocketSensorManager] High-G failed, switching to LOW-G");
            }
            break;

        case MODE_FUSED:
            // Future: could implement fusion in crossover zone (12-15g)
            // For now, treat as low-G
            if (accelMag > accelThresholdHigh && hasHighG)
            {
                newMode = MODE_HIGH_G;
            }
            else if (accelMag < accelThresholdLow && hasLowG)
            {
                newMode = MODE_LOW_G;
            }
            break;
        }

        // Update mode if changed
        if (newMode != accelMode)
        {
            accelMode = newMode;
            lastModeSwitchTime = currentTimeMs;
        }
    }

    // Select output based on current mode
    switch (accelMode)
    {
    case MODE_LOW_G:
        if (hasLowG)
        {
            bodyData.accel = lowGBodyAccel;
            activeAccel = lowGAccel;
            bodyData.hasAccel = true;
        }
        else
        {
            bodyData.hasAccel = false;
        }
        break;

    case MODE_HIGH_G:
        if (hasHighG)
        {
            bodyData.accel = highGBodyAccel;
            activeAccel = highGAccel;
            bodyData.hasAccel = true;
        }
        else
        {
            bodyData.hasAccel = false;
        }
        break;

    case MODE_FUSED:
        // Future: weighted average or Kalman fusion
        // For now, simple average if both available
        if (hasLowG && hasHighG)
        {
            bodyData.accel = (lowGBodyAccel + highGBodyAccel) * 0.5;
            bodyData.hasAccel = true;
        }
        else if (hasLowG)
        {
            bodyData.accel = lowGBodyAccel;
            bodyData.hasAccel = true;
        }
        else if (hasHighG)
        {
            bodyData.accel = highGBodyAccel;
            bodyData.hasAccel = true;
        }
        else
        {
            bodyData.hasAccel = false;
        }
        break;

    case MODE_NONE:
    default:
        bodyData.hasAccel = false;
        break;
    }

    return bodyData.hasAccel;
}

bool RocketSensorManager::updateGyro(uint32_t currentTimeMs)
{
    if (!gyro)
    {
        bodyData.hasGyro = false;
        return false;
    }

    if (!checkSensorHealth(gyro, gyroHealth, currentTimeMs))
    {
        bodyData.hasGyro = false;
        return false;
    }

    Vector<3> rawGyro = gyro->getAngVel();

    bodyData.gyro = gyroMount.transform(rawGyro);
    bodyData.hasGyro = true;

    return true;
}

bool RocketSensorManager::updateMag(uint32_t currentTimeMs)
{
    if (!mag)
    {
        bodyData.hasMag = false;
        return false;
    }

    if (!checkSensorHealth(mag, magHealth, currentTimeMs))
    {
        bodyData.hasMag = false;
        return false;
    }

    Vector<3> rawMag = mag->getMag();

    bodyData.mag = magMount.transform(rawMag);
    bodyData.hasMag = true;

    return true;
}

bool RocketSensorManager::updateBaro(uint32_t currentTimeMs)
{
    if (!baro)
    {
        bodyData.hasBaro = false;
        return false;
    }

    if (!checkSensorHealth(baro, baroHealth, currentTimeMs))
    {
        bodyData.hasBaro = false;
        return false;
    }

    double pressure = baro->getPressure();
    double temperature = baro->getTemp();
    double altitude = baro->getASLAltM();

    bodyData.pressure = pressure;
    bodyData.temperature = temperature;
    bodyData.baroAltASL = altitude;
    bodyData.hasBaro = true;

    return true;
}

// ========================= Health Monitoring =========================

bool RocketSensorManager::checkSensorHealth(Sensor* sensor, SensorHealthInfo& health, uint32_t currentTimeMs)
{
    if (!sensor)
    {
        return false;
    }

    // Check if sensor is responding
    return sensor->isInitialized();
}

// ========================= Validation Helpers =========================

bool RocketSensorManager::isValidVector(const Vector<3>& vec, double maxMagnitude) const
{
    // Check for NaN or infinity
    if (!std::isfinite(vec.x()) || !std::isfinite(vec.y()) || !std::isfinite(vec.z()))
    {
        return false;
    }

    // Check magnitude is within reasonable range
    double mag = vec.magnitude();
    if (!std::isfinite(mag) || mag > maxMagnitude)
    {
        return false;
    }

    return true;
}

bool RocketSensorManager::isValidScalar(double value) const
{
    return std::isfinite(value);
}

// ========================= Auto-Detection Methods =========================

MountingTransform RocketSensorManager::detectOrientationFromGravity(Accel* accel)
{
    if (!accel || !accel->isInitialized())
    {
        Serial.println("[RocketSensorManager] Cannot detect orientation: accelerometer not initialized");
        return MountingTransform(MountingOrientation::IDENTITY);
    }

    // Read accelerometer (should be sitting still, vertical)
    Vector<3> reading = accel->getAccel();

    // Validate reading
    if (!isValidVector(reading, 15.0))
    {
        Serial.println("[RocketSensorManager] Cannot detect orientation: invalid accelerometer reading");
        return MountingTransform(MountingOrientation::IDENTITY);
    }

    // Find which axis has magnitude closest to 1g (9.81 m/s²)
    double ax = std::abs(reading.x());
    double ay = std::abs(reading.y());
    double az = std::abs(reading.z());

    const double GRAVITY = 9.81;
    const double TOLERANCE = 3.0;  // Allow ±3 m/s² tolerance

    // Check if we have a clear vertical axis
    double maxAxis = std::max({ax, ay, az});
    if (std::abs(maxAxis - GRAVITY) > TOLERANCE)
    {
        Serial.print("[RocketSensorManager] No clear vertical axis detected. Reading: [");
        Serial.print(reading.x());
        Serial.print(", ");
        Serial.print(reading.y());
        Serial.print(", ");
        Serial.print(reading.z());
        Serial.println("]");
        return MountingTransform(MountingOrientation::IDENTITY);
    }

    // Determine which axis is vertical and map to body Z (up when vertical)
    // Body frame: Z points up when rocket is vertical (nose up)
    MountingTransform transform;

    if (ax > ay && ax > az)
    {
        // X axis is vertical
        if (reading.x() > 0)
        {
            // Sensor +X points up → rotate to body +Z
            // Need: sensor X → body Z (positive)
            transform = MountingTransform(MountingOrientation::ROTATE_90_Y);
            Serial.println("[RocketSensorManager] Detected: Sensor +X up → ROTATE_90_Y");
        }
        else
        {
            // Sensor -X points up (X reads negative) → rotate -X to body +Z
            // Need: sensor -X → body Z (so sensor X → body -Z)
            transform = MountingTransform(MountingOrientation::ROTATE_NEG90_Y);
            Serial.println("[RocketSensorManager] Detected: Sensor -X up → ROTATE_NEG90_Y");
        }
    }
    else if (ay > ax && ay > az)
    {
        // Y axis is vertical
        if (reading.y() > 0)
        {
            // Sensor +Y points up → rotate to body +Z
            // Need: sensor Y → body Z (positive)
            transform = MountingTransform(MountingOrientation::ROTATE_NEG90_X);
            Serial.println("[RocketSensorManager] Detected: Sensor +Y up → ROTATE_NEG90_X");
        }
        else
        {
            // Sensor -Y points up → rotate -Y to body +Z
            // Need: sensor -Y → body Z (so sensor Y → body -Z)
            transform = MountingTransform(MountingOrientation::ROTATE_90_X);
            Serial.println("[RocketSensorManager] Detected: Sensor -Y up → ROTATE_90_X");
        }
    }
    else
    {
        // Z axis is vertical
        if (reading.z() > 0)
        {
            // Sensor +Z points up → already aligned with body +Z
            transform = MountingTransform(MountingOrientation::IDENTITY);
            Serial.println("[RocketSensorManager] Detected: Sensor +Z up → IDENTITY");
        }
        else
        {
            // Sensor -Z points up (Z reads negative) → flip Z to point up
            // Need: Z→-Z, flip Z axis only
            double flipZ[9] = {
                1, 0, 0,
                0, 1, 0,
                0, 0, -1
            };
            transform = MountingTransform(flipZ);
            Serial.println("[RocketSensorManager] Detected: Sensor -Z up → FLIP_Z (custom)");
        }
    }

    Serial.print("[RocketSensorManager] Accelerometer reading: [");
    Serial.print(reading.x());
    Serial.print(", ");
    Serial.print(reading.y());
    Serial.print(", ");
    Serial.print(reading.z());
    Serial.println("] m/s²");

    return transform;
}

bool RocketSensorManager::autoDetectMountingOrientations()
{
    Serial.println("[RocketSensorManager] Auto-detecting mounting orientations...");
    Serial.println("[RocketSensorManager] Assuming rocket is vertical (nose up, body Z up)");

    bool detectedAny = false;

    // Detect low-G accelerometer orientation
    if (lowGAccel)
    {
        Serial.println("[RocketSensorManager] Detecting low-G accelerometer orientation...");
        MountingTransform detected = detectOrientationFromGravity(lowGAccel);
        lowGAccelMount = detected;

        // For IMUs, apply same transform to gyro and mag
        // Assumption: if gyro/mag exist, they're likely part of the same IMU as low-G accel
        if (gyro)
        {
            Serial.println("[RocketSensorManager] Applying same transform to gyro (assuming same IMU)");
            gyroMount = detected;
        }
        if (mag)
        {
            Serial.println("[RocketSensorManager] Applying same transform to mag (assuming same IMU)");
            magMount = detected;
        }

        detectedAny = true;
    }

    // Detect high-G accelerometer orientation
    if (highGAccel)
    {
        Serial.println("[RocketSensorManager] Detecting high-G accelerometer orientation...");
        MountingTransform detected = detectOrientationFromGravity(highGAccel);
        highGAccelMount = detected;
        detectedAny = true;
    }

    if (!detectedAny)
    {
        Serial.println("[RocketSensorManager] Warning: No accelerometers available for orientation detection");
        return false;
    }

    Serial.println("[RocketSensorManager] Orientation detection complete");
    Serial.println("[RocketSensorManager] Note: Gyro and mag must be manually configured if not part of same IMU as accel");

    return true;
}
