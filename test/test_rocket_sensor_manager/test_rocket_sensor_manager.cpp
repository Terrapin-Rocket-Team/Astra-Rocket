#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"

// Include RocketSensorManager
#include "../../src/RocketSensorManager.h"

using namespace astra;

// ========================= Test Fixtures =========================

RocketSensorManager* sensorManager = nullptr;
FakeAccel* lowGAccel = nullptr;
FakeAccel* highGAccel = nullptr;
FakeGyro* gyro = nullptr;
FakeBarometer* baro = nullptr;

// ========================= Setup / Teardown =========================

void setUp(void)
{
    // Create fresh instances for each test
    sensorManager = new RocketSensorManager();
    lowGAccel = new FakeAccel();
    highGAccel = new FakeAccel();
    gyro = new FakeGyro();
    baro = new FakeBarometer();
}

void tearDown(void)
{
    // Clean up after each test
    delete sensorManager;
    delete lowGAccel;
    delete highGAccel;
    delete gyro;
    delete baro;
}

// ========================= Orientation Detection Tests =========================
// Test orientation detection through public auto-detect method

void test_orientation_detection_z_up_identity()
{
    // Sensor reads +Z up (9.81 on Z axis)
    lowGAccel->set(Vector<3>{0.0, 0.0, 9.81});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    // Auto-detect - should succeed
    bool success = sensorManager->autoDetectMountingOrientations();
    TEST_ASSERT_TRUE(success);

    // Verify transform by checking data pass-through
    lowGAccel->set(Vector<3>{1.0, 2.0, 3.0});
    sensorManager->update();
    BodyFrameData data = sensorManager->getBodyFrameData();

    // Should be identity transform - values pass through unchanged
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.0, data.accel.x());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 2.0, data.accel.y());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 3.0, data.accel.z());
}

void test_orientation_detection_x_up()
{
    // Sensor reads +X up
    lowGAccel->set(Vector<3>{9.81, 0.0, 0.0});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();
    sensorManager->autoDetectMountingOrientations();

    // Now sensor X should map to body Z
    // Set sensor X=10, should appear in body Z
    lowGAccel->set(Vector<3>{10.0, 0.0, 0.0});
    sensorManager->update();
    BodyFrameData data = sensorManager->getBodyFrameData();

    // X→Z rotation
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.x());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.y());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 10.0, data.accel.z());
}

void test_orientation_detection_y_up()
{
    // Sensor reads +Y up
    lowGAccel->set(Vector<3>{0.0, 9.81, 0.0});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();
    sensorManager->autoDetectMountingOrientations();

    // Now sensor Y should map to body Z
    lowGAccel->set(Vector<3>{0.0, 10.0, 0.0});
    sensorManager->update();
    BodyFrameData data = sensorManager->getBodyFrameData();

    // Y→Z rotation
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.x());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.y());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 10.0, data.accel.z());
}

void test_orientation_detection_with_tilt()
{
    // Sensor reads mostly Z up, but with some tilt (should still detect)
    lowGAccel->set(Vector<3>{1.0, 0.5, 9.5});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    bool success = sensorManager->autoDetectMountingOrientations();
    TEST_ASSERT_TRUE(success);
}

void test_orientation_detection_fails_with_bad_reading()
{
    // Invalid reading (NaN)
    lowGAccel->set(Vector<3>{NAN, 0.0, 0.0});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    // Should still return true (detection ran) but fall back to IDENTITY
    bool success = sensorManager->autoDetectMountingOrientations();
    TEST_ASSERT_TRUE(success);
}

// ========================= Accelerometer Mode Switching Tests =========================

void test_accel_mode_starts_low_g()
{
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->begin();

    // Should start in LOW_G mode
    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_LOW_G, sensorManager->getAccelMode());
    TEST_ASSERT_EQUAL_PTR(lowGAccel, sensorManager->getActiveAccel());
}

void test_accel_mode_switches_to_high_g_above_threshold()
{
    // Explicitly initialize sensors
    lowGAccel->begin();
    highGAccel->begin();

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->begin();

    // Set low-G reading above 15g threshold (e.g., 20g)
    lowGAccel->set(Vector<3>{0.0, 0.0, 20.0 * 9.81});
    highGAccel->set(Vector<3>{0.0, 0.0, 20.0 * 9.81});

    // First update - should still be LOW_G (just read)
    sensorManager->update();

    // Second update - should switch to HIGH_G
    delay(150);  // Wait for debounce
    sensorManager->update();

    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_HIGH_G, sensorManager->getAccelMode());
    TEST_ASSERT_EQUAL_PTR(highGAccel, sensorManager->getActiveAccel());
}

void test_accel_mode_switches_back_to_low_g_with_hysteresis()
{
    // Explicitly initialize sensors
    lowGAccel->begin();
    highGAccel->begin();

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->setModeSwitchDebounce(50);  // Reduce for faster testing
    sensorManager->begin();

    // Start high (20g)
    lowGAccel->set(Vector<3>{0.0, 0.0, 20.0 * 9.81});
    highGAccel->set(Vector<3>{0.0, 0.0, 20.0 * 9.81});
    sensorManager->update();
    delay(60);
    sensorManager->update();

    // Verify in HIGH_G mode
    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_HIGH_G, sensorManager->getAccelMode());

    // Drop to 10g (below 12g threshold)
    lowGAccel->set(Vector<3>{0.0, 0.0, 10.0 * 9.81});
    highGAccel->set(Vector<3>{0.0, 0.0, 10.0 * 9.81});
    delay(60);
    sensorManager->update();

    // Should switch back to LOW_G
    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_LOW_G, sensorManager->getAccelMode());
}

void test_accel_mode_hysteresis_prevents_oscillation()
{
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->setModeSwitchDebounce(50);
    sensorManager->begin();

    // Set to 13g (in hysteresis zone: 12-15g)
    lowGAccel->set(Vector<3>{0.0, 0.0, 13.0 * 9.81});
    highGAccel->set(Vector<3>{0.0, 0.0, 13.0 * 9.81});

    sensorManager->update();
    RocketSensorManager::AccelMode initialMode = sensorManager->getAccelMode();

    // Multiple updates at 13g should not cause switching
    for (int i = 0; i < 5; i++)
    {
        delay(60);
        sensorManager->update();
        TEST_ASSERT_EQUAL(initialMode, sensorManager->getAccelMode());
    }
}

void test_accel_mode_only_low_g_available()
{
    // Only register low-G accelerometer
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    // Should use LOW_G mode
    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_LOW_G, sensorManager->getAccelMode());

    // Even at high acceleration, should stay in LOW_G
    lowGAccel->set(Vector<3>{0.0, 0.0, 20.0 * 9.81});
    sensorManager->update();

    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_LOW_G, sensorManager->getAccelMode());
}

void test_accel_mode_only_high_g_available()
{
    // Only register high-G accelerometer
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->begin();

    // Should use HIGH_G mode
    TEST_ASSERT_EQUAL(RocketSensorManager::MODE_HIGH_G, sensorManager->getAccelMode());
}

// ========================= Health Monitoring Tests =========================

void test_health_monitoring_healthy_sensor()
{
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    lowGAccel->set(Vector<3>{0.0, 0.0, 9.81});
    sensorManager->update();

    TEST_ASSERT_EQUAL(SensorHealth::HEALTHY, sensorManager->getAccelHealth());
}

void test_health_monitoring_invalid_data()
{
    lowGAccel->begin();
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->setMaxFailures(3);
    sensorManager->begin();

    // Provide invalid data (NaN)
    lowGAccel->set(Vector<3>{NAN, 0.0, 0.0});

    // Update multiple times to trigger failure
    for (int i = 0; i < 3; i++)
    {
        sensorManager->update();
    }

    // Health should be degraded or failed
    SensorHealth health = sensorManager->getAccelHealth();
    TEST_ASSERT_TRUE(health == SensorHealth::DEGRADED || health == SensorHealth::FAILED);
}

// ========================= Body Frame Data Tests =========================

void test_body_frame_data_valid_accel()
{
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    lowGAccel->set(Vector<3>{1.0, 2.0, 3.0});
    sensorManager->update();

    BodyFrameData data = sensorManager->getBodyFrameData();

    TEST_ASSERT_TRUE(data.hasAccel);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, data.accel.x());
    TEST_ASSERT_EQUAL_DOUBLE(2.0, data.accel.y());
    TEST_ASSERT_EQUAL_DOUBLE(3.0, data.accel.z());
}

void test_body_frame_data_valid_gyro()
{
    gyro->begin();
    sensorManager->withGyro(gyro);
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    gyro->set(Vector<3>{0.1, 0.2, 0.3});
    gyro->update();
    sensorManager->update();

    BodyFrameData data = sensorManager->getBodyFrameData();

    TEST_ASSERT_TRUE(data.hasGyro);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, data.gyro.x());
    TEST_ASSERT_EQUAL_DOUBLE(0.2, data.gyro.y());
    TEST_ASSERT_EQUAL_DOUBLE(0.3, data.gyro.z());
}

void test_body_frame_data_valid_baro()
{
    baro->begin();
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withBaro(baro);
    sensorManager->begin();

    baro->set(1013.25, 20.0);
    baro->update();
    sensorManager->update();

    BodyFrameData data = sensorManager->getBodyFrameData();

    TEST_ASSERT_TRUE(data.hasBaro);
    TEST_ASSERT_EQUAL_DOUBLE(1013.25, data.pressure);
    TEST_ASSERT_EQUAL_DOUBLE(20.0, data.temperature);
}

void test_body_frame_data_mounting_transform_applied()
{
    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->begin();

    // Set a rotation that maps Y→Z (sensor Y becomes body Z)
    sensorManager->setAccelMount(MountingTransform(MountingOrientation::ROTATE_NEG90_X));

    // Sensor reads Y=10
    lowGAccel->set(Vector<3>{0.0, 10.0, 0.0});
    sensorManager->update();

    BodyFrameData data = sensorManager->getBodyFrameData();

    // After rotation, Y should map to Z
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.x());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, data.accel.y());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 10.0, data.accel.z());
}

// ========================= Full Auto-Detection Test =========================

void test_auto_detect_mounting_orientations()
{
    // Set low-G accel to read +Z up
    lowGAccel->set(Vector<3>{0.0, 0.0, 9.81});

    // Set high-G accel to read +Y up (rotated 90°)
    highGAccel->set(Vector<3>{0.0, 9.81, 0.0});

    sensorManager->withLowGAccel(lowGAccel);
    sensorManager->withHighGAccel(highGAccel);
    sensorManager->begin();

    // Auto-detect orientations
    bool success = sensorManager->autoDetectMountingOrientations();

    TEST_ASSERT_TRUE(success);

    // Verify low-G transform is identity
    // (We can't directly check the internal transform, but we can verify behavior)
    // Update and check that data comes through correctly
    lowGAccel->set(Vector<3>{1.0, 2.0, 3.0});
    sensorManager->update();

    BodyFrameData data = sensorManager->getBodyFrameData();
    // Low-G should have IDENTITY, so values should pass through unchanged
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.0, data.accel.x());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 2.0, data.accel.y());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 3.0, data.accel.z());
}

// ========================= Main Test Runner =========================

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Orientation detection tests
    RUN_TEST(test_orientation_detection_z_up_identity);
    RUN_TEST(test_orientation_detection_x_up);
    RUN_TEST(test_orientation_detection_y_up);
    RUN_TEST(test_orientation_detection_with_tilt);
    RUN_TEST(test_orientation_detection_fails_with_bad_reading);

    // Accelerometer mode switching tests
    RUN_TEST(test_accel_mode_starts_low_g);
    RUN_TEST(test_accel_mode_switches_to_high_g_above_threshold);
    RUN_TEST(test_accel_mode_switches_back_to_low_g_with_hysteresis);
    RUN_TEST(test_accel_mode_hysteresis_prevents_oscillation);
    RUN_TEST(test_accel_mode_only_low_g_available);
    RUN_TEST(test_accel_mode_only_high_g_available);

    // Health monitoring tests
    RUN_TEST(test_health_monitoring_healthy_sensor);
    RUN_TEST(test_health_monitoring_invalid_data);

    // Body frame data tests
    RUN_TEST(test_body_frame_data_valid_accel);
    RUN_TEST(test_body_frame_data_valid_gyro);
    RUN_TEST(test_body_frame_data_valid_baro);
    RUN_TEST(test_body_frame_data_mounting_transform_applied);

    // Auto-detection test
    RUN_TEST(test_auto_detect_mounting_orientations);

    return UNITY_END();
}
