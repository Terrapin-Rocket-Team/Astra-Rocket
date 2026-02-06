#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <Sensors/SensorManager/SensorManager.h>
#include "../../src/RocketState.h"
#include "../mocks/MockLinearKalmanFilter.h"
#include "../mocks/MockMahony.h"
#include <cmath>

using namespace astra_rocket;
using namespace astra;
using namespace astra_mocks;

// ===== Test Fixtures =====
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
SensorManager* sensorManager;
MockLinearKalmanFilter* kalmanFilter;
MockMahony* orientationFilter;
RocketState* state;
double testGroundLevel = 0.0;

void setUp(void)
{
    // Set up minimal sensors just to keep State class happy
    // NOTE: We won't use these sensors for testing - we directly manipulate KF state
    fakeBaro.init();
    fakeIMU.init();

    sensorManager = new SensorManager();
    sensorManager->setAccelSource(fakeIMU.getAccelSensor());
    sensorManager->setGyroSource(fakeIMU.getGyroSensor());
    sensorManager->setBaroSource(&fakeBaro);
    sensorManager->begin();

    // Create mock filters
    // MockLinearKalmanFilter(measurementSize, controlSize, stateSize)
    // State is 9D: [px, py, pz, vx, vy, vz, ax, ay, az]
    kalmanFilter = new MockLinearKalmanFilter(6, 0, 9);
    orientationFilter = new MockMahony();

    // Create RocketState with filters and sensor manager
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();

    testGroundLevel = 0.0;
    state->setGroundLevel(0.0);
    setMillis(0);
}

void tearDown(void)
{
    delete state;
    delete kalmanFilter;
    delete orientationFilter;
    delete sensorManager;
    state = nullptr;
    kalmanFilter = nullptr;
    orientationFilter = nullptr;
    sensorManager = nullptr;
    resetMillis();
}

// ===== Helper Functions =====

// Helper to set KF state and update RocketState
void setStateAndUpdate(double altitude, double velocity, double accelZ, unsigned long timeMs) {
    Matrix kfState = kalmanFilter->getState();
    kfState(2, 0) = altitude;   // pz (altitude AGL)
    kfState(5, 0) = velocity;   // vz (vertical velocity)
    kfState(8, 0) = accelZ;     // az (z-axis acceleration)
    kalmanFilter->setState(kfState);

    setMillis(timeMs);
    double timeSec = millis() / 1000.0;

    // Call both predictState and update to properly populate state variables
    state->predictState(timeSec);
    state->update(timeSec);
}

// Overload without acceleration for cases where we don't care about accel
void setStateAndUpdate(double altitude, double velocity, unsigned long timeMs) {
    setStateAndUpdate(altitude, velocity, -9.81, timeMs);
}

// ===== BASIC INITIALIZATION TESTS =====

void test_rocket_state_initialization() {
    // Test that RocketState initializes with mocks
    MockLinearKalmanFilter kf(6, 0, 9);
    MockMahony ahrs;
    RocketState emptyState(&kf, &ahrs);

    // Verify initial flight stage is PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, emptyState.getFlightStage());

    // Verify initial values are reasonable
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getTimeInStage());
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getAltitudeAGL());
}

void test_ground_level_setting() {
    // Set ground level altitude
    double groundAlt = 100.0; // 100m MSL
    state->setGroundLevel(groundAlt);
    testGroundLevel = groundAlt;

    // Set KF state to ground level (0 AGL)
    setStateAndUpdate(0.0, 0.0, 100);

    // AGL should be 0 when at ground level
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, state->getAltitudeAGL());
}

void test_flight_stage_manual_setting() {
    // Manually set flight stage
    state->setFlightStage(FlightStage::BOOST);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    state->setFlightStage(FlightStage::COAST);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    state->setFlightStage(FlightStage::LANDED);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

// ===== GROUND LEVEL EDGE CASES =====

void test_ground_level_negative() {
    // Test ground level below sea level (e.g., Death Valley)
    state->setGroundLevel(-86.0); // Death Valley elevation
    testGroundLevel = -86.0;

    // Set altitude to 0 AGL (which is -86 MSL)
    setStateAndUpdate(0.0, 0.0, 100);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, state->getAltitudeAGL());

    // Set altitude to 86m AGL
    setStateAndUpdate(86.0, 0.0, 200);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 86.0, state->getAltitudeAGL());
}

void test_ground_level_high_altitude() {
    // Test ground level at high altitude (e.g., La Paz, Bolivia)
    testGroundLevel = 3640.0;
    state->setGroundLevel(3640.0); // ~12,000 ft

    // AGL should be 0 at launch site
    setStateAndUpdate(0.0, 0.0, 100);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, state->getAltitudeAGL());

    // Climb to 360m AGL
    setStateAndUpdate(360.0, 50.0, 200);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 360.0, state->getAltitudeAGL());
}

void test_ground_level_not_set() {
    // Test behavior when ground level is at 0
    state->setGroundLevel(0.0);

    // Set altitude to 100m
    setStateAndUpdate(100.0, 0.0, 100);

    // AGL should equal altitude when ground = 0
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 100.0, state->getAltitudeAGL());
}

void test_negative_altitude_agl() {
    // Test that AGL can go negative (landing below launch site)
    testGroundLevel = 100.0;
    state->setGroundLevel(100.0);

    // 5m below launch site
    setStateAndUpdate(-5.0, 0.0, 100);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, -5.0, state->getAltitudeAGL());
}

// ===== SENSOR UPDATE TESTS =====

void test_altitude_updates() {
    // Test that altitude updates correctly
    setStateAndUpdate(100.0, 0.0, 100);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 100.0, state->getAltitudeAGL());

    // Update to higher altitude
    setStateAndUpdate(200.0, 0.0, 200);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 200.0, state->getAltitudeAGL());
}

// ===== ALTITUDE TRACKING TESTS =====

void test_altitude_ascent() {
    // Simulate ascent to 1000m
    setStateAndUpdate(1000.0, 50.0, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 1000.0, state->getAltitudeAGL());

    // Simulate descent to 500m
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    setStateAndUpdate(500.0, -15.0, 2000);

    double currentAlt = state->getAltitudeAGL();
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 500.0, currentAlt);
}

// ===== OFF-VERTICAL ANGLE EDGE CASES =====

void test_off_vertical_angle_vertical_flight() {
    // Test perfectly vertical flight
    orientationFilter->setMockOrientation(Quaternion(1, 0, 0, 0));
    setStateAndUpdate(100.0, 50.0, 1000);

    double angle = state->getOffVerticalAngle();
    // Should be close to 0 degrees
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, angle);
}

void test_off_vertical_angle_horizontal_flight() {
    // Test horizontal flight
    // Set orientation to 90 degrees pitch BEFORE updating
    Quaternion q;
    q.fromAxisAngle(Vector<3>(0, 1, 0), M_PI / 2.0); // 90 degree rotation around Y axis
    orientationFilter->setMockOrientation(q);

    // Now update - this will read the mock orientation
    setStateAndUpdate(100.0, 50.0, 1000);

    double angle = state->getOffVerticalAngle();
    // Should be 90 degrees
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 90.0, angle);
}

void test_off_vertical_angle_zero_acceleration() {
    // Test with zero acceleration
    orientationFilter->setMockOrientation(Quaternion(1, 0, 0, 0));
    setStateAndUpdate(100.0, 0.0, 0.0, 1000);

    double angle = state->getOffVerticalAngle();
    // Should default to 0
    TEST_ASSERT_EQUAL_DOUBLE(0.0, angle);
}

void test_pad_alignment_and_frame_lock() {
    // Body +X points up (rotate -90 deg about Y)
    Quaternion q;
    q.fromAxisAngle(Vector<3>(0, 1, 0), -M_PI / 2.0);
    orientationFilter->setMockOrientation(q);

    // Hold on pad long enough to pass hysteresis
    for (int i = 0; i < 12; i++) {
        setStateAndUpdate(0.0, 0.0, -9.81, 100 + i * 20);
    }

    double alignedAngle = state->getOffVerticalAngle();
    TEST_ASSERT_DOUBLE_WITHIN(5.0, 0.0, alignedAngle);

    // Lock frame for test (simulate liftoff)
    state->setFlightStage(FlightStage::BOOST);
    state->lockFrameForTest();

    // Change orientation to identity; locked frame should preserve previous mapping
    orientationFilter->setMockOrientation(Quaternion(1, 0, 0, 0));
    for (int i = 0; i < 12; i++) {
        setStateAndUpdate(0.0, 0.0, 40.0, 700 + i * 20);
    }

    double lockedAngle = state->getOffVerticalAngle();
    TEST_ASSERT_TRUE(lockedAngle > 45.0);
}

// ===== TIME IN STAGE EDGE CASES =====

void test_time_in_stage_rollover() {
    // Test behavior at long durations
    state->setFlightStage(FlightStage::PAD_IDLE);
    setMillis(0);
    setStateAndUpdate(0.0, 0.0, 100);

    // Simulate 1 hour on pad
    setMillis(3600000); // 1 hour in ms
    setStateAndUpdate(0.0, 0.0, 3600000);

    double timeInStage = state->getTimeInStage();
    TEST_ASSERT_TRUE(timeInStage >= 3590.0 && timeInStage <= 3610.0); // ~3600 seconds
}

void test_time_in_stage_rapid_transitions() {
    // Test rapid stage changes
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(10.0, 10.0, 100);

    setMillis(100);
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(15.0, 12.0, 200);

    double time1 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time1 >= 0.0 && time1 <= 0.2);

    setMillis(200);
    state->setFlightStage(FlightStage::APOGEE);
    setStateAndUpdate(20.0, 1.0, 300);

    double time2 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time2 >= 0.0 && time2 <= 0.2);
}

// ===== GETTER SANITY CHECKS =====

void test_all_getters_return_valid_values() {
    // Verify all getters return finite values
    setStateAndUpdate(100.0, 50.0, -19.62, 1000);

    TEST_ASSERT_TRUE(std::isfinite(state->getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(state->getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(state->getTimeInStage()));
}

void test_getters_no_sensors() {
    // Test that getters work with minimal setup
    MockLinearKalmanFilter kf(6, 0, 9);
    MockMahony ahrs;
    RocketState emptyState(&kf, &ahrs);

    // Should not crash and return valid (possibly zero) values
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getTimeInStage()));
}

// ===== SENSOR DATA EDGE CASES =====

void test_extreme_altitude_values() {
    // Test high altitude
    setStateAndUpdate(5000.0, 100.0, 5000);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 5000.0, state->getAltitudeAGL());

    // Test near-ground level
    setStateAndUpdate(0.01, -5.0, 10000);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.01, state->getAltitudeAGL());
}

void test_sensor_update_frequency() {
    // Test many rapid updates work correctly
    for (int i = 0; i < 1000; i++) {
        double alt = i * 0.1; // 0 to 100m
        setStateAndUpdate(alt, 10.0, i);
    }

    TEST_ASSERT_DOUBLE_WITHIN(1.0, 100.0, state->getAltitudeAGL());
}

// ===== FLIGHT STAGE TRANSITION TRACKING =====

void test_stage_transition_time_reset() {
    // Verify time in stage resets on transition
    setMillis(0);
    setStateAndUpdate(100.0, 50.0, -9.81, 0);
    state->setFlightStage(FlightStage::BOOST);

    setMillis(2000);
    setStateAndUpdate(150.0, 60.0, -30.0, 2000);
    double boostTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(boostTime >= 1.7 && boostTime <= 2.3);

    // Transition to coast
    setMillis(2100);
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(160.0, 55.0, -9.81, 2100);

    double coastTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(coastTime >= 0.0 && coastTime <= 0.4); // Should reset
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Basic tests
    RUN_TEST(test_rocket_state_initialization);
    RUN_TEST(test_ground_level_setting);
    RUN_TEST(test_flight_stage_manual_setting);

    // Ground level edge cases
    RUN_TEST(test_ground_level_negative);
    RUN_TEST(test_ground_level_high_altitude);
    RUN_TEST(test_ground_level_not_set);
    RUN_TEST(test_negative_altitude_agl);

    // Sensor updates
    RUN_TEST(test_altitude_updates);

    // Altitude tracking
    RUN_TEST(test_altitude_ascent);

    // Off-vertical angle
    RUN_TEST(test_off_vertical_angle_vertical_flight);
    RUN_TEST(test_off_vertical_angle_horizontal_flight);
    RUN_TEST(test_off_vertical_angle_zero_acceleration);
    RUN_TEST(test_pad_alignment_and_frame_lock);

    // Time in stage
    RUN_TEST(test_time_in_stage_rollover);
    RUN_TEST(test_time_in_stage_rapid_transitions);

    // Getter sanity
    RUN_TEST(test_all_getters_return_valid_values);
    RUN_TEST(test_getters_no_sensors);

    // Sensor data edge cases
    RUN_TEST(test_extreme_altitude_values);
    RUN_TEST(test_sensor_update_frequency);

    // Stage transitions
    RUN_TEST(test_stage_transition_time_reset);

    UNITY_END();
}
