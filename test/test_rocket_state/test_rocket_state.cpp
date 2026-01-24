#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <State/State.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Filters/Filter.h>
#include <Filters/Mahony.h>
#include "../../src/RocketState.h"
#include "../../src/RocketKF.h"
#include <cmath>

using namespace astra_rocket;
using namespace astra;

// Mock sensors for testing
FakeBarometer fakeBaro;
FakeAccel fakeAccel;
FakeGyro fakeGyro;
SensorManager* sensorManager;
RocketKF* kalmanFilter;
MahonyAHRS* orientationFilter;
RocketState* state;

void setUp(void)
{
    // Initialize sensors
    fakeBaro.init();
    fakeAccel.init();
    fakeGyro.init();

    // Create sensor manager
    sensorManager = new SensorManager();
    sensorManager->setPrimaryAccel(&fakeAccel);
    sensorManager->setPrimaryGyro(&fakeGyro);
    sensorManager->setPrimaryBaro(&fakeBaro);
    sensorManager->begin();

    // Create filters
    kalmanFilter = new RocketKF();
    orientationFilter = new MahonyAHRS();

    // Create RocketState with filters
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();
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

// Helper to simulate an update cycle
void simulateUpdate(double dt = 0.02) {
    // Advance time
    unsigned long currentMillis = millis();
    setMillis(currentMillis + (unsigned long)(dt * 1000.0));

    // Update sensor manager
    sensorManager->update();

    // Update state (expects time in seconds, not milliseconds)
    double newTime = millis() / 1000.0;
    state->update(newTime);
}

// ===== BASIC INITIALIZATION TESTS =====

void test_rocket_state_initialization() {
    // Test that RocketState can be instantiated
    RocketKF kf;
    MahonyAHRS ahrs;
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

    // Set barometer to ground level
    fakeBaro.setAltitude(100.0);
    simulateUpdate();

    // AGL should be 0 when at ground level
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getAltitudeAGL());
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

    fakeBaro.setAltitude(0.0); // Sea level
    simulateUpdate();

    // AGL should be 86m above ground
    TEST_ASSERT_EQUAL_DOUBLE(86.0, state->getAltitudeAGL());
}

void test_ground_level_high_altitude() {
    // Test ground level at high altitude (e.g., La Paz, Bolivia)
    state->setGroundLevel(3640.0); // ~12,000 ft

    fakeBaro.setAltitude(3640.0);
    simulateUpdate();

    // AGL should be 0 at launch site
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getAltitudeAGL());

    // At 4000m MSL = 360m AGL
    fakeBaro.setAltitude(4000.0);
    setMillis(100);
    simulateUpdate();
    TEST_ASSERT_EQUAL_DOUBLE(360.0, state->getAltitudeAGL());
}

void test_ground_level_not_set() {
    // Test behavior when ground level is never set (defaults to 0)
    fakeBaro.setAltitude(100.0);
    simulateUpdate();

    // AGL should be altitude MSL when ground = 0
    TEST_ASSERT_EQUAL_DOUBLE(100.0, state->getAltitudeAGL());
}

void test_negative_altitude_agl() {
    // Test that AGL can go negative (sensor drift, landing below launch site)
    state->setGroundLevel(100.0);

    fakeBaro.setAltitude(95.0); // 5m below launch site
    simulateUpdate();

    TEST_ASSERT_EQUAL_DOUBLE(-5.0, state->getAltitudeAGL());
}

// ===== SENSOR UPDATE TESTS =====

void test_altitude_updates() {
    // Test that altitude updates correctly
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    simulateUpdate();

    TEST_ASSERT_EQUAL_DOUBLE(100.0, state->getAltitudeAGL());

    fakeBaro.setAltitude(200.0);
    setMillis(100);
    simulateUpdate();

    TEST_ASSERT_EQUAL_DOUBLE(200.0, state->getAltitudeAGL());
}

// ===== ALTITUDE TRACKING TESTS =====

void test_altitude_ascent() {
    // Ascend to apogee
    for (int i = 0; i < 50; i++) {
        fakeBaro.setAltitude(i * 20.0); // Up to 1000m
        fakeAccel.set(Vector<3>{0, 0, -9.81});
        setMillis(i * 100);
        simulateUpdate();
    }

    double apogeeAlt = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(apogeeAlt >= 900.0 && apogeeAlt <= 1100.0);

    // Descend
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    for (int i = 0; i < 50; i++) {
        fakeBaro.setAltitude(1000.0 - i * 10.0);
        setMillis(5000 + i * 100);
        simulateUpdate();
    }

    double currentAlt = state->getAltitudeAGL();

    // Current altitude should be lower than apogee
    TEST_ASSERT_TRUE(currentAlt < apogeeAlt);
}

// ===== OFF-VERTICAL ANGLE EDGE CASES =====

void test_off_vertical_angle_vertical_flight() {
    // Test perfectly vertical flight
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -30.0}); // Straight up
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should be close to 0 degrees
    TEST_ASSERT_TRUE(angle < 5.0);
}

void test_off_vertical_angle_horizontal_flight() {
    // Test horizontal flight
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{30.0, 0, 0}); // Horizontal
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should be close to 90 degrees
    TEST_ASSERT_TRUE(angle > 60.0); // Allow some tolerance
}

void test_off_vertical_angle_zero_acceleration() {
    // Test with zero acceleration (no orientation info)
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, 0});
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should default to 0 when no acceleration
    TEST_ASSERT_EQUAL_DOUBLE(0.0, angle);
}

// ===== TIME IN STAGE EDGE CASES =====

void test_time_in_stage_rollover() {
    // Test behavior at long durations
    state->setFlightStage(FlightStage::PAD_IDLE);
    setMillis(0);
    simulateUpdate();

    // Simulate 1 hour on pad
    setMillis(3600000); // 1 hour in ms
    simulateUpdate();

    double timeInStage = state->getTimeInStage();
    TEST_ASSERT_TRUE(timeInStage >= 3590.0 && timeInStage <= 3610.0); // ~3600 seconds
}

void test_time_in_stage_rapid_transitions() {
    // Test rapid stage changes
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);
    simulateUpdate();

    setMillis(100);
    state->setFlightStage(FlightStage::COAST);
    simulateUpdate();

    double time1 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time1 >= 0.0 && time1 <= 0.2);

    setMillis(200);
    state->setFlightStage(FlightStage::APOGEE);
    simulateUpdate();

    double time2 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time2 >= 0.0 && time2 <= 0.2);
}

// ===== GETTER SANITY CHECKS =====

void test_all_getters_return_valid_values() {
    // Verify all getters return finite values
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -19.62});
    simulateUpdate();

    TEST_ASSERT_TRUE(std::isfinite(state->getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(state->getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(state->getTimeInStage()));
}

void test_getters_no_sensors() {
    // Test that getters work even without sensor updates
    RocketKF kf;
    MahonyAHRS ahrs;
    RocketState emptyState(&kf, &ahrs);

    // Should not crash and return valid (possibly zero) values
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getTimeInStage()));
}

// ===== SENSOR DATA EDGE CASES =====

void test_extreme_altitude_values() {
    // Test very high altitude (space boundary ~100km)
    fakeBaro.setAltitude(100000.0);
    simulateUpdate();

    TEST_ASSERT_EQUAL_DOUBLE(100000.0, state->getAltitudeAGL());

    // Test very low altitude
    fakeBaro.setAltitude(0.01);
    setMillis(100);
    simulateUpdate();

    TEST_ASSERT_EQUAL_DOUBLE(0.01, state->getAltitudeAGL());
}

void test_sensor_update_frequency() {
    // Test that multiple updates work correctly
    for (int i = 0; i < 1000; i++) {
        fakeBaro.setAltitude(i * 0.1);
        fakeAccel.set(Vector<3>{0, 0, -9.81});
        setMillis(i * 10);
        simulateUpdate();
    }

    // Should complete without errors
    TEST_ASSERT_TRUE(state->getAltitudeAGL() >= 90.0);
}

// ===== FLIGHT STAGE TRANSITION TRACKING =====

void test_stage_transition_time_reset() {
    // Verify time in stage resets on transition
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);

    // Set high acceleration to stay in BOOST (above 1.5G threshold)
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -30.0}); // 3G
    simulateUpdate();

    setMillis(2000);
    simulateUpdate();
    double boostTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(boostTime >= 1.8 && boostTime <= 2.2);

    // Transition to coast
    setMillis(2100);
    state->setFlightStage(FlightStage::COAST);
    // Set low acceleration for coast
    fakeAccel.set(Vector<3>{0, 0, -9.81}); // 1G
    simulateUpdate();

    double coastTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(coastTime >= 0.0 && coastTime <= 0.3); // Should reset
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
