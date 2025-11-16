#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"
#include <State/State.h>  // Force TRT-Astra to be included
#include "../../src/RocketState.h"
#include <cmath>

using namespace astra_rocket;

// Mock sensors for testing
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
Sensor* testSensors[2];
RocketState* state;

void setUp(void)
{
    // set stuff up before each test here, if needed
    testSensors[0] = &fakeBaro;
    testSensors[1] = &fakeIMU;

    fakeBaro.init();
    fakeIMU.init();

    state = new RocketState(testSensors, 2);
    state->setGroundLevel(0.0);

    setMillis(0);
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
    delete state;
    state = nullptr;
    resetMillis();
}

// ===== BASIC INITIALIZATION TESTS =====

void test_rocket_state_initialization() {
    // Test that RocketState can be instantiated with no sensors
    RocketState emptyState(nullptr, 0);

    // Verify initial flight stage is PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, emptyState.getFlightStage());

    // Verify initial values are reasonable
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getTimeInStage());
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getAltitudeAGL());
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getMaxAcceleration());
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getMaxVelocity());
}

void test_ground_level_setting() {
    // Set ground level altitude
    double groundAlt = 100.0; // 100m MSL
    state->setGroundLevel(groundAlt);

    // Set barometer to ground level
    fakeBaro.setAltitude(100.0);
    state->update();

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
    state->update();

    // AGL should be 86m above ground
    TEST_ASSERT_EQUAL_DOUBLE(86.0, state->getAltitudeAGL());
}

void test_ground_level_high_altitude() {
    // Test ground level at high altitude (e.g., La Paz, Bolivia)
    state->setGroundLevel(3640.0); // ~12,000 ft

    fakeBaro.setAltitude(3640.0);
    state->update();

    // AGL should be 0 at launch site
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getAltitudeAGL());

    // At 4000m MSL = 360m AGL
    fakeBaro.setAltitude(4000.0);
    setMillis(100);
    state->update();
    TEST_ASSERT_EQUAL_DOUBLE(360.0, state->getAltitudeAGL());
}

void test_ground_level_not_set() {
    // Test behavior when ground level is never set (defaults to 0)
    fakeBaro.setAltitude(100.0);
    state->update();

    // AGL should be altitude MSL when ground = 0
    TEST_ASSERT_EQUAL_DOUBLE(100.0, state->getAltitudeAGL());
}

void test_negative_altitude_agl() {
    // Test that AGL can go negative (sensor drift, landing below launch site)
    state->setGroundLevel(100.0);

    fakeBaro.setAltitude(95.0); // 5m below launch site
    state->update();

    TEST_ASSERT_EQUAL_DOUBLE(-5.0, state->getAltitudeAGL());
}

// ===== VERTICAL COMPONENT CALCULATION EDGE CASES =====

void test_vertical_acceleration_zero() {
    // Test with zero acceleration (free fall)
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    double vertAccel = state->getVerticalAcceleration();
    // Should be close to 0
    TEST_ASSERT_TRUE(fabs(vertAccel) < 0.1);
}

void test_vertical_acceleration_high_g() {
    // Test with very high acceleration (200G)
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -1962.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 200G
    state->update();

    double vertAccel = state->getVerticalAcceleration();
    // Should be approximately 200G
    TEST_ASSERT_TRUE(fabs(vertAccel - 200.0) < 10.0); // Allow some tolerance
}

void test_vertical_acceleration_negative() {
    // Test with upward acceleration in NED frame (negative z)
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, 9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // -1G in NED
    state->update();

    double vertAccel = state->getVerticalAcceleration();
    // Should be approximately -1G (downward)
    TEST_ASSERT_TRUE(vertAccel < 0.0);
}

// ===== MAX VALUE TRACKING EDGE CASES =====

void test_max_acceleration_tracking() {
    state->setFlightStage(FlightStage::BOOST);

    // Start with low acceleration
    fakeIMU.set(Vector<3>{0, 0, -19.62}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 2G
    state->update();
    TEST_ASSERT_TRUE(state->getMaxAcceleration() >= 1.9);

    // Increase to 10G
    fakeIMU.set(Vector<3>{0, 0, -98.1}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 10G
    setMillis(100);
    state->update();
    TEST_ASSERT_TRUE(state->getMaxAcceleration() >= 9.0);

    // Drop back to 2G - max should stay at 10G
    fakeIMU.set(Vector<3>{0, 0, -19.62}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    setMillis(200);
    state->update();
    TEST_ASSERT_TRUE(state->getMaxAcceleration() >= 9.0);
}

void test_max_velocity_tracking() {
    state->setFlightStage(FlightStage::COAST);

    // Simulate increasing then decreasing velocity
    // Note: Velocity depends on State class calculations
    for (int i = 0; i < 10; i++) {
        fakeBaro.setAltitude(100.0 + i * 10.0);
        setMillis(i * 100);
        state->update();
    }

    double maxVel1 = state->getMaxVelocity();

    // Descend
    for (int i = 0; i < 10; i++) {
        fakeBaro.setAltitude(200.0 - i * 10.0);
        setMillis(1000 + i * 100);
        state->update();
    }

    double maxVel2 = state->getMaxVelocity();

    // Max velocity should not decrease
    TEST_ASSERT_TRUE(maxVel2 >= maxVel1);
}

void test_max_altitude_tracking() {
    // Ascend to apogee
    for (int i = 0; i < 50; i++) {
        fakeBaro.setAltitude(i * 20.0); // Up to 1000m
        setMillis(i * 100);
        state->update();
    }

    double maxAlt1 = state->getAltitudeAGL();

    // Descend
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    for (int i = 0; i < 50; i++) {
        fakeBaro.setAltitude(1000.0 - i * 10.0);
        setMillis(5000 + i * 100);
        state->update();
    }

    double currentAlt = state->getAltitudeAGL();

    // Current altitude should be lower than max
    TEST_ASSERT_TRUE(currentAlt < maxAlt1);
}

// ===== OFF-VERTICAL ANGLE EDGE CASES =====

void test_off_vertical_angle_vertical_flight() {
    // Test perfectly vertical flight
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // Straight up
    state->update();

    double angle = state->getOffVerticalAngle();
    // Should be close to 0 degrees
    TEST_ASSERT_TRUE(angle < 5.0);
}

void test_off_vertical_angle_horizontal_flight() {
    // Test horizontal flight
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{30.0, 0, 0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // Horizontal
    state->update();

    double angle = state->getOffVerticalAngle();
    // Should be close to 90 degrees
    TEST_ASSERT_TRUE(angle > 60.0); // Allow some tolerance
}

void test_off_vertical_angle_zero_acceleration() {
    // Test with zero acceleration (no orientation info)
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    double angle = state->getOffVerticalAngle();
    // Should default to 0 when no acceleration
    TEST_ASSERT_EQUAL_DOUBLE(0.0, angle);
}

// ===== TIME IN STAGE EDGE CASES =====

void test_time_in_stage_rollover() {
    // Test behavior at long durations
    state->setFlightStage(FlightStage::PAD_IDLE);
    setMillis(0);
    state->update();

    // Simulate 1 hour on pad
    setMillis(3600000); // 1 hour in ms
    state->update();

    double timeInStage = state->getTimeInStage();
    TEST_ASSERT_TRUE(timeInStage >= 3590.0 && timeInStage <= 3610.0); // ~3600 seconds
}

void test_time_in_stage_rapid_transitions() {
    // Test rapid stage changes
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);
    state->update();

    setMillis(100);
    state->setFlightStage(FlightStage::COAST);
    state->update();

    double time1 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time1 >= 0.0 && time1 <= 0.2);

    setMillis(200);
    state->setFlightStage(FlightStage::APOGEE);
    state->update();

    double time2 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time2 >= 0.0 && time2 <= 0.2);
}

// ===== GETTER SANITY CHECKS =====

void test_all_getters_return_valid_values() {
    // Verify all getters return finite values
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -19.62}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    TEST_ASSERT_TRUE(std::isfinite(state->getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(state->getVerticalVelocity()));
    TEST_ASSERT_TRUE(std::isfinite(state->getVerticalAcceleration()));
    TEST_ASSERT_TRUE(std::isfinite(state->getMaxAcceleration()));
    TEST_ASSERT_TRUE(std::isfinite(state->getMaxVelocity()));
    TEST_ASSERT_TRUE(std::isfinite(state->getApogeeEstimate()));
    TEST_ASSERT_TRUE(std::isfinite(state->getTimeToApogee()));
    TEST_ASSERT_TRUE(std::isfinite(state->getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(state->getTimeInStage()));
}

void test_getters_no_sensors() {
    // Test that getters work even with no sensors
    RocketState emptyState(nullptr, 0);
    emptyState.update();

    // Should not crash and return valid (possibly zero) values
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getVerticalVelocity()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getMaxAcceleration()));
}

// ===== SENSOR DATA EDGE CASES =====

void test_extreme_altitude_values() {
    // Test very high altitude (space boundary ~100km)
    fakeBaro.setAltitude(100000.0);
    state->update();

    TEST_ASSERT_EQUAL_DOUBLE(100000.0, state->getAltitudeAGL());

    // Test very low altitude
    fakeBaro.setAltitude(0.01);
    setMillis(100);
    state->update();

    TEST_ASSERT_EQUAL_DOUBLE(0.01, state->getAltitudeAGL());
}

void test_sensor_update_frequency() {
    // Test that multiple updates work correctly
    for (int i = 0; i < 1000; i++) {
        fakeBaro.setAltitude(i * 0.1);
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        setMillis(i * 10);
        state->update();
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
    fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 3G
    state->update();

    setMillis(2000);
    state->update();
    double boostTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(boostTime >= 1.8 && boostTime <= 2.2);

    // Transition to coast
    setMillis(2100);
    state->setFlightStage(FlightStage::COAST);
    // Set low acceleration for coast
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 1G
    state->update();

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

    // Vertical component calculations
    RUN_TEST(test_vertical_acceleration_zero);
    RUN_TEST(test_vertical_acceleration_high_g);
    RUN_TEST(test_vertical_acceleration_negative);

    // Max value tracking
    RUN_TEST(test_max_acceleration_tracking);
    RUN_TEST(test_max_velocity_tracking);
    RUN_TEST(test_max_altitude_tracking);

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
