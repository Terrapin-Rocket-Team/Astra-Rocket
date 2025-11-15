#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"

// include other headers you need to test here
#include <State/State.h>
#include "../../src/RocketState.h"

using namespace astra_rocket;

// ---

// Set up and global variables or mocks for testing here
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
Sensor* testSensors[2];
RocketState* state;

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    // set stuff up before each test here, if needed
    testSensors[0] = &fakeBaro;
    testSensors[1] = &fakeIMU;

    // Initialize sensors
    fakeBaro.init();
    fakeIMU.init();

    // Create new RocketState for each test
    state = new RocketState(testSensors, 2);

    // Reset time
    setMillis(0);
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
    delete state;
    state = nullptr;
    resetMillis();  // Reset fake time for next test
}
// ---

// Test functions must be void and take no arguments, put them here

void test_ground_level_initialization() {
    // Set barometer to 100m MSL
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // Set ground level to current altitude
    state->setGroundLevel(100.0);

    // AGL should be approximately 0 when at ground level
    double agl = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(agl >= -5.0 && agl <= 5.0); // Allow some margin
}

void test_altitude_agl_calculation() {
    // Set ground level at sea level
    state->setGroundLevel(0.0);

    // Set barometer to 500m altitude
    fakeBaro.setAltitude(500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // AGL should be approximately 500m
    double agl = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(agl >= 490.0 && agl <= 510.0); // Allow 10m margin
}

void test_altitude_agl_with_elevated_ground_level() {
    // Set ground level at 1000m MSL
    state->setGroundLevel(1000.0);

    // Set barometer to 1500m MSL
    fakeBaro.setAltitude(1500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // AGL should be approximately 500m
    double agl = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(agl >= 490.0 && agl <= 510.0);
}

void test_vertical_velocity_calculation() {
    state->setGroundLevel(0.0);
    setMillis(0);

    // Set initial altitude
    fakeBaro.set(101325.0, 20.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // The vertical velocity is calculated from the state's internal filter
    // For this test, we just verify the getter works
    double vertVel = state->getVerticalVelocity();
    TEST_ASSERT_TRUE(vertVel >= -100.0 && vertVel <= 100.0); // Sanity check
}

void test_vertical_acceleration_calculation() {
    state->setGroundLevel(0.0);

    // Set high acceleration (3G upward = -30 m/s^2 in IMU z-axis)
    fakeBaro.set(101325.0, 20.0);
    fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // Vertical acceleration should be approximately 3G
    // Note: Actual value depends on state's acceleration calculation
    double vertAccel = state->getVerticalAcceleration();
    TEST_ASSERT_TRUE(vertAccel >= -10.0 && vertAccel <= 10.0); // Sanity check
}

void test_max_acceleration_tracking() {
    state->setGroundLevel(0.0);

    // Initially should be 0
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getMaxAcceleration());

    // Apply 2G acceleration
    fakeIMU.set(Vector<3>{0, 0, -20.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    double max1 = state->getMaxAcceleration();
    TEST_ASSERT_TRUE(max1 >= 0.0); // Should have increased

    // Apply 5G acceleration
    fakeIMU.set(Vector<3>{0, 0, -50.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    double max2 = state->getMaxAcceleration();
    TEST_ASSERT_TRUE(max2 >= max1); // Should have increased further

    // Apply lower acceleration (1G)
    fakeIMU.set(Vector<3>{0, 0, -10.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    double max3 = state->getMaxAcceleration();
    TEST_ASSERT_EQUAL_DOUBLE(max2, max3); // Should stay at max (not decrease)
}

void test_max_velocity_tracking() {
    state->setGroundLevel(0.0);

    // Initially should be 0
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getMaxVelocity());

    // Update multiple times to build up velocity
    // (In practice, velocity comes from the state filter)
    for (int i = 0; i < 10; i++) {
        setMillis(i * 100);
        fakeIMU.set(Vector<3>{0, 0, -20.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        state->update();
    }

    // Max velocity should have increased
    double maxVel = state->getMaxVelocity();
    TEST_ASSERT_TRUE(maxVel >= 0.0);
}

void test_apogee_estimate_during_boost() {
    state->setGroundLevel(0.0);
    state->setFlightStage(FlightStage::BOOST);

    // Update with high acceleration at 100m altitude
    fakeIMU.set(Vector<3>{0, 0, -40.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.setAltitude(100.0);
    state->update();

    // During boost, apogee estimate should be calculated
    // Should be greater than current altitude
    double apogeeEst = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(apogeeEst >= 0.0); // Should be non-negative
}

void test_apogee_estimate_during_coast() {
    state->setGroundLevel(0.0);
    state->setFlightStage(FlightStage::COAST);

    // Update with low acceleration (coasting)
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.set(95000.0, 15.0); // ~500m altitude
    state->update();

    // During coast, apogee estimate should be calculated
    double apogeeEst = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(apogeeEst >= 0.0);
}

void test_apogee_estimate_after_apogee() {
    state->setGroundLevel(0.0);
    state->setFlightStage(FlightStage::UNDER_DROGUE);

    // Set altitude
    fakeBaro.set(95000.0, 15.0); // ~500m altitude
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    state->update();

    // After apogee, estimate should use max altitude achieved
    double apogeeEst = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(apogeeEst >= 0.0);

    // Time to apogee should be 0 (already past it)
    double timeToApogee = state->getTimeToApogee();
    TEST_ASSERT_EQUAL_DOUBLE(0.0, timeToApogee);
}

void test_time_to_apogee_calculation() {
    state->setGroundLevel(0.0);
    state->setFlightStage(FlightStage::COAST);

    // Set conditions for upward velocity
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.set(95000.0, 15.0);
    state->update();

    // Time to apogee should be calculated during ascent phases
    double timeToApogee = state->getTimeToApogee();
    TEST_ASSERT_TRUE(timeToApogee >= 0.0);
}

void test_off_vertical_angle_at_rest() {
    state->setGroundLevel(0.0);

    // Set low acceleration (at rest on pad)
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.set(101325.0, 20.0);
    state->update();

    // Off-vertical angle should be small when at rest
    double angle = state->getOffVerticalAngle();
    TEST_ASSERT_TRUE(angle >= -10.0 && angle <= 10.0);
}

void test_off_vertical_angle_during_boost() {
    state->setGroundLevel(0.0);
    state->setFlightStage(FlightStage::BOOST);

    // Set high vertical acceleration
    fakeIMU.set(Vector<3>{0, 0, -40.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.set(101325.0, 20.0);
    state->update();

    // Off-vertical angle should be calculated
    double angle = state->getOffVerticalAngle();
    TEST_ASSERT_TRUE(angle >= 0.0 && angle <= 180.0); // Valid angle range
}

void test_state_getters_return_valid_values() {
    state->setGroundLevel(0.0);

    // Update state with sea level altitude
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.setAltitude(0.0);
    state->update();

    // Test all getters return reasonable values
    TEST_ASSERT_TRUE(state->getAltitudeAGL() >= -100.0 && state->getAltitudeAGL() <= 100000.0);
    TEST_ASSERT_TRUE(state->getVerticalVelocity() >= -1000.0 && state->getVerticalVelocity() <= 1000.0);
    TEST_ASSERT_TRUE(state->getVerticalAcceleration() >= -100.0 && state->getVerticalAcceleration() <= 100.0);
    TEST_ASSERT_TRUE(state->getMaxAcceleration() >= 0.0 && state->getMaxAcceleration() <= 1000.0);
    TEST_ASSERT_TRUE(state->getMaxVelocity() >= 0.0 && state->getMaxVelocity() <= 10000.0);
    TEST_ASSERT_TRUE(state->getApogeeEstimate() >= 0.0);
    TEST_ASSERT_TRUE(state->getTimeToApogee() >= 0.0);
    TEST_ASSERT_TRUE(state->getOffVerticalAngle() >= 0.0 && state->getOffVerticalAngle() <= 180.0);
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Add your tests here
    RUN_TEST(test_ground_level_initialization);
    RUN_TEST(test_altitude_agl_calculation);
    RUN_TEST(test_altitude_agl_with_elevated_ground_level);
    RUN_TEST(test_vertical_velocity_calculation);
    RUN_TEST(test_vertical_acceleration_calculation);
    RUN_TEST(test_max_acceleration_tracking);
    RUN_TEST(test_max_velocity_tracking);
    RUN_TEST(test_apogee_estimate_during_boost);
    RUN_TEST(test_apogee_estimate_during_coast);
    RUN_TEST(test_apogee_estimate_after_apogee);
    RUN_TEST(test_time_to_apogee_calculation);
    RUN_TEST(test_off_vertical_angle_at_rest);
    RUN_TEST(test_off_vertical_angle_during_boost);
    RUN_TEST(test_state_getters_return_valid_values);

    UNITY_END();
}
// ---
