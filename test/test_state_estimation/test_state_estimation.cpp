#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>

// include other headers you need to test here
#include <State/State.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Filters/Filter.h>
#include <Filters/Mahony.h>
#include "../../src/RocketState.h"
#include "../../src/RocketKF.h"

using namespace astra_rocket;
using namespace astra;

// ---

// Set up and global variables or mocks for testing here
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
SensorManager* sensorManager;
RocketKF* kalmanFilter;
MahonyAHRS* orientationFilter;
RocketState* state;

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    // Initialize sensors
    fakeBaro.init();
    fakeIMU.init();

    // Create sensor manager
    sensorManager = new SensorManager();
    sensorManager->setPrimaryAccel(fakeIMU.getAccelSensor());
    sensorManager->setPrimaryGyro(fakeIMU.getGyroSensor());
    sensorManager->setPrimaryBaro(&fakeBaro);
    sensorManager->begin();

    // Create filters
    kalmanFilter = new RocketKF();
    orientationFilter = new MahonyAHRS();

    // Create new RocketState for each test
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();

    // Reset time
    setMillis(0);
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
    delete state;
    delete kalmanFilter;
    delete orientationFilter;
    delete sensorManager;
    state = nullptr;
    kalmanFilter = nullptr;
    orientationFilter = nullptr;
    sensorManager = nullptr;
    resetMillis();  // Reset fake time for next test
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

// ---

// Test functions must be void and take no arguments, put them here

void test_ground_level_initialization() {
    // Set barometer to 100m MSL
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    simulateUpdate();

    // Set ground level to current altitude
    state->setGroundLevel(100.0);
    simulateUpdate();  // Update to recalculate AGL

    // AGL should be approximately 0 when at ground level
    double agl = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(agl >= -5.0 && agl <= 5.0); // Allow some margin
}

void test_altitude_agl_calculation() {
    // Set barometer to 500m altitude
    fakeBaro.setAltitude(500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    // Set ground level at sea level
    state->setGroundLevel(0.0);
    simulateUpdate();

    // AGL should be approximately 500m
    double agl = state->getAltitudeAGL();
    printf("AGL: %.2f m (expected 490-510)\n", agl);
    TEST_ASSERT_TRUE(agl >= 490.0 && agl <= 510.0); // Allow 10m margin
}

void test_altitude_agl_with_elevated_ground_level() {
    // Set barometer to 1500m MSL
    fakeBaro.setAltitude(1500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    // Set ground level at 1000m MSL
    state->setGroundLevel(1000.0);
    simulateUpdate();

    // AGL should be approximately 500m
    double agl = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(agl >= 490.0 && agl <= 510.0);
}

// Note: The following tests have been removed because the methods were removed from RocketState:
// - getVerticalVelocity()
// - getVerticalAcceleration()
// - getMaxAcceleration()
// - getMaxVelocity()
// - getApogeeEstimate()
// - getTimeToApogee()
//
// The library has been pruned to focus on core flight stage detection.

void test_basic_state_update() {
    // Test that state updates work without errors
    state->setGroundLevel(0.0);
    setMillis(0);

    fakeBaro.set(101325.0, 20.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    simulateUpdate();

    // Verify basic getters work
    TEST_ASSERT_TRUE(state->getAltitudeAGL() >= -100.0);
    TEST_ASSERT_TRUE(state->getOffVerticalAngle() >= 0.0);
}

void test_off_vertical_angle_at_rest() {
    state->setGroundLevel(0.0);

    // Set low acceleration (at rest on pad)
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.set(101325.0, 20.0);
    simulateUpdate();

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
    simulateUpdate();

    // Off-vertical angle should be calculated
    double angle = state->getOffVerticalAngle();
    TEST_ASSERT_TRUE(angle >= 0.0 && angle <= 180.0); // Valid angle range
}

void test_state_getters_return_valid_values() {
    // Update state with sea level altitude
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
    fakeBaro.setAltitude(0.0);

    state->setGroundLevel(0.0);
    simulateUpdate();

    // Test all available getters return reasonable values
    TEST_ASSERT_TRUE(state->getAltitudeAGL() >= -100.0 && state->getAltitudeAGL() <= 100000.0);
    TEST_ASSERT_TRUE(state->getOffVerticalAngle() >= 0.0 && state->getOffVerticalAngle() <= 180.0);
    TEST_ASSERT_TRUE(state->getTimeInStage() >= 0.0);
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
    RUN_TEST(test_basic_state_update);
    RUN_TEST(test_off_vertical_angle_at_rest);
    RUN_TEST(test_off_vertical_angle_during_boost);
    RUN_TEST(test_state_getters_return_valid_values);

    UNITY_END();
}
// ---
