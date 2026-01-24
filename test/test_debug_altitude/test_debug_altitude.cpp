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

void setUp(void)
{
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

    // Create state with filters
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();

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
    // Update sensor manager
    sensorManager->update();

    // Update state
    state->update(dt * 1000.0); // Convert to milliseconds
}

// ---

void test_fake_barometer_altitude() {
    // Test that FakeBarometer.setAltitude() works
    fakeBaro.setAltitude(100.0);
    fakeBaro.read();

    double alt = fakeBaro.getASLAltM();
    printf("FakeBarometer altitude: %.2f m\n", alt);
    TEST_ASSERT_TRUE(alt >= 95.0 && alt <= 105.0);
}

void test_rocket_state_reads_barometer() {
    // Test that RocketState reads barometer altitude
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    simulateUpdate();

    // Now set ground level
    state->setGroundLevel(100.0);

    // Update state again to recalculate AGL with new ground level
    simulateUpdate();

    double aglAfter = state->getAltitudeAGL();

    // Should be near 0
    TEST_ASSERT_TRUE(aglAfter >= -10.0 && aglAfter <= 10.0);
}

// ---

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_fake_barometer_altitude);
    RUN_TEST(test_rocket_state_reads_barometer);

    UNITY_END();
}
// ---
