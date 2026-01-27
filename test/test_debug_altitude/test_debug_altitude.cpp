#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <Sensors/SensorManager/SensorManager.h>
#include "../../src/RocketState.h"
#include "../mocks/MockLinearKalmanFilter.h"
#include "../mocks/MockMahony.h"

using namespace astra_rocket;
using namespace astra;
using namespace astra_mocks;

// ---

FakeBarometer fakeBaro;
FakeIMU fakeIMU;
SensorManager* sensorManager;
MockLinearKalmanFilter* kalmanFilter;
MockMahony* orientationFilter;
RocketState* state;

// ---

void setUp(void)
{
    // Set up minimal sensors just to keep State class happy
    fakeBaro.init();
    fakeIMU.init();

    sensorManager = new SensorManager();
    sensorManager->setPrimaryAccel(fakeIMU.getAccelSensor());
    sensorManager->setPrimaryGyro(fakeIMU.getGyroSensor());
    sensorManager->setPrimaryBaro(&fakeBaro);
    sensorManager->begin();

    // Create mock filters
    kalmanFilter = new MockLinearKalmanFilter(9, 0, 6);
    orientationFilter = new MockMahony();

    // Create state with filters and sensor manager
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

// ---

void test_debug_altitude() {
    // Set ground level to 0
    state->setGroundLevel(0.0);

    // Directly set KF state to have altitude of 100m AGL
    Matrix kfState = kalmanFilter->getState();
    kfState(2, 0) = 100.0;  // pz = 100m AGL
    kfState(5, 0) = 10.0;   // vz = 10 m/s (ascending)
    kalmanFilter->setState(kfState);

    // Advance time and call update to pull KF state into RocketState
    setMillis(20);
    double currentTime = millis() / 1000.0;
    state->update(currentTime);

    // Verify altitude is read correctly from KF state
    double agl = state->getAltitudeAGL();
    printf("AGL: %.2f m (expected 100.0)\n", agl);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 100.0, agl);
}

// ---

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_debug_altitude);

    UNITY_END();
}
// ---
