#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>

// include other headers you need to test here
#include <State/State.h>
#include "../../src/RocketState.h"
#include "../../src/RocketSensorManager.h"

using namespace astra_rocket;

// ---

// Set up and global variables or mocks for testing here
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
RocketSensorManager sensorManager;
RocketState* state;

// ---

void setUp(void)
{
    fakeBaro.init();
    fakeIMU.init();

    sensorManager.withLowGAccel(fakeIMU.getAccelSensor());
    sensorManager.withGyro(fakeIMU.getGyroSensor());
    sensorManager.withBaro(&fakeBaro);
    sensorManager.begin();

    state = new RocketState();
    state->withSensorManager(&sensorManager);
    state->begin();

    setMillis(0);
}

void tearDown(void)
{
    delete state;
    state = nullptr;
    resetMillis();
}

// Helper to simulate an update cycle
void simulateUpdate(double dt = 0.02) {
    // Get sensor data
    Vector<3> accel = fakeIMU.getAccelSensor()->getAccel();
    Vector<3> gyro = fakeIMU.getGyroSensor()->getAngVel();
    double baroAlt = fakeBaro.getASLAltM();

    // Call split update methods like Astra does
    state->updateOrientation(gyro, accel, dt);
    state->updateMeasurements(Vector<3>(0, 0, 0), baroAlt, false, true, -1);
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
