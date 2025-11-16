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

void setUp(void)
{
    testSensors[0] = &fakeBaro;
    testSensors[1] = &fakeIMU;

    fakeBaro.init();
    fakeIMU.init();

    state = new RocketState(testSensors, 2);

    setMillis(0);
}

void tearDown(void)
{
    delete state;
    state = nullptr;
    resetMillis();
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

    state->update();

    // Now set ground level
    state->setGroundLevel(100.0);

    // Update state again to recalculate AGL with new ground level
    state->update();

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
