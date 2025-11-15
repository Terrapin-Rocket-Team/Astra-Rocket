#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include <State/State.h>  // Force TRT-Astra to be included
#include "../../src/RocketState.h"

using namespace astra_rocket;

// Mock sensors for testing
Sensor* testSensors[1];

void setUp(void)
{
    // set stuff up before each test here, if needed
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_rocket_state_initialization() {
    // Test that RocketState can be instantiated
    RocketState state(testSensors, 0);

    // Verify initial flight stage is PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state.getFlightStage());

    // Verify initial values are reasonable
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state.getTimeInStage());
}

void test_ground_level_setting() {
    RocketState state(testSensors, 0);

    // Set ground level altitude
    double groundAlt = 100.0; // 100m MSL
    state.setGroundLevel(groundAlt);

    // AGL should be 0 when at ground level
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state.getAltitudeAGL());
}

void test_flight_stage_manual_setting() {
    RocketState state(testSensors, 0);

    // Manually set flight stage
    state.setFlightStage(FlightStage::BOOST);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state.getFlightStage());

    state.setFlightStage(FlightStage::COAST);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state.getFlightStage());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_rocket_state_initialization);
    RUN_TEST(test_ground_level_setting);
    RUN_TEST(test_flight_stage_manual_setting);

    UNITY_END();
}
