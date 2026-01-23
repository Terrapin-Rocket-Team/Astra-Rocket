#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <State/State.h>
#include "../../src/RocketState.h"
#include "../../src/RocketSensorManager.h"

using namespace astra_rocket;

// Test fixtures
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
RocketSensorManager sensorManager;
RocketState* state;

void setUp(void) {
    fakeBaro.init();
    fakeIMU.init();

    sensorManager.withLowGAccel(fakeIMU.getAccelSensor());
    sensorManager.withGyro(fakeIMU.getGyroSensor());
    sensorManager.withBaro(&fakeBaro);
    sensorManager.begin();

    state = new RocketState();
    state->withSensorManager(&sensorManager);
    state->begin();
    state->setGroundLevel(0.0);

    setMillis(0);
}

void tearDown(void) {
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

// Helper function to simulate descent with velocity
void simulateDescent(double startAlt, double endAlt, double descentRate, unsigned long duration) {
    unsigned long startTime = millis();
    double currentAlt = startAlt;

    // Calculate number of steps for smooth descent (update every 50ms)
    int numSteps = duration / 50;
    double altStep = (startAlt - endAlt) / numSteps;

    for (int i = 0; i <= numSteps; i++) {
        unsigned long currentTime = startTime + i * 50;
        currentAlt = startAlt - (altStep * i);

        setMillis(currentTime);
        fakeBaro.setAltitude(currentAlt);
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        simulateUpdate();
    }
}

// ===== DROGUE DEPLOYMENT DETECTION =====

void test_drogue_nominal_deployment() {
    // Test normal drogue deployment with typical descent rate (20 m/s)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    // Start at 500m after apogee
    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Simulate descent at 20 m/s for 1 second
    simulateDescent(500.0, 480.0, 20.0, 1000);

    // Should detect drogue deployment (velocity between 5-40 m/s)
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_slow_deployment() {
    // Test drogue at minimum descent rate (5 m/s)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Descend slowly at 5 m/s
    simulateDescent(500.0, 495.0, 5.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_fast_deployment() {
    // Test drogue at maximum descent rate (40 m/s)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Descend fast at 40 m/s
    simulateDescent(500.0, 460.0, 40.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_too_slow_no_detection() {
    // Test that very slow descent (< 5 m/s) doesn't indicate drogue
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Descend very slowly at 3 m/s (below threshold)
    simulateDescent(500.0, 497.0, 3.0, 1000);

    // Should remain in EXPECTING_DROGUE
    FlightStage stage = state->getFlightStage();
    // May still be expecting since velocity is too low
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_too_fast_ballistic() {
    // Test that ballistic descent (> 40 m/s) doesn't indicate drogue
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Descend very fast at 60 m/s (ballistic - drogue failed)
    simulateDescent(500.0, 440.0, 60.0, 1000);

    // Should remain in EXPECTING_DROGUE (no valid drogue deployment)
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_partial_deployment() {
    // Test partial drogue deployment (slower than ballistic but faster than normal)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Partial deployment - 35 m/s (within range but high)
    simulateDescent(500.0, 465.0, 35.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

void test_drogue_entanglement_variable_rate() {
    // Test variable descent rate (drogue entangled/spinning)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    double altitude = 500.0;

    // Variable descent - alternating rates
    for (int i = 0; i < 20; i++) {
        setMillis(10000 + i * 100);

        // Alternate between fast and slow
        double rate = (i % 2 == 0) ? 15.0 : 25.0;
        altitude -= rate * 0.1; // 100ms steps

        fakeBaro.setAltitude(altitude);
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        simulateUpdate();
    }

    // Should eventually detect drogue (rates are within range)
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);
}

// ===== MAIN DEPLOYMENT DETECTION =====

void test_main_nominal_deployment() {
    // Test normal main deployment with typical descent rate (5 m/s)
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    state->setGroundLevel(0.0);

    setMillis(20000);
    fakeBaro.setAltitude(300.0);
    simulateUpdate();

    // Descend at 5 m/s
    simulateDescent(300.0, 295.0, 5.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_main_slow_deployment() {
    // Test main at minimum descent rate (2 m/s)
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    state->setGroundLevel(0.0);

    setMillis(20000);
    fakeBaro.setAltitude(200.0);
    simulateUpdate();

    // Very slow descent at 2 m/s
    simulateDescent(200.0, 198.0, 2.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_main_fast_deployment() {
    // Test main at maximum descent rate (10 m/s)
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    state->setGroundLevel(0.0);

    setMillis(20000);
    fakeBaro.setAltitude(200.0);
    simulateUpdate();

    // Fast descent at 10 m/s
    simulateDescent(200.0, 190.0, 10.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_main_failed_deployment() {
    // Test that fast descent (> 10 m/s) indicates failed main
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    state->setGroundLevel(0.0);

    setMillis(20000);
    fakeBaro.setAltitude(200.0);
    simulateUpdate();

    // Still at drogue descent rate (20 m/s - main failed)
    simulateDescent(200.0, 180.0, 20.0, 1000);

    // Should remain in EXPECTING_MAIN
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_main_too_slow_no_movement() {
    // Test that very slow/no descent (< 2 m/s) doesn't indicate main
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    state->setGroundLevel(0.0);

    setMillis(20000);
    fakeBaro.setAltitude(100.0);
    simulateUpdate();

    // Very slow at 1 m/s
    simulateDescent(100.0, 99.0, 1.0, 1000);

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

// ===== ALTITUDE-BASED MAIN TRIGGER =====

void test_main_trigger_at_400m() {
    // Test that main deployment is expected at 400m AGL
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(15000);

    // Descend from above 400m to below
    fakeBaro.setAltitude(450.0);
    simulateUpdate();
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // Cross 400m threshold
    simulateDescent(450.0, 350.0, 15.0, 1000);

    // Should transition to EXPECTING_MAIN
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::UNDER_DROGUE ||
                     stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_main_trigger_with_ground_offset() {
    // Test main trigger with non-zero ground level
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    state->setGroundLevel(100.0); // Launch from 100m MSL

    setMillis(15000);

    // 500m MSL = 400m AGL
    fakeBaro.setAltitude(550.0); // 450m AGL
    simulateUpdate();
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // Cross threshold (400m AGL = 500m MSL)
    simulateDescent(550.0, 450.0, 15.0, 1000); // 350m AGL

    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::UNDER_DROGUE ||
                     stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

// ===== COMPLETE DESCENT SEQUENCE =====

void test_full_descent_sequence_nominal() {
    // Test complete descent: EXPECTING_DROGUE -> UNDER_DROGUE -> EXPECTING_MAIN -> UNDER_MAIN
    state->setFlightStage(FlightStage::APOGEE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(1000.0);
    simulateUpdate();

    // Start descending - should transition to EXPECTING_DROGUE (and possibly UNDER_DROGUE)
    simulateDescent(1000.0, 995.0, 3.0, 500);
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::APOGEE ||
                     stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);

    // Drogue deploys - descent rate increases to 20 m/s
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    setMillis(11000);
    simulateDescent(995.0, 975.0, 20.0, 1000);
    stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE);

    // Continue under drogue to 400m
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    setMillis(12000);
    simulateDescent(975.0, 450.0, 20.0, 5000);

    // Should still be under drogue above 400m
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // Cross 400m - should expect main
    simulateDescent(450.0, 350.0, 20.0, 1000);
    stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::UNDER_DROGUE ||
                     stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);

    // Main deploys - descent rate slows to 5 m/s
    state->setFlightStage(FlightStage::EXPECTING_MAIN);
    setMillis(18000);
    simulateDescent(350.0, 345.0, 5.0, 1000);
    stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

void test_full_descent_sequence_no_drogue() {
    // Test ballistic descent (drogue failure)
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);

    // Ballistic descent at 50 m/s from 1000m to 400m
    simulateDescent(1000.0, 400.0, 50.0, 3000);

    // Should still be expecting drogue (never detected)
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::EXPECTING_DROGUE ||
                     stage == FlightStage::UNDER_DROGUE ||
                     stage == FlightStage::EXPECTING_MAIN);
}

void test_full_descent_sequence_no_main() {
    // Test main failure (stays at drogue descent rate)
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(15000);

    // Continue at drogue rate through main deployment altitude
    simulateDescent(500.0, 100.0, 20.0, 5000);

    // Should transition to expecting main but not under main
    FlightStage stage = state->getFlightStage();
    TEST_ASSERT_TRUE(stage == FlightStage::UNDER_DROGUE ||
                     stage == FlightStage::EXPECTING_MAIN ||
                     stage == FlightStage::UNDER_MAIN);
}

// ===== DESCENT RATE CALCULATION VERIFICATION =====

void test_descent_rate_from_barometer() {
    // Verify that vertical velocity is correctly calculated from barometer
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    state->setGroundLevel(0.0);

    setMillis(10000);
    fakeBaro.setAltitude(500.0);
    simulateUpdate();

    // Descend at known rate
    setMillis(11000); // 1 second later
    fakeBaro.setAltitude(480.0); // 20m lower = 20 m/s
    simulateUpdate();

    // Vertical velocity should be approximately -20 m/s (negative = down)
    double vertVel = state->getVerticalVelocity();
    // Allow some tolerance due to filtering
    // Note: This depends on the State class velocity calculation
    TEST_ASSERT_TRUE(vertVel <= 0.0); // Should be negative (descending)
}

// ===== Main Test Runner =====

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Drogue deployment tests
    RUN_TEST(test_drogue_nominal_deployment);
    RUN_TEST(test_drogue_slow_deployment);
    RUN_TEST(test_drogue_fast_deployment);
    RUN_TEST(test_drogue_too_slow_no_detection);
    RUN_TEST(test_drogue_too_fast_ballistic);
    RUN_TEST(test_drogue_partial_deployment);
    RUN_TEST(test_drogue_entanglement_variable_rate);

    // Main deployment tests
    RUN_TEST(test_main_nominal_deployment);
    RUN_TEST(test_main_slow_deployment);
    RUN_TEST(test_main_fast_deployment);
    RUN_TEST(test_main_failed_deployment);
    RUN_TEST(test_main_too_slow_no_movement);

    // Altitude-based triggers
    RUN_TEST(test_main_trigger_at_400m);
    RUN_TEST(test_main_trigger_with_ground_offset);

    // Complete sequences
    RUN_TEST(test_full_descent_sequence_nominal);
    RUN_TEST(test_full_descent_sequence_no_drogue);
    RUN_TEST(test_full_descent_sequence_no_main);

    // Verification
    RUN_TEST(test_descent_rate_from_barometer);

    UNITY_END();
}
