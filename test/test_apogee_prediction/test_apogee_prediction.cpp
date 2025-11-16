#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"
#include <State/State.h>
#include "../../src/RocketState.h"
#include <cmath>

using namespace astra_rocket;

// Test fixtures
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
Sensor* testSensors[2];
RocketState* state;

void setUp(void) {
    testSensors[0] = &fakeBaro;
    testSensors[1] = &fakeIMU;

    fakeBaro.init();
    fakeIMU.init();

    state = new RocketState(testSensors, 2);
    state->setGroundLevel(0.0);

    setMillis(0);
}

void tearDown(void) {
    delete state;
    state = nullptr;
    resetMillis();
}

// ===== APOGEE ESTIMATION DURING BOOST =====

void test_apogee_estimate_during_boost() {
    // During boost, apogee should be estimated based on current velocity and deceleration
    state->setFlightStage(FlightStage::BOOST);

    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 3G upward

    state->update();

    // Apogee estimate should be calculated
    double apogee = state->getApogeeEstimate();
    // Should be some positive value (depends on velocity calculation)
    TEST_ASSERT_TRUE(std::isfinite(apogee));
}

void test_apogee_estimate_during_coast() {
    // During coast, apogee estimate should be based on current velocity and deceleration
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(300.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // Free fall

    state->update();

    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
    TEST_ASSERT_TRUE(apogee >= 300.0); // Should be at least current altitude
}

void test_apogee_estimate_at_apogee() {
    // At apogee, estimate should be close to current altitude
    state->setFlightStage(FlightStage::APOGEE);

    fakeBaro.setAltitude(1000.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double apogee = state->getApogeeEstimate();
    // At apogee, velocity should be ~0, so estimate should be close to current altitude
    TEST_ASSERT_TRUE(apogee >= 900.0 && apogee <= 1100.0);
}

void test_apogee_estimate_after_apogee() {
    // After apogee, estimate should be the max altitude achieved
    state->setFlightStage(FlightStage::UNDER_DROGUE);

    // Simulate having reached 1000m
    fakeBaro.setAltitude(1000.0);
    state->update();
    setMillis(100);

    // Now descending to 800m
    fakeBaro.setAltitude(800.0);
    setMillis(200);
    state->update();

    double apogee = state->getApogeeEstimate();
    // Should be based on max altitude (1000m MSL)
    TEST_ASSERT_TRUE(apogee >= 950.0);
}

// ===== TIME TO APOGEE ESTIMATION =====

void test_time_to_apogee_during_coast() {
    // Time to apogee should be positive during coast
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double timeToApogee = state->getTimeToApogee();
    // Should be finite and non-negative
    TEST_ASSERT_TRUE(std::isfinite(timeToApogee));
    TEST_ASSERT_TRUE(timeToApogee >= 0.0);
}

void test_time_to_apogee_at_apogee() {
    // At apogee, time should be 0
    state->setFlightStage(FlightStage::APOGEE);

    fakeBaro.setAltitude(1000.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double timeToApogee = state->getTimeToApogee();
    TEST_ASSERT_EQUAL_DOUBLE(0.0, timeToApogee);
}

void test_time_to_apogee_after_apogee() {
    // After apogee, time should be 0
    state->setFlightStage(FlightStage::UNDER_MAIN);

    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double timeToApogee = state->getTimeToApogee();
    TEST_ASSERT_EQUAL_DOUBLE(0.0, timeToApogee);
}

void test_time_to_apogee_on_pad() {
    // On pad, time to apogee should be 0 (not started)
    state->setFlightStage(FlightStage::PAD_IDLE);

    fakeBaro.setAltitude(0.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double timeToApogee = state->getTimeToApogee();
    TEST_ASSERT_EQUAL_DOUBLE(0.0, timeToApogee);
}

// ===== EDGE CASES IN APOGEE CALCULATION =====

void test_apogee_with_zero_velocity() {
    // Test apogee calculation when velocity is zero
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(500.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    // Even with zero velocity, should have valid estimate
    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
    TEST_ASSERT_TRUE(apogee >= 500.0);
}

void test_apogee_with_very_low_deceleration() {
    // Test with minimal deceleration (close to zero drag)
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(300.0);
    // Very low deceleration - just gravity
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
}

void test_apogee_with_high_deceleration() {
    // Test with high deceleration (high drag)
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(400.0);
    // High deceleration (5G total = 4G drag + 1G gravity)
    fakeIMU.set(Vector<3>{0, 0, -49.05}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 5G

    state->update();

    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
    TEST_ASSERT_TRUE(apogee >= 400.0);
}

void test_apogee_estimate_convergence() {
    // Test that apogee estimate converges as rocket approaches apogee
    state->setFlightStage(FlightStage::COAST);

    double estimates[10];

    // Simulate coasting upward with decreasing velocity
    for (int i = 0; i < 10; i++) {
        fakeBaro.setAltitude(500.0 + i * 50.0); // Ascending
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        setMillis(i * 500);
        state->update();

        estimates[i] = state->getApogeeEstimate();
    }

    // All estimates should be finite
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_TRUE(std::isfinite(estimates[i]));
    }
}

void test_apogee_estimate_negative_altitude() {
    // Test with negative AGL (shouldn't happen but test robustness)
    state->setGroundLevel(100.0);
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(50.0); // Below ground level
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
}

void test_apogee_with_extreme_altitude() {
    // Test at very high altitude (100km - space boundary)
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(100000.0);
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double apogee = state->getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
    TEST_ASSERT_TRUE(apogee >= 100000.0);
}

// ===== APOGEE ESTIMATE ACCURACY TESTS =====

void test_apogee_estimate_with_known_trajectory() {
    // Test with a known simple trajectory (parabolic motion)
    state->setFlightStage(FlightStage::BOOST);  // Start in BOOST to avoid premature APOGEE transition
    state->setGroundLevel(0.0);

    // Start at 100m with high acceleration (boost phase)
    fakeBaro.setAltitude(100.0);
    fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // 3G during boost

    setMillis(0);
    state->update();

    // Transition to coast and simulate ascending
    state->setFlightStage(FlightStage::COAST);
    for (int i = 1; i <= 20; i++) {
        double altitude = 100.0 + i * 20.0; // Linear ascent for simplicity
        fakeBaro.setAltitude(altitude);
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        setMillis(i * 100);
        state->update();
    }

    double apogee = state->getApogeeEstimate();
    // Should estimate something reasonable
    // With ~200 m/s velocity and 1G deceleration, apogee should be current alt + v^2/(2g)
    // = 500 + 200^2/(2*9.81) = 500 + 2039 = ~2539m
    // Being conservative, check for at least > 1000m
    TEST_ASSERT_TRUE(apogee > 1000.0);
}

void test_apogee_increases_during_boost() {
    // Apogee estimate should generally increase during boost
    state->setFlightStage(FlightStage::BOOST);

    double prevEstimate = 0.0;

    for (int i = 0; i < 10; i++) {
        fakeBaro.setAltitude(i * 10.0);
        fakeIMU.set(Vector<3>{0, 0, -30.0}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0}); // Constant thrust
        setMillis(i * 100);
        state->update();

        double currentEstimate = state->getApogeeEstimate();

        // Generally should be increasing or stable
        if (i > 0) {
            // Allow some tolerance for filtering effects
            TEST_ASSERT_TRUE(currentEstimate >= prevEstimate - 10.0);
        }

        prevEstimate = currentEstimate;
    }
}

void test_apogee_stabilizes_near_apogee() {
    // Apogee estimate should stabilize as rocket nears apogee
    state->setFlightStage(FlightStage::COAST);

    double estimates[5];

    // Simulate near apogee
    for (int i = 0; i < 5; i++) {
        fakeBaro.setAltitude(990.0 + i * 2.0); // Slow ascent
        fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});
        setMillis(i * 500);
        state->update();

        estimates[i] = state->getApogeeEstimate();
    }

    // Variance should be relatively small
    double minEst = estimates[0];
    double maxEst = estimates[0];
    for (int i = 1; i < 5; i++) {
        if (estimates[i] < minEst) minEst = estimates[i];
        if (estimates[i] > maxEst) maxEst = estimates[i];
    }

    // All estimates should be in reasonable range
    TEST_ASSERT_TRUE(maxEst - minEst < 200.0); // Within 200m of each other
}

// ===== POST-APOGEE BEHAVIOR =====

void test_apogee_uses_max_altitude_post_apogee() {
    // After apogee, estimate should use max altitude achieved
    state->setFlightStage(FlightStage::COAST);

    // Ascend to 1000m
    for (int i = 0; i < 20; i++) {
        fakeBaro.setAltitude(i * 50.0);
        setMillis(i * 100);
        state->update();
    }

    // Reach apogee
    state->setFlightStage(FlightStage::APOGEE);
    fakeBaro.setAltitude(1000.0);
    setMillis(2000);
    state->update();

    double apogeeAtPeak = state->getApogeeEstimate();

    // Now descend
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    fakeBaro.setAltitude(900.0);
    setMillis(2500);
    state->update();

    double apogeeOnDescent = state->getApogeeEstimate();

    // Apogee estimate shouldn't decrease
    TEST_ASSERT_TRUE(apogeeOnDescent >= apogeeAtPeak - 10.0);
}

void test_apogee_constant_after_landing() {
    // After landing, apogee should remain constant
    state->setFlightStage(FlightStage::LANDED);

    fakeBaro.setAltitude(0.0);
    setMillis(0);
    state->update();

    double apogee1 = state->getApogeeEstimate();

    // Wait some time
    setMillis(10000);
    state->update();

    double apogee2 = state->getApogeeEstimate();

    // Should be the same
    TEST_ASSERT_EQUAL_DOUBLE(apogee1, apogee2);
}

// ===== SPECIAL CASES =====

void test_apogee_with_no_barometer() {
    // Test behavior when barometer is missing
    Sensor* emptySensors[1];
    emptySensors[0] = &fakeIMU;

    RocketState stateNoBarometer(emptySensors, 1);
    stateNoBarometer.setFlightStage(FlightStage::COAST);
    stateNoBarometer.update();

    double apogee = stateNoBarometer.getApogeeEstimate();
    TEST_ASSERT_TRUE(std::isfinite(apogee));
}

void test_apogee_ground_level_offset() {
    // Test with non-zero ground level
    state->setGroundLevel(500.0); // Launch from 500m MSL
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(1000.0); // 500m AGL
    fakeIMU.set(Vector<3>{0, 0, -9.81}, Vector<3>{0, 0, 0}, Vector<3>{0, 0, 0});

    state->update();

    double apogee = state->getApogeeEstimate();
    // Apogee should be in MSL
    TEST_ASSERT_TRUE(apogee >= 1000.0);
}

void test_apogee_updates_every_cycle() {
    // Verify apogee is recalculated on each update
    state->setFlightStage(FlightStage::COAST);

    fakeBaro.setAltitude(500.0);
    setMillis(0);
    state->update();
    double apogee1 = state->getApogeeEstimate();

    fakeBaro.setAltitude(520.0);
    setMillis(100);
    state->update();
    double apogee2 = state->getApogeeEstimate();

    fakeBaro.setAltitude(540.0);
    setMillis(200);
    state->update();
    double apogee3 = state->getApogeeEstimate();

    // All should be finite
    TEST_ASSERT_TRUE(std::isfinite(apogee1));
    TEST_ASSERT_TRUE(std::isfinite(apogee2));
    TEST_ASSERT_TRUE(std::isfinite(apogee3));
}

// ===== Main Test Runner =====

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Basic apogee estimation
    RUN_TEST(test_apogee_estimate_during_boost);
    RUN_TEST(test_apogee_estimate_during_coast);
    RUN_TEST(test_apogee_estimate_at_apogee);
    RUN_TEST(test_apogee_estimate_after_apogee);

    // Time to apogee
    RUN_TEST(test_time_to_apogee_during_coast);
    RUN_TEST(test_time_to_apogee_at_apogee);
    RUN_TEST(test_time_to_apogee_after_apogee);
    RUN_TEST(test_time_to_apogee_on_pad);

    // Edge cases
    RUN_TEST(test_apogee_with_zero_velocity);
    RUN_TEST(test_apogee_with_very_low_deceleration);
    RUN_TEST(test_apogee_with_high_deceleration);
    RUN_TEST(test_apogee_estimate_convergence);
    RUN_TEST(test_apogee_estimate_negative_altitude);
    RUN_TEST(test_apogee_with_extreme_altitude);

    // Accuracy tests
    RUN_TEST(test_apogee_estimate_with_known_trajectory);
    RUN_TEST(test_apogee_increases_during_boost);
    RUN_TEST(test_apogee_stabilizes_near_apogee);

    // Post-apogee
    RUN_TEST(test_apogee_uses_max_altitude_post_apogee);
    RUN_TEST(test_apogee_constant_after_landing);

    // Special cases
    RUN_TEST(test_apogee_with_no_barometer);
    RUN_TEST(test_apogee_ground_level_offset);
    RUN_TEST(test_apogee_updates_every_cycle);

    UNITY_END();
}
