#include <unity.h>
#include <NativeTestHelper.h>
#include "../mocks/UnitTestSensors.h"
#include <Sensors/SensorManager/SensorManager.h>
#include "../../src/RocketState.h"
#include "../mocks/MockLinearKalmanFilter.h"
#include "../mocks/MockMahony.h"

using namespace astra_rocket;
using namespace astra;
using namespace astra_mocks;

// ===== Test Fixtures =====
FakeBarometer fakeBaro;
FakeIMU fakeIMU;
SensorManager* sensorManager;
MockLinearKalmanFilter* kalmanFilter;
MockMahony* orientationFilter;
RocketState* state;

void setUp(void) {
    // Set up minimal sensors just to keep State class happy
    // NOTE: We won't use these sensors for testing - we directly manipulate KF state
    fakeBaro.init();
    fakeIMU.init();

    sensorManager = new SensorManager();
    sensorManager->setAccelSource(fakeIMU.getAccelSensor());
    sensorManager->setGyroSource(fakeIMU.getGyroSensor());
    sensorManager->setBaroSource(&fakeBaro);
    sensorManager->begin();

    // Create mock filters
    // MockLinearKalmanFilter(measurementSize, controlSize, stateSize)
    // State is 9D: [px, py, pz, vx, vy, vz, ax, ay, az]
    // Measurements are 6D: [px, py, pz, ax, ay, az]
    kalmanFilter = new MockLinearKalmanFilter(6, 0, 9);
    orientationFilter = new MockMahony();

    // Create state with filters and sensor manager
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();

    state->setGroundLevel(0.0);
    setMillis(0);
}

void tearDown(void) {
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

// ===== Helper Functions =====

// Helper to set KF state and update RocketState
void setStateAndUpdate(double altitude, double velocity, double accelZ, unsigned long timeMs) {
    // DefaultKalmanFilter uses 6-state model: [px, py, pz, vx, vy, vz]
    // Acceleration is NOT part of the state - it comes from orientation filter
    Matrix kfState = kalmanFilter->getState();
    kfState(2, 0) = altitude;   // pz (altitude AGL)
    kfState(5, 0) = velocity;   // vz (vertical velocity)
    kalmanFilter->setState(kfState);

    // Mock the orientation filter to return the desired acceleration
    // This simulates what the real Mahony filter would provide
    orientationFilter->setMockEarthAcceleration(Vector<3>(0, 0, accelZ));

    setMillis(timeMs);
    double timeSec = millis() / 1000.0;

    // Call both predictState and update to properly populate state variables
    // predictState reads from orientation filter and KF state
    // update does measurement update (but with mock KF it does nothing)
    state->predictState(timeSec);
    state->update();
}

// Overload without acceleration for cases where we don't care about accel
void setStateAndUpdate(double altitude, double velocity, unsigned long timeMs) {
    setStateAndUpdate(altitude, velocity, -9.81, timeMs);
}


// ================================================================================
// LIFTOFF DETECTION TESTS
// ================================================================================

void test_liftoff_nominal() {
    // Test normal liftoff detection with 4G sustained acceleration
    setStateAndUpdate(0.0, 0.0, -9.81, 0);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());

    // Apply 4.0G acceleration (39.24 m/s²) - well above 3.0G threshold
    // This starts the detection timer at 10ms
    setStateAndUpdate(0.0, 0.0, -39.24, 10);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage()); // Not yet - need 100ms

    // Sustain for > 100ms - should trigger
    // 10ms + 101ms = 111ms total, so detection triggers
    setStateAndUpdate(5.0, 10.0, -39.24, 111);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());
}

void test_liftoff_at_threshold() {
    // Test detection at 3.0G threshold (needs to be slightly above due to > comparison)
    setStateAndUpdate(0.0, 0.0, -9.81, 0);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());

    // Apply 3.01G at 10ms (starts timer) - slightly above 3.0G threshold
    setStateAndUpdate(0.0, 0.0, -29.53, 10);

    // Sustain for > 100ms (10ms + 101ms = 111ms)
    setStateAndUpdate(2.0, 5.0, -29.53, 111);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());
}

void test_liftoff_below_threshold() {
    // Test that 2.9G doesn't trigger liftoff
    setStateAndUpdate(0.0, 0.0, -9.81, 0);

    // Apply 2.9G (28.45 m/s²) - just below threshold
    setStateAndUpdate(0.0, 0.0, -28.45, 10);

    // Even if sustained for long time, should NOT trigger
    setStateAndUpdate(1.0, 3.0, -28.45, 500);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());
}

void test_liftoff_brief_spike() {
    // Test that brief acceleration spikes don't trigger liftoff
    setStateAndUpdate(0.0, 0.0, -9.81, 0);

    // Brief spike for only 80ms (below 100ms threshold)
    setStateAndUpdate(0.0, 0.0, -35.0, 10);
    setStateAndUpdate(0.1, 0.5, -35.0, 80);

    // Drop back to normal
    setStateAndUpdate(0.2, 1.0, -9.81, 100);

    // Should still be on pad
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());
}

void test_liftoff_timing_boundary() {
    // Test that exactly 100ms is sufficient
    setStateAndUpdate(0.0, 0.0, -9.81, 0);

    // Start high acceleration
    setStateAndUpdate(0.0, 0.0, -40.0, 1);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());

    // At 99ms - should not trigger yet
    setStateAndUpdate(1.0, 5.0, -40.0, 99);
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());

    // At 101ms - should trigger
    setStateAndUpdate(2.0, 10.0, -40.0, 102);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());
}


// ================================================================================
// BURNOUT DETECTION TESTS
// ================================================================================

void test_burnout_nominal() {
    // Test normal burnout detection with typical coast acceleration (1.0G)
    state->setFlightStage(FlightStage::BOOST);
    setMillis(1000);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // Drop to 1.0G (9.81 m/s²) - below 1.5G threshold (starts timer at 1010ms)
    setStateAndUpdate(102.0, 52.0, -9.81, 1010);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage()); // Not yet

    // Sustain for > 200ms - should trigger (1010ms + 201ms = 1211ms)
    setStateAndUpdate(110.0, 55.0, -9.81, 1211);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}

void test_burnout_at_threshold() {
    // Test burnout detection exactly at 1.5G threshold (14.715 m/s²)
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);

    // Set exactly 1.5G
    setStateAndUpdate(105.0, 52.0, -14.715, 1050);

    // Sustain for > 200ms
    setStateAndUpdate(115.0, 55.0, -14.715, 1260);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}

void test_burnout_strong_downward_acceleration() {
    // Strong downward acceleration still qualifies as burnout because the
    // signed vertical acceleration has already fallen below the threshold.
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);

    // This starts the burnout timer but should not transition immediately.
    setStateAndUpdate(105.0, 52.0, -15.7, 1050);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // Sustained low/negative vertical acceleration transitions to coast.
    setStateAndUpdate(120.0, 58.0, -15.7, 1500);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}

void test_burnout_timing_boundary() {
    // With signed vertical acceleration, this fixture is already below the
    // burnout threshold at 1000ms, so the debounce window starts there.
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // Still below the threshold.
    setStateAndUpdate(102.0, 51.0, -9.81, 1010);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // At 199ms after the first qualifying sample, it should not trigger yet.
    setStateAndUpdate(110.0, 54.0, -9.81, 1199);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // At 201ms it should trigger.
    setStateAndUpdate(112.0, 55.0, -9.81, 1201);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}

void test_burnout_fluctuating_negative_acceleration() {
    // Fluctuations that remain below the signed threshold should still
    // accumulate toward burnout.
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);

    // Fluctuate between high and low acceleration
    for (int i = 0; i < 10; i++) {
        double accel = (i % 2 == 0) ? -30.0 : -10.0;
        setStateAndUpdate(100.0 + i, 50.0 + i, accel, 1000 + i * 50);
    }

    // All samples are still below the signed threshold, so burnout should occur.
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}

void test_multiple_motor_burns() {
    // Test staged rocket - multiple boost phases
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);

    // First motor burnout
    setStateAndUpdate(110.0, 52.0, -9.81, 1100);
    setStateAndUpdate(120.0, 54.0, -9.81, 1320);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    // Second motor ignition (manual override - would be commanded in real system)
    // The manual stage change should clear stale burnout debounce state.
    state->setFlightStage(FlightStage::BOOST);
    setStateAndUpdate(130.0, 56.0, -35.0, 2000);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // Second burnout
    setStateAndUpdate(200.0, 70.0, -9.81, 2100);
    setStateAndUpdate(220.0, 72.0, -9.81, 2320);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}


// ================================================================================
// APOGEE DETECTION TESTS
// ================================================================================

void test_apogee_nominal() {
    // Apogee is only declared once descent is clearly underway.
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(500.0, 5.0, 5000);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    // Slightly negative velocity is still too close to zero.
    setStateAndUpdate(501.0, -0.1, 5050);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    // Clear descent beyond the threshold triggers apogee.
    setStateAndUpdate(500.8, -2.1, 5100);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());
}

void test_apogee_at_velocity_threshold() {
    // Apogee detection occurs at the exact descent threshold.
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(500.0, 2.5, 5000);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    setStateAndUpdate(501.0, -2.0, 5050);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());
}

void test_apogee_very_low_velocity_no_detection() {
    // Slow upward motion near zero is still not enough to declare apogee.
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(500.0, 3.0, 5000);

    // Still climbing slowly.
    setStateAndUpdate(501.0, 0.3, 5100);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    // Apogee arrives once the vehicle is descending decisively.
    setStateAndUpdate(500.5, -2.3, 5200);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());
}

void test_apogee_high_velocity_no_detection() {
    // Test that high upward velocity doesn't trigger apogee
    state->setFlightStage(FlightStage::COAST);
    setStateAndUpdate(500.0, 50.0, 5000);

    // Still high velocity
    setStateAndUpdate(520.0, 40.0, 5100);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
}


// ================================================================================
// APOGEE TO EXPECTING_DROGUE TRANSITION
// ================================================================================

void test_apogee_to_expecting_drogue_transition() {
    // Test transition when descending > 2 m/s
    state->setFlightStage(FlightStage::APOGEE);
    setStateAndUpdate(500.0, -0.1, 10000);

    // Should stay in APOGEE with slow descent
    setStateAndUpdate(499.0, -1.0, 10100);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());

    // Transition when descending faster than 2 m/s
    setStateAndUpdate(498.0, -2.1, 10200);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());
}


// ================================================================================
// DROGUE DEPLOYMENT DETECTION TESTS
// ================================================================================

void test_drogue_nominal_deployment() {
    // Test normal drogue deployment with typical descent rate (20 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // Set descent velocity in valid drogue range (5-40 m/s)
    setStateAndUpdate(500.0, -20.0, 10000);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());

    // Should stay in EXPECTING_DROGUE for just under 2 seconds
    setStateAndUpdate(460.0, -20.0, 11990);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());

    // After 2 seconds sustained, should transition to UNDER_DROGUE
    setStateAndUpdate(458.0, -20.0, 12010);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());
}

void test_drogue_at_min_boundary() {
    // Test drogue at minimum descent rate (just above 5 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // 5.1 m/s descent (needs to be > 5.0)
    setStateAndUpdate(500.0, -5.1, 10000);
    setStateAndUpdate(489.8, -5.1, 12010);

    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());
}

void test_drogue_at_max_boundary() {
    // Test drogue at maximum descent rate (just below 40 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // 39.9 m/s descent (needs to be < 40.0)
    setStateAndUpdate(500.0, -39.9, 10000);
    setStateAndUpdate(420.2, -39.9, 12010);

    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());
}

void test_drogue_too_slow_no_detection() {
    // Test that very slow descent (< 5 m/s) doesn't indicate drogue
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // 3 m/s - below minimum threshold
    setStateAndUpdate(500.0, -3.0, 10000);
    setStateAndUpdate(494.0, -3.0, 12100);

    // Should remain in EXPECTING_DROGUE
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());
}

void test_drogue_too_fast_ballistic() {
    // Test that ballistic descent (> 40 m/s) doesn't indicate drogue
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // 60 m/s - ballistic descent (drogue failed)
    setStateAndUpdate(500.0, -60.0, 10000);
    setStateAndUpdate(380.0, -60.0, 12100);

    // Should remain in EXPECTING_DROGUE
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());
}

void test_drogue_detection_timing_boundary() {
    // Test that exactly 2 seconds is required
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    setStateAndUpdate(500.0, -20.0, 10000);

    // At 1.99 seconds - should not trigger
    setStateAndUpdate(462.0, -20.0, 11990);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());

    // At 2.01 seconds - should trigger
    setStateAndUpdate(460.0, -20.0, 12010);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());
}

void test_drogue_velocity_fluctuation() {
    // Test that velocity fluctuations reset the detection timer
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_DROGUE);

    // Start with valid velocity
    setStateAndUpdate(500.0, -20.0, 10000);

    // Wait 1.5 seconds, then velocity goes out of range
    setStateAndUpdate(470.0, -20.0, 11500);
    setStateAndUpdate(468.0, -50.0, 11600); // Too fast!

    // Come back to valid range
    setStateAndUpdate(460.0, -20.0, 11700);

    // Should not trigger yet - timer was reset
    setStateAndUpdate(420.0, -20.0, 13500);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());

    // Need another 2 seconds from reset
    setStateAndUpdate(410.0, -20.0, 13710);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());
}


// ================================================================================
// MAIN DEPLOYMENT ALTITUDE TRIGGER TESTS
// ================================================================================

void test_main_trigger_at_400m() {
    // Test that main deployment is expected at 400m AGL
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    setStateAndUpdate(450.0, -15.0, 15000);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // Cross 400m threshold
    setStateAndUpdate(390.0, -15.0, 15500);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());
}

void test_main_trigger_altitude_boundary() {
    // Test boundary at exactly 400m
    state->setFlightStage(FlightStage::UNDER_DROGUE);

    // At 401m - should not trigger
    setStateAndUpdate(401.0, -15.0, 15000);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // At 399m - should trigger
    setStateAndUpdate(399.0, -15.0, 15100);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());
}

void test_main_trigger_exactly_400m() {
    // Test at exactly 400.0m (condition is <, so 400.0 won't trigger)
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    setStateAndUpdate(450.0, -15.0, 15000);

    // Set just below 400m
    setStateAndUpdate(399.9, -15.0, 15100);

    // Should trigger below 400m
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());
}


// ================================================================================
// MAIN DEPLOYMENT DETECTION TESTS
// ================================================================================

void test_main_nominal_deployment() {
    // Test normal main deployment with typical descent rate (5 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    // Set descent velocity in valid main range (2-10 m/s)
    setStateAndUpdate(300.0, -5.0, 20000);
    setStateAndUpdate(290.0, -5.0, 22010);

    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}

void test_main_at_min_boundary() {
    // Test main at minimum descent rate (just above 2 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    // 2.1 m/s descent (needs to be > 2.0)
    setStateAndUpdate(200.0, -2.1, 20000);
    setStateAndUpdate(195.8, -2.1, 22010);

    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}

void test_main_at_max_boundary() {
    // Test main at maximum descent rate (just below 10 m/s)
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    // 9.9 m/s descent (needs to be < 10.0)
    setStateAndUpdate(200.0, -9.9, 20000);
    setStateAndUpdate(180.2, -9.9, 22010);

    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}

void test_main_failed_deployment() {
    // Test that fast descent (> 10 m/s) indicates failed main
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    // Still at drogue descent rate (20 m/s - main failed)
    setStateAndUpdate(200.0, -20.0, 20000);
    setStateAndUpdate(160.0, -20.0, 22100);

    // Should remain in EXPECTING_MAIN
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());
}

void test_main_too_slow_no_movement() {
    // Test that very slow descent (< 2 m/s) doesn't indicate main
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    // 1 m/s - below minimum threshold
    setStateAndUpdate(100.0, -1.0, 20000);
    setStateAndUpdate(98.0, -1.0, 22100);

    // Should remain in EXPECTING_MAIN
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());
}

void test_main_detection_timing_boundary() {
    // Test that exactly 2 seconds is required
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    setStateAndUpdate(200.0, -5.0, 20000);

    // At 1.99 seconds - should not trigger
    setStateAndUpdate(190.0, -5.0, 21990);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());

    // At 2.01 seconds - should trigger
    setStateAndUpdate(189.5, -5.0, 22010);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}

void test_main_velocity_fluctuation() {
    // Test that velocity fluctuations reset the detection timer
    setStateAndUpdate(1000.0, 0.0, 0);
    state->setFlightStage(FlightStage::EXPECTING_MAIN);

    setStateAndUpdate(200.0, -5.0, 20000);

    // Wait 1.5 seconds, then velocity goes out of range
    setStateAndUpdate(192.5, -5.0, 21500);
    setStateAndUpdate(192.0, -15.0, 21600); // Too fast!

    // Come back to valid range
    setStateAndUpdate(190.0, -5.0, 21700);

    // Should not trigger yet - timer was reset
    setStateAndUpdate(180.0, -5.0, 23500);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());

    // Need another 2 seconds from reset
    setStateAndUpdate(178.0, -5.0, 23710);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}


// ================================================================================
// LANDING DETECTION TESTS
// ================================================================================

void test_landing_nominal() {
    // Test normal landing detection with low altitude and velocity
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(5.0, -0.5, 30000);

    // Should stay in UNDER_MAIN for < 3 seconds
    setStateAndUpdate(4.0, -0.3, 32900);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());

    // After 3 seconds, should land
    setStateAndUpdate(3.5, -0.2, 33010);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_landing_timing_boundary() {
    // Test that exactly 3000ms is required for landing
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(5.0, -0.5, 30000);

    // At 2999ms - should not trigger
    setStateAndUpdate(3.5, -0.3, 32999);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());

    // At 3001ms - should trigger
    setStateAndUpdate(3.5, -0.2, 33001);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_landing_at_velocity_threshold() {
    // Test landing just below 1.0 m/s threshold
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(5.0, -0.9, 30000);

    // Wait 3+ seconds at low velocity
    setStateAndUpdate(2.3, -0.9, 33100);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_landing_at_altitude_threshold() {
    // Test landing at exactly 10m AGL threshold
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(10.0, -0.5, 30000);

    // Wait 3+ seconds at threshold altitude
    setStateAndUpdate(9.0, -0.3, 33100);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_landing_bouncing() {
    // Test that bouncing resets the landing timer
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(5.0, -0.5, 30000);

    // Wait 1 second
    setStateAndUpdate(4.5, -0.5, 31000);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());

    // Bounce up briefly
    setStateAndUpdate(5.0, 2.0, 31500);

    // Back down
    setStateAndUpdate(4.0, -0.5, 32000);

    // Should still be under main - bounce reset the timer
    setStateAndUpdate(3.5, -0.3, 34500);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());

    // Need full 3 seconds from reset
    setStateAndUpdate(3.0, -0.2, 35010);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_landing_high_altitude_no_detection() {
    // Test that high altitude with moderate velocity prevents landing
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(50.0, -5.0, 30000);

    // High velocity prevents landing even after time passes
    setStateAndUpdate(35.0, -5.0, 33100);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}

void test_landing_high_velocity_no_detection() {
    // Test that high velocity prevents landing even at low altitude
    state->setFlightStage(FlightStage::UNDER_MAIN);
    setStateAndUpdate(5.0, -5.0, 30000);

    // Even at low altitude, high velocity prevents landing
    setStateAndUpdate(3.0, -4.0, 33100);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());
}


// ================================================================================
// TERMINAL STATE TESTS
// ================================================================================

void test_landed_is_terminal() {
    // Verify that landed state never transitions
    state->setFlightStage(FlightStage::LANDED);
    setStateAndUpdate(0.0, 0.0, 40000);

    // Try various extreme conditions
    setStateAndUpdate(100.0, 50.0, -50.0, 40100);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());

    setStateAndUpdate(500.0, 100.0, -100.0, 40200);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());

    // Should always remain landed
    setStateAndUpdate(0.0, 0.0, 50000);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}


// ================================================================================
// FULL FLIGHT SEQUENCE TESTS
// ================================================================================

void test_complete_nominal_flight_sequence() {
    // Test a complete flight from pad to landing

    // 1. PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());

    // 2. LIFTOFF -> BOOST
    // Need to start at rest first, then apply high acceleration
    setStateAndUpdate(0.0, 0.0, -9.81, 0);
    setStateAndUpdate(0.0, 0.0, -40.0, 10);  // Start high accel timer at 10ms
    setStateAndUpdate(5.0, 10.0, -40.0, 120); // Trigger at 120ms (>100ms sustained)
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    // 3. BURNOUT -> COAST
    setStateAndUpdate(200.0, 100.0, -9.81, 5000);
    setStateAndUpdate(250.0, 80.0, -9.81, 5220);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    // 4. APOGEE
    setStateAndUpdate(500.0, 1.0, 10000);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());
    setStateAndUpdate(499.5, -2.2, 10100);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());

    // 5. EXPECTING_DROGUE (needs velocity < -2.0 m/s)
    setStateAndUpdate(498.0, -2.5, 10200);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_DROGUE, state->getFlightStage());

    // 6. UNDER_DROGUE
    setStateAndUpdate(450.0, -20.0, 12000);
    setStateAndUpdate(410.0, -20.0, 14020);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_DROGUE, state->getFlightStage());

    // 7. EXPECTING_MAIN (altitude trigger)
    setStateAndUpdate(390.0, -20.0, 15000);
    TEST_ASSERT_EQUAL(FlightStage::EXPECTING_MAIN, state->getFlightStage());

    // 8. UNDER_MAIN
    setStateAndUpdate(350.0, -5.0, 16000);
    setStateAndUpdate(340.0, -5.0, 18020);
    TEST_ASSERT_EQUAL(FlightStage::UNDER_MAIN, state->getFlightStage());

    // 9. LANDED
    setStateAndUpdate(5.0, -0.5, 30000);
    setStateAndUpdate(3.0, -0.3, 33100);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}


// ================================================================================
// UTILITY TESTS
// ================================================================================

void test_manual_stage_setting() {
    // Test that we can manually set stages for testing/override
    state->setFlightStage(FlightStage::BOOST);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    state->setFlightStage(FlightStage::COAST);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    state->setFlightStage(FlightStage::APOGEE);
    TEST_ASSERT_EQUAL(FlightStage::APOGEE, state->getFlightStage());

    state->setFlightStage(FlightStage::LANDED);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

void test_time_in_stage_tracking() {
    // Test that time in stage is tracked correctly
    // First update to set initial time
    setStateAndUpdate(100.0, 50.0, -30.0, 1000);

    // Set stage - this records stageStartTime at current time (1.0s)
    state->setFlightStage(FlightStage::BOOST);

    // Advance time by 2 seconds (1000ms -> 3000ms)
    setStateAndUpdate(120.0, 60.0, -30.0, 3000);

    // Should be approximately 2.0 seconds in stage
    double timeInStage = state->getTimeInStage();
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 2.0, timeInStage);
}

void test_initial_state() {
    // Test that initial state is PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, state->getFlightStage());
}


// ================================================================================
// MAIN TEST RUNNER
// ================================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Liftoff tests
    RUN_TEST(test_liftoff_nominal);
    RUN_TEST(test_liftoff_at_threshold);
    RUN_TEST(test_liftoff_below_threshold);
    RUN_TEST(test_liftoff_brief_spike);
    RUN_TEST(test_liftoff_timing_boundary);

    // Burnout tests
    RUN_TEST(test_burnout_nominal);
    RUN_TEST(test_burnout_at_threshold);
    RUN_TEST(test_burnout_strong_downward_acceleration);
    RUN_TEST(test_burnout_timing_boundary);
    RUN_TEST(test_burnout_fluctuating_negative_acceleration);
    RUN_TEST(test_multiple_motor_burns);

    // Apogee tests
    RUN_TEST(test_apogee_nominal);
    RUN_TEST(test_apogee_at_velocity_threshold);
    RUN_TEST(test_apogee_very_low_velocity_no_detection);
    RUN_TEST(test_apogee_high_velocity_no_detection);
    RUN_TEST(test_apogee_to_expecting_drogue_transition);

    // Drogue deployment tests
    RUN_TEST(test_drogue_nominal_deployment);
    RUN_TEST(test_drogue_at_min_boundary);
    RUN_TEST(test_drogue_at_max_boundary);
    RUN_TEST(test_drogue_too_slow_no_detection);
    RUN_TEST(test_drogue_too_fast_ballistic);
    RUN_TEST(test_drogue_detection_timing_boundary);
    RUN_TEST(test_drogue_velocity_fluctuation);

    // Main deployment altitude trigger tests
    RUN_TEST(test_main_trigger_at_400m);
    RUN_TEST(test_main_trigger_altitude_boundary);
    RUN_TEST(test_main_trigger_exactly_400m);

    // Main deployment detection tests
    RUN_TEST(test_main_nominal_deployment);
    RUN_TEST(test_main_at_min_boundary);
    RUN_TEST(test_main_at_max_boundary);
    RUN_TEST(test_main_failed_deployment);
    RUN_TEST(test_main_too_slow_no_movement);
    RUN_TEST(test_main_detection_timing_boundary);
    RUN_TEST(test_main_velocity_fluctuation);

    // Landing tests
    RUN_TEST(test_landing_nominal);
    RUN_TEST(test_landing_timing_boundary);
    RUN_TEST(test_landing_at_velocity_threshold);
    RUN_TEST(test_landing_at_altitude_threshold);
    RUN_TEST(test_landing_bouncing);
    RUN_TEST(test_landing_high_altitude_no_detection);
    RUN_TEST(test_landing_high_velocity_no_detection);

    // Terminal state tests
    RUN_TEST(test_landed_is_terminal);

    // Full sequence tests
    RUN_TEST(test_complete_nominal_flight_sequence);

    // Utility tests
    RUN_TEST(test_manual_stage_setting);
    RUN_TEST(test_time_in_stage_tracking);
    RUN_TEST(test_initial_state);

    UNITY_END();
}
