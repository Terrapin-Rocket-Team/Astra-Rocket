#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <State/State.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Filters/Filter.h>
#include <Filters/Mahony.h>
#include "../../src/RocketState.h"
#include "../../src/RocketKF.h"
#include <cmath>

using namespace astra_rocket;
using namespace astra;

// Mock sensors for testing
FakeBarometer fakeBaro;
FakeAccel fakeAccel;
FakeGyro fakeGyro;
SensorManager* sensorManager;
RocketKF* kalmanFilter;
MahonyAHRS* orientationFilter;
RocketState* state;

void setUp(void)
{
    // Initialize sensors
    fakeBaro.init();
    fakeAccel.init();
    fakeGyro.init();

    // Create sensor manager
    sensorManager = new SensorManager();
    sensorManager->setPrimaryAccel(&fakeAccel);
    sensorManager->setPrimaryGyro(&fakeGyro);
    sensorManager->setPrimaryBaro(&fakeBaro);
    sensorManager->begin();

    // Create filters
    kalmanFilter = new RocketKF();
    orientationFilter = new MahonyAHRS();

    // Create RocketState with filters
    state = new RocketState(kalmanFilter, orientationFilter);
    state->withSensorManager(sensorManager);
    state->begin();
    state->setGroundLevel(0.0);
    LOGD("HELLO THERE");
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
    // Advance time
    unsigned long currentMillis = millis();
    setMillis(currentMillis + (unsigned long)(dt * 1000.0));

    // Update sensor manager
    sensorManager->update();

    // Update state (expects time in seconds, not milliseconds)
    double newTime = millis() / 1000.0;
    state->update(newTime);
}

// Helper to simulate realistic altitude change with matching acceleration
// Simulates a trajectory with acceleration, coast, and deceleration phases
// ENU frame: Z is up, gravity is -9.81
void simulateAltitudeChange(double startAlt, double endAlt, double duration, int steps = 50) {
    double deltaAlt = endAlt - startAlt;
    double dt = duration / steps;

    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;  // 0 to 1

        // Use smoothstep for realistic trajectory: accelerate → coast → decelerate
        // s(t) = 3t² - 2t³
        double s = 3 * t * t - 2 * t * t * t;
        double altitude = startAlt + deltaAlt * s;

        // Velocity: ds/dt = 6t - 6t²
        double velocity = (6 * t - 6 * t * t) * deltaAlt / duration;

        // Inertial acceleration in earth frame (ENU): d²s/dt² = 6 - 12t
        double inertial_accel_z = (6 - 12 * t) * deltaAlt / (duration * duration);

        // Accelerometer measures specific force (reaction force)
        // specific_force = -inertial_accel - gravity
        // In ENU: at rest, inertial_accel = 0, gravity = -9.81, so specific_force = 0 - (-9.81) = -9.81
        // When accelerating up: specific_force_z = -inertial_accel_z + 9.81
        fakeBaro.setAltitude(altitude);
        fakeAccel.set(Vector<3>{0, 0, -inertial_accel_z - 9.81});

        simulateUpdate(dt);
    }
}

// ===== BASIC INITIALIZATION TESTS =====

void test_rocket_state_initialization() {
    // Test that RocketState can be instantiated
    RocketKF kf;
    MahonyAHRS ahrs;
    RocketState emptyState(&kf, &ahrs);

    // Verify initial flight stage is PAD_IDLE
    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, emptyState.getFlightStage());

    // Verify initial values are reasonable
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getTimeInStage());
    TEST_ASSERT_EQUAL_DOUBLE(0.0, emptyState.getAltitudeAGL());
}

void test_ground_level_setting() {
    // Set ground level altitude
    double groundAlt = 100.0; // 100m MSL
    state->setGroundLevel(groundAlt);

    // Set barometer to ground level
    fakeBaro.setAltitude(100.0);
    simulateUpdate();

    // AGL should be 0 when at ground level
    TEST_ASSERT_EQUAL_DOUBLE(0.0, state->getAltitudeAGL());
}

void test_flight_stage_manual_setting() {
    // Manually set flight stage
    state->setFlightStage(FlightStage::BOOST);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, state->getFlightStage());

    state->setFlightStage(FlightStage::COAST);
    TEST_ASSERT_EQUAL(FlightStage::COAST, state->getFlightStage());

    state->setFlightStage(FlightStage::LANDED);
    TEST_ASSERT_EQUAL(FlightStage::LANDED, state->getFlightStage());
}

// ===== GROUND LEVEL EDGE CASES =====

void test_ground_level_negative() {
    // Test ground level below sea level (e.g., Death Valley)
    state->setGroundLevel(-86.0); // Death Valley elevation

    fakeBaro.setAltitude(0.0); // Sea level
    // Give filter multiple updates to converge
    for (int i = 0; i < 5; i++) {
        simulateUpdate();
    }

    // AGL should be 86m above ground (allow small tolerance for filter convergence)
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 86.0, state->getAltitudeAGL());
}

void test_ground_level_high_altitude() {
    // Test ground level at high altitude (e.g., La Paz, Bolivia)
    state->setGroundLevel(3640.0); // ~12,000 ft

    fakeBaro.setAltitude(3640.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    // Give filter multiple updates to converge to initial altitude
    for (int i = 0; i < 10; i++) {
        simulateUpdate();
    }

    // AGL should be 0 at launch site
    TEST_ASSERT_DOUBLE_WITHIN(5.0, 0.0, state->getAltitudeAGL());

    // Simulate realistic ascent to 4000m MSL (360m AGL) over 15 seconds
    // Use more steps for better KF convergence
    simulateAltitudeChange(3640.0, 4000.0, 15.0, 200);

    // Add settling time at final altitude
    fakeBaro.setAltitude(4000.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    for (int i = 0; i < 50; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(30.0, 360.0, state->getAltitudeAGL());
}

void test_ground_level_not_set() {
    // Test behavior when ground level is never set (defaults to 0)
    fakeBaro.setAltitude(100.0);
    // Give filter multiple updates to converge
    for (int i = 0; i < 5; i++) {
        simulateUpdate();
    }

    // AGL should be altitude MSL when ground = 0
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 100.0, state->getAltitudeAGL());
}

void test_negative_altitude_agl() {
    // Test that AGL can go negative (sensor drift, landing below launch site)
    state->setGroundLevel(100.0);

    fakeBaro.setAltitude(95.0); // 5m below launch site
    // Give filter multiple updates to converge
    for (int i = 0; i < 5; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(0.5, -5.0, state->getAltitudeAGL());
}

// ===== SENSOR UPDATE TESTS =====

void test_altitude_updates() {
    // Test that altitude updates correctly
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    // Give filter time to converge
    for (int i = 0; i < 10; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(5.0, 100.0, state->getAltitudeAGL());

    // Simulate realistic climb from 100m to 200m over 8 seconds
    simulateAltitudeChange(100.0, 200.0, 8.0, 100);

    // Add settling time at final altitude
    fakeBaro.setAltitude(200.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    for (int i = 0; i < 50; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(15.0, 200.0, state->getAltitudeAGL());
}

// ===== ALTITUDE TRACKING TESTS =====

void test_altitude_ascent() {
    // Simulate realistic ascent to 1000m over 20 seconds
    simulateAltitudeChange(0.0, 1000.0, 20.0, 100);

    double apogeeAlt = state->getAltitudeAGL();
    TEST_ASSERT_TRUE(apogeeAlt >= 900.0 && apogeeAlt <= 1100.0);

    // Simulate realistic descent from 1000m to 500m over 15 seconds
    state->setFlightStage(FlightStage::UNDER_DROGUE);
    simulateAltitudeChange(1000.0, 500.0, 15.0, 75);

    double currentAlt = state->getAltitudeAGL();

    // Current altitude should be lower than apogee
    TEST_ASSERT_TRUE(currentAlt < apogeeAlt);
}

// ===== OFF-VERTICAL ANGLE EDGE CASES =====

void test_off_vertical_angle_vertical_flight() {
    // Test perfectly vertical flight
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -30.0}); // Straight up
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should be close to 0 degrees
    TEST_ASSERT_TRUE(angle < 5.0);
}

void test_off_vertical_angle_horizontal_flight() {
    // Test horizontal flight
    // Note: This test is problematic - orientation can't be inferred from single accel reading
    // without gyro data. Keeping for now but may need rework.
    fakeBaro.setAltitude(100.0);
    // Horizontal accel in X direction, still feeling gravity in -Z
    fakeAccel.set(Vector<3>{30.0, 0, -9.81}); // Horizontal accel + gravity
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should be close to 90 degrees (but may not converge from single update)
    TEST_ASSERT_TRUE(angle > 60.0); // Allow some tolerance
}

void test_off_vertical_angle_zero_acceleration() {
    // Test with zero acceleration (no orientation info)
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, 0});
    simulateUpdate();

    double angle = state->getOffVerticalAngle();
    // Should default to 0 when no acceleration
    TEST_ASSERT_EQUAL_DOUBLE(0.0, angle);
}

// ===== TIME IN STAGE EDGE CASES =====

void test_time_in_stage_rollover() {
    // Test behavior at long durations
    state->setFlightStage(FlightStage::PAD_IDLE);
    setMillis(0);
    simulateUpdate();

    // Simulate 1 hour on pad
    setMillis(3600000); // 1 hour in ms
    simulateUpdate();

    double timeInStage = state->getTimeInStage();
    TEST_ASSERT_TRUE(timeInStage >= 3590.0 && timeInStage <= 3610.0); // ~3600 seconds
}

void test_time_in_stage_rapid_transitions() {
    // Test rapid stage changes
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);
    simulateUpdate();

    setMillis(100);
    state->setFlightStage(FlightStage::COAST);
    simulateUpdate();

    double time1 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time1 >= 0.0 && time1 <= 0.2);

    setMillis(200);
    state->setFlightStage(FlightStage::APOGEE);
    simulateUpdate();

    double time2 = state->getTimeInStage();
    TEST_ASSERT_TRUE(time2 >= 0.0 && time2 <= 0.2);
}

// ===== GETTER SANITY CHECKS =====

void test_all_getters_return_valid_values() {
    // Verify all getters return finite values
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -19.62});
    simulateUpdate();

    TEST_ASSERT_TRUE(std::isfinite(state->getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(state->getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(state->getTimeInStage()));
}

void test_getters_no_sensors() {
    // Test that getters work even without sensor updates
    RocketKF kf;
    MahonyAHRS ahrs;
    RocketState emptyState(&kf, &ahrs);

    // Should not crash and return valid (possibly zero) values
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getAltitudeAGL()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getOffVerticalAngle()));
    TEST_ASSERT_TRUE(std::isfinite(emptyState.getTimeInStage()));
}

// ===== SENSOR DATA EDGE CASES =====

void test_extreme_altitude_values() {
    // Test high altitude climb (simulate going up to 5km over 40 seconds)
    // Note: Very high altitudes are unrealistic for barometer-based rockets
    simulateAltitudeChange(0.0, 5000.0, 40.0, 300);

    // Add settling time
    fakeBaro.setAltitude(5000.0);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    for (int i = 0; i < 100; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(500.0, 5000.0, state->getAltitudeAGL());

    // Simulate descent back to near-ground level over 80 seconds
    simulateAltitudeChange(5000.0, 0.01, 80.0, 400);

    // Add settling time
    fakeBaro.setAltitude(0.01);
    fakeAccel.set(Vector<3>{0, 0, -9.81});
    for (int i = 0; i < 100; i++) {
        simulateUpdate();
    }

    TEST_ASSERT_DOUBLE_WITHIN(100.0, 0.01, state->getAltitudeAGL());
}

void test_sensor_update_frequency() {
    // Test that many updates work correctly with realistic trajectory
    // Climb to 100m over 10 seconds with high update frequency
    simulateAltitudeChange(0.0, 100.0, 10.0, 1000);

    // Should complete without errors and reach approximately 100m
    TEST_ASSERT_TRUE(state->getAltitudeAGL() >= 90.0 && state->getAltitudeAGL() <= 110.0);
}

// ===== FLIGHT STAGE TRANSITION TRACKING =====

void test_stage_transition_time_reset() {
    // Verify time in stage resets on transition
    setMillis(0);
    state->setFlightStage(FlightStage::BOOST);

    // Set high acceleration to stay in BOOST (above 1.5G threshold)
    fakeBaro.setAltitude(100.0);
    fakeAccel.set(Vector<3>{0, 0, -30.0}); // 3G
    simulateUpdate();

    setMillis(2000);
    simulateUpdate();
    double boostTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(boostTime >= 1.8 && boostTime <= 2.2);

    // Transition to coast
    setMillis(2100);
    state->setFlightStage(FlightStage::COAST);
    // Set low acceleration for coast
    fakeAccel.set(Vector<3>{0, 0, -9.81}); // 1G
    simulateUpdate();

    double coastTime = state->getTimeInStage();
    TEST_ASSERT_TRUE(coastTime >= 0.0 && coastTime <= 0.3); // Should reset
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Basic tests
    RUN_TEST(test_rocket_state_initialization);
    RUN_TEST(test_ground_level_setting);
    RUN_TEST(test_flight_stage_manual_setting);

    // Ground level edge cases
    RUN_TEST(test_ground_level_negative);
    RUN_TEST(test_ground_level_high_altitude);
    RUN_TEST(test_ground_level_not_set);
    RUN_TEST(test_negative_altitude_agl);

    // Sensor updates
    RUN_TEST(test_altitude_updates);

    // Altitude tracking
    RUN_TEST(test_altitude_ascent);

    // Off-vertical angle
    RUN_TEST(test_off_vertical_angle_vertical_flight);
    RUN_TEST(test_off_vertical_angle_horizontal_flight);
    RUN_TEST(test_off_vertical_angle_zero_acceleration);

    // Time in stage
    RUN_TEST(test_time_in_stage_rollover);
    RUN_TEST(test_time_in_stage_rapid_transitions);

    // Getter sanity
    RUN_TEST(test_all_getters_return_valid_values);
    RUN_TEST(test_getters_no_sensors);

    // Sensor data edge cases
    RUN_TEST(test_extreme_altitude_values);
    RUN_TEST(test_sensor_update_frequency);

    // Stage transitions
    RUN_TEST(test_stage_transition_time_reset);

    UNITY_END();
}
