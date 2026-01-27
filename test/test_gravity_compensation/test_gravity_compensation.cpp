#include <unity.h>
#include <NativeTestHelper.h>
#include "../../src/RocketKF.h"
#include "../../.pio/libdeps/native/TRT-Astra/src/Filters/Mahony.h"
#include "../../.pio/libdeps/native/TRT-Astra/src/Math/Vector.h"
#include <cmath>

using namespace astra_rocket;
using namespace astra;

/**
 * This test suite verifies the gravity compensation chain:
 * 1. Accelerometer measures specific force
 * 2. Mahony's getEarthAcceleration() converts specific force -> inertial accel
 * 3. KF receives and tracks inertial acceleration
 */

// Test fixture
MahonyAHRS* mahony;
RocketKF* kf;

void setUp(void) {
    mahony = new MahonyAHRS();
    mahony->setMode(MahonyMode::CALIBRATING);

    // Calibrate with "at rest" accelerometer reading
    Vector<3> accel_at_rest(0, 0, 9.81);  // Specific force pointing up
    Vector<3> gyro_zero(0, 0, 0);

    // Accumulate calibration samples
    for (int i = 0; i < 200; i++) {
        mahony->update(accel_at_rest, gyro_zero, 0.01);
    }
    mahony->finalizeCalibration();

    kf = new RocketKF();
    kf->initialize();
}

void tearDown(void) {
    delete mahony;
    delete kf;
    mahony = nullptr;
    kf = nullptr;
}

// ===== COORDINATE SYSTEM TESTS =====

void test_enu_coordinate_system() {
    // In ENU (East-North-Up):
    // - X = East
    // - Y = North
    // - Z = Up (away from Earth)
    // - Gravity vector: g = [0, 0, -9.81] (points down)

    // This test just documents the expected convention
    Vector<3> gravity_enu(0, 0, -9.81);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -9.81, gravity_enu.z());
}

// ===== SPECIFIC FORCE vs INERTIAL ACCELERATION =====

void test_specific_force_at_rest() {
    // At rest on pad:
    // - Inertial acceleration = 0 (not moving)
    // - Specific force = +9.81 Z (structure pushes up to support mass)

    Vector<3> accel_sensor(0, 0, 9.81);  // What accelerometer measures

    // Expected: sensor measures +9.81 (reaction force)
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 9.81, accel_sensor.z());
}

void test_specific_force_freefall() {
    // In freefall:
    // - Inertial acceleration = -9.81 Z (falling)
    // - Specific force = 0 (no support force, weightless)

    Vector<3> accel_sensor_freefall(0, 0, 0);  // What accelerometer measures

    // Expected: sensor measures 0 (weightless)
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, accel_sensor_freefall.z());
}

void test_specific_force_under_parachute() {
    // Under parachute at terminal velocity:
    // - Inertial acceleration = 0 (constant velocity descent)
    // - Specific force = +9.81 Z (parachute pulls up to balance gravity)

    Vector<3> accel_sensor_parachute(0, 0, 9.81);  // What accelerometer measures

    // Expected: sensor measures +9.81 (same as at rest)
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 9.81, accel_sensor_parachute.z());
}

// ===== MAHONY GRAVITY COMPENSATION =====

void test_mahony_conversion_at_rest() {
    // At rest: specific force = +9.81, inertial accel = 0
    Vector<3> specific_force(0, 0, 9.81);

    // Debug: Check what the quaternion is
    Quaternion q = mahony->getQuaternion();
    fprintf(stderr, "\n[DEBUG] Mahony quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
            q.w(), q.x(), q.y(), q.z());

    // Check mode
    fprintf(stderr, "[DEBUG] Mahony mode: %d (0=CALIBRATING, 1=CORRECTING, 2=GYRO_ONLY)\n",
            (int)mahony->getMode());

    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    // Expected: inertial_accel.z = 0
    fprintf(stderr, "At rest: specific_force.z=%.2f -> inertial_accel.z=%.2f (expected 0)\n",
            specific_force.z(), inertial_accel.z());

    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, inertial_accel.z());
}

void test_mahony_conversion_freefall() {
    // Freefall: specific force = 0, inertial accel = -9.81
    Vector<3> specific_force(0, 0, 0);

    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    // Expected: inertial_accel.z = -9.81
    fprintf(stderr, "Freefall: specific_force.z=%.2f -> inertial_accel.z=%.2f (expected -9.81)\n",
            specific_force.z(), inertial_accel.z());

    TEST_ASSERT_DOUBLE_WITHIN(0.5, -9.81, inertial_accel.z());
}

void test_mahony_conversion_under_parachute() {
    // Under parachute: specific force = +9.81, inertial accel = 0
    Vector<3> specific_force(0, 0, 9.81);

    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    // Expected: inertial_accel.z = 0 (same as at rest)
    fprintf(stderr, "Under parachute: specific_force.z=%.2f -> inertial_accel.z=%.2f (expected 0)\n",
            specific_force.z(), inertial_accel.z());

    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, inertial_accel.z());
}

void test_mahony_conversion_boost() {
    // During motor burn with 30 m/s² thrust (upward):
    // - Inertial accel = +30 m/s² (accelerating upward)
    // - Specific force = 30 + 9.81 = 39.81 m/s² (thrust + gravity support)

    Vector<3> specific_force(0, 0, 39.81);

    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    // Expected: inertial_accel.z = +30
    fprintf(stderr, "Boost: specific_force.z=%.2f -> inertial_accel.z=%.2f (expected 30)\n",
            specific_force.z(), inertial_accel.z());

    TEST_ASSERT_DOUBLE_WITHIN(0.5, 30.0, inertial_accel.z());
}

// ===== KALMAN FILTER MEASUREMENT =====

void test_kf_measurement_at_rest() {
    // Simulate at rest on pad
    // Measurement: [px, py, pz, ax, ay, az] where a is INERTIAL acceleration

    double measurement_data[6] = {
        0, 0, 0,      // Position at origin
        0, 0, 0       // Inertial accel = 0 (at rest)
    };
    Matrix measurement(6, 1, measurement_data);

    // Feed to KF
    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double kf_accel_z = state(8, 0);

    fprintf(stderr, "KF at rest: measurement_az=%.2f -> kf_state_az=%.2f (expected 0)\n",
            measurement_data[5], kf_accel_z);

    // KF should estimate near-zero acceleration
    TEST_ASSERT_DOUBLE_WITHIN(2.0, 0.0, kf_accel_z);
}

void test_kf_measurement_boost() {
    // Simulate boost phase with +30 m/s² inertial acceleration

    double measurement_data[6] = {
        0, 0, 0,      // Position (simplification)
        0, 0, 30.0    // Inertial accel = +30 m/s²
    };
    Matrix measurement(6, 1, measurement_data);

    // Feed to KF
    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double kf_accel_z = state(8, 0);

    fprintf(stderr, "KF boost: measurement_az=%.2f -> kf_state_az=%.2f (expected 30)\n",
            measurement_data[5], kf_accel_z);

    // KF should estimate +30 m/s²
    TEST_ASSERT_DOUBLE_WITHIN(5.0, 30.0, kf_accel_z);
}

void test_kf_measurement_freefall() {
    // Simulate freefall with -9.81 m/s² inertial acceleration

    double measurement_data[6] = {
        0, 0, 100,        // At altitude
        0, 0, -9.81       // Inertial accel = -9.81 m/s² (falling)
    };
    Matrix measurement(6, 1, measurement_data);

    // Feed to KF
    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double kf_accel_z = state(8, 0);

    fprintf(stderr, "KF freefall: measurement_az=%.2f -> kf_state_az=%.2f (expected -9.81)\n",
            measurement_data[5], kf_accel_z);

    // KF should estimate -9.81 m/s²
    TEST_ASSERT_DOUBLE_WITHIN(2.0, -9.81, kf_accel_z);
}

// ===== FULL CHAIN TEST =====

void test_full_chain_at_rest() {
    // Full chain: sensor -> Mahony -> KF

    // 1. Sensor reads specific force
    Vector<3> specific_force(0, 0, 9.81);  // At rest

    // 2. Mahony converts to inertial accel
    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    fprintf(stderr, "Full chain at rest: sensor=%.2f -> mahony=%.2f\n",
            specific_force.z(), inertial_accel.z());

    // 3. Feed to KF as measurement
    double measurement_data[6] = {
        0, 0, 0,
        inertial_accel.x(), inertial_accel.y(), inertial_accel.z()
    };
    Matrix measurement(6, 1, measurement_data);

    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double kf_accel_z = state(8, 0);

    fprintf(stderr, "Full chain at rest: sensor=%.2f -> mahony=%.2f -> kf=%.2f (expected 0)\n",
            specific_force.z(), inertial_accel.z(), kf_accel_z);

    // Final KF estimate should be ~0
    TEST_ASSERT_DOUBLE_WITHIN(2.0, 0.0, kf_accel_z);
}

void test_full_chain_boost() {
    // Full chain: sensor -> Mahony -> KF during boost

    // 1. Sensor reads specific force during 30 m/s² boost
    Vector<3> specific_force(0, 0, 39.81);  // 30 + 9.81

    // 2. Mahony converts to inertial accel
    Vector<3> inertial_accel = mahony->getEarthAcceleration(specific_force);

    fprintf(stderr, "Full chain boost: sensor=%.2f -> mahony=%.2f\n",
            specific_force.z(), inertial_accel.z());

    // 3. Feed to KF as measurement
    double measurement_data[6] = {
        0, 0, 0,
        inertial_accel.x(), inertial_accel.y(), inertial_accel.z()
    };
    Matrix measurement(6, 1, measurement_data);

    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double kf_accel_z = state(8, 0);

    fprintf(stderr, "Full chain boost: sensor=%.2f -> mahony=%.2f -> kf=%.2f (expected 30)\n",
            specific_force.z(), inertial_accel.z(), kf_accel_z);

    // Final KF estimate should be ~+30
    TEST_ASSERT_DOUBLE_WITHIN(5.0, 30.0, kf_accel_z);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Coordinate system
    RUN_TEST(test_enu_coordinate_system);

    // Specific force concepts
    RUN_TEST(test_specific_force_at_rest);
    RUN_TEST(test_specific_force_freefall);
    RUN_TEST(test_specific_force_under_parachute);

    // Mahony gravity compensation
    RUN_TEST(test_mahony_conversion_at_rest);
    RUN_TEST(test_mahony_conversion_freefall);
    RUN_TEST(test_mahony_conversion_under_parachute);
    RUN_TEST(test_mahony_conversion_boost);

    // KF measurements
    RUN_TEST(test_kf_measurement_at_rest);
    RUN_TEST(test_kf_measurement_boost);
    RUN_TEST(test_kf_measurement_freefall);

    // Full chain
    RUN_TEST(test_full_chain_at_rest);
    RUN_TEST(test_full_chain_boost);

    UNITY_END();
}
