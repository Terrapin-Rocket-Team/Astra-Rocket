#include <unity.h>
#include <NativeTestHelper.h>
#include "../../src/RocketKF.h"
#include <cmath>

using namespace astra_rocket;
using namespace astra;

// Test fixture
RocketKF* kf;

void setUp(void) {
    kf = new RocketKF();
    kf->initialize();
}

void tearDown(void) {
    delete kf;
    kf = nullptr;
}

// Helper to create measurement vector [px, py, pz, ax, ay, az]
Matrix createMeasurement(double px, double py, double pz, double ax, double ay, double az) {
    double* data = new double[6]{px, py, pz, ax, ay, az};
    return Matrix(6, 1, data);
}

// ===== INITIALIZATION TESTS =====

void test_kf_initialization() {
    // Test that KF initializes with correct dimensions
    Matrix state = kf->getState();
    TEST_ASSERT_EQUAL(9, state.getRows());
    TEST_ASSERT_EQUAL(1, state.getCols());

    // Initial state should be zeros
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_EQUAL_DOUBLE(0.0, state(i, 0));
    }
}

void test_kf_dimensions() {
    // Test dimensions are correct
    TEST_ASSERT_EQUAL(6, kf->getMeasurementSize());
    TEST_ASSERT_EQUAL(0, kf->getInputSize());
    TEST_ASSERT_EQUAL(9, kf->getStateSize());
}

// ===== MATRIX DIMENSION TESTS =====

void test_kf_matrix_dimensions() {
    double dt = 0.02;

    // Test state transition matrix
    Matrix stateTransition = kf->getF(dt);
    TEST_ASSERT_EQUAL(9, stateTransition.getRows());
    TEST_ASSERT_EQUAL(9, stateTransition.getCols());

    // Test measurement matrix
    Matrix measurement = kf->getH();
    TEST_ASSERT_EQUAL(6, measurement.getRows());
    TEST_ASSERT_EQUAL(9, measurement.getCols());

    // Test measurement noise matrix
    Matrix measNoise = kf->getR();
    TEST_ASSERT_EQUAL(6, measNoise.getRows());
    TEST_ASSERT_EQUAL(6, measNoise.getCols());

    // Test process noise matrix
    Matrix processNoise = kf->getQ(dt);
    TEST_ASSERT_EQUAL(9, processNoise.getRows());
    TEST_ASSERT_EQUAL(9, processNoise.getCols());

    // Test control matrix - should be empty
    Matrix control = kf->getG(dt);
    TEST_ASSERT_EQUAL(9, control.getRows());
    TEST_ASSERT_EQUAL(0, control.getCols());
}

// ===== MATRIX PHYSICS TESTS =====

void test_kf_state_transition_matrix() {
    // Test F matrix encodes correct physics
    double dt = 1.0;
    Matrix stateTransition = kf->getF(dt);

    // Position rows should have [1, 0, 0, dt, 0, 0, 0.5*dt^2, 0, 0]
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, stateTransition(0, 0));     // pos_x = pos_x
    TEST_ASSERT_DOUBLE_WITHIN(0.001, dt, stateTransition(0, 3));      // pos_x += vel_x * dt
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.5*dt*dt, stateTransition(0, 6)); // pos_x += 0.5 * acc_x * dt^2

    // Velocity rows should have [0, 0, 0, 1, 0, 0, dt, 0, 0]
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, stateTransition(3, 3));     // vel_x = vel_x
    TEST_ASSERT_DOUBLE_WITHIN(0.001, dt, stateTransition(3, 6));      // vel_x += acc_x * dt

    // Acceleration rows should have [0, 0, 0, 0, 0, 0, 1, 0, 0]
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, stateTransition(6, 6));     // acc_x = acc_x (constant)
}

void test_kf_measurement_matrix() {
    // Test measurement matrix measures position and acceleration (not velocity)
    Matrix measMat = kf->getH();

    // First 3 rows should measure position (indices 0, 1, 2)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(0, 0)); // meas[0] = state[0] (px)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(1, 1)); // meas[1] = state[1] (py)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(2, 2)); // meas[2] = state[2] (pz)

    // Last 3 rows should measure acceleration (indices 6, 7, 8)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(3, 6)); // meas[3] = state[6] (ax)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(4, 7)); // meas[4] = state[7] (ay)
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1.0, measMat(5, 8)); // meas[5] = state[8] (az)
}

void test_kf_noise_matrices() {
    // Test noise matrices have reasonable values
    Matrix measNoise = kf->getR();
    Matrix processNoise = kf->getQ(0.02);

    // Measurement noise should have positive diagonal values
    TEST_ASSERT_TRUE(measNoise(0, 0) > 0); // Position noise
    TEST_ASSERT_TRUE(measNoise(3, 3) > 0); // Acceleration noise

    // Process noise should have positive diagonal values for acceleration
    TEST_ASSERT_TRUE(processNoise(6, 6) > 0); // Acceleration process noise
    TEST_ASSERT_TRUE(processNoise(7, 7) > 0);
    TEST_ASSERT_TRUE(processNoise(8, 8) > 0);
}

// ===== PREDICTION TESTS =====

void test_kf_prediction_updates_state() {
    // Get initial state
    Matrix initialState = kf->getState();
    double initialPz = initialState(2, 0);

    // Perform prediction
    Matrix control(0, 1);
    kf->predict(0.02, control);

    // State should still be accessible (not necessarily changed since initial is 0)
    Matrix newState = kf->getState();
    TEST_ASSERT_EQUAL(9, newState.getRows());
}

void test_kf_multiple_predictions() {
    // Multiple predictions should not crash
    Matrix control(0, 1);

    for (int i = 0; i < 100; i++) {
        kf->predict(0.02, control);
    }

    Matrix state = kf->getState();
    TEST_ASSERT_EQUAL(9, state.getRows());
}

// ===== MEASUREMENT UPDATE TESTS =====

void test_kf_measurement_update() {
    // Provide a measurement
    Matrix measurement = createMeasurement(0, 0, 100.0, 0, 0, -9.81);
    kf->update(measurement);

    // State should be updated
    Matrix state = kf->getState();
    double pz = state(2, 0);

    // Position should move toward measurement (100m)
    TEST_ASSERT_TRUE(pz > 0.0);
}

void test_kf_measurement_convergence() {
    // Test that repeated measurements cause convergence
    for (int i = 0; i < 100; i++) {
        // Predict
        Matrix control(0, 1);
        kf->predict(0.02, control);

        // Update with constant measurement: pz=100m, az=-9.81
        Matrix measurement = createMeasurement(0, 0, 100.0, 0, 0, -9.81);
        kf->update(measurement);
    }

    // After many updates, state should converge close to measurements
    Matrix state = kf->getState();
    TEST_ASSERT_DOUBLE_WITHIN(10.0, 100.0, state(2, 0));    // position
    TEST_ASSERT_DOUBLE_WITHIN(5.0, -9.81, state(8, 0));     // acceleration
}

void test_kf_altitude_tracking() {
    // Simulate altitude increasing
    for (int i = 0; i < 50; i++) {
        double altitude = i * 2.0; // 0 to 100m

        Matrix control(0, 1);
        kf->predict(0.02, control);

        Matrix measurement = createMeasurement(0, 0, altitude, 0, 0, -9.81);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double pz = state(2, 0);

    // Should be tracking the rising altitude
    TEST_ASSERT_TRUE(pz > 50.0);
}

// ===== REALISTIC FLIGHT SCENARIO TESTS =====

void test_kf_ascent_phase() {
    // Simulate simple ascent by providing increasing altitude measurements
    double dt = 0.02;

    for (int i = 0; i < 100; i++) {
        double altitude = i * 1.0; // Steadily increasing altitude

        Matrix control(0, 1);
        kf->predict(dt, control);

        Matrix measurement = createMeasurement(0, 0, altitude, 0, 0, -9.81);
        kf->update(measurement);
    }

    Matrix finalState = kf->getState();
    double finalAlt = finalState(2, 0);

    // Should be tracking increasing altitude
    TEST_ASSERT_TRUE(finalAlt > 50.0);   // Should have tracked the climb
}

void test_kf_descent_phase() {
    // Initialize at altitude with downward velocity
    // First, get to altitude
    for (int i = 0; i < 50; i++) {
        Matrix control(0, 1);
        kf->predict(0.02, control);

        Matrix measurement = createMeasurement(0, 0, 500.0, 0, 0, -9.81);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    double startAlt = state(2, 0);

    // Now simulate descent
    for (int i = 0; i < 50; i++) {
        double alt = 500.0 - i * 5.0; // Descending

        Matrix control(0, 1);
        kf->predict(0.02, control);

        Matrix measurement = createMeasurement(0, 0, alt, 0, 0, -9.81);
        kf->update(measurement);
    }

    Matrix finalState = kf->getState();
    double finalAlt = finalState(2, 0);
    double finalVel = finalState(5, 0);

    // Should be descending
    TEST_ASSERT_TRUE(finalAlt < startAlt);
    TEST_ASSERT_TRUE(finalVel < 0.0); // Negative velocity = descending
}

// ===== EDGE CASE TESTS =====

void test_kf_zero_dt_prediction() {
    // Get initial state
    Matrix initialState = kf->getState();

    // Predict with dt=0
    Matrix control(0, 1);
    kf->predict(0.0, control);

    Matrix finalState = kf->getState();

    // State should not change significantly
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_DOUBLE_WITHIN(0.001, initialState(i, 0), finalState(i, 0));
    }
}

void test_kf_large_dt_prediction() {
    // Test prediction with large dt doesn't crash
    Matrix control(0, 1);
    kf->predict(10.0, control); // 10 second prediction

    Matrix state = kf->getState();
    TEST_ASSERT_EQUAL(9, state.getRows());
}

void test_kf_rapid_measurements() {
    // Test rapid measurement updates don't break anything
    for (int i = 0; i < 1000; i++) {
        Matrix measurement = createMeasurement(0, 0, 100.0, 0, 0, -9.81);
        kf->update(measurement);
    }

    Matrix state = kf->getState();
    // Should converge very close to measurement
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 100.0, state(2, 0));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Initialization tests
    RUN_TEST(test_kf_initialization);
    RUN_TEST(test_kf_dimensions);

    // Matrix dimension tests
    RUN_TEST(test_kf_matrix_dimensions);

    // Matrix physics tests
    RUN_TEST(test_kf_state_transition_matrix);
    RUN_TEST(test_kf_measurement_matrix);
    RUN_TEST(test_kf_noise_matrices);

    // Prediction tests
    RUN_TEST(test_kf_prediction_updates_state);
    RUN_TEST(test_kf_multiple_predictions);

    // Measurement update tests
    RUN_TEST(test_kf_measurement_update);
    RUN_TEST(test_kf_measurement_convergence);
    RUN_TEST(test_kf_altitude_tracking);

    // Realistic flight scenarios
    RUN_TEST(test_kf_ascent_phase);
    RUN_TEST(test_kf_descent_phase);

    // Edge cases
    RUN_TEST(test_kf_zero_dt_prediction);
    RUN_TEST(test_kf_large_dt_prediction);
    RUN_TEST(test_kf_rapid_measurements);

    UNITY_END();
}
