#include "RocketKF.h"

namespace astra_rocket {

// 1. Update Constructor: 3 measurements (3 Pos), 3 control (3 Accel), 6 states
RocketKF::RocketKF() : LinearKalmanFilter(3, 3, 6) {}

void RocketKF::initialize() {
    // State: [px, py, pz, vx, vy, vz]
    double *state_data = new double[6]{0};
    X = Matrix(6, 1, state_data);

    // Initialize Covariance P
    // We need a 6x6 matrix now.
    double *cov_data = new double[36]{0};

    // Set diagonal values
    // High uncertainty in Position
    cov_data[0] = 10000.0; cov_data[7] = 10000.0; cov_data[14] = 10000.0;
    // High uncertainty in Velocity
    cov_data[21] = 100.0;  cov_data[28] = 100.0;   cov_data[35] = 100.0;

    P = Matrix(6, 6, cov_data);
}

// 2. Update F: The physics of motion (state transition without control)
// Pos += Vel*dt
// Vel += 0 (changes come from control input)
Matrix RocketKF::getF(double dt) {
    double *data = new double[36]{
        // Pos (rows 0-2) -> depends on Pos, Vel
        1, 0, 0,  dt, 0, 0,
        0, 1, 0,  0, dt, 0,
        0, 0, 1,  0, 0, dt,

        // Vel (rows 3-5) -> identity (changes from control)
        0, 0, 0,  1, 0, 0,
        0, 0, 0,  0, 1, 0,
        0, 0, 0,  0, 0, 1
    };
    return Matrix(6, 6, data);
}

// Control Matrix: How acceleration (control input) affects state
// Pos += 0.5*Acc*dt^2
// Vel += Acc*dt
Matrix RocketKF::getG(double dt) {
    double half_dt2 = 0.5 * dt * dt;
    double *data = new double[18]{
        // How control (accel) affects Position
        half_dt2, 0, 0,
        0, half_dt2, 0,
        0, 0, half_dt2,

        // How control (accel) affects Velocity
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };
    return Matrix(6, 3, data);
}

// 3. Update H: We measure Position only
// We do NOT measure Velocity directly.
Matrix RocketKF::getH() {
    double *data = new double[18]{
        // Measuring Position (matches State indexes 0,1,2)
        1, 0, 0,  0, 0, 0,
        0, 1, 0,  0, 0, 0,
        0, 0, 1,  0, 0, 0
    };
    // 3 Measurements (rows), 6 States (cols)
    return Matrix(3, 6, data);
}

// 4. Update R: Measurement Noise
Matrix RocketKF::getR() {
    double *data = new double[9]{0};

    // Position noise (e.g., GPS accuracy ~2.0m)
    data[0] = data[4] = 5.0;
    //baro
    data[8] = 0.5;

    return Matrix(3, 3, data);
}

// 5. Update Q: Process Noise
// Models uncertainty in state dynamics (primarily from unmodeled accelerations/jerk)
Matrix RocketKF::getQ(double dt) {
    double *data = new double[36]{0};

    // Position uncertainty (from unmodeled dynamics)
    data[0] = data[7] = data[14] = (dt * dt * dt * dt) / 4.0;

    data[3] = data[10] = data[17] = (dt * dt * dt) / 2.0;

    data[18] = data[25] = data[32] = (dt * dt * dt) / 2.0;

    // Velocity uncertainty (from unmodeled accelerations)
    data[21] = data[28] = data[35] = (dt * dt);

    return Matrix(6, 6, data) * (.2 * .2); // Scale overall process noise
}

} // namespace astra