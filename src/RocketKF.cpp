#include "RocketKF.h"

namespace astra_rocket {

// 1. Update Constructor: 6 measurements (3 Pos + 3 Accel), 0 control, 9 states
RocketKF::RocketKF() : LinearKalmanFilter(6, 0, 9) {}

void RocketKF::initialize() {
    // State: [px, py, pz, vx, vy, vz, ax, ay, az]
    double *state_data = new double[9]{0}; 
    X = Matrix(9, 1, state_data);

    // Initialize Covariance P
    // We need a 9x9 matrix now.
    double *cov_data = new double[81]{0};
    
    // Set diagonal values
    // High uncertainty in Position
    cov_data[0] = 10000.0; cov_data[10] = 10000.0; cov_data[20] = 10000.0;
    // High uncertainty in Velocity
    cov_data[30] = 100.0;  cov_data[40] = 100.0;   cov_data[50] = 100.0;
    // High uncertainty in Acceleration
    cov_data[60] = 10.0;   cov_data[70] = 10.0;    cov_data[80] = 10.0;

    P = Matrix(9, 9, cov_data);
}

// 2. Update F: The physics of motion
// Pos += Vel*dt + 0.5*Acc*dt^2
// Vel += Acc*dt
// Acc += 0 (Constant assumption)
Matrix RocketKF::getF(double dt) {
    double half_dt2 = 0.5 * dt * dt;
    double *data = new double[81]{
        // Pos (rows 0-2) -> depends on Pos, Vel, Acc
        1, 0, 0,  dt, 0, 0,  half_dt2, 0, 0,
        0, 1, 0,  0, dt, 0,  0, half_dt2, 0,
        0, 0, 1,  0, 0, dt,  0, 0, half_dt2,
        
        // Vel (rows 3-5) -> depends on Vel, Acc
        0, 0, 0,  1, 0, 0,   dt, 0, 0,
        0, 0, 0,  0, 1, 0,   0, dt, 0,
        0, 0, 0,  0, 0, 1,   0, 0, dt,
        
        // Acc (rows 6-8) -> depends on Acc (Identity)
        0, 0, 0,  0, 0, 0,   1, 0, 0,
        0, 0, 0,  0, 0, 0,   0, 1, 0,
        0, 0, 0,  0, 0, 0,   0, 0, 1
    };
    return Matrix(9, 9, data);
}

// Control Matrix is now empty or zero, as we don't have external control inputs
Matrix RocketKF::getG(double dt) {
    return Matrix(9, 0, nullptr); 
}

// 3. Update H: We measure Position AND Acceleration
// We do NOT measure Velocity directly usually.
Matrix RocketKF::getH() {
    double *data = new double[54]{
        // Measuring Position (matches State indexes 0,1,2)
        1, 0, 0,  0, 0, 0,  0, 0, 0,
        0, 1, 0,  0, 0, 0,  0, 0, 0,
        0, 0, 1,  0, 0, 0,  0, 0, 0,

        // Measuring Acceleration (matches State indexes 6,7,8)
        0, 0, 0,  0, 0, 0,  1, 0, 0,
        0, 0, 0,  0, 0, 0,  0, 1, 0,
        0, 0, 0,  0, 0, 0,  0, 0, 1
    };
    // 6 Measurements (rows), 9 States (cols)
    return Matrix(6, 9, data);
}

// 4. Update R: Measurement Noise
Matrix RocketKF::getR() {
    double *data = new double[36]{0};
    
    // Position noise (e.g., GPS accuracy ~2.0m)
    data[0] = 2.0; data[7] = 2.0; data[14] = 2.0;
    
    // Acceleration noise (e.g., Accelerometer noise ~0.5 m/s^2)
    data[21] = 3; data[28] = 3; data[35] = 3;
    
    return Matrix(6, 6, data);
}

// 5. Update Q: Process Noise
// We trust the physics for Pos/Vel, but we expect Accel to change (Jerk).
// So we inject noise primarily into the Acceleration states.
Matrix RocketKF::getQ(double dt) {
    double *data = new double[81]{0};
    double jerk_uncertainty = 1.0; // Allow accel to vary by ~1 m/s^2 per step

    // Rows 6,7,8 (Acceleration Indices)
    data[60] = jerk_uncertainty; 
    data[70] = jerk_uncertainty; 
    data[80] = jerk_uncertainty;

    return Matrix(9, 9, data);
}

} // namespace astra