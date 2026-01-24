#include "RocketKF.h"

namespace astra_rocket {

// Define the measurement size, control size, and state size
RocketKF::RocketKF() : LinearKalmanFilter(3, 3, 6) {}

void RocketKF::initialize() {
    // Initialize state vector to zero (position and velocity)
    // State: [px, py, pz, vx, vy, vz]
    double *state_data = new double[6]{0, 0, 0, 0, 0, 0};
    X = Matrix(6, 1, state_data);

    // Initialize covariance matrix P with very high uncertainty
    // This tells the filter we have no confidence in our initial zero state,
    // so it should trust the first measurements heavily
    double *cov_data = new double[36]{
        10000.0, 0, 0, 0, 0, 0,      // Very high uncertainty in X position (10000m²)
        0, 10000.0, 0, 0, 0, 0,      // Very high uncertainty in Y position (10000m²)
        0, 0, 10000.0, 0, 0, 0,      // Very high uncertainty in Z position (10000m²)
        0, 0, 0, 100.0, 0, 0,        // High uncertainty in X velocity (100 m²/s²)
        0, 0, 0, 0, 100.0, 0,        // High uncertainty in Y velocity (100 m²/s²)
        0, 0, 0, 0, 0, 100.0         // High uncertainty in Z velocity (100 m²/s²)
    };
    P = Matrix(6, 6, cov_data);
}

Matrix RocketKF::getF(double dt) {
    double *data = new double[36]{
        1.0, 0, 0, dt, 0, 0,
        0, 1.0, 0, 0, dt, 0,
        0, 0, 1.0, 0, 0, dt,
        0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 1.0
    };
    return Matrix(6, 6, data);
}

Matrix RocketKF::getG(double dt) {
    double *data = new double[18]{
        0.5 * dt * dt, 0, 0,
        0, 0.5 * dt * dt, 0,
        0, 0, 0.5 * dt * dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };
    return Matrix(6, 3, data);
}

Matrix RocketKF::getH() {
    double *data = new double[18]{
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0
    };
    return Matrix(3, 6, data);
}

Matrix RocketKF::getR() {
    double *data = new double[9]{
        1.0, 0, 0,
        0, 1.0, 0,
        0, 0, 0.5
    };
    return Matrix(3, 3, data);
}

Matrix RocketKF::getQ(double dt) {
    double *data = new double[36]{
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    };
    return Matrix(6, 6, data);
}

} // namespace astra
