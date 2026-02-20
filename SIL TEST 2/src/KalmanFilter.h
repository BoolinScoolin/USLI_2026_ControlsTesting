#pragma once

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include <stdint.h>

class KalmanFilter {
public:
    KalmanFilter();

    // Safety-hardened update function
    void update(float z_measured, uint32_t current_micros);

    float getAltitude() const { return x[0]; }
    float getVelocity() const { return x[1]; }

    // Call this in setup() to set the starting ground altitude
    void initialize(float initial_alt);

private:
    // State: x = [altitude, velocity]
    float x[2];

    // Covariance
    float P[2][2];

    // Process noise covariance
    float Q[2][2];

    // Measurement noise (scalar)
    float R;

    uint32_t last_micros;
    bool is_initialized;

    // Helpers
    static inline void mat2_zero(float M[2][2]) {
        M[0][0] = 0.0f; M[0][1] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 0.0f;
    }

    static inline void mat2_identity(float M[2][2]) {
        M[0][0] = 1.0f; M[0][1] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 1.0f;
    }
};

#endif
