#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
: R(60.0f),
  last_micros(0),
  is_initialized(false)
{
    // Initial state
    x[0] = 0.0f; // altitude
    x[1] = 0.0f; // velocity

    // Initial covariance
    mat2_identity(P);

    // Process noise covariance (same values you had)
    mat2_zero(Q);
    Q[0][0] = 0.5f;
    Q[1][1] = 1.5f;

    // (Q off-diagonals are 0 by default)
}

void KalmanFilter::initialize(float initial_alt) {
    x[0] = initial_alt;
    x[1] = 0.0f;
    last_micros = micros();
    is_initialized = true;
}

void KalmanFilter::update(float z, uint32_t current_micros) {
    if (!is_initialized) {
        initialize(z);
        return;
    }

    // --- Dynamic dt Calculation with Safety ---
    float dt = (current_micros - last_micros) * 1.0e-6f;
    last_micros = current_micros;

    // Guard against first-loop spikes or zero-time errors
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.031f;
    }

    // ============================================================
    // 1) PREDICT
    // Model: x' = A x,  A = [1 dt; 0 1]
    // ============================================================

    // Predicted state xp = A*x
    const float xp0 = x[0] + dt * x[1];
    const float xp1 = x[1];

    // Predicted covariance: Pp = A*P*A' + Q
    // With A = [1 dt; 0 1], expand in closed-form for speed.
    const float P00 = P[0][0], P01 = P[0][1];
    const float P10 = P[1][0], P11 = P[1][1];

    const float dtP11 = dt * P11;
    const float dt2P11 = dt * dtP11;

    float Pp00 = P00 + dt * (P10 + P01) + dt2P11 + Q[0][0];
    float Pp01 = P01 + dtP11 + Q[0][1];
    float Pp10 = P10 + dtP11 + Q[1][0];
    float Pp11 = P11 + Q[1][1];

    // ============================================================
    // 2) UPDATE
    // H = [1 0]  (measure altitude only)
    // S = H*Pp*H' + R = Pp00 + R
    // K = Pp*H'/S = [Pp00; Pp10] / S
    // ============================================================

    const float S = Pp00 + R;

    // Safety: avoid divide-by-zero / weird negatives (shouldn't happen, but embedded life)
    if (S <= 1.0e-9f) {
        // If S is degenerate, just accept the prediction and keep covariance as-is
        x[0] = xp0;
        x[1] = xp1;
        P[0][0] = Pp00; P[0][1] = Pp01;
        P[1][0] = Pp10; P[1][1] = Pp11;
        return;
    }

    const float invS = 1.0f / S;
    const float K0 = Pp00 * invS;
    const float K1 = Pp10 * invS;

    const float innovation = z - xp0;

    // Updated state
    x[0] = xp0 + K0 * innovation;
    x[1] = xp1 + K1 * innovation;

    // Updated covariance: P = Pp - K*(H*Pp)
    // H*Pp = [Pp00 Pp01]
    float newP00 = Pp00 - K0 * Pp00;
    float newP01 = Pp01 - K0 * Pp01;
    float newP10 = Pp10 - K1 * Pp00;
    float newP11 = Pp11 - K1 * Pp01;

    // Optional: enforce symmetry (helps with float drift)
    const float sym01 = 0.5f * (newP01 + newP10);
    newP01 = sym01;
    newP10 = sym01;

    P[0][0] = newP00; P[0][1] = newP01;
    P[1][0] = newP10; P[1][1] = newP11;
}
