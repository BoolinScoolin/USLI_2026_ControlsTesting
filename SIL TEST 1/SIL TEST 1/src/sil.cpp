#include "sil.h"

int findTimeIndexBinary(float t_runtime) {
    if (SIL_N <= 1) return 0;
    if (t_runtime <= sil_time_s[0]) return 0;
    if (t_runtime >= sil_time_s[SIL_N-1]) return SIL_N-1;

    int lo = 0;
    int hi = SIL_N - 1;

    // Invariant: t[lo] <= t_runtime < t[hi]
    while (hi - lo > 1) {
        int mid = lo + (hi - lo) / 2;
        if (sil_time_s[mid] <= t_runtime) lo = mid;
        else hi = mid;
    }
    return lo;
}

void SIL_MTi::readMessages(float t_s) {
    int k = findTimeIndexBinary(t_s);

    euler_rad[0]  = sil_euler_1_rad[k];
    euler_rad[1]  = sil_euler_2_rad[k];
    euler_rad[2]  = sil_euler_3_rad[k];

    accel_mps2[0] = sil_accel_x_mps2[k];
    accel_mps2[1] = sil_accel_y_mps2[k];
    accel_mps2[2] = sil_accel_z_mps2[k];

    gyro_rps[0]   = sil_gyro_x_rps[k];
    gyro_rps[1]   = sil_gyro_y_rps[k];
    gyro_rps[2]   = sil_gyro_z_rps[k];
}


float* SIL_MTi::getEulerAngles() { return euler_rad; }
float* SIL_MTi::getAcceleration() { return accel_mps2; }
float* SIL_MTi::getRateOfTurn() { return gyro_rps; }

float SIL_Adafruit_BMP3XX::readAltitude(float sea_level_pressure_pa, float t_s) {
    int k = findTimeIndexBinary(t_s);
    return sil_alt_z_m[k];
}

float sil_true_alt(float t_s) {
    int k = findTimeIndexBinary(t_s);
    return sil_true_z_m[k];
}