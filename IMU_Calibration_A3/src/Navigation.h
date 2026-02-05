#pragma once

#include <Arduino.h>
#include "Quaternion.h"
#include "Measurements.h"

const float gz_n_mps2 = 9.80665;

struct INS_State {
    float p1_n_m, p2_n_m, p3_n_m;
    float v1_n_mps, v2_n_mps, v3_n_mps;
    Quaternion q_nb;
    float p_b_rps, q_b_rps, r_b_rps;
    float b1_g_rps, b2_g_rps, b3_g_rps;
    float b1_a_mps2, b2_a_mps2, b3_a_mps2;
};


void attitude_propagate(INS_State& ins, const IMU_Measurements& imu_meas, const float deltat_s);
void navigation_propagate(INS_State& ins, const IMU_Measurements& imu_meas, const float deltat_s);
void predict(INS_State& ins, const IMU_Measurements& imu_meas);