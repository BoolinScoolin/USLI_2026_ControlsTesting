#include "Navigation.h"

void attitude_propagate(INS_State& ins, const IMU_Measurements& imu_meas, const float deltat_s) {

    // Angular rates = gyro readings - gyro biases
    float p_b_rps = imu_meas.gyroX - ins.b1_g_rps;
    float q_b_rps = imu_meas.gyroY - ins.b2_g_rps;
    float r_b_rps = imu_meas.gyroZ - ins.b3_g_rps;

    // Compute sigma
    float sigma_x_rad = p_b_rps*deltat_s;
    float sigma_y_rad = q_b_rps*deltat_s;
    float sigma_z_rad = r_b_rps*deltat_s;
    float sigma_rad_sqrd =
        sigma_x_rad * sigma_x_rad +
        sigma_y_rad * sigma_y_rad +
        sigma_z_rad * sigma_z_rad;

    // Compute coefficients
    // float ac = 1.0f - sigma_rad_sqrd/8.0f;
    // float as = 0.5f - sigma_rad_sqrd/48.0f;
    
    float sigma_rad = std::sqrt(sigma_rad_sqrd);
    float ac = std::cos(sigma_rad/2);
    float as = std::sin(sigma_rad/2)/sigma_rad;
    
    // Compute rotation quaternion
    Quaternion rq_k = {ac, as*sigma_x_rad, as*sigma_y_rad, as*sigma_z_rad};

    // Update quaternion
    ins.q_nb = quat_normalize(quat_mult(ins.q_nb, rq_k));
}

    float ax_n_mps2;
    float ay_n_mps2;
    float az_n_mps2;
void navigation_propagate(INS_State& ins, const IMU_Measurements& imu_meas, const float deltat_s) {
    
    // Specific force in body frame
    float ax_b_mps2 = imu_meas.accelX - ins.b1_a_mps2;
    float ay_b_mps2 = imu_meas.accelY - ins.b2_a_mps2;
    float az_b_mps2 = imu_meas.accelZ - ins.b3_a_mps2;

    // Resolve acceleration in body frame
    // float ax_n_mps2;
    // float ay_n_mps2;
    // float az_n_mps2;
    quat_transform(ins.q_nb,
                   ax_b_mps2, ax_n_mps2,
                   ay_b_mps2, ay_n_mps2,
                   az_b_mps2, az_n_mps2);

    // Gravity compensation
    az_n_mps2 += gz_n_mps2;

    // Propagation
    const float half_dt2 = 0.5f * deltat_s*deltat_s;
    ins.p1_n_m += ins.v1_n_mps*deltat_s + ax_n_mps2*half_dt2;
    ins.p2_n_m += ins.v2_n_mps*deltat_s + ay_n_mps2*half_dt2;
    ins.p3_n_m += ins.v3_n_mps*deltat_s + az_n_mps2*half_dt2;
    ins.v1_n_mps += ax_n_mps2*deltat_s;
    ins.v2_n_mps += ay_n_mps2*deltat_s;
    ins.v3_n_mps += az_n_mps2*deltat_s;
    
}

void predict(INS_State& ins, const IMU_Measurements& imu_meas) {

    // Time since last prop step
    constexpr float US_TO_S = 1.0e-6f;
    float deltat_s = imu_meas.IMU_dt_us * US_TO_S; 

    // Clamp time
    if (deltat_s <= 0.0f || deltat_s > 1.0f) {
        return;
    }

    attitude_propagate(ins, imu_meas, deltat_s);
    navigation_propagate(ins, imu_meas, deltat_s);

    // PRINTS
    if (millis() % 100 < 10) {
        float phi_rad, theta_rad, psi_rad;
        quat2eul(ins.q_nb, phi_rad, theta_rad, psi_rad);

        // Serial.print(imu_meas.accelX, 6);
        // Serial.print(" ");
        // Serial.print(imu_meas.accelY, 6);
        // Serial.print(" ");
        // Serial.print(imu_meas.accelZ, 6);
        // Serial.print("   ");

        // Serial.print(ax_n_mps2, 6);
        // Serial.print(" ");
        // Serial.print(ay_n_mps2, 6);
        // Serial.print(" ");
        // Serial.print(az_n_mps2, 6);
        // Serial.print("   ");

        // Serial.print(quat_norm2(ins.q_nb), 6);
        // Serial.print("   ");
        
        Serial.print(phi_rad*180.0/PI, 6);
        Serial.print(" ");
        Serial.print(theta_rad*180.0/PI, 6);
        Serial.print(" ");
        Serial.print(psi_rad*180.0/PI, 6);
        Serial.print("   ");

        Serial.print(imu_meas.roll, 6);
        Serial.print(" ");
        Serial.print(imu_meas.pitch, 6);
        Serial.print(" ");
        Serial.print(imu_meas.yaw, 6);
        Serial.print("   ");

        // Serial.print(ins.p1_n_m, 6);
        // Serial.print(" ");
        // Serial.print(ins.p2_n_m, 6);
        // Serial.print(" ");
        // Serial.print(ins.p3_n_m, 6);
        // Serial.print("   ");

        // Serial.print(ins.v1_n_mps, 6);
        // Serial.print(" ");
        // Serial.print(ins.v2_n_mps, 6);
        // Serial.print(" ");
        // Serial.print(ins.v3_n_mps, 6);
        // Serial.print("   ");

        Serial.println(deltat_s, 6);
    }

}