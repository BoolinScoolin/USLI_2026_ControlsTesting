#include "Navigation.h"

void attitude_propagate(INS_State& ins, Measurements& measurement) {


    // Time since last prop step
    constexpr float US_TO_S = 1.0e-6f;
    float deltat_s = measurement.IMU_dt_us * US_TO_S; 

    // Clamp
    if (deltat_s <= 0.0f || deltat_s > 1.0f) {
        Serial.println("Clamped");
        return;
    }

    // Angular rates = gyro readings - gyro biases
    float p_b_rps = measurement.gyroX - ins.b1_g_rps;
    float q_b_rps = measurement.gyroY - ins.b2_g_rps;
    float r_b_rps = measurement.gyroZ - ins.b3_g_rps;

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
    Quaternion r_k = {ac, as*sigma_x_rad, as*sigma_y_rad, as*sigma_z_rad};

    // Update quaternion
    ins.q_nb = quat_normalize(quat_mult(ins.q_nb, r_k));


    if (millis() % 100 < 10) {
    // // QUATERNION PRINTS
    // Serial.print(ins.q_nb.q0);
    // Serial.print(" ");
    // Serial.print(ins.q_nb.q1);
    // Serial.print(" ");
    // Serial.print(ins.q_nb.q2);
    // Serial.print(" ");
    // Serial.print(ins.q_nb.q3);
    // Serial.print(" ");

    float phi_rad, theta_rad, psi_rad;
    quat2eul(ins.q_nb, phi_rad, theta_rad, psi_rad);

    // EULER ANGLE PRINTS
    Serial.print(phi_rad*180.0/PI, 6);
    Serial.print(" ");
    Serial.print(theta_rad*180.0/PI, 6);
    Serial.print(" ");
    Serial.print(psi_rad*180.0/PI, 6);
    Serial.print(" ");
    Serial.println(deltat_s, 6);

    // // GYRO PRINTS
    // Serial.print(p_b_rps, 6);
    // Serial.print(" ");
    // Serial.print(q_b_rps, 6);
    // Serial.print(" ");
    // Serial.println(r_b_rps, 6);  
    }
}

void predict(INS_State& ins, Measurements& measurement) {
    
    attitude_propagate(ins, measurement);

}