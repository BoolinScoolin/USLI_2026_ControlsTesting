#include "Navigation.h"

void correctImuMeasurements(INS_State& ins, const IMU_Measurements& imu_meas) {
    estimate_true_accel(&ins.tumble_calibration_data, &imu_meas);
}

void attitude_propagate(INS_State& ins, const IMU_Measurements& imu_meas) {

    // Time since last prop step
    constexpr float US_TO_S = 1.0e-6f;
    float deltat_s = imu_meas.IMU_dt_us * US_TO_S; 

    // Clamp time
    if (deltat_s <= 0.0f || deltat_s > 1.0f) {
        return;
    }

    // Angular rates = gyro readings - gyro biases
    float p_b_rps = -(imu_meas.gyroX - ins.b1_g_rps);
    float q_b_rps = imu_meas.gyroY - ins.b2_g_rps;
    float r_b_rps = -(imu_meas.gyroZ - ins.b3_g_rps);

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

    // Update ins rates
    ins.p_b_rps = p_b_rps;
    ins.q_b_rps = q_b_rps;
    ins.r_b_rps = r_b_rps;
}

void compute_attitude(INS_State& ins, FlightPhase& currentPhase, const IMU_Measurements& imu_meas) {
    if (currentPhase == ARMED) {
        const float d2r = PI/180;
        float phi_rad = -imu_meas.roll_deg*d2r;
        float theta_rad = imu_meas.pitch_deg*d2r;
        float psi_rad = -imu_meas.yaw_deg*d2r;
        ins.p_b_rps = -(imu_meas.gyroX - ins.b1_g_rps);
        ins.q_b_rps = imu_meas.gyroY - ins.b2_g_rps;
        ins.r_b_rps = -(imu_meas.gyroZ - ins.b3_g_rps);
        ins.q_nb = eul2quat(phi_rad, theta_rad, psi_rad);
    } else {
        attitude_propagate(ins, imu_meas);
    }
}

void parse_reading(INS_State& ins, FlightPhase& currentPhase, const IMU_Measurements& imu_meas) {
    compute_attitude(ins, currentPhase, imu_meas);
    correctImuMeasurements(ins, imu_meas);

    ins.accel_2_norm = sqrt(ins.tumble_calibration_data.estimatedTrueAcceleration[0] * ins.tumble_calibration_data.estimatedTrueAcceleration[0] +
                            ins.tumble_calibration_data.estimatedTrueAcceleration[1] * ins.tumble_calibration_data.estimatedTrueAcceleration[1] +
                            ins.tumble_calibration_data.estimatedTrueAcceleration[2] * ins.tumble_calibration_data.estimatedTrueAcceleration[2]) -
                            gz_n_mps2;
}

void initialize_orientation(INS_State& ins, IMU_Measurements& imu_meas) {
    Serial.println("Initializing Orientation...");
    while(!digitalRead(IMU_DRDY_PIN)) { /* wait */ };
    readIMU(imu_meas);
    delay(10);
    readIMU(imu_meas);
    delay(10);
    readIMU(imu_meas);
    delay(10);
    readIMU(imu_meas);
    delay(10);
    Serial.print(imu_meas.roll_deg);
    Serial.print(" ");
    Serial.print(imu_meas.pitch_deg);
    Serial.print(" ");
    Serial.println(imu_meas.yaw_deg);
    constexpr float d2r = PI/180.0f;
    float roll_rad = imu_meas.roll_deg*d2r;
    float pitch_rad = -imu_meas.pitch_deg*d2r;
    float yaw_rad = -imu_meas.yaw_deg*d2r;
    ins.q_nb = eul2quat(roll_rad, pitch_rad, yaw_rad);
}

void apply_imu_calibration(INS_State& ins) {
    // Parse IMU calibration data from SD Card
    float invGainMatrixValues[9] = {
        1.002844,-0.007338,-0.002772,0.011808,1.001216,-0.032589,-0.004214,0.031073,1.001119
    };
    float axisOffsetValues[3] = {
        0.000454,-0.032508,-0.008450
    };
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            ins.tumble_calibration_data.invGainMatrix[row][col] =
                invGainMatrixValues[row * 3 + col];
        }
        ins.tumble_calibration_data.axisOffset[row] = axisOffsetValues[row];
    }
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[0][0], 6);
    // Serial.print(" ");
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[0][1], 6);
    // Serial.print(" ");
    // Serial.println(ins.tumble_calibration_data.invGainMatrix[0][2], 6);
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[1][0], 6);
    // Serial.print(" ");
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[1][1], 6);
    // Serial.print(" ");
    // Serial.println(ins.tumble_calibration_data.invGainMatrix[1][2], 6);
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[2][0], 6);
    // Serial.print(" ");
    // Serial.print(ins.tumble_calibration_data.invGainMatrix[2][1], 6);
    // Serial.print(" ");
    // Serial.println(ins.tumble_calibration_data.invGainMatrix[2][2], 6);

    // Serial.print("\n");
    // Serial.print(ins.tumble_calibration_data.axisOffset[0], 6);
    // Serial.print(" ");
    // Serial.print(ins.tumble_calibration_data.axisOffset[1], 6);
    // Serial.print(" ");
    // Serial.println(ins.tumble_calibration_data.axisOffset[2], 6);
}