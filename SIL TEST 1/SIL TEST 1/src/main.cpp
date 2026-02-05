#include <Arduino.h>
#include "main.h"

IntervalTimer read_baro;
IMU_Measurements imu_meas = {0};
BARO_Measurements baro_meas = {0};
INS_State ins = {0};
FlightPhase currentPhase = ARMED;


// Calibration data
TUMBLE_Data calib_data = {0};
KalmanFilter KF;
bool read_baro_flag = false;
bool read_imu_flag = false;


void new_imu_reading() {
    uint32_t now = micros();
    readIMU(imu_meas);  // updates imu_meas with latest readings
    parse_reading(ins, currentPhase, imu_meas);  // computes attitude

    if (read_baro_flag) {
        readBarometer(baro_meas);  // updates baro_meas with latest barometer readings
        ins.p3_n_m = baro_meas.baroAltitude - groundAltitude;  // subtracts tare
        KF.update(ins.p3_n_m, now);  // updates KF state
        ins.p3_n_m = KF.getAltitude();  // extracts KF state and stores in ins object
        ins.v3_n_mps = KF.getVelocity();
        read_baro_flag = false; 
    }

}

void read_baro_ISR() {
    read_baro_flag = true;
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Wire.begin();
    if (!initializeIMU() || !initializeBarometer()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    performIMUTare();
    performBarometerTare();
    
    // Initialize Orientation {
    delay(1000);
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
    {
    constexpr float d2r = PI/180.0f;
    float roll_rad = imu_meas.roll_deg*d2r;
    float pitch_rad = -imu_meas.pitch_deg*d2r;
    float yaw_rad = -imu_meas.yaw_deg*d2r;
    ins.q_nb = eul2quat(roll_rad, pitch_rad, yaw_rad);
    }

    // Setup SD Card
    if (!SD.begin(BUILTIN_SDCARD)) {
        tone(BUZZER_PIN, NOTE_E8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_C8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A7, 500);
        delay(2000);

        // Two beeps for bad SD Card read
        tone(BUZZER_PIN, NOTE_D8, 500);
        delay(600);
        tone(BUZZER_PIN, NOTE_D8, 500);
        while (true);
    }

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
    Serial.print(ins.tumble_calibration_data.invGainMatrix[0][0], 6);
    Serial.print(" ");
    Serial.print(ins.tumble_calibration_data.invGainMatrix[0][1], 6);
    Serial.print(" ");
    Serial.println(ins.tumble_calibration_data.invGainMatrix[0][2], 6);
    Serial.print(ins.tumble_calibration_data.invGainMatrix[1][0], 6);
    Serial.print(" ");
    Serial.print(ins.tumble_calibration_data.invGainMatrix[1][1], 6);
    Serial.print(" ");
    Serial.println(ins.tumble_calibration_data.invGainMatrix[1][2], 6);
    Serial.print(ins.tumble_calibration_data.invGainMatrix[2][0], 6);
    Serial.print(" ");
    Serial.print(ins.tumble_calibration_data.invGainMatrix[2][1], 6);
    Serial.print(" ");
    Serial.println(ins.tumble_calibration_data.invGainMatrix[2][2], 6);

    Serial.print("\n");
    Serial.print(ins.tumble_calibration_data.axisOffset[0], 6);
    Serial.print(" ");
    Serial.print(ins.tumble_calibration_data.axisOffset[1], 6);
    Serial.print(" ");
    Serial.println(ins.tumble_calibration_data.axisOffset[2], 6);

    delay(2000);

    // Attach IMU Interrupt
    if (digitalRead(IMU_DRDY_PIN)) readIMU(imu_meas);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);

    // Attach barometer intervaltimer
    read_baro.begin(read_baro_ISR, 100000);

    Serial.println("Setup complete.");
}

void loop() {

    uint32_t now = micros();
    static uint32_t last_print_us = 0;
    updateFlightPhase(ins);

    if (currentPhase == ARMED) {
        static RollingMeanFifo<600> groundAlt;
        static uint32_t last_groundalt_update_ms = millis();
        groundAlt.push(baro_meas.baroAltitude);
        if (millis() - last_groundalt_update_ms > 60*1000) {
            groundAltitude = groundAlt.mean();
            last_groundalt_update_ms = millis();
        }
    }

    if (shouldControl()) {
        checkLockout(ins, currentPhase);
    }


    // DEBUG PRINTS
    if (true && now - last_print_us > 20000) {

        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        quat2eul(ins.q_nb, roll_rad, pitch_rad, yaw_rad);

        Serial.print(roll_rad*180/PI, 4);
        Serial.print(" ");
        Serial.print(pitch_rad*180/PI,4);
        Serial.print(" ");
        Serial.print(yaw_rad*180/PI,4);
        Serial.print("    ");

        float pitch_rate_rps = sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps);
        Serial.print(pitch_rate_rps, 4);
        Serial.print("   ");

        Serial.print(baro_meas.baroAltitude - groundAltitude, 4);
        Serial.print("  ");
        Serial.print(ins.p3_n_m, 4);
        Serial.print("  ");
        Serial.print(ins.v3_n_mps, 4);
        Serial.print("    ");

        Serial.print(ins.accel_2_norm);
        Serial.print("   ");

        Serial.print(imu_meas.IMU_dt_us);
        Serial.print("   ");
        
        Serial.println(currentPhase);

        last_print_us = now;
    }
}