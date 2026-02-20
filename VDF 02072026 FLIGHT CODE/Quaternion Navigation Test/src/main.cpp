/*
This is the flight code for UF's Swamp Launch Rocket Team USLI VDF on February 7th, 2026.
Generative AI was used in the creation of this code.

The macros SERVO_OPEN and SERVO_CLOSE were changed a little since landing.

Authors:    Jindamai, Yim and Riba, Colin
*/

#include <Arduino.h>
#include "main.h"

IntervalTimer read_baro;
IMU_Measurements imu_meas = {0};
BARO_Measurements baro_meas = {0};
INS_State ins = {0};
FlightPhase currentPhase = ARMED;
ApogeeController controller;


// Calibration data
TUMBLE_Data calib_data = {0};
KalmanFilter KF;

volatile bool read_baro_flag = false;
bool read_imu_flag = false;
volatile bool update_phase_flag = false;
volatile bool write_data_flag = false;

// Servo
CTServo servo(SERVO_ID);


void new_imu_reading() {
    uint32_t now = micros();
    readIMU(imu_meas);  // updates imu_meas with latest readings
    parse_reading(ins, currentPhase, imu_meas);  // computes attitude

    if (read_baro_flag) {
        readBarometer(baro_meas);  // updates baro_meas with latest barometer readings
        ins.p3_n_m = baro_meas.baroAltitude - groundAltitude;  // subtracts tare
    }

    KF.update(ins.p3_n_m, now);  // updates KF state
    ins.p3_n_m = KF.getAltitude();  // extracts KF state and stores in ins object
    ins.v3_n_mps = KF.getVelocity();
    read_baro_flag = false; 
    update_phase_flag = true;

    // Logging
    static uint32_t k = 0;
    k++;
    if (k >= 10) {
        write_data_flag = true;
        k = 0;
    }   

}

void read_baro_ISR() {
    read_baro_flag = true;
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // LED Initialize
    pinMode(RGB_R_PIN, OUTPUT);
    pinMode(RGB_G_PIN, OUTPUT);
    pinMode(RGB_B_PIN, OUTPUT);
    digitalWrite(RGB_R_PIN, HIGH);

    // Chime to signify start of initialization
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, NOTE_B7, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_D8, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100);
    delay(2000);

    Wire2.begin();
    
    // Setup Servo
    servo.initialize();
    servo.writePosition(SERVO_CLOSE);

    // setup controller
    controller.initialize();

    if (!initializeIMU() || !initializeBarometer(Wire2)) {
        digitalWrite(RGB_R_PIN, LOW);
        while (1) {
            digitalWrite(RGB_B_PIN, !digitalRead(RGB_B_PIN)); // blink LED if sensor init fails
            tone(BUZZER_PIN, NOTE_B7, 50);      // Every 2 seconds for bad sensor
            delay(2000);
        }
    }

    makeOutputFile();
    writeHeaders();
    apply_imu_calibration(ins);

    performIMUTare();
    performBarometerTare();
    initialize_orientation(ins, imu_meas);

    // Attach IMU Interrupt
    if (digitalRead(IMU_DRDY_PIN)) readIMU(imu_meas);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);

    // Attach barometer intervaltimer
    read_baro.begin(read_baro_ISR, 100000);

    tone(BUZZER_PIN, NOTE_A8, 100);
    delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100);
    delay(1000);
}

void loop() {

    uint32_t now = micros();
    static uint32_t last_print_us = 0;

    // State detection
    if (update_phase_flag) {
        updateFlightPhase(ins);
        update_phase_flag = false;
    }

    if (currentPhase == ARMED) {

        // this is designed to re-tare ground altitude every minute
        static RollingMeanFifo<600> groundAlt;
        static uint32_t last_groundalt_update_us = micros();
        groundAlt.push(baro_meas.baroAltitude);
        if (now - last_groundalt_update_us > 60*1000000) {
            groundAltitude = groundAlt.mean();
            last_groundalt_update_us = micros();
        }

        // This just blinks the LED if its armed
        static uint32_t last_blink_update_us = micros();
        if (now - last_blink_update_us > 1*1000000) {
            digitalWrite(RGB_R_PIN, !digitalRead(RGB_R_PIN));
            last_blink_update_us = micros();
        }

        static uint32_t last_beep_update_us = micros();
        // This just beeps if its armed
        if (now - last_beep_update_us > 10*1000000) {
            tone(BUZZER_PIN, NOTE_G8, 100);
            last_beep_update_us = micros();
        }
    }

    // Controls stuff
    if (controller.shouldControl(currentPhase)) {
        if(checkLockout(ins, currentPhase)) {
            controller.shouldControl(currentPhase);
        }
        else if (currentPhase == COASTING) { 
            controller.update(ins, currentPhase);
            servo.writePosition(controller.getActuatorCommand());
        }
    }

    // Write data
    static uint32_t c = 0;  // Flush counter
    if (write_data_flag) {
        servo.readPosition();
        output_file.print(now*1.0e-6f,6);
        output_file.print(",");
        output_file.print(imu_meas.accelX, 6);
        output_file.print(",");
        output_file.print(imu_meas.accelY, 6);
        output_file.print(",");
        output_file.print(imu_meas.accelZ, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroX, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroY, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroZ, 6);
        output_file.print(",");
        output_file.print(baro_meas.baroAltitude, 6);
        output_file.print(",");
        output_file.print(ins.p3_n_m);
        output_file.print(",");
        output_file.print(ins.v3_n_mps);
        output_file.print(",");
        output_file.print(ins.q_nb.q0);
        output_file.print(",");
        output_file.print(ins.q_nb.q1);
        output_file.print(",");
        output_file.print(ins.q_nb.q2);
        output_file.print(",");
        output_file.print(ins.q_nb.q3);
        output_file.print(",");
        output_file.print(currentPhase);
        output_file.print(",");
        output_file.print(servo.getPositionDeg(), 4);
        output_file.print(",");
        output_file.println(controller.getActuatorCommand(), 4);

        c++;
        if (c >= 200) {
            output_file.flush();
            c = 0;
        }

        write_data_flag = false;
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

        // float pitch_rate_rps = sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps);
        // Serial.print(pitch_rate_rps, 4);
        // Serial.print("   ");

        Serial.print(ins.accel_2_norm);
        Serial.print("   ");

        Serial.print(baro_meas.baroAltitude - groundAltitude, 4);
        Serial.print("      ");
        
        Serial.print(ins.p3_n_m, 4);
        Serial.print("  ");
        Serial.print(ins.v3_n_mps, 4);
        Serial.print("      ");

        Serial.print(imu_meas.IMU_dt_us);
        Serial.print("   ");

        Serial.print(servo.getPositionDeg());
        Serial.print("   ");

        
        Serial.println(currentPhase);

        last_print_us = now;
    }
}