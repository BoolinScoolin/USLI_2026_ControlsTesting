#include "main.h"

IntervalTimer read_baro;
IntervalTimer sil_imu_drdy;
IMU_Measurements imu_meas = {0};
BARO_Measurements baro_meas = {0};
INS_State ins = {0};
FlightPhase currentPhase = ARMED;
ApogeeController controller;


// Calibration data
TUMBLE_Data calib_data = {0};
KalmanFilter KF;
File output_file;
bool read_baro_flag = false;
bool read_imu_flag = false;
volatile bool update_phase_flag = false;
volatile bool write_data_flag = false;

// Servo
CTServo servo(SERVO_ID);

// SIL TESTING: LOOP START TIME OBJECT
float now_s;
uint32_t start_time = 0;

void new_imu_reading() {
    uint32_t now = micros() - start_time;
    now_s = now/1000000.0;
    readIMU(imu_meas, now_s);  // updates imu_meas with latest readings
    parse_reading(ins, currentPhase, imu_meas);  // computes attitude

    if (read_baro_flag) {
        readBarometer(baro_meas, now_s);  // updates baro_meas with latest barometer readings
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
    }   
}

void read_baro_ISR() {
    read_baro_flag = true;
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    Serial.println("Beginning Setup...");

    // Wire.begin();

    // Setup Servo
    servo.initialize();
    servo.writePosition(CLOSE_POSITION);

    // setup controller
    controller.initialize();
    

    if (!initializeIMU() || !initializeBarometer()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    // performIMUTare();
    // performBarometerTare();
    
    // Initialize Orientation {
    uint32_t now = micros();
    static float now_s = now/1000000.0;
    readIMU(imu_meas, now_s);
    delay(10);
    now = micros();
    now_s = now/1000000.0;
    readIMU(imu_meas, now_s);
    delay(10);
    now = micros();
    now_s = now/1000000.0;
    readIMU(imu_meas, now_s);
    delay(10);
    now = micros();
    now_s = now/1000000.0;
    readIMU(imu_meas, now_s);
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
    // Make new file
        // Remove old debug file if exists
    if (SD.exists(OUTPUT_FILENAME)) {
        SD.remove(OUTPUT_FILENAME);
    }
    output_file = SD.open(OUTPUT_FILENAME, FILE_WRITE);
    output_file.print("Time [s]");
    output_file.print(",");
    output_file.print("ax_b_mps2");
    output_file.print(",");
    output_file.print("ax_b_mps2");
    output_file.print(",");
    output_file.print("az_b_mps2");
    output_file.print(",");
    output_file.print("gx_b_rps");
    output_file.print(",");
    output_file.print("gy_b_rps");
    output_file.print(",");
    output_file.print("gz_b_rps");
    output_file.print(",");
    output_file.print("p3_n_m");
    output_file.print(",");
    output_file.print("v3_n_mps");
    output_file.print(",");
    output_file.print("q0");
    output_file.print(",");
    output_file.print("q1");
    output_file.print(",");
    output_file.print("q2");
    output_file.print(",");
    output_file.print("q3");
    output_file.print(",");
    output_file.println("state");
    output_file.print(",");
    output_file.print("encoder");
    output_file.print(",");
    output_file.print("command");


    // Parse IMU calibration data from SD Card
    float invGainMatrixValues[9] = {
        1.00,0.00,0.00,0.00,1.00,0.00,0.00,0.00,1.00
    };
    float axisOffsetValues[3] = {
        0.000,0.00,0.00
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

    start_time = micros();

    // Attach barometer intervaltimer
    read_baro.begin(read_baro_ISR, 100000);

    // Attach SIL intervaltimer to replace IMU DRDY pin
    sil_imu_drdy.begin(new_imu_reading, 10000);}

void loop() {

    uint32_t now = micros();
    static uint32_t last_print_us = 0;

    if (update_phase_flag) {
        updateFlightPhase(ins);
        update_phase_flag = false;
    }

    if (currentPhase == ARMED) {
        static RollingMeanFifo<600> groundAlt;
        static uint32_t last_groundalt_update_ms = millis();
        groundAlt.push(baro_meas.baroAltitude);
        if (millis() - last_groundalt_update_ms > 60*1000) {
            groundAltitude = groundAlt.mean();
            last_groundalt_update_ms = millis();
        }
    }

    if (controller.shouldControl(currentPhase)) {
        if(checkLockout(ins, currentPhase)) {
            controller.shouldControl(currentPhase);
        }
        else {
            controller.update(ins, currentPhase);
            servo.writePosition(controller.getActuatorCommand());
        }
    }

    // if (now - imu_meas.last_IMU_reading_time_us > 100000) {
    //     readIMU(imu_meas, now_s);
    // }

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
        output_file.println(currentPhase);
        output_file.print(",");
        output_file.print(servo.getPositionDeg(), 4);
        output_file.print(",");
        output_file.print(controller.getActuatorCommand(), 4);

        c++;
        if (c >= 200) {
            output_file.flush();
            Serial.println("Flushed.");
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

        // static float max_prrps = 0.0f;
        // float pitch_rate_rps = sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps);
        // if (pitch_rate_rps > max_prrps) {
        //     max_prrps = pitch_rate_rps;
        // }
        // Serial.print(max_prrps, 4);
        // Serial.print("   ");

        // Serial.print(baro_meas.baroAltitude - groundAltitude, 4);
        // Serial.print("  ");
        Serial.print(ins.p3_n_m, 4);
        Serial.print("  ");

        // Serial.print(ins.v3_n_mps, 4);
        // Serial.print("    ");

        // Serial.print(ins.accel_2_norm);
        // Serial.print("   ");

        // Serial.print( pitch_rad );
        // Serial.print("    ");
        // Serial.print( sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps) );
        // Serial.print("    ");

        // Serial.print(imu_meas.IMU_dt_us);
        // Serial.print("   ");

        servo.readPosition();
        Serial.print(servo.getPositionDeg());
        Serial.print("   ");
        Serial.print(controller.getActuatorCommand());
        Serial.print("   ");

        float true_z_m = sil_true_alt(now_s);
        Serial.print(currentPhase);
        Serial.print("    SIL [t z]: ");
        Serial.println(now_s);

        last_print_us = now;
    }
}