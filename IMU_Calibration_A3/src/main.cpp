#include <Arduino.h>
#include "main.h"
#include "imu_calibration.h"

#define CALIBRATION_FILENAME "six_point_calibration.txt"
File calibrationFile;


IMU_Measurements imu_meas = {0};
BARO_Measurements baro_meas = {0};
INS_State ins = {0};

volatile bool imu_reading_new = false;

void new_imu_reading() {
    readIMU(imu_meas);
    imu_reading_new = true;
}

void setup() {

    Serial.begin(9600);
    delay(1000);
    Serial.println("Starting Calibration Setup.");

    // // Chime to signify start of initialization
    // pinMode(BUZZER_PIN, OUTPUT);
    // tone(BUZZER_PIN, NOTE_B7, 50);
    // delay(50);
    // tone(BUZZER_PIN, NOTE_D8, 50);
    // delay(50);
    // tone(BUZZER_PIN, NOTE_G8, 100);
    // delay(2000);

    Wire.begin();
    if (!initializeIMU()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
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
    // Remove calibration file & open new one
    if (SD.exists(CALIBRATION_FILENAME)) {
        SD.remove(CALIBRATION_FILENAME);
    }
    calibrationFile = SD.open(CALIBRATION_FILENAME, FILE_WRITE);

    // Attach IMU Interrupt
    if (digitalRead(IMU_DRDY_PIN)) readIMU(imu_meas);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);

    TUMBLE_Data calib_data = tumble_data_init(10, 30);
    run_6pt_tumble_calibration(&imu_meas, &calib_data);  // run calibration
    write_tumble_outputs(&calib_data, calibrationFile);  // write calibration data to SD

    Serial.println("\nCalibration complete.");
}

void loop() {
    delay(1000);
}