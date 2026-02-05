#include <Arduino.h>
#include "main.h"

void new_imu_reading() {
    readIMU(imu_meas);
}

void setup() {

    Wire.begin();
    if (!initializeIMU()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    performIMUTare();
    
    // Initialize Orientation {
    {
    constexpr float d2r = PI/180.0f;
    float roll_rad = 180.0f*d2r;
    float pitch_rad = 0.0f*d2r;
    float yaw_rad = 0.0f*d2r;
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

    // Attach IMU Interrupt
    if (digitalRead(IMU_DRDY_PIN)) readIMU(imu_meas);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);

    Serial.println("Setup complete.");
}

void loop() {
    delay(100);
}