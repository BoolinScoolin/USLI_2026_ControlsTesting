#include <Arduino.h>
#include "main.h"
#include <SD.h>

#define IMU_DRDY_PIN 20
#define IMU_ADDRESS 0x6B
#define BUZZER_PIN 33

// Initialize beeper c:
#define BUZZER_PIN 33
const int NOTE_A7 = 3520;
const int NOTE_B7 = 3951;
const int NOTE_C8 = 4186;
const int NOTE_D8 = 4698;
const int NOTE_E8 = 5274;
const int NOTE_F8 = 5588;
const int NOTE_G8 = 6272;
const int NOTE_A8 = 7040;
const int NOTE_B8 = 7902;


Measurements measurement = {0};
INS_State ins = {0};

File output_file;
#define OUTPUT_FILENAME "IMU_Output.txt" // include .txt

volatile bool write_data_flag = false;

void new_imu_reading() {
    // readIMU(measurement);
    // predict(ins, measurement);

    readIMU(measurement);

    // // Write data
    // output_file.print(t_s,6);
    // output_file.print(",");
    // output_file.print(measurement.accelX, 6);
    // output_file.print(",");
    // output_file.print(measurement.accelY, 6);
    // output_file.print(",");
    // output_file.print(measurement.accelZ, 6);
    // output_file.print(",");
    // output_file.print(measurement.gyroX, 6);
    // output_file.print(",");
    // output_file.print(measurement.gyroY, 6);
    // output_file.print(",");
    // output_file.println(measurement.gyroZ, 6);

    // Serial.print(t_s);
    // Serial.println(",   Data Write.")

    static uint32_t k = 0;

    k++;

    if (k >= 10) {
        write_data_flag = true;
    }    
}

uint32_t last_flush_ms = millis();

void setup() {

    Wire.begin();
    if (!initializeIMU()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    performIMUTare();
    
    // Initialize Orientation
    constexpr float d2r = PI/180.0;
    float roll_rad = 0*d2r;
    float pitch_rad = 0*d2r;
    float yaw_rad = 0*d2r;
    ins.q_nb = eul2quat(roll_rad, pitch_rad, yaw_rad);

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

    // Remove old debug file if exists
    if (SD.exists(OUTPUT_FILENAME)) {
        SD.remove(OUTPUT_FILENAME);
    }

    // Make new file
    output_file = SD.open(OUTPUT_FILENAME, FILE_WRITE);

    // Headers
    output_file.print("time [s]");
    output_file.print(",");
    output_file.print("accelX [mps2]");
    output_file.print(",");
    output_file.print("accelY [mps2]");
    output_file.print(",");
    output_file.print("accelZ [mps2]");
    output_file.print(",");
    output_file.print("gyroX [rps]");
    output_file.print(",");
    output_file.print("gyroY [rps]");
    output_file.print(",");
    output_file.println("gyroZ [rps]");
    output_file.flush();

    // Attach IMU Interrupt
    if (digitalRead(IMU_DRDY_PIN)) readIMU(measurement);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);

    // Chime to signify start of logging
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, NOTE_B7, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_D8, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100);
    delay(2000);
    Serial.println("Setup complete.");
    last_flush_ms = millis();
}

void loop() {

    delay(50);

    // if (millis() - last_flush_ms > 10*1000) {
    //     output_file.flush();
    //     Serial.println("Flushed");
    //     last_flush_ms = millis();
    //     //while (1);
    // }

    static uint32_t c = 0;  // counter

    if (write_data_flag) {
        // Write data
        float t_s = measurement.last_IMU_reading_time_us * 1.0e-6f;
        output_file.print(t_s,6);
        output_file.print(",");
        output_file.print(measurement.accelX, 6);
        output_file.print(",");
        output_file.print(measurement.accelY, 6);
        output_file.print(",");
        output_file.print(measurement.accelZ, 6);
        output_file.print(",");
        output_file.print(measurement.gyroX, 6);
        output_file.print(",");
        output_file.print(measurement.gyroY, 6);
        output_file.print(",");
        output_file.println(measurement.gyroZ, 6);
        Serial.print(t_s);
        Serial.println(",   Data Write.");

        c++;

        if (c >= 200) {
            output_file.flush();
            Serial.println("Flushed.");
            c = 0;
        }

        write_data_flag = false;
    }
    
}