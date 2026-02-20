
#include "Sensors.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

SIL_MTi sil_imu;
SIL_MTi *imu = &sil_imu;
SIL_Adafruit_BMP3XX baro;


float imuTare[6] = {0.0f};
bool imuTared = false;
float groundAltitude = 0.0f;

bool initializeIMU() {
    return true;
}

// bool performIMUTare() {
//     float sum[6] = {0};
//     int samples = 0;
//     unsigned long start = millis();

//     while (millis() - start < 10000) {
//         if (digitalRead(IMU_DRDY_PIN)) {
//             imu->readMessages();
//             float *acc  = imu->getAcceleration();
//             float *gyro = imu->getRateOfTurn();

//             sum[0] += acc[0];
//             sum[1] += acc[1];
//             sum[2] += acc[2];
//             sum[3] += gyro[0];
//             sum[4] += gyro[1];
//             sum[5] += gyro[2];
//             samples++;
//         }
//         delay(10);
//     }

//     if (samples > 0) {
//         for (int i = 0; i < 6; i++) {
//             imuTare[i] = sum[i] / samples;
//         }
//         imuTared = true;
//         return true;
//     }
//     return false;
// }

bool readIMU(IMU_Measurements &imu_meas, float t_s) {
    if (!imu) {
        Serial.println("!imu proc.");
        return false;
    }

    imu->readMessages(t_s);
    
    float *acc  = imu->getAcceleration();
    float *gyro = imu->getRateOfTurn();
    float *eulerAngles = imu->getEulerAngles();

    if (imuTared) {
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0] - imuTare[3];
        imu_meas.gyroY  = gyro[1] - imuTare[4];
        imu_meas.gyroZ  = gyro[2] - imuTare[5];
        imu_meas.roll_deg = eulerAngles[0];
        imu_meas.pitch_deg = eulerAngles[1];
        imu_meas.yaw_deg = eulerAngles[2];
    } else {
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0];
        imu_meas.gyroY  = gyro[1];
        imu_meas.gyroZ  = gyro[2];
        imu_meas.roll_deg = eulerAngles[0];
        imu_meas.pitch_deg = eulerAngles[1];
        imu_meas.yaw_deg = eulerAngles[2];
    }

    // Timer updates
    uint32_t now = micros();
    imu_meas.IMU_dt_us = now - imu_meas.last_IMU_reading_time_us;
    imu_meas.last_IMU_reading_time_us = now;

    return true;
}

bool initializeBarometer() {
    return true;
}

// bool performBarometerTare() {
//     float altSum = 0.0f;
//     int altSamples = 0;

//     const int DISCARD_READINGS = 10;
//     const int TOTAL_READINGS   = 110;  // 10 discard + 100 used

//     int validCount = 0;

//     for (int i = 0; i < TOTAL_READINGS; i++) {
//         if (baro.performReading()) {
//             float alt = baro.readAltitude(1013.25);

//             // Skip first few valid readings
//             if (validCount >= DISCARD_READINGS) {
//                 altSum += alt;
//                 altSamples++;
//             }

//             validCount++;
//         }
//         delay(10);
//     }  

//     if (altSamples > 0) {
//         groundAltitude = altSum / altSamples;
//         Serial.print("\n Barometer Tare: ");
//         Serial.println(groundAltitude);
//         return true;
//     } else {
//         return false;
//     }
// }

bool readBarometer(BARO_Measurements &baro_meas, float t_s) {

    baro_meas.baroAltitude    = baro.readAltitude(1013.25f, t_s);
    // baro_meas.baroPressure    = baro.pressure / 100.0f;
    // baro_meas.baroTemperature = baro.temperature;
    
    return true;
}