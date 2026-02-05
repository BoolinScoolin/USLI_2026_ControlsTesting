
#include "Sensors.h"

MTi *imu = nullptr;

float imuTare[6] = {0.0f};
bool imuTared = false;
float groundAltitude = 0.0f;

bool initializeIMU() {

    pinMode(IMU_DRDY_PIN, INPUT);
    imu = new MTi(IMU_ADDRESS, IMU_DRDY_PIN);

    if (!imu->detect(1000)) {
        return false;
    }

    imu->goToConfig();
    delay(100);
    imu->requestDeviceInfo();
    delay(100);
    imu->configureOutputs();
    delay(100);
    imu->goToMeasurement();
    delay(100);
    delay(500);

    return true;
}

void performIMUTare() {
    float sum[6] = {0};
    int samples = 0;
    unsigned long start = millis();

    while (millis() - start < 10000) {
        if (digitalRead(IMU_DRDY_PIN)) {
            imu->readMessages();
            float *acc  = imu->getAcceleration();
            float *gyro = imu->getRateOfTurn();

            sum[0] += acc[0];
            sum[1] += acc[1];
            sum[2] += acc[2];
            sum[3] += gyro[0];
            sum[4] += gyro[1];
            sum[5] += gyro[2];
            samples++;
        }
        delay(10);
    }

    if (samples > 0) {
        for (int i = 0; i < 6; i++) {
            imuTare[i] = sum[i] / samples;
        }
        imuTared = true;

    }
}

bool readIMU(Measurements &measurement) {
    if (!imu) return;
    
    // Only read if data is ready
    if (!digitalRead(IMU_DRDY_PIN)) return;
    
    imu->readMessages();
    
    float *acc  = imu->getAcceleration();
    float *gyro = imu->getRateOfTurn();

    // PLACEHOLDER for now
    //float *eulerAngles = imu->getEulerAngles();

    if (imuTared) {
        measurement.accelX = acc[0] - imuTare[0];
        measurement.accelY = acc[1] - imuTare[1];
        measurement.accelZ = acc[2] - imuTare[2];
        measurement.gyroX  = gyro[0] - imuTare[3];
        measurement.gyroY  = gyro[1] - imuTare[4];
        measurement.gyroZ  = gyro[2] - imuTare[5];
        //yeah ik the function doesn't work but filler for now
        measurement.roll = 0.0;//eulerAngles[0];
        measurement.pitch = 0.0; //eulerAngles[1];
        measurement.yaw = 0.0; //eulerAngles[2];
    } else {
        measurement.accelX = acc[0];
        measurement.accelY = acc[1];
        measurement.accelZ = acc[2];
        measurement.gyroX  = gyro[0];
        measurement.gyroY  = gyro[1];
        measurement.gyroZ  = gyro[2];
        //yeah ik the function doesn't work but filler for now
        measurement.roll = 0.0; //eulerAngles[0];
        measurement.pitch = 0.0; //eulerAngles[1];
        measurement.yaw = 0.0; //eulerAngles[2];
    }

    // Timer updates
    uint32_t now = micros();
    measurement.IMU_dt_us = now - measurement.last_IMU_reading_time_us;
    measurement.last_IMU_reading_time_us = now;

    return true;
}
