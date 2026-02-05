#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include "MTi.h"
#include "Measurements.h"

#define IMU_DRDY_PIN 20
#define IMU_ADDRESS 0x6B

extern MTi *imu;
extern float groundAltitude;

bool initializeIMU();
bool performIMUTare();
bool readIMU(IMU_Measurements& imu_meas);

bool initializeBarometer();
bool performBarometerTare();
bool readBarometer(BARO_Measurements &baro_meas);



#endif
