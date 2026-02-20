#pragma once

#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include "sil.h"
#include "Measurements.h"

#define IMU_DRDY_PIN 12
#define IMU_ADDRESS 0x6B

extern SIL_MTi *imu;
extern float groundAltitude;

bool initializeIMU();
bool performIMUTare();
bool readIMU(IMU_Measurements& imu_meas, float t_s);

bool initializeBarometer();
bool performBarometerTare();
bool readBarometer(BARO_Measurements &baro_meas, float t_s);



#endif
