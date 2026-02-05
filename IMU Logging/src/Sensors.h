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
void performIMUTare();
bool readIMU(Measurements& measurement);

#endif
