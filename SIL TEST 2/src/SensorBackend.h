// SensorBackend.h
#pragma once
#include "Measurements.h"

class SensorBackend {
public:
  virtual ~SensorBackend() = default;

  virtual bool initializeIMU() = 0;
  virtual bool performIMUTare() = 0;
  virtual bool readIMU(IMU_Measurements& imu) = 0;

  virtual bool initializeBarometer() = 0;
  virtual bool performBarometerTare() = 0;
  virtual bool readBarometer(BARO_Measurements& baro) = 0;
};
