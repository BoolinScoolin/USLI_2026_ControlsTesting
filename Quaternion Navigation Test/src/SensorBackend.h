#pragma once

#include "Measurements.h"

// Abstract base class:
//  General sensor class blueprint
class SensorBackend {
    public:
    
        // 
        virtual ~SensorBackend() = default;

        // sensor functions
        virtual bool initializeIMU() = 0;
        virtual bool performIMUTare() = 0;
        virtual bool readIMU(IMU_Measurements& imu_meas) = 0;
        virtual bool initializeBarometer() = 0;
        virtual bool performBarometerTare() = 0;
        virtual bool readBarometer(BARO_Measurements &baro_meas) = 0;
};