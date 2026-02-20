#pragma once

#include "sil_data.h"

int findTimeIndexBinary(float t_runtime);

class SIL_MTi {
  public:

    float euler_rad[3];
    float accel_mps2[3];
    float gyro_rps[3];

    void readMessages(float t_s);

    // custom
    float* getEulerAngles();
    float* getAcceleration();
    float* getRateOfTurn();

};

class SIL_Adafruit_BMP3XX {
    public:
        float readAltitude(float sea_level_pressure_pa, float t_s);
};

float sil_true_alt(float t_s);
