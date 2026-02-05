#pragma once

// Includes
#include "Measurements.h"
#include "Navigation.h"
#include "Sensors.h"
#include "Quaternion.h"
#include "SD.h"

// IMU
#define IMU_DRDY_PIN 20
#define IMU_ADDRESS 0x6B

// Initialize beeper 
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

// Data logging
#define OUTPUT_FILENAME "IMU_Output.txt" // include .txt

// Calibration flag
extern volatile bool imu_reading_new;
