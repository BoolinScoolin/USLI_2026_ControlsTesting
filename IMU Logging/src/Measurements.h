#pragma once

// Current state (holds latest sensor readings)
struct Measurements {
    // IMU
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // rad/s
    float roll, pitch, yaw;
    uint32_t last_IMU_reading_time_us;
    uint32_t IMU_dt_us;

    // Barometer
    float baroAltitude;            // meters
    float baroPressure;            // hPa
    float baroTemperature;         // °C
    
    // GPS
    float gpsLat;
    float gpsLon;
    float gpsAltitude;
    float gpsSpeed;
    uint8_t gpsSatellites;
    bool gpsHasFix;
    float gpsXMeters;
    float gpsYMeters;
};
