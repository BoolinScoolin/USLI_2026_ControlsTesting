#pragma once
#include "main.h"
#include "SCServo.h"

#define SERVO_ID 1

#define SERVOSerial Serial2

#define SERVO_HOME 0  // update with closed position
#define SERVO_MAX 4095 // update with max position (25 deg flap angle for barometer safety)

const float deg2servo = 4096.0 / 360;

#define SERVO_MAX_MAX 280.0f
#define SERVO_OPEN 265.0f
#define SERVO_CLOSE 190.0f

class CTServo {
public:
    CTServo(uint8_t id);

    void initialize();              // init servo object
    void writePosition(float deg);  // write servo deg
    void readPosition();     // return servo deg
    float getPositionDeg() const;


private:
    uint8_t _id;                    // scservo id
    float   _position_deg;          
    SMS_STS _st;                    // servo object
};


