#pragma once
#include "main.h"
#include "SCServo.h"

#define SERVO_ID 1

#define SERVOSerial Serial2

#define SERVO_HOME 0  // update with closed position
#define SERVO_MAX 4095 // update with max position (25 deg flap angle for barometer safety)


#define OPEN_POSITION 250.0f;
#define CLOSE_POSITION 182.0f;

const float deg2servo = 4096.0 / 360;

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


