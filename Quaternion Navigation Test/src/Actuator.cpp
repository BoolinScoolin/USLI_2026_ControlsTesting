#include "Actuator.h"

CTServo::CTServo(uint8_t id)
: _id(id), _position_deg(0.0f) {
}

void CTServo::initialize() {
    SERVOSerial.begin(1000000, SERIAL_8N1);
    _st.pSerial = &SERVOSerial;
}

void CTServo::writePosition(float deg) {
    float ticks = deg*deg2servo;
    _st.WritePosEx(_id, ticks, 4095, 255);
}

void CTServo::readPosition() {
    int pos_ach = _st.ReadPos(_id);
    if (pos_ach < 0) {
        return;  // read failed
    }
    _position_deg = pos_ach/deg2servo;
}

float CTServo::getPositionDeg() const {
    return _position_deg;
}

