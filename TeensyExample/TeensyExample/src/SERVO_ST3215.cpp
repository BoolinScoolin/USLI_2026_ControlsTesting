#include <SERVO_ST3215.h>

SMS_STS st;

void setup_ST3215() {
  SERVOSerial.begin(1000000, SERIAL_8N1); // Open servo serial line at 1 mbps and 8N1 serial data format configuration
  st.pSerial = &SERVOSerial; // assign serial pointer to serial port
  delay(500);
  Serial.println("Servo Initialization done.");
}
