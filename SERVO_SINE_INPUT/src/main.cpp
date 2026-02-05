#include <Arduino.h>
#include <SCServo.h>
#include <SD.h>

// Initialize beeper c:
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

// Initialize Servo Parameters
SMS_STS st; // create servo object
#define SERVOSerial Serial4
#define SERVO_ID 1
const float deg2servo = 4096.0 / 360;
File output_file;
#define OUTPUT_FILENAME "servo_sine_0p5Hz.txt" // include .txt


// Sine wave parameter
float amplitude = 44.5*deg2servo;
float offset = 44.5*deg2servo;       // center position
float freq = 0.5;          // Hz
uint32_t t0;
uint32_t next_sample_us;
const uint32_t SAMPLE_PERIOD_US = 2000; // period in microseconds

void setup() {

  // Turn on led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Chime to signify start of initialization
  pinMode(BUZZER_PIN, OUTPUT);
  tone(BUZZER_PIN, NOTE_B7, 50);
  delay(50);
  tone(BUZZER_PIN, NOTE_D8, 50);
  delay(50);
  tone(BUZZER_PIN, NOTE_G8, 100);
  delay(2000);

  // Setup Servo
  SERVOSerial.begin(1000000, SERIAL_8N1); // Open servo serial line at 1 mbps and 8N1 serial data format configuration
  st.pSerial = &SERVOSerial; // assign serial pointer to serial port

  // Setup SD Card
  if (!SD.begin(BUILTIN_SDCARD)) {
    tone(BUZZER_PIN, NOTE_E8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_C8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_A7, 500);
    delay(2000);

    // Two beeps for bad SD Card read
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    while (true);
  }

  // Remove old debug file if exists
  if (SD.exists(OUTPUT_FILENAME)) {
      SD.remove(OUTPUT_FILENAME);
  }

  // Make new file
  output_file = SD.open(OUTPUT_FILENAME, FILE_WRITE);

  // Headers
  output_file.print("time [s]");
  output_file.print(",");
  output_file.print("cmd");
  output_file.print(",");
  output_file.println("ach");
  output_file.flush();

  // Set sine wave initial time
  t0 = micros();
  next_sample_us = t0;

}

void loop() {

    // Check runtime
    uint32_t now = micros();

    if ((int32_t) (now - next_sample_us) < 0) {
      return;
    }

    next_sample_us += SAMPLE_PERIOD_US;

    // Initialize loop counter
    static int n = 0;

    // Generate sine wave
    double t = (now - t0) / 1e6f;  // time in seconds
    float pos = offset + amplitude * sin(2 * M_PI * freq * t);
    
    // Write servo command
    st.WritePosEx(SERVO_ID, pos, 3800, 50);

    // Read servo encoder data
    int pos_ach = st.ReadPos(SERVO_ID);

    // Write data
    output_file.print(t,6);
    output_file.print(",");
    output_file.print(pos);
    output_file.print(",");
    output_file.println(pos_ach);

    // Update loop counter
    n++;

    // Stop readings after 20 seconds
    if (t > 20.0) {
      output_file.flush();
      st.WritePosEx(SERVO_ID, 0, 3800, 50);
      tone(BUZZER_PIN, NOTE_D8, 500);
      while(true);
    }
}
