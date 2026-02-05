// No abstraction since that broke it last time and I'm on a deadline ... 

const float MOTOR_BURNOUT_TIME = 3.1; // Add factor of safety
const float M_PI = 3.1415926;

// Includes
#include <Arduino.h>  // For teensy
#include <SCServo.h>  // For servo motor
#include "MTi.h"  // For xsens imu
#include <Wire.h>  // For i2c ? idr tbh
#include <SD.h>  // For data logging to microcontroller microsd card
#include <Adafruit_Sensor.h>  // For barometer
#include <Adafruit_BMP3XX.h>  // For barometer
#include <string> // for strings
#include <cmath> // for math
// #include <Eigen> // For math

// namespace eg = Eigen;
using std::string;

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

// Initialize GPS Parameters
#define GPSSerial Serial8
File gps_output;
string gps_output_filename = "gps_output.txt";

// Initialize Barometer Parameters
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)
File barometer_output;
string barometer_output_filename = "barometer_output.txt";

// Initialize IMU Parameters
#define DRDY 20                    //Arduino Digital IO pin used as input for MTi-DRDY
#define IMU_ADDRESS 0x6B                //MTi I2C address 0x6B (default I2C address for MTi 1-series)
MTi *MyMTi = NULL;
File IMU_output;
string IMU_output_filename = "IMU_output.txt"; // include .txt

// Initialize Servo Parameters
SMS_STS st; // create servo object
#define SERVOSerial Serial4
#define SERVO_ID 1
const float deg2servo = 4096.0 / 360;

// Sine wave parameter
float amplitude = 44.5*deg2servo;     // ±800 counts around center
float offset = 44.5*deg2servo;       // center position
float freq = 0.5;          // 0.2 Hz = one cycle every 5 seconds
unsigned long t0;
static bool test1 = false;

void setup() {

  // Turn on on-board LED
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

  // Setup GPS
  GPSSerial.begin(115200);

  // Setup Servo
  SERVOSerial.begin(1000000, SERIAL_8N1); // Open servo serial line at 1 mbps and 8N1 serial data format configuration
  st.pSerial = &SERVOSerial; // assign serial pointer to serial port

  // Setup Barometer
  if (!bmp.begin_I2C()) {
    tone(BUZZER_PIN, NOTE_E8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_C8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_A7, 500);
    delay(2000);

    // Three beeps for bad barometer
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    while (true);
  }

  Wire.begin();                     //Initialize Wire library for I2C communication
  pinMode(DRDY, INPUT);             //Data Ready pin, indicates whether data/notifications are available to be read
  MyMTi = new MTi(IMU_ADDRESS, DRDY);   //Create our new MTi object
  if (!MyMTi->detect(1000)) {       //Check if MTi is detected before moving on
    tone(BUZZER_PIN, NOTE_E8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_C8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_A7, 500);
    delay(2000);

    // Four beeps for bad IMU
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    tone(BUZZER_PIN, NOTE_D8, 500);
    while (true) {
    }
  } else {
    delay(100);
    MyMTi->goToConfig();            //Switch device to Config mode
    delay(100);
    MyMTi->requestDeviceInfo();     //Request the device's product code and firmware version
    delay(100);
    MyMTi->configureOutputs();      //Configure the device's outputs based on its functionality. See MTi::configureOutputs() for more alternative output configurations.
    delay(100);
    MyMTi->goToMeasurement();       //Switch device to Measurement mode
    delay(100);
  }

  IMU_output = SD.open(IMU_output_filename.c_str(), FILE_WRITE);
  if (!IMU_output) {
    tone(BUZZER_PIN, NOTE_E8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_C8, 200);
    delay(200);
    tone(BUZZER_PIN, NOTE_A7, 500);
    delay(2000);

    // One beep for other (PANIC)
    tone(BUZZER_PIN, NOTE_D8, 500);
    delay(600);
    while (true);
  }
  else {
    IMU_output.print("ax ");
    IMU_output.print("ay ");
    IMU_output.print("az ");
    IMU_output.print("wx ");
    IMU_output.print("wy ");
    IMU_output.println("wz");
  }

  // Rotate servo slightly
  st.WritePosEx(SERVO_ID, 85*deg2servo, 3800, 50);
  delay(500);
  st.WritePosEx(SERVO_ID, 0, 3800, 50);

  // Turn off onboard LED
  digitalWrite(LED_BUILTIN, LOW);

  // Set sine wave initial time
  t0 = millis();

  
  // Initialization complete
  tone(BUZZER_PIN, NOTE_D8, 100);   // play 2 kHz tone for 500 ms
  delay(100);
  tone(BUZZER_PIN, NOTE_F8, 100);   // play 2 kHz tone for 500 ms
  delay(100);
  tone(BUZZER_PIN, NOTE_A8, 200);   // play 2 kHz tone for 500 ms
  delay(500);
  
}

void loop() {

  static bool TARE_FLAG = false; // tare flag to check if tare has completed
  static int ii = 0;  // counter to check how many readings are in tare
  static unsigned long lastFlush = 0; // initialize timer since last flush
  static float tare[6] = {0.0f};  // all elements = 0.0f
  static float alt_tare = 0.0;
  static float altitude;

  // Initialize accelerometer readings
  float* acc_b_N = nullptr;
  float* omega_b_rps = nullptr;

  if (digitalRead(MyMTi->drdy)) {   //MTi reports that new data/notifications are available
    MyMTi->readMessages();          //Read new data messages
    acc_b_N = MyMTi->getAcceleration();
    omega_b_rps = MyMTi->getRateOfTurn();
    if (TARE_FLAG) {

      acc_b_N[0] = acc_b_N[0] - tare[0];
      acc_b_N[1] = acc_b_N[1] - tare[1];
      acc_b_N[2] = acc_b_N[2] - tare[2];
      omega_b_rps[0] = omega_b_rps[0] - tare[3];
      omega_b_rps[1] = omega_b_rps[1] - tare[4];
      omega_b_rps[2] = omega_b_rps[2] - tare[5];

      altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - alt_tare;

      
      // Export
      IMU_output.print(acc_b_N[0]);
      IMU_output.print(' ');
      IMU_output.print(acc_b_N[1]);
      IMU_output.print(' ');
      IMU_output.print(acc_b_N[2]);
      IMU_output.print(' ');
      IMU_output.print(omega_b_rps[0]);
      IMU_output.print(' ');
      IMU_output.print(omega_b_rps[1]);
      IMU_output.print(' ');
      IMU_output.println(omega_b_rps[2]);

      if (!test1) {
        tone(BUZZER_PIN, NOTE_A8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A8, 200);
        test1 = true;
      }

    } else if (millis() - lastFlush > 5000) {
      tare[0] += acc_b_N[0];  // store values
      tare[1] += acc_b_N[1];
      tare[2] += acc_b_N[2];
      tare[3] += omega_b_rps[0];
      tare[4] += omega_b_rps[1];
      tare[5] += omega_b_rps[2];
      ii++;  // count values in tare
      if (!test1) {
        tone(BUZZER_PIN, NOTE_A8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A8, 200);
      test1 = true;
    }
    } else {
      if (ii>0) {   // make sure you dont divide by zero
        for (int j = 0; j < 6; j++) tare[j] /= ii;  // compute average
        alt_tare = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }
      // Export tare
      IMU_output.print(tare[0]);
      IMU_output.print(' ');
      IMU_output.print(tare[1]);
      IMU_output.print(' ');
      IMU_output.print(tare[2]);
      IMU_output.print(' ');
      IMU_output.print(tare[3]);
      IMU_output.print(' ');
      IMU_output.print(tare[4]);
      IMU_output.print(' ');
      IMU_output.println(tare[5]);
      TARE_FLAG = true;
        // Tare complete
      tone(BUZZER_PIN, NOTE_D8, 200);   // play 2 kHz tone for 500 ms
      delay(200);
      tone(BUZZER_PIN, NOTE_F8, 200);   // play 2 kHz tone for 500 ms
      delay(200);
      tone(BUZZER_PIN, NOTE_A8, 400);   // play 2 kHz tone for 500 ms
      delay(500);
    }

    if (millis() - lastFlush > 1000) {
      IMU_output.flush();
      lastFlush = millis();
    }
  }


  // Parse IMU Data
 // eg::Vector3f acc_vec = eg::Vector3f(acc_b_N[0], acc_b_N[1], acc_b_N[2]);


  // Initialize stage detection events
  static bool LAUNCH_FLAG = false;
  static bool BURNOUT_FLAG = false;
  static bool ABORT_FLAG = false;
  static bool APOGEE_FLAG = false;
  static bool LANDING_FLAG = false;
  static float LAUNCH_TIME = 0;
  static int ABORT_ROTRATE_DPS = 180;

  if (LANDING_FLAG &&  TARE_FLAG) {
    // Start beeping like an animal to make kevin's life harder

  } else if (APOGEE_FLAG &&  TARE_FLAG) {
    // Check for landing



  } else if (ABORT_FLAG &&  TARE_FLAG) {

    // Return Servo
    // Turn off servo serial port
    // Check for apogee
    st.WritePosEx(SERVO_ID, 0, 3800, 50);
    while(true);

  } else if (BURNOUT_FLAG &&  TARE_FLAG) {

    // Continue integration
    // Turn on airbrake / GNC logic
    // Check for ABORT conditions
    // Check for apogee
    if (std::fabs(omega_b_rps[1]) > ABORT_ROTRATE_DPS || std::fabs(omega_b_rps[2]) >  ABORT_ROTRATE_DPS) {
      ABORT_FLAG = true;
    }
    
    float t = (millis() - t0) / 1000.0;  // time in seconds
    float pos = offset + amplitude * sin(2 * M_PI * freq * t);

    st.WritePosEx(SERVO_ID, pos, 3800, 50);

  } else if (LAUNCH_FLAG &&  TARE_FLAG) {

    // Begin Timer for burnout
    if (millis() - LAUNCH_TIME > 1000*MOTOR_BURNOUT_TIME) {
      if (alt_tare > 30) {
        BURNOUT_FLAG = true;
      } else {
        LAUNCH_FLAG = false;
      }
    }

  } else if (TARE_FLAG) {  // On the rail

    // Testing (DELETE)
    if (!test1) {
      tone(BUZZER_PIN, NOTE_A8, 200);
      test1 = true;
    }

    Serial.begin(115200);
    Serial.println(acc_b_N[0]);

    // Compute norm of measured acceleration
    float acc_mag = std::sqrt(acc_b_N[0]*acc_b_N[0] + acc_b_N[1]*acc_b_N[1] + acc_b_N[2]*acc_b_N[2]);

    if (false) {
      LAUNCH_FLAG = true;
      LAUNCH_TIME = millis();

      // Testing (DELETE)
      tone(BUZZER_PIN, NOTE_A7, 200);
      delay(500);
      tone(BUZZER_PIN, NOTE_A8, 200);

    }
      // Initialize orientation and position
      // Check for Launch

  }

}