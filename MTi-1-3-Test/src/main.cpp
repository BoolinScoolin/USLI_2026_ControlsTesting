//This library demonstrates basic Xbus communication between an Arduino (Uno) and an Xsens MTi 1-series shield board (MTi-#-DK) using the I2C interface.

//At least the following hardware connections are required:
//MTi_SCL - Arduino_SCL
//MTi_SDA - Arduino_SDA
//MTi_GND - Arduino_GND
//MTi_3.3V - Arduino_3.3V
//MTi_DRDY - Arduino_D3
//Additionally, make sure to:
//  -Enable I2C by switching PSEL0,1 to "1"
//  -Supply the MTi-#-DK with 3.3V (since 5V will force USB mode)
//  -Add 2.7 kOhm pullup resistors to the SCL/SDA lines (only for MTi-#-DK board Rev 2.3 or older - newer revisions come with onboard pullup resistors)

//This example code is merely a starting point for code development. Its functionality can be extended by making use of the Xbus protocol.
//Xbus is the proprietary binary communication protocol that is supported by all Xsens MTi products, and it is fully documented at https://mtidocs.xsens.com/mt-low-level-communication-protocol-documentation

//For further details on this example code, please refer to BASE: https://base.xsens.com/s/article/Interfacing-the-MTi-1-series-DK-with-an-Arduino?language=en_US


#include "MTi.h"
#include <Wire.h>
#include <SD.h>
#include <string>

using std::string;


#define DRDY 17                      //Arduino Digital IO pin used as input for MTi-DRDY
#define ADDRESS 0x6B                //MTi I2C address 0x6B (default I2C address for MTi 1-series)
MTi *MyMTi = NULL;

// Intialize Output Files
File IMU_output;
string IMU_output_filename = "IMU_output.txt"; // include .txt

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);             //Initialize communication for serial monitor output (Ctrl+Shift+M)
  while (!Serial) delay(10);
  Serial.println("\nInitializing...");
  delay(100);
  Wire.begin();                     //Initialize Wire library for I2C communication
  pinMode(DRDY, INPUT);             //Data Ready pin, indicates whether data/notifications are available to be read
  
  Serial.println("Past pinMode...");
  delay(100);

  MyMTi = new MTi(ADDRESS, DRDY);   //Create our new MTi object
  Serial.println("Past MTi Object init...");
  delay(100);
  Serial.println("Entering if...");
  delay(100);

  if (!MyMTi->detect(1000)) {       //Check if MTi is detected before moving on
    Serial.println("Please check your hardware connections.");
    while (1) {
      //Cannot continue because no MTi was detected.
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

    // Export data
  bool sd_ok = SD.begin(BUILTIN_SDCARD);
  Serial.print("SD.begin() = ");
  Serial.println(sd_ok ? "SUCCESS" : "FAIL");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed. Please try again.");
    while (true);
  }
  Serial.println("Initialization done.");
  IMU_output = SD.open(IMU_output_filename.c_str(), FILE_WRITE);
  if (!IMU_output) {
    Serial.println("Error opening file! Please try again.");
    while (true);
  }
  else {
    Serial.println("SD Output file opened.");
    IMU_output.print("ax ");
    IMU_output.print("ay ");
    IMU_output.print("az ");
    IMU_output.print("wx ");
    IMU_output.print("wy ");
    IMU_output.println("wz");
  }

  Serial.println("\nInitialization complete.");
  delay(1000);
}

void loop() {
  static bool TARE_FLAG = false; // tare flag to check if tare has completed
  static int ii = 0;  // counter to check how many readings are in tare
  static unsigned long lastFlush = 0; // initialize timer since last flush
  static float tare[6] = {0.0f};  // all elements = 0.0f

  if (digitalRead(MyMTi->drdy)) {   //MTi reports that new data/notifications are available
    MyMTi->readMessages();          //Read new data messages
    float* acc_b_N = MyMTi->getAcceleration();
    float* omega_b_rps = MyMTi->getRateOfTurn();
    if (TARE_FLAG) {
      Serial.print(acc_b_N[0] - tare[0]);
      Serial.print(' ');
      Serial.print(acc_b_N[1] - tare[1]);
      Serial.print(' ');
      Serial.print(acc_b_N[2] - tare[2]);
      Serial.print(' ');
      Serial.print(omega_b_rps[0] - tare[3]);
      Serial.print(' ');
      Serial.print(omega_b_rps[1] - tare[4]);
      Serial.print(' ');
      Serial.println(omega_b_rps[2] - tare[5]);

      // Export
      IMU_output.print(acc_b_N[0] - tare[0]);
      IMU_output.print(' ');
      IMU_output.print(acc_b_N[1] - tare[1]);
      IMU_output.print(' ');
      IMU_output.print(acc_b_N[2] - tare[2]);
      IMU_output.print(' ');
      IMU_output.print(omega_b_rps[0] - tare[3]);
      IMU_output.print(' ');
      IMU_output.print(omega_b_rps[1] - tare[4]);
      IMU_output.print(' ');
      IMU_output.println(omega_b_rps[2] - tare[5]);
    } else if (millis() - lastFlush > 2500) {
      tare[0] += acc_b_N[0];  // store values
      tare[1] += acc_b_N[1];
      tare[2] += acc_b_N[2];
      tare[3] += omega_b_rps[0];
      tare[4] += omega_b_rps[1];
      tare[5] += omega_b_rps[2];
      ii++;  // count values in tare
    } else {
      if (ii>0) {   // make sure you dont divide by zero
        for (int j = 0; j < 6; j++) tare[j] /= ii;  // compute average
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
      Serial.println("Tare Complete");
      delay(1000);
    }

    if (millis() - lastFlush > 1000) {
      IMU_output.flush();
      lastFlush = millis();
    }
  }
}