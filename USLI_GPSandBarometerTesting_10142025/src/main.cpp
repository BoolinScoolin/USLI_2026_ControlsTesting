#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include "Adafruit_BMP3XX.h"
#include <SD.h>

// Barometer Stuff
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)

// Intialize Output Files
File barometer_output;

void setup() {

  // Turn on Teensy LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);

  // Barometer
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire

  Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Barometer setup complete.");

  // Export data
  bool sd_ok = SD.begin(BUILTIN_SDCARD);
  Serial.print("SD.begin() = ");
  Serial.println(sd_ok ? "SUCCESS" : "FAIL");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed. Please try again.");
    while (true);
  }
  Serial.println("Initialization done.");
  barometer_output = SD.open("barometer_7f_test1.txt", FILE_WRITE);
  if (!barometer_output) {
    Serial.println("Error opening file! Please try again.");
    while (true);
  }
  else {
    Serial.println("SD Output file opened.");
    barometer_output.println("Altitude [m]");
  }
}

void loop() {
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  barometer_output.println(altitude);
  Serial.println(altitude);
  static unsigned long lastFlush = 0;
  if (millis() - lastFlush > 1000) {
    barometer_output.flush();
    lastFlush = millis();
  }
}