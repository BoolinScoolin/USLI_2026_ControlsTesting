#include <Arduino.h>
#include <SD.h>

// GPS Serial Port
#define GPSSerial Serial8
// GPS Output Modes
#define PMTK_SET_NMEA_OUTPUT_OFF      "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY  "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA   "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_GGAVTG   "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_VTGONLY  "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGAONLY  "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// GPS Refresh Rates
#define PMTK_SET_NMEA_UPDATE_1HZ   "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_2HZ   "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_5HZ   "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ  "$PMTK220,100*2F"
// GPS Serial Baud Rates
#define PMTK_SET_BAUD_4800    "$PMTK251,4800*14"
#define PMTK_SET_BAUD_9600    "$PMTK251,9600*17"
#define PMTK_SET_BAUD_19200   "$PMTK251,19200*22"
#define PMTK_SET_BAUD_38400   "$PMTK251,38400*27\r\n"
#define PMTK_SET_BAUD_57600   "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_115200  "$PMTK251,115200*1F"
#define PMTK_SET_BAUD_230400  "$PMTK251,230400*1C"

// Initialize export file
File gps_output;

void setup() {

  // Turn on Teensy LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(2000);
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Starting setup.");
  GPSSerial.begin(9600);
  //GPSSerial.begin(19200);
  //GPSSerial.begin(38400);
  //GPSSerial.begin(76800);
  //GPSSerial.begin(115200);
  while (!GPSSerial) delay(10);
  GPSSerial.println(PMTK_SET_BAUD_115200); // change here (2/4 changes for baud rate)
  delay(100);
  GPSSerial.end(); //  close gps serial
  delay(100);
  GPSSerial.begin(115200); // change here (3/4 changes for baud rate)
  Serial.println("Now listening at 115200..."); // change here (4/4 changes for baud rate)
  delay(100);
  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_GGAONLY);  // change output data here (1/1)
  delay(100);
  GPSSerial.println(PMTK_SET_NMEA_UPDATE_10HZ); // change sample rate here (1/1)
  delay(100); 
  Serial.println("GPS Initialization done!");
  Serial.println("Initialization Complete.");

  // Export data
  bool sd_ok = SD.begin(BUILTIN_SDCARD);
  Serial.print("SD.begin() = ");
  Serial.println(sd_ok ? "SUCCESS" : "FAIL");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed. Please try again.");
    while (true);
  }
  Serial.println("SD Card Initialization done.");
  
  gps_output = SD.open("gps_7f_test1.txt", FILE_WRITE);
  if (!gps_output) {
    Serial.println("Error opening file! Please try again.");
    while (true);
  }
  else {
    Serial.println("SD Output file opened.");
    gps_output.println("Setup Test");
  }
}

void loop() {

  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps_output.print(c);
    if (c == '\n') {
      gps_output.flush();
    }
    Serial.print(c);
  }


}
