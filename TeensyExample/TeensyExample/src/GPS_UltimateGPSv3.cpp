#include <GPS_UltimateGPSv3.h>

void setup_UltimateGPSv3() { 
  // To communicate with GPS, we need to select the correct baud rate.
  // To change the baud rate of the GPS, we need to communicate with the GPS...
  // So, we first must find the current baud rate so that we can send the command to change it.
  Serial.println("Searching for current GPS baud rate...");
  Serial.println();
  long gps_baud_rates[] = {115200, 9600, 19200, 38400, 57600, 4800}; // store possible baud rates for iteration
  bool BAUD_FLAG = false;  // iniitalize baud flag to indicate right baud found
  for (int i=0; i<6; i++) {
    Serial.print("Trying baud rate ");
    Serial.print(i+1);
    Serial.println("/6");
    GPSSerial.begin(gps_baud_rates[i]); // Try i'th baud
    delay(1500);
    GPSSerial.println(PMTK_SET_NMEA_OUTPUT_OFF);  // tell GPS to shut up
    delay(1500);
    while (GPSSerial.available() > 0) { // clear GPS Serial
      GPSSerial.read();  // discard incoming bytes
    }
    delay(1500);
    GPSSerial.println("$PMTK605*31"); // query firmware version (produces a known output if at correct baud rate)
    delay(1500);
    unsigned long start = millis(); // store current time
    int j = 0; // initialize counter to prevent infinite loop
    String buffer = ""; // initialize a buffer string to store GPS serial output
    while (millis() - start < 500 && j < 10000) { // run for 0.5 s
      while (GPSSerial.available()) {
        char c = GPSSerial.read(); // grab latest character
        buffer += c; // add in buffer (buffer becomes the latest GPS Serial text)

        if (buffer.endsWith("$PMTK705")) { // compare end of buffer to known output
          BAUD_FLAG = true; // update flag to mark correct baud rate
          Serial.print("Buffer (Right Baud): "); // print buffer for visual verification
          Serial.println(buffer);
          Serial.println();
          break;
        }

        if (buffer.length() > 100) buffer.remove(0, 50); // dont let buffer get too large

      }

      if (BAUD_FLAG) { // check if baud was found this iteration
        Serial.print("Found GPS at ");
        Serial.println(gps_baud_rates[i]);
        Serial.println();
        GPSSerial.begin(gps_baud_rates[i]); // initialize gps at correct baud rate
        delay(1500);
        Serial.println("Breaking for loop..."); // debug
        break;
      }
      j++; // prevent while loop from hanging
    }

    Serial.print("Buffer (Wrong Baud): "); // print wrong buffer
    Serial.println(buffer);
    Serial.println();
  }
  if (!BAUD_FLAG) {
    Serial.println("Did not find GPS baud rate. Please try again.");
    while (true); // dont run program if baud rate unknown
  }

  // Set new baud rate
  Serial.println("Switching GPS to baud 115200..."); // change baud rate here (1/4 changes for baud rate)
  // Send command to change baud
  GPSSerial.println(PMTK_SET_BAUD_115200); // change here (2/4 changes for baud rate)
  delay(500);
  GPSSerial.end(); //  close gps serial
  delay(500);
  GPSSerial.begin(115200); // change here (3/4 changes for baud rate)
  Serial.println("Now listening at 115200..."); // change here (4/4 changes for baud rate)
  delay(500);
  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // change output data here (1/1)
  delay(500);
  GPSSerial.println(PMTK_SET_NMEA_UPDATE_2HZ); // change sample rate here (1/1)
  delay(500); 
  Serial.println("GPS Initialization done!");

}