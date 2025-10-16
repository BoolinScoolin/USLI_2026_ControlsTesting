#include <Arduino.h>
// #include <Adafruit_Sensor.h>
// #include "Adafruit_BMP5xx.h"
// #include "Adafruit_BMP3XX.h"
//#include <SERVO_ST3215.h>
//#include <GPS_UltimateGPSv3.h>
//#include <BAROMETER_BMP390.h>
#define GPSSerial Serial8

// // GPS Serial Port
// #define GPSSerial Serial8
// // GPS Output Modes
// #define PMTK_SET_NMEA_OUTPUT_OFF      "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// #define PMTK_SET_NMEA_OUTPUT_RMCONLY  "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// #define PMTK_SET_NMEA_OUTPUT_RMCGGA   "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// #define PMTK_SET_NMEA_OUTPUT_ALLDATA  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// // GPS Refresh Rates
// #define PMTK_SET_NMEA_UPDATE_1HZ   "$PMTK220,1000*1F"
// #define PMTK_SET_NMEA_UPDATE_2HZ   "$PMTK220,500*2B"
// #define PMTK_SET_NMEA_UPDATE_5HZ   "$PMTK220,200*2C"
// #define PMTK_SET_NMEA_UPDATE_10HZ  "$PMTK220,100*2F"
// // GPS Serial Baud Rates
// #define PMTK_SET_BAUD_4800    "$PMTK251,4800*14"
// #define PMTK_SET_BAUD_9600    "$PMTK251,9600*17"
// #define PMTK_SET_BAUD_19200   "$PMTK251,19200*22"
// #define PMTK_SET_BAUD_38400   "$PMTK251,38400*27\r\n"
// #define PMTK_SET_BAUD_57600   "$PMTK251,57600*2C"
// #define PMTK_SET_BAUD_115200  "$PMTK251,115200*1F"
// #define PMTK_SET_BAUD_230400  "$PMTK251,230400*1C"

// Barometer Stuff (BMP581)
// Adafruit_BMP5xx bmp; // Create BMP5xx object
// bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode

// Barometer Stuff (BMP390)
// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11
// #define BMP_CS 10

void setup() {

  /// Setup LED used to verify power
  pinMode(LED_BUILTIN, OUTPUT);

  /// Turn on the LED to verify power
  digitalWrite(LED_BUILTIN, HIGH);

  delay(3000);  // give time for USB serial to reconnect after upload
  Serial.begin(115200); // Open USB Serial monitor,  make this baud rate fast enough to we aren't waiting on it
  while (!Serial); // Wait for serial monitor to open
  Serial.println("\nInitialization starting.");
  
  ///////////////////////////
  ////// SERVO SETUP ////////
  ///////////////////////////
  //setup_ST3215();

  ///////////////////////////
  //////// GPS SETUP ////////
  ///////////////////////////

  //setup_UltimateGPSv3();

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(115200);

  ///////////////////////////
  ////////// BMP581 /////////
  ///////////////////////////
  
  // Serial.println(F("Adafruit BMP5xx Comprehensive Test!"));
  // // Try to initialize the sensor
  // // For I2C mode (default):
  // bool is_bmp5xx = true;  // initialize barometer flag to check if ones plugged in (debug)
  // if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
  // // For SPI mode (uncomment the line below and comment out the I2C line above):
  // // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
  //   Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
  //   // while (1) delay(10); // Hang if theres no barometer
  //   is_bmp5xx = false; // Used to continue without a barometer 
  // }

  // if (is_bmp5xx) {

  //   Serial.println(F("BMP5xx found!"));
  //   Serial.println();

  //   // Demonstrate all setter functions with range documentation
  //   Serial.println(F("=== Setting Up Sensor Configuration ==="));
    
  //   /* Temperature Oversampling Settings:
  //   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
  //   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
  //   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
  //   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
  //   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
  //   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
  //   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
  //   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
  //   */
  //   Serial.println(F("Setting temperature oversampling to 2X..."));
  //   bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  //   /* Pressure Oversampling Settings (same options as temperature):
  //   * Higher oversampling = better accuracy but slower readings
  //   * Recommended: 16X for good balance of speed/accuracy
  //   */
  //   Serial.println(F("Setting pressure oversampling to 16X..."));
  //   bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

  //   /* IIR Filter Coefficient Settings:
  //   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
  //   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
  //   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
  //   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
  //   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
  //   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
  //   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
  //   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
  //   */
  //   Serial.println(F("Setting IIR filter to coefficient 3..."));
  //   bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  //   /* Output Data Rate Settings (Hz):
  //   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
  //   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
  //   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
  //   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
  //   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
  //   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
  //   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
  //   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
  //   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
  //   */
  //   Serial.println(F("Setting output data rate to 50 Hz..."));
  //   bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);

  //   /* Power Mode Settings:
  //   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
  //   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
  //   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
  //   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
  //   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
  //   */
  //   Serial.println(F("Setting power mode to normal..."));
  //   desiredMode = BMP5XX_POWERMODE_NORMAL;
  //   bmp.setPowerMode(desiredMode);

  //   /* Enable/Disable Pressure Measurement:
  //   * true  - Enable pressure measurement (default)
  //   * false - Disable pressure measurement (temperature only)
  //   */
  //   Serial.println(F("Enabling pressure measurement..."));
  //   bmp.enablePressure(true);

  //   /* Interrupt Configuration:
  //   * BMP5XX_INTERRUPT_PULSED / BMP5XX_INTERRUPT_LATCHED - Interrupt mode
  //   * BMP5XX_INTERRUPT_ACTIVE_LOW / BMP5XX_INTERRUPT_ACTIVE_HIGH - Interrupt polarity  
  //   * BMP5XX_INTERRUPT_PUSH_PULL / BMP5XX_INTERRUPT_OPEN_DRAIN - Interrupt drive
  //   * BMP5XX_INTERRUPT_DATA_READY, BMP5XX_INTERRUPT_FIFO_FULL, etc. - Interrupt sources (can combine with |)
  //   */
  //   Serial.println(F("Configuring interrupt pin with data ready source..."));
  //   bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);

  //   Serial.println();
  //   Serial.println(F("=== Current Sensor Configuration ==="));
    
  //   // Pretty print temperature oversampling inline
  //   Serial.print(F("Temperature Oversampling: "));
  //   switch(bmp.getTemperatureOversampling()) {
  //     case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
  //     case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
  //     case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
  //     case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
  //     case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
  //     case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
  //     case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
  //     case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
  //     default: Serial.println(F("Unknown")); break;
  //   }
    
  //   // Pretty print pressure oversampling inline
  //   Serial.print(F("Pressure Oversampling: "));
  //   switch(bmp.getPressureOversampling()) {
  //     case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
  //     case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
  //     case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
  //     case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
  //     case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
  //     case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
  //     case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
  //     case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
  //     default: Serial.println(F("Unknown")); break;
  //   }
    
  //   // Pretty print IIR filter coefficient inline
  //   Serial.print(F("IIR Filter Coefficient: "));
  //   switch(bmp.getIIRFilterCoeff()) {
  //     case BMP5XX_IIR_FILTER_BYPASS:   Serial.println(F("Bypass (No filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_1:  Serial.println(F("1 (Light filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_3:  Serial.println(F("3 (Medium filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_7:  Serial.println(F("7 (More filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_15: Serial.println(F("15 (Heavy filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_31: Serial.println(F("31 (Very heavy filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_63: Serial.println(F("63 (Maximum filtering)")); break;
  //     case BMP5XX_IIR_FILTER_COEFF_127:Serial.println(F("127 (Maximum filtering)")); break;
  //     default: Serial.println(F("Unknown")); break;
  //   }
    
  //   // Pretty print output data rate inline
  //   Serial.print(F("Output Data Rate: "));
  //   switch(bmp.getOutputDataRate()) {
  //     case BMP5XX_ODR_240_HZ:   Serial.println(F("240 Hz")); break;
  //     case BMP5XX_ODR_218_5_HZ: Serial.println(F("218.5 Hz")); break;
  //     case BMP5XX_ODR_199_1_HZ: Serial.println(F("199.1 Hz")); break;
  //     case BMP5XX_ODR_179_2_HZ: Serial.println(F("179.2 Hz")); break;
  //     case BMP5XX_ODR_160_HZ:   Serial.println(F("160 Hz")); break;
  //     case BMP5XX_ODR_149_3_HZ: Serial.println(F("149.3 Hz")); break;
  //     case BMP5XX_ODR_140_HZ:   Serial.println(F("140 Hz")); break;
  //     case BMP5XX_ODR_129_8_HZ: Serial.println(F("129.8 Hz")); break;
  //     case BMP5XX_ODR_120_HZ:   Serial.println(F("120 Hz")); break;
  //     case BMP5XX_ODR_110_1_HZ: Serial.println(F("110.1 Hz")); break;
  //     case BMP5XX_ODR_100_2_HZ: Serial.println(F("100.2 Hz")); break;
  //     case BMP5XX_ODR_89_6_HZ:  Serial.println(F("89.6 Hz")); break;
  //     case BMP5XX_ODR_80_HZ:    Serial.println(F("80 Hz")); break;
  //     case BMP5XX_ODR_70_HZ:    Serial.println(F("70 Hz")); break;
  //     case BMP5XX_ODR_60_HZ:    Serial.println(F("60 Hz")); break;
  //     case BMP5XX_ODR_50_HZ:    Serial.println(F("50 Hz")); break;
  //     case BMP5XX_ODR_45_HZ:    Serial.println(F("45 Hz")); break;
  //     case BMP5XX_ODR_40_HZ:    Serial.println(F("40 Hz")); break;
  //     case BMP5XX_ODR_35_HZ:    Serial.println(F("35 Hz")); break;
  //     case BMP5XX_ODR_30_HZ:    Serial.println(F("30 Hz")); break;
  //     case BMP5XX_ODR_25_HZ:    Serial.println(F("25 Hz")); break;
  //     case BMP5XX_ODR_20_HZ:    Serial.println(F("20 Hz")); break;
  //     case BMP5XX_ODR_15_HZ:    Serial.println(F("15 Hz")); break;
  //     case BMP5XX_ODR_10_HZ:    Serial.println(F("10 Hz")); break;
  //     case BMP5XX_ODR_05_HZ:    Serial.println(F("5 Hz")); break;
  //     case BMP5XX_ODR_04_HZ:    Serial.println(F("4 Hz")); break;
  //     case BMP5XX_ODR_03_HZ:    Serial.println(F("3 Hz")); break;
  //     case BMP5XX_ODR_02_HZ:    Serial.println(F("2 Hz")); break;
  //     case BMP5XX_ODR_01_HZ:    Serial.println(F("1 Hz")); break;
  //     case BMP5XX_ODR_0_5_HZ:   Serial.println(F("0.5 Hz")); break;
  //     case BMP5XX_ODR_0_250_HZ: Serial.println(F("0.25 Hz")); break;
  //     case BMP5XX_ODR_0_125_HZ: Serial.println(F("0.125 Hz")); break;
  //     default: Serial.println(F("Unknown")); break;
  //   }
    
  //   // Pretty print power mode inline
  //   Serial.print(F("Power Mode: "));
  //   switch(bmp.getPowerMode()) {
  //     case BMP5XX_POWERMODE_STANDBY:     Serial.println(F("Standby")); break;
  //     case BMP5XX_POWERMODE_NORMAL:      Serial.println(F("Normal")); break;
  //     case BMP5XX_POWERMODE_FORCED:      Serial.println(F("Forced")); break;
  //     case BMP5XX_POWERMODE_CONTINUOUS:  Serial.println(F("Continuous")); break;
  //     case BMP5XX_POWERMODE_DEEP_STANDBY:Serial.println(F("Deep Standby")); break;
  //     default: Serial.println(F("Unknown")); break;
  //   }
  // }

  ///////////////////////////
  ////////// BMP390 /////////
  ///////////////////////////

  //setup_BMP390();

  Serial.println();

  Serial.println("Initiailization done!");
}

void loop() {

  ///////////////////////////
  ////////// SERVO //////////
  ///////////////////////////

  // // Move servo back
  // st.WritePosEx(SERVO_ID, random(100, 3000), 3800, 50);
  // int pos = st.ReadPos(SERVO_ID);
  // Serial.println(pos);



  ///////////////////////////
  /////////// GPS ///////////
  ///////////////////////////

  // if (GPSSerial.available()) {
    // char c = GPSSerial.read();
    // Serial.write(c);
    // if (c == '$' && false) { // new line from GPS
      
    //   // Print Servo Encoder Data
    //   Serial.print("Servo Position: ");
    //   Serial.println(pos);

    //   // Print Barometer data    
    //   Serial.print(F("Temperature = "));
    //   Serial.print(bmp.temperature);
    //   Serial.println(F(" °C"));

    //   Serial.print(F("Pressure = "));
    //   Serial.print(bmp.pressure);
    //   Serial.println(F(" hPa"));

    //   Serial.print(F("Approx. Altitude = "));
    //   Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    //   Serial.println(F(" m"));
    //   Serial.println(F("---"));  
      
    //   Serial.print('$');
    // }
  // }
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
  
  ///////////////////////////
  ///// BAROMETER / TEMP ////
  ///////////////////////////

}