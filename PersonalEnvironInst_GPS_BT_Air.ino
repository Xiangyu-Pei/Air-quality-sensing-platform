# Copyright ownership belongs to XY Pei, aerosol_research@163.com

#include <DHT.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include <RTClib.h>

// Software and hard ware serial setting
// software serial: connect TX of Buetooth to digital pin 6, RX of Buetooth to digital pin 7
SoftwareSerial portBluetooth(6,7);
//HardwareSerial portBluetooth = Serial2;
// hardware serial: connect TX of GPS to RX1 on MEGA, RX of GPS to TX1 on MEGA
Adafruit_GPS GPS(&Serial1);
HardwareSerial portGPS = Serial1;

// T&RH sensor digital pin
#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT11 or DHT22  (AM2302)
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Analog input
// Pin 0: O3
// Pin 1: CO
// Pin 2: NO2
// Pin 3: NO
// Pin 4: SO2

// MicroSD card adaptor digital pin
 // SD card attached to SPI bus as follows:
 // DI - pin 51
 // DO - pin 50
 // CLK - pin 52
 // CS - pin SD_chipSelect
const int SD_chipSelect = 5;

// OLED display
// You can use any (4 or) 5 pins 
#define OLED_MOSI   48
#define OLED_CLK   49
#define OLED_DC    45
#define OLED_CS    46
#define OLED_RESET 47
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS); 

// GPS
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/*

 Interface to Shinyei Model PPD42NS Particle Sensor
 
 Based on code by Christopher Nafis April 2012
 Modified by Xiangyu Pei June 2015
 
 http://www.seeedstudio.com/depot/grove-dust-sensor-p-1050.html
 http://www.sca-shinyei.com/pdf/PPD42NS.pdf
 
 JST Pin 1 (Black Wire)  => Arduino GND
 JST Pin 2               => Arduino Digital Pin 7  PM 2.5-10
 JST Pin 3 (Red wire)    => Arduino 5VDC
 JST Pin 4 (Yellow wire) => Arduino Digital Pin 6  PM 1-10
 
 */

unsigned long starttime;

unsigned long triggerOnP1;
unsigned long triggerOffP1;
unsigned long pulseLengthP1;
unsigned long durationP1;
boolean valP1 = HIGH;
boolean triggerP1 = false;

unsigned long triggerOnP2;
unsigned long triggerOffP2;
unsigned long pulseLengthP2;
unsigned long durationP2;
boolean valP2 = HIGH;
boolean triggerP2 = false;

// Pow Pulse Occupancy (LPO)
float ratioP1 = 0;
float ratioP2 = 0;
// Sample time (ms)
unsigned long sampletime_ms = 30000;

// RTC
// GND connected to pin 53
// 5V connected to pin 22
// SDA connected to pin 20
// SCL connected to pin 21
RTC_DS1307 rtc;

void setup()
{  
  // Open serial communications and wait for port to open:
   Serial.begin(115200);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
    }
  
  // Start each serial port
  portBluetooth.begin(9600);
  portGPS.begin(9600);
  
  // Start DHT sensor
  dht.begin();
  
// Display test
  Serial.print("Display testing...");

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // initiate done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  Serial.println("Done");
  delay(1000);  
  
    // Setup microSD card adaptor
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    // return;
  }
  else {Serial.println("Card initialized.");} 
  
  // Setup Shinyei PM sensor
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  starttime = millis();
  
  // Setup GPS 
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  portGPS.println(PMTK_Q_RELEASE);
  
     // Setup RTC
  #ifdef AVR
    Wire.begin();
  #else
    Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
  #endif
    rtc.begin();
    
    if (! rtc.isrunning()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c); //UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop()
{  
// Timing
    unsigned long time1;
    time1 = millis();
// RTC time
  DateTime now = rtc.now();
  
//--------GPS Data-------------------------------------------------
  // Serial.println();
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c); //Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
    
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    portBluetooth.print("\nTime: ");
    portBluetooth.print(GPS.hour, DEC); portBluetooth.print(':');
    portBluetooth.print(GPS.minute, DEC); portBluetooth.print(':');
    portBluetooth.print(GPS.seconds, DEC); portBluetooth.print('.');
    portBluetooth.println(GPS.milliseconds);
    portBluetooth.print("Date: ");
    portBluetooth.print(GPS.day, DEC); portBluetooth.print('/');
    portBluetooth.print(GPS.month, DEC); portBluetooth.print("/20");
    portBluetooth.println(GPS.year, DEC);
    portBluetooth.print("Fix: "); portBluetooth.print((int)GPS.fix);
    portBluetooth.print(" quality: "); portBluetooth.println((int)GPS.fixquality); 
    
    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", "); 
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Location (in degrees, works with Google Maps): ");
//      Serial.print(GPS.latitudeDegrees, 6);
//      Serial.print(", "); 
//      Serial.println(GPS.longitudeDegrees, 6);
//      
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude (cm): "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      
      portBluetooth.print("Location: ");
      portBluetooth.print(GPS.latitude, 4); portBluetooth.print(GPS.lat);
      portBluetooth.print(", "); 
      portBluetooth.print(GPS.longitude, 4); portBluetooth.println(GPS.lon);
      portBluetooth.print("Location (in degrees, works with Google Maps): ");
      portBluetooth.print(GPS.latitudeDegrees, 6);
      portBluetooth.print(", "); 
      portBluetooth.println(GPS.longitudeDegrees, 6);
      
      portBluetooth.print("Speed (knots): "); portBluetooth.println(GPS.speed);
      portBluetooth.print("Angle: "); portBluetooth.println(GPS.angle);
      portBluetooth.print("Altitude (cm): "); portBluetooth.println(GPS.altitude);
      portBluetooth.print("Satellites: "); portBluetooth.println((int)GPS.satellites);
    }

// display date, time
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(now.hour(), DEC); display.print(':');
  display.print(now.minute(), DEC); display.print(':');
  display.print(now.second(), DEC); display.print(' ');
  display.print(" GPS Fix: "); display.println((int)GPS.fix);
  
  if (GPS.fix) {
      display.print("Lat: ");
      display.print(GPS.latitude, 4); display.println(GPS.lat);
      display.print("Lon: "); 
      display.print(GPS.longitude, 4); display.println(GPS.lon);
      }
  else {
      display.print("Lat: ");
      display.println("0000");
      display.print("Lon: "); 
      display.println("0000");
      }
      
  // print time to the serial port:
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print("\t");
    
//-------T&RH Data----------------------------------------------- 
  // Read temperature as Celsius
  float temp = dht.readTemperature();
  // Read relative humidity 
  float humi = dht.readHumidity();
  
  if(isnan(humi) || isnan(temp) )
  {
    Serial.println();
    Serial.print("Failed to read from DHT sensor!");
    Serial.println(); 
   
    portBluetooth.println();
    portBluetooth.print("Failed to read from DHT sensor!");
    portBluetooth.println(); 
  }
  else
  {  
  }

 
 //---------O3 CO NO2 NO SO2 Analog data working electrode (mV)------------------------------------------ 
    // O3
    int sensor_O3 = analogRead(0);
    float v_O3 = 4.9*float(sensor_O3);
    
    // CO
    int sensor_CO = analogRead(1);
    float v_CO = 4.9*float(sensor_CO);
    
    // NO2
    int sensor_NO2 = analogRead(2);
    float v_NO2 = 4.9*float(sensor_NO2);
  
    // NO
    int sensor_NO = analogRead(3);
    float v_NO = 4.9*float(sensor_NO);
    
    // SO2
    int sensor_SO2 = analogRead(4);
    float v_SO2 = 4.9*float(sensor_SO2);


 //---------O3 CO NO2 NO SO2 Analog data auxiliary electrode (mV)------------------------------------------ 
    // O3
    int sensor_O3_a = analogRead(8);
    float v_O3_a = 4.9*float(sensor_O3_a);
    
    // CO
    int sensor_CO_a = analogRead(9);
    float v_CO_a = 4.9*float(sensor_CO_a);
    
    // NO2
    int sensor_NO2_a= analogRead(10);
    float v_NO2_a = 4.9*float(sensor_NO2_a);
    
    // NO
    int sensor_NO_a = analogRead(11);
    float v_NO_a = 4.9*float(sensor_NO_a);
    
    // SO2
    int sensor_SO2_a = analogRead(12);
    float v_SO2_a = 4.9*float(sensor_SO2_a);
    

 //---------O3 CO NO2 NO SO2 Calibration ------------------------------------------ 
     // O3
    float conc_O3 = ((v_O3-349)-(1.56)*(v_O3_a-359))/253;
     // CO
    float conc_CO = ((v_CO-333)-(-1)*(v_CO_a-348))/0.443;
     // NO2
    float conc_NO2 = ((v_NO2-237)-(0.68)*(v_NO2_a-229))/0.197;
    // NO
    float conc_NO = ((v_NO-283)-(1.82)*(v_CO_a-282))/582; 
    // SO2
    float conc_SO2 = ((v_SO2-333)-(1.34)*(v_SO2_a-341))/0.327;
 
    
  //---------------Make Date String------------------------------------
   char str1[12]; // year
   char str1t[12] = "0"; // year
   char str2[12]; // month
   char str2t[12] = "0"; // month
   char str3[12]; // day
   char str3t[12] = "0"; // day
   
   char strG[12] = "G";
   char strP[12] = "P";
   char strT[12] = ".txt";
   
   int rtc_year = now.year();
   int rtc_month = now.month();
   int rtc_day = now.day();
   rtc_year = rtc_year-2000;
   
   sprintf(str1,"%d",rtc_year);
   int len1 = strlen(str1);
   if (len1 == 1){
   strcat(str1t, str1);
   strcpy(str1, str1t);
   }
   else {}

   sprintf(str2,"%d",rtc_month);
   int len2 = strlen(str2);
   if (len2 == 1){
   strcat(str2t, str2);
   strcpy(str2, str2t);
   }
   else {}

   sprintf(str3,"%d",rtc_day);
   int len3 = strlen(str3);
   if (len3 == 1){
   strcat(str3t, str3);
   strcpy(str3, str3t);
   }
   else {}   
   
   // make gas data file name, format: Gyymmdd.txt
   strcat(str1, str2);
   strcat(str1, str3);
   strcat(strG, str1);
   strcat(strG, strT);
   // make PM data file name, format: Pyymmdd.txt
   strcat(strP, str1);
   strcat(strP, strT);
    
//-----------Logging Data to MicroSD Card--------------------------------  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(strG, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(now.year(), DEC); dataFile.print("\t");
    dataFile.print(now.month(), DEC); dataFile.print("\t");   
    dataFile.print(now.day(), DEC); dataFile.print("\t");
    dataFile.print(now.hour(), DEC); dataFile.print("\t");
    dataFile.print(now.minute(), DEC); dataFile.print("\t");
    dataFile.print(now.second(), DEC); dataFile.print("\t");
    
    dataFile.print(GPS.year, DEC); dataFile.print("\t");
    dataFile.print(GPS.month, DEC); dataFile.print("\t");   
    dataFile.print(GPS.day, DEC); dataFile.print("\t");
    dataFile.print(GPS.hour, DEC); dataFile.print("\t");
    dataFile.print(GPS.minute, DEC); dataFile.print("\t");
    dataFile.print(GPS.seconds, DEC); dataFile.print("\t");
    dataFile.print(GPS.milliseconds); dataFile.print("\t");     
    dataFile.print((int)GPS.fix); dataFile.print("\t");
    dataFile.print((int)GPS.fixquality); dataFile.print("\t"); 
    if (GPS.fix) {
      dataFile.print(GPS.latitude, 4); dataFile.print("\t");
      dataFile.print(GPS.lat); dataFile.print("\t");
      dataFile.print(GPS.longitude, 4); dataFile.print("\t");
      dataFile.print(GPS.lon); dataFile.print("\t");
      dataFile.print(GPS.latitudeDegrees, 4); dataFile.print("\t");
      dataFile.print(GPS.longitudeDegrees, 4); dataFile.print("\t");
      
      dataFile.print(GPS.speed); dataFile.print("\t"); 
      dataFile.print(GPS.angle); dataFile.print("\t"); 
      dataFile.print(GPS.altitude); dataFile.print("\t"); 
      dataFile.println((int)GPS.satellites);
    }
    else {
      dataFile.print("999"); dataFile.print("\t");
      dataFile.print("999"); dataFile.print("\t");
      dataFile.print("999"); dataFile.print("\t");
      dataFile.print("999"); dataFile.print("\t");
      dataFile.print("999"); dataFile.print("\t");
      dataFile.print("999"); dataFile.print("\t");
      
      dataFile.print("999"); dataFile.print("\t"); 
      dataFile.print("999"); dataFile.print("\t"); 
      dataFile.print("999"); dataFile.print("\t"); 
      dataFile.println("999");   
    }
    
    dataFile.print(temp); dataFile.print("\t");
    dataFile.print(humi); dataFile.print("\t");
    dataFile.print(v_O3); dataFile.print("\t");
    dataFile.print(v_CO); dataFile.print("\t");
    dataFile.print(v_NO2); dataFile.print("\t");
    dataFile.print(v_NO); dataFile.print("\t");
    dataFile.print(v_SO2); dataFile.print("\t");    
    
    dataFile.print(v_O3_a); dataFile.print("\t");
    dataFile.print(v_CO_a); dataFile.print("\t");
    dataFile.println(v_NO2_a); dataFile.print("\t");  
    dataFile.print(v_NO_a); dataFile.print("\t");
    dataFile.println(v_SO2_a);
    
    display.print("Gas: OK");
    dataFile.close();
    }
  // if the file isn't open, pop up an error:
  else {
    // Serial.println("error opening gasdata.txt");
    portBluetooth.println("error opening gasdata.txt");
    display.print("Gas: Fail");
  }
    
    // Serial.print("T ");
    Serial.print(temp);
    Serial.print("\t");
    // Serial.print(" degree C, RH ");
    Serial.print(humi);
    Serial.print("\t");
    // Serial.print(" %, O3 ");
    Serial.print(v_O3);
    Serial.print("\t");
    // Serial.print(" ppb, CO ");
    Serial.print(v_CO);
    Serial.print("\t");
    // Serial.print(" ppb, NO2 ");
    Serial.print(v_NO2);
    Serial.print("\t");
    // Serial.print(" ppb, NO ");
    Serial.print(v_NO);
    Serial.print("\t");
    // Serial.print(" ppb, SO2 ");
    Serial.print(v_SO2);
    Serial.print("\t");
    // Serial.println(" ppb");
    
    // Serial.print("O3_a ");
    Serial.print(v_O3);
    Serial.print("\t");
    // Serial.print(", CO_a ");
    Serial.print(v_CO);
    Serial.print("\t");
    // Serial.print(", NO2 ");
    Serial.print(v_NO2_a);
    Serial.print("\t");
    // Serial.print(", NO ");
    Serial.print(v_NO_a);
    Serial.print("\t");
    // Serial.print(", SO2 ");
    Serial.print(v_SO2_a);
    Serial.print("\t");
    
    portBluetooth.print("T ");
    portBluetooth.print(temp);
    portBluetooth.print(" degree C, RH ");
    portBluetooth.print(humi);
    portBluetooth.print(" %, O3 ");
    portBluetooth.print(conc_O3);
    portBluetooth.print(" ppb, CO ");
    portBluetooth.print(conc_CO);
    portBluetooth.print(" ppb, NO2 ");
    portBluetooth.print(conc_NO2);
    portBluetooth.println(" ppb");  
    portBluetooth.print(" ppb, NO ");
    portBluetooth.print(conc_NO);
    portBluetooth.print(" ppb, SO2 ");
    portBluetooth.print(conc_SO2);
    portBluetooth.println(" ppb");    

    portBluetooth.print("O3_a ");
    portBluetooth.print(v_O3_a);
    portBluetooth.print(", CO_a ");
    portBluetooth.print(v_CO_a);
    portBluetooth.print(", NO2 ");
    portBluetooth.print(v_NO2_a);
    portBluetooth.print(", NO ");
    portBluetooth.print(v_NO_a);
    portBluetooth.print(", SO2 ");
    portBluetooth.println(v_SO2_a);
    
    
// Shinyei PM sensor
  
  valP1 = digitalRead(6);
  valP2 = digitalRead(7);
  
  if(valP1 == LOW && triggerP1 == false){
    triggerP1 = true;
    triggerOnP1 = micros();
  }
  
  if (valP1 == HIGH && triggerP1 == true){
      triggerOffP1 = micros();
      pulseLengthP1 = triggerOffP1 - triggerOnP1;
      durationP1 = durationP1 + pulseLengthP1;
      triggerP1 = false;
  }
  
    if(valP2 == LOW && triggerP2 == false){
    triggerP2 = true;
    triggerOnP2 = micros();
  }
  
    if (valP2 == HIGH && triggerP2 == true){
      triggerOffP2 = micros();
      pulseLengthP2 = triggerOffP2 - triggerOnP2;
      durationP2 = durationP2 + pulseLengthP2;
      triggerP2 = false;
  }

    // Function creates particle count and mass concentration
    // from PPD-42 low pulse occupancy (LPO).
    if ((millis() - starttime) > sampletime_ms) {
      
      // Percent Full Scale (%FS)   
      ratioP1 = durationP1/(sampletime_ms*10.0);
      ratioP2 = durationP2/(sampletime_ms*10.0);
      
      durationP1 = 0;
      durationP2 = 0;
      starttime = millis();
      
      // save PM data

        File dataFile2 = SD.open(strP, FILE_WRITE);
        // if the file is available, write to it:
        if (dataFile2) {
          dataFile2.print(now.year(), DEC); dataFile2.print("\t");
          dataFile2.print(now.month(), DEC); dataFile2.print("\t");   
          dataFile2.print(now.day(), DEC); dataFile2.print("\t");
          dataFile2.print(now.hour(), DEC); dataFile2.print("\t");
          dataFile2.print(now.minute(), DEC); dataFile2.print("\t");
          dataFile2.print(now.second(), DEC); dataFile2.print("\t");
          
          dataFile2.print(GPS.year, DEC); dataFile2.print("\t");
          dataFile2.print(GPS.month, DEC); dataFile2.print("\t");   
          dataFile2.print(GPS.day, DEC); dataFile2.print("\t");
          dataFile2.print(GPS.hour, DEC); dataFile2.print("\t");
          dataFile2.print(GPS.minute, DEC); dataFile2.print("\t");
          dataFile2.print(GPS.seconds, DEC); dataFile2.print("\t");
          dataFile2.print(GPS.milliseconds); dataFile2.print("\t");     
          dataFile2.print((int)GPS.fix); dataFile2.print("\t");
          dataFile2.print((int)GPS.fixquality); dataFile2.print("\t"); 
      
          dataFile2.print(ratioP1); dataFile2.print("\t");
          dataFile2.println(ratioP2); 
         
          dataFile2.close();
          }
      }   

       File dataFile3 = SD.open(strP, FILE_WRITE);  
       if (dataFile3) {
              display.println("  PM: OK");
              dataFile3.close();
              }
        // if the file isn't open, pop up an error:
        else {
              // Serial.println("error opening pmdata.txt");
              portBluetooth.println("error opening pmdata.txt");
              display.println("  PM: Fail");
             }

          // Print PM data
          // Serial.print("LPO1 = ");
          Serial.print(ratioP1);
          Serial.print("\t");
          // Serial.print("; LPO2 = ");
          Serial.println(ratioP2);
                    
          portBluetooth.print("LPO1 = ");
          portBluetooth.print(ratioP1);
          portBluetooth.print("; LPO2 = ");
          portBluetooth.println(ratioP2);
          
//-------------OLED Display--------------------------------------------
        display.print("T ");
        display.print(temp);
        display.print(" C, RH ");
        display.print(humi);
        display.println("%");
        display.print("O3  ");
        display.print(conc_O3);
        display.print(" ");
        display.print("CO ");
        display.println(conc_CO);
        display.print("NO2 ");
        display.print(conc_NO2);
        display.print(" ");
        display.print("NO ");
        display.println(conc_NO);
        display.print("SO2 ");
        display.print(conc_SO2);
        display.print(" ");
        //display.println(" ppb");  
        
//        display.print("LPO1 ");
//        display.print(ratioP1);
//        display.print("  ");
//        display.print("LPO2 ");
//        display.print(ratioP2);
//        display.println(" ");

        display.print(ratioP1);
        display.print("/");
        display.println(ratioP2);   
        
        display.display();
        display.clearDisplay(); 
    
//---------Time Interval (1s)----------------------------------------------  
    unsigned long time2;
    time2 = millis();
    unsigned long time_int;
    time_int = 1000-(time2-time1);
    delay(time_int); //delay for reread 
}





