// Your GPRS credentials (leave empty, if not needed)
#define TELE2              false
#define DEBUG              false
#define IBEACON            false

#if TELE2
  const char apn[]      = "4g.tele2.se"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
#else
  const char apn[]      = "services.telenor.se";
#endif

const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using

#if DEBUG
  const char server[] = "luminare.se";      // domain name: example.com, maker.ifttt.com, etc
  const char resource[] = "update?";        // resource path, for example: /post-data.php
  const int  port = 5555;                    // server port number
#else
  const char server[] = "thingspeak.com";   // domain name: example.com, maker.ifttt.com, etc
  const char resource[] = "update?";        // resource path, for example: /post-data.php
  const int  port = 80;                     // server port number
#endif

// Keep this API Key value to be compatible with the PHP code provided in the project page. 
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key 
String apiKeyValue = "";

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
#define BATTERY_VOLTAGE      35
// BMP180 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19
// UBLOX GPS pins
#define GPS_TX               32 // orange (TX on GPS)
#define GPS_RX               33 // red (RX on GPS)

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>
#include <deque>


//For BLE iBeacon
#include "sys/time.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"


/***
 * Position database
 */
class positionDb
{
  public:
  
  struct positionElement
  {
    uint32_t timestamp;
    float    longitude;
    float    latitude;
    float    altitude;
    float    barometricAltitude;
    float    temperature;
    uint8_t  signalStrength;
    float    batteryVoltage;
  };

  void addPosition(positionElement&& pe)
  {
    if(db.size() >= elementLimit)
    {
      db.pop_back();
    }
    db.emplace_front(pe);
  }

  positionElement consumeNewestElement()
  {
    auto elem = db.front();
    db.pop_front();
    return elem;
  }
  positionElement consumeOldestElement()
  {
    auto elem = db.back();
    db.pop_back();
    return elem;;
  }

  bool hasPositionElements()
  {
    return !db.empty();
  }

  uint16_t getSize()
  {
    return db.size();
  }

  private:
  uint16_t elementLimit = 5120; // approximatly 100kb if positionElement is ~20 bytes
  std::deque<positionElement> db;
};



#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

//------Barometer config-------

#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmpBarometer = Adafruit_BMP085_Unified(10085);
#define SEA_LEVEL_PRESSURE 103000

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
//TwoWire I2CBME = TwoWire(1);
//Adafruit_BME280 bme; 


//-------GPS config-----
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
SoftwareSerial gpsInterface(GPS_TX, GPS_RX);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

//BLE iBeacon config
#ifdef IBEACON
BLEAdvertising *pAdvertising;
struct timeval now;
#define BEACON_UUID           "8ec76ea3-6668-48da-9866-75be8bc86f4d"
RTC_DATA_ATTR static time_t last;        // remember last boot in RTC Memory
RTC_DATA_ATTR static uint32_t bootcount; // remember number of boots in RTC Memory
#endif

//-------ESP32 power save configuration------
#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

void sendGsmData(TinyGPSPlus &gps);
bool getBarometricData(float &barAltitude, float &temperature);
void setBeacon();

void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);
  gpsInterface.begin(9600);
  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  //I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.init();
  //modem.restart();  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }

  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Initialize barometric sensor
  Wire.begin(I2C_SDA_2, I2C_SCL_2);
  if(!bmpBarometer.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    SerialMon.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  std::deque<positionDb::positionElement> queue;
  SerialMon.println("Max size: " + String(queue.max_size()) + " Size of element: " + String(sizeof(positionDb::positionElement)) );

  // Activate pin for reading battery voltage
  pinMode(BATTERY_VOLTAGE, INPUT);

  //BLE iBeacon setup
  #ifdef IBEACON
  gettimeofday(&now, NULL);
  last = now.tv_sec;
  BLEDevice::init("");
  pAdvertising = BLEDevice::getAdvertising();
  setBeacon();
  #endif
}

void loop() {
  TinyGPSPlus gps;
  positionDb posDb;
  float barometricAltitude, temperature;
  bool runNext = true;
  auto start = millis();
  while(runNext)
  {
    if (getBarometricData(barometricAltitude, temperature))
    {
      SerialMon.println("Altitude: " + String(barometricAltitude, 6) + " m Temperature: " + String(temperature, 6) + " C");
    }
    if(readGPS(gps))
    {
      extractGpsData(gps,posDb,barometricAltitude, temperature, (uint8_t)modem.getSignalQuality(), (float)((analogRead(BATTERY_VOLTAGE) * 7.45 ) / 4095));
    }
    else{
      SerialMon.println("Gps Not available");
    }

    if(start + 5000 < millis())
    {
      start = millis();
      sendGsmData(posDb);
    }
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 30 seconds

    //iBeacon 
    #ifdef IBEACON
    pAdvertising->start();
    Serial.println("Advertizing started...");
    delay(100);
    pAdvertising->stop();
    #endif
    delay(5000);
  }
}

void extractGpsData(TinyGPSPlus& gps, positionDb& posDb, float& barometricAltitude, float& temperature, uint8_t signalStrength, float batteryVoltage)
{
  if (gps.location.lat() == 0 || gps.location.lng() == 0 || gps.satellites.value() < 3 ){
    SerialMon.println("No GPS lock, sattelites: " + String(gps.satellites.value()));
    return;
  }

  posDb.addPosition({gps.time.value(),gps.location.lng(),gps.location.lat(),gps.altitude.meters(), barometricAltitude, temperature, signalStrength, batteryVoltage});
  
}

void sendGsmData(positionDb& posDb){
  bool sendNext = true;

  if (!posDb.hasPositionElements()){
    return;
  }
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
  }
  else {
    SerialMon.println(" OK");
    
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");
      SerialMon.println(millis());
    
      // Making an HTTP GET request
      SerialMon.println("Performing HTTP GET request...");
      String httpRequestData = "GET https://api.thingspeak.com/update.json?api_key=9N13CC4IWEVHIBLL";
      
      while(posDb.hasPositionElements() && sendNext)
      {
        SerialMon.println("Number of elements is DB: " + String(posDb.getSize()));
        sendNext = false;
        auto posElem = posDb.consumeNewestElement();

        SerialMon.println("Latitude: " + String(posElem.latitude,6) + " Longitude: " + String(posElem.longitude,6));
        SerialMon.println("Altitude: " + String(posElem.altitude,6) + "m Battery Voltage: " + String(posElem.batteryVoltage,2) + " v");
        SerialMon.println("Barometric Altitude: " + String(posElem.barometricAltitude,2) + "m Temperature: " + String(posElem.temperature,2) + " C");
        SerialMon.println("Signal Strength: " + String(modem.getSignalQuality()) + " dB");
        
        client.print(httpRequestData);
        client.print("&field1=" + String(posElem.barometricAltitude,2));
        client.print("&field2=" + String(posElem.temperature,2));
        client.print("&field3=" + String(posElem.signalStrength));
        client.print("&field4=" + String(posElem.batteryVoltage,2));
        client.print("&latitude=" + String(posElem.latitude,6));
        client.print("&longitude=" + String(posElem.longitude,6));
        client.print("&elevation=" + String(posElem.altitude,6));
        //client.print("&status=" + String("Splendid"));
        client.println(String(" HTTP/1.0"));
        client.println();
        client.println();
        client.println();
        
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 2000L) 
        {
          // Print available data (HTTP response from server)
          while (client.available()) 
          {
            sendNext=true;
            char c = client.read();
            SerialMon.print(c);
            timeout = millis();
          }
        }
                if(!sendNext) // we failed to send current value ( never got a response ), put back message in db
        {
          SerialMon.println("Failed to send position element...");
          posDb.addPosition(std::move(posElem));
        }
        delay(4000);
      }
      

      SerialMon.println();
    
      // Close client and disconnect
      client.stop();
      SerialMon.println(F("Server disconnected"));
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
    }
  }
}

bool readGPS(TinyGPSPlus &gps)
{
  bool newdata = false;
  unsigned long start = millis();
  // Every seconds we print an update
  while (millis() - start < 1000) 
  {
    if (gpsInterface.available()) 
    {
      char c = gpsInterface.read();
      if (gps.encode(c)) 
      {
        newdata = true;
        //break;  // uncomment to print new data immediately!
      }else{
        //SerialMon.print(c);  // uncomment to see raw GPS data
      }
    }
  }
  return newdata;
}

bool getBarometricData(float &barometricAltitude, float &temperature)
{
  
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmpBarometer.getEvent(&event);
  
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    float pressure;
    bmpBarometer.getPressure(&pressure);
    bmpBarometer.getTemperature(&temperature);
    barometricAltitude = bmpBarometer.pressureToAltitude(SEA_LEVEL_PRESSURE, pressure, temperature);
    return true;
  }
  else
  {
    SerialMon.println("Sensor error");
    return false;
  }
}

void setBeacon() {

  BLEBeacon oBeacon = BLEBeacon();
  oBeacon.setManufacturerId(0x4C00); // fake Apple 0x004C LSB (ENDIAN_CHANGE_U16!)
  oBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
  oBeacon.setMajor((bootcount & 0xFFFF0000) >> 16);
  oBeacon.setMinor(bootcount&0xFFFF);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
  
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  
  std::string strServiceData = "";
  
  strServiceData += (char)26;     // Len
  strServiceData += (char)0xFF;   // Type
  strServiceData += oBeacon.getData(); 
  oAdvertisementData.addData(strServiceData);
  
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setScanResponseData(oScanResponseData);

}
