// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = "4g.tele2.se"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[] = "luminare.se"; // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "update?";         // resource path, for example: /post-data.php
const int  port = 5555;                             // server port number

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
// BME280 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19
// UBLOX GPS pins
#define GPS_TX               14 // orange (TX on GPS)
#define GPS_RX               12 // red (RX on GPS)

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
    uint8_t  satFixes;
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
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
TwoWire I2CBME = TwoWire(1);
//Adafruit_BME280 bme; 


//-------GPS config-----
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//HardwareSerial serialGPS(2);
//serialGPS.begin(9600, SERIAL_8N1, 32, 33);
SoftwareSerial gpsInterface(GPS_TX, GPS_RX);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);


//-------ESP32 power save configuration------
#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

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

void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);
  gpsInterface.begin(9600);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

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
  //modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  
  // You might need to change the BME280 I2C address, in our case it's 0x76
/*  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }*/

  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  

  std::deque<positionDb::positionElement> queue;
  SerialMon.println("Max size: " + String(queue.max_size()) + " Size of element: " + String(sizeof(positionDb::positionElement)) );
  
  
}

void loop() {
  TinyGPSPlus gps;
  positionDb posDb;
  
  bool runNext = true;
  auto start = millis();
  while(runNext)
  {
    if(readGPS(gps))
    {
      extractGpsData(gps,posDb);
    }

    if(start + 5000 < millis())
    {
      start = millis();
      sendGsmData(posDb);
    }

  }
  
  sendGsmData(posDb);
  
  delay(2000);

  // Put ESP32 into deep sleep mode (with timer wake up)
  //esp_deep_sleep_start();
}

void extractGpsData(TinyGPSPlus& gps, positionDb& posDb)
{
  SerialMon.println("Latitude: " + String(gps.location.lat(), 6) + " Longitude: " + String(gps.location.lng(), 6));
  SerialMon.println("Altitude: " + String(gps.altitude.meters()) + " Sattelites: " + String(gps.satellites.value()));
  if (gps.location.lat() == 0 || gps.location.lng() == 0 || gps.satellites.value() < 3 ){
    return;
  }

  posDb.addPosition({0,gps.location.lng(),gps.location.lat(),gps.altitude.meters(),gps.satellites.value()});
  
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
        sendNext = false;
        auto posElem = posDb.consumeNewestElement();
        client.print(httpRequestData);
        client.print("&latitude=" + String(posElem.latitude,6));
        client.print("&longitude=" + String(posElem.longitude,6));
        client.print("&field1=" + String(posElem.altitude,6));
        client.print("&field2=" + String(posElem.satFixes));
        client.println(String(" HTTP/1.0"));
        client.println();
        client.println();
        client.println();
        
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 1000L) 
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
