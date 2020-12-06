/*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <string.h>

String Arsp, Grsp;
SoftwareSerial gsmInterface(7, 6); // RX, TX
SoftwareSerial gpsInterface(10, 11);
TinyGPS gps;

String APN_NAME = "\"services.telenor.se\",";
String APN_PIN  = "\"\",";
String APN_PASS = "\"\"";

bool internetOk = false;
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

String doGsmCommand(String cmd, int timeout = 1000);
void sendToThingSpeak();
int openTcp(String adress, String port);
int sendTcpString(String data,int terminate);
int sendTcpByte(char byte, int terminate);
int closeTcp();

using namespace::std;

void doPassthrouhg()
{
  //#define PASSTHROUGH

  #ifdef PASSTHROUGH
  Serial.println("START:");
  for(;;)
  {
    if (Serial.available()) {      // If anything comes in Serial (USB),
      gsmInterface.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
    }

    if (gsmInterface.available()) {     // If anything comes in Serial1 (pins 0 & 1)
      Serial.write(gsmInterface.read());   // read it and send it out Serial (USB)
    }
  }
  #endif
}

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  gsmInterface.begin(9600);
  Serial.println("start stuff");
  delay(20000);
  doPassthrouhg();
  String CP;
  //setupGprs();
  //CP=doGsmCommand("AT+CIFSR",1000);
  CP=doGsmCommand("AT+CGREG?");
  delay(1000);  
  Serial.println(doGsmCommand("AT+CGATT?"));
  delay(1000);
 
  Serial.println(doGsmCommand("AT+CIPSHUT"));
  delay(1000);
 
  Serial.println(doGsmCommand("AT+CIPSTATUS"));
  delay(2000);
 
  Serial.println(doGsmCommand("AT+CIPMUX=0"));
  delay(2000);
  if(CP.indexOf("OK" ))//CP.length() != 27)
  {
    Serial.println("CP_Ok="+CP);
    setupGprs();
    internetOk = true;
  }
  else
    internetOk = false;
    Serial.println("CP="+CP + " CP_length= " +CP.length());
}

void loop() // run over and over
{
 //doGsmTransfer();
 if( internetOk)
 {
  sendToThingSpeak();
 }
 else
 {
  Serial.println("Serial Not Ok");
 }
 delay(10000);
}

void setupGprs()
{

  Serial.println("Waiting for GSM module to be ready...");
  String reportString = "";
  for(;;)
  {
    reportString = doGsmCommand("AT");
    if(reportString.indexOf("OK") != -1)
      break;
      
    if(reportString.indexOf("TIMEOUT") != -1)
      Serial.println("Modem did not respond\n");

    delay(250);
  }

  printPower();
  
  reportString = doGsmCommand("ATI");
  Serial.println("Module:" + reportString );

  Serial.println("Enable all features...");

  while ((reportString = doGsmCommand("AT+CFUN=1", 2000)).indexOf("OK") == -1 )
  {
    Serial.println("Try again, enable all features... \nLast response was:\n" + reportString);    
    delay(2000);
    Serial.println("Features = " + reportString);
  }
  
  Serial.println("Check that SIM is ready for GPRS ");
  while ((reportString = doGsmCommand("AT+CPIN?", 1000)).indexOf("OK") == -1)
  {
    Serial.println("Still waiting for SIM to be ready for GPRS... \nLast response was:\n" + reportString);   

     delay(2000);
  }
  Serial.println("Response from SIM ready is: "+ reportString);
  Serial.println("Enable extended result code reporting\n");
  reportString = doGsmCommand("AT+CMEE=1");
  //Serial.println("Response was " + reportString);

  Serial.println("Set APN to: " + APN_NAME + APN_PIN + APN_PASS + "\n");
  reportString = doGsmCommand("AT+CSTT=" + APN_NAME + APN_PIN + APN_PASS); //  AT+CSTT="services.telenor.se","",""
  //Serial.println("Response was " + reportString);
  delay(2000);
  Serial.println("Start GPRS connection");
  while((reportString = doGsmCommand("AT+CIICR")).indexOf("OK" )== -1) //Starting GPRS connection
  {
    Serial.println("Failed to establish GPRS session\n");
    Serial.println(reportString+"\n");
    delay(3000);
  }
  
  reportString = doGsmCommand("AT+CIFSR");
  Serial.println("Obtained IP: " + reportString);
  reportString = doGsmCommand("AT+CIPSPRT=1"); //activates echo >
  Serial.println("Prompt set" + reportString);
  delay(10000);
}




void sendToThingSpeak()
{
  String reportString;
  int time = millis();

  String str="GET https://api.thingspeak.com/update?api_key=9N13CC4IWEVHIBLL&field1=" + String(time);
  Serial.println(str.length());
  
  delay(3000);

  reportString = doGsmCommand("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//start up the connection
  Serial.println("Prompt set" + reportString);
  delay(1000);

  reportString = doGsmCommand("AT+CIPSEND=" + str.length());//begin send data to remote server
  Serial.println("Begin send data: " + reportString);
  delay(4000);
  
  
  Serial.println("Adding string" + str);
  reportString = doGsmCommand(str);//begin send data to remote server
  Serial.println("Add response" + reportString);
  delay(4000);

  reportString = doGsmCommand((char)26);//sending
  Serial.println("Sending" + reportString);
  delay(5000);//waiting for reply, important! the time is base on the condition of internet 
  
  reportString = doGsmCommand("AT+CIPACK");//ask for acknowledge details
  Serial.println("Ask for Ack "+ reportString);

  reportString = doGsmCommand("AT+CIPSHUT");//close the connection
  Serial.println("Connection Closed" + reportString);
  delay(100);

} 


void printPower()
{
  Serial.println("Check power\n");
  String reportString = doGsmCommand("AT+CBC");
 delay(1000);
  Serial.println("Response was " + reportString);
}

String doGsmCommand(String cmd, int timeout = 1000)
{
  unsigned long time = millis();
  Serial.println("SEND: "+cmd);
  gsmInterface.println(cmd+"\r\n");
  String response = "";
  delay(10);

  for(;;)
  {
    delay(150);
    response = gsmInterface.readString();
    if(response.indexOf("OK") != -1)
      break;
    if(response.length() > 3)
      return response;
    if((time + timeout ) > millis())
      return "TIMEOUT";
    Serial.println("Data on interface: '" + response + "'");
  }
  return response;
}


void sendText()
{
//Set SMS format to ASCII
  gsmInterface.println("AT+CMGF=1\r\n");
  delay(1000);
  Serial.println(gsmInterface.read());
  //Send new SMS command and message number
  gsmInterface.println("AT+CMGS=\"+46707959528\"\r\n");
  delay(1000);
  Serial.println(gsmInterface.read());
  //Send SMS content
  gsmInterface.println("This is balloon speaking\r\n");
  delay(50);
  //gsmInterface.write(String(millis()/1000).toCharArray());
  delay(1000);
   
  //Send Ctrl+Z / ESC to denote SMS message is complete
  gsmInterface.write((char)26);
  delay(1000);
  Serial.println(gsmInterface.read());
  Serial.println("SMS Sent!");
}

void readGPS()
{
  bool newdata = false;
  unsigned long start = millis();
  // Every 5 seconds we print an update
  while (millis() - start < 5000) 
  {
    if (gpsInterface.available()) 
    
    {
      char c = gpsInterface.read();
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  // uncomment to print new data immediately!
      }else{
        Serial.print(c);  // uncomment to see raw GPS data
      }
    }
  }
  
  if (newdata) 
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour+8));  Serial.print(":"); //Serial.print("UTC +08:00 Malaysia");
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths)); Serial.print(" UTC +08:00 Malaysia");
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

int openTcp(String adress, String port)
{
  //String tcpOpen()
  //gsmInterface.println("AT+CIPSTART=\"TCP\",\""+adress+"\",\""+port+"\"");
  //delay(150);
  return 0;
}
