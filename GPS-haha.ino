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

void printStartup();
void doGetRequestTest(String server, String port, String message, String protocol = "TCP");
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
  printStartup();
  CP=doGsmCommand("AT+CGREG?");
  if(CP.indexOf("OK" ))//CP.length() != 27)
  {
    Serial.println("CP_Ok="+CP);
    setupGprs();
    internetOk = true;
  }
  else
  {
    Serial.println("NOT GOOD PLACE");
    internetOk = false;
    Serial.println("CP="+CP + " CP_length= " +CP.length());
  }
}

void loop() // run over and over
{
 Serial.println("ACTION TIME!!");
 if( internetOk)
 {
  //doGetRequestTest("luminare.se","5555", "GET Hejtomtegubbar HTTP/1.0"); // Christian test server
  //doGetRequestTest("asksensors.com","80", "GET http://asksensors.com/api.asksensors/write/uGwRYe9At5EKDib5QoIgqBPk6x9C7gIk?module1=loltest HTTP/1.1\r\nHost: asksensors.com\r\nConnection: close\r\n\r\n");
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
  Serial.println("SETUP DONE!!");
}

void doGetRequestTest(String server, String port, String message, String protocol)
{
  String reportString;
  reportString = doGsmCommand("AT+CIPSTART=\"" + protocol + "\",\"" + server + "\",\"" + port + "\"");//start up the connection
  
  Serial.println("Prompt set" + reportString);
  delay(1000);
  
  reportString = doGsmCommand("AT+CIPSEND=" + (String)(message.length()));//begin send data to remote server
  Serial.println("Begin send data: " + reportString);
  reportString = doGsmCommand(message + (String)((char)26));
  Serial.println("sent all data: " + reportString);
  delay(5000);
  reportString = doGsmCommand("AT+CIPACK");//ask for acknowledge details
  Serial.println("Ask for Ack "+ reportString);
  reportString = doGsmCommand("AT+CIPCLOSE");//begin send data to remote server
  Serial.println("Closing connection: " + reportString);
  delay(1000);
}

void printStartup()
{  
  Serial.println(doGsmCommand("AT+CGATT?")); //Check GPRS attachment
  delay(1000);
 
  Serial.println(doGsmCommand("AT+CIPSHUT")); //close the GPRS PDP context.
  delay(1000);
 
  Serial.println(doGsmCommand("AT+CIPSTATUS")); //returns the current connection status. This command returns the applicable server status, client status, conenction number (for multi-ip) and GPRS bearer info.
  delay(2000);
 
  Serial.println(doGsmCommand("AT+CIPMUX=0"));
  delay(2000);
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
    unsigned long timeNow = millis();
    response = gsmInterface.readString();
    if(response.indexOf("OK") != -1)
      break;
    if(response.length() > 3)
      return response;
    if((time + timeout ) > timeNow)
      return "TIMEOUT";
    Serial.println("Data on interface: '" + response + "'");
  }
  return response;
}
