/*
  M5Stack NMEA 2000 display and NMEA 0183 WiFi Gateway
  Reads Data from CAN bus and displays is on the M5Stack module
  Converts NMEA2000 to NMEA0183 and send it to TCP clients via WiFi (port 2222)

  Version 0.1 / 23.11.2019
*/

#define ENABLE_WIFI 0  // Set to 1 to enable M5Stack acts also as NMEA0183 WiFi Gateway. Set to 0 to disable.
#define ENABLE_DEBUG_LOG 0 // Debug log, set to 1 to enable

#define ESP32_CAN_TX_PIN GPIO_NUM_2  // Set CAN TX port to 2 for M5 Stack
#define ESP32_CAN_RX_PIN GPIO_NUM_5  // Set CAN RX port to 5 for M5 Stack

#define LOAD_GFXFF
#define GFXFF 1
#define GLCD  0
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8
#define FSS9 &FreeSans9pt7b
#define FSSB9 &FreeSansBold9pt7b
#define FSSB12 &FreeSansBold12pt7b
#define FSS12 &FreeSans12pt7b
#define FSS18 &FreeSans18pt7b
#define FSS24 &FreeSans24pt7b
#define FSSB24 &FreeSansBold24pt7b

#include <M5Stack.h>
#include "M5StackUpdater.h"
#include <Time.h>
#include <sys/time.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Seasmart.h>
#include <N2kMessages.h>
#include <WiFi.h>

#include "N2kDataToNMEA0183.h"
#include "List.h"
#include "BoatData.h"
#include "battery12pt7b.h"
#define BAT12 &battery12pt7b 


tBoatData BoatData;  // Struct to store Boat Data (from BoatData.h)

// Wifi AP
const char *ssid = "MyM5Stack";
const char *password = "password";

// Put IP address details here
IPAddress local_ip(192, 168, 15, 1);
IPAddress gateway(192, 168, 15, 1);
IPAddress subnet(255, 255, 255, 0);

const uint16_t ServerPort = 2222; // Define the port, where server sends data. Use this e.g. on OpenCPN or Navionics

const size_t MaxClients = 10;
bool SendNMEA0183Conversion = true; // Do we send NMEA2000 -> NMEA0183 conversion
bool SendSeaSmart = false; // Do we send NMEA2000 messages in SeaSmart format

WiFiServer server(ServerPort, MaxClients);

using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

tN2kDataToNMEA0183 vN2kDataToNMEA0183(&NMEA2000, 0);


const unsigned long ReceiveMessages[] PROGMEM = {
  127250L, // Heading
  127258L, // Magnetic variation
  128259UL,// Boat speed
  128267UL,// Depth
  129025UL,// Position
  129026L, // COG and SOG
  129029L, // GNSS
  130306L, // Wind
  128275UL,// Log
  127245UL,// Rudder
  130310UL,// Water temp
  130311UL,// Water temp
  0
};


// Forward declarations
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg);

double t = 0;  // Time
int page = 0;  // Initial page to show
int pages = 5; // Number of pages -1
int LCD_Brightness = 250;
int LCD_Dif = 10;
int counter = 0;
boolean firstTime = true;
int bakBat = 10;
int bat = 0;
char bakDate[12];
char bakLat1[4];
char bakLat2[4];
char bakLon1[4];
char bakLon2[4];

uint16_t textColor = TFT_WHITE;
uint16_t lightBackground = 0x101D;//0x1E9F;
uint16_t darkBackground = 0x0811;//0x439;

long MyTime = 0;  // System Time from NMEA2000
bool TimeSet = false;


void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}

void setup() {
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;

  M5.begin();
  M5.Power.begin();
  Wire.begin();
  if(digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setCursor(0, 0, 1);

  ledcDetachPin(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, INPUT);

  Serial.begin(115200); delay(500);

#if ENABLE_WIFI == 1

  // Init wifi connection
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  delay(100);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Start TCP server
  server.begin();

#endif

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega

  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "M5 Display",  // Manufacturer's Model ID
                                 "1.0.2.25 (2019-07-07)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 32);

  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.AttachMsgHandler(&vN2kDataToNMEA0183); // NMEA 2000 -> NMEA 0183 conversion
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg); // Also send all NMEA2000 messages in SeaSmart format

  vN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);

  NMEA2000.Open();

  Display_Main();

}


//*****************************************************************************
void SendBufToClients(const char *buf) {
  for (auto it = clients.begin() ; it != clients.end(); it++) {
    if ( (*it) != NULL && (*it)->connected() ) {
      (*it)->println(buf);
    }
  }
}

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500
//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {

  if ( !SendSeaSmart ) return;

  char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
  if ( N2kToSeasmart(N2kMsg, millis(), buf, MAX_NMEA2000_MESSAGE_SEASMART_SIZE) == 0 ) return;
  SendBufToClients(buf);
}


#define MAX_NMEA0183_MESSAGE_SIZE 150
//*****************************************************************************
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
  if ( !SendNMEA0183Conversion ) return;

  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if ( !NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE) ) return;
  SendBufToClients(buf);
}


//*****************************************************************************
void AddClient(WiFiClient &client) {
  Serial.println("New Client.");
  clients.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

//*****************************************************************************
void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
  Serial.println("Client Disconnected.");
  (*it)->stop();
  it = clients.erase(it);
}

//*****************************************************************************
void CheckConnections() {
  WiFiClient client = server.available();   // listen for incoming clients

  if ( client ) AddClient(client);

  for (auto it = clients.begin(); it != clients.end(); it++) {
    if ( (*it) != NULL ) {
      if ( !(*it)->connected() ) {
        StopClient(it);
      } else {
        if ( (*it)->available() ) {
          char c = (*it)->read();
          if ( c == 0x03 ) StopClient(it); // Close connection by ctrl-c
        }
      }
    } else {
      it = clients.erase(it); // Should have been erased by StopClient
    }
  }
}


void set_system_time(void) {
  MyTime = (BoatData.DaysSince1970 * 3600 * 24) + BoatData.GPSTime;

  if (MyTime != 0 && !TimeSet) { // Set system time from valid NMEA2000 time (only once)
    struct timeval tv;
    tv.tv_sec = MyTime;
    settimeofday(&tv, NULL);
    TimeSet = true;
  }
}



void loop() {
  M5.update();

  if (millis() > t) {
    t = millis() + 1000;

    counter++;
    if (page == 0) Page_1();
    if (page == 1) Page_2();
    if (page == 2) Page_3();
    if (page == 3) Page_4();
    if (page == 4) Page_5();
    if (page == 5) Page_6();

    set_system_time();
    DiplayDateTime();
  }

  if (M5.BtnB.wasReleased() == true) {
    page++;                        // Button B pressed -> Next page
    if (page > pages) page = 0;
    t -= 1000;
    Display_Main();
  } else if (M5.BtnA.wasReleased() == true) {
    page--;
    if (page < 0) page = pages;
    t -= 1000;
    Display_Main();
  } else if (M5.BtnC.wasReleased() == true) {               /* Button C pressed ? --> Change brightness */
      //      M5.Speaker.tone(NOTE_DH2, 1);
    if (LCD_Brightness >= 250) { LCD_Dif = -10; } 
    else if (LCD_Brightness <= 0) { LCD_Dif = 10; }

    LCD_Brightness = LCD_Brightness + LCD_Dif;            /* Increase brightness */
    M5.Lcd.setBrightness(LCD_Brightness);                   /* Change brightness value */
  }
  
  if (M5.BtnC.wasReleasefor(700) == true) {
    LCD_Dif = - LCD_Dif;
  }

#if ENABLE_WIFI == 1
  CheckConnections();
#endif

  NMEA2000.ParseMessages();
  vN2kDataToNMEA0183.Update(&BoatData);
}

void format21_30(char* outstr, float x) {
  unsigned int xi = abs(trunc(x));
  unsigned count=1;
  unsigned int value= 10;
	while (xi>=value)
	{
		value*=10;
		count++;
	}
	if      (count == 1) sprintf(outstr, "  %1.1f", x);
  else if (count == 2) sprintf(outstr, "%1.1f", x);
  else if (count == 3) sprintf(outstr, " %3.0f", x);
  else if (count >= 4) sprintf(outstr, " %4.0f", x);
}

void format30(char* outstr, float x) {
  unsigned int xi = abs(trunc(x));
  unsigned count=1;
  unsigned int value= 10;
	while (xi>=value)
	{
		value*=10;
		count++;
	}
	if      (count == 1) sprintf(outstr, "%5.0f", x);
  else if (count == 2) sprintf(outstr, "%4.0f", x);
  else if (count == 3) sprintf(outstr, "%3.0f", x);
}

void formatpm180(char* outstr, float x) {
  unsigned int xi = abs(trunc(x));
  unsigned count=1;
  unsigned int value= 10;
	while (xi>=value)
	{
		value*=10;
		count++;
	}
  if (x<0) count++;
	if      (count == 1) sprintf(outstr, "%7.0f", x);
  else if (count == 2) sprintf(outstr, "%6.0f", x);
  else if (count == 3) sprintf(outstr, "%5.0f", x);
  else if (count == 4) sprintf(outstr, "%4.0f", x);
}

void Page_6(void) {
  char buffer[20];
  int xpos = 280;
  int txtLeft = 2;
  int txtRight =  M5.Lcd.width() - 2;
  int y2lines1 = 112;
  int y2lines2 = 196;
  double HDG = 0;
  double Water = 0;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);

  if (BoatData.Heading >= 0 && BoatData.Heading <= 360.) HDG = BoatData.Heading;
  else HDG = 0.;
  if (BoatData.WaterTemperature >= 0 && BoatData.WaterTemperature < 100.) Water = BoatData.WaterTemperature;
  else Water = 0.;

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setTextDatum(L_BASELINE);
    M5.Lcd.setFreeFont(FSS18);
    M5.Lcd.drawString("HDG",txtLeft, y2lines1,GFXFF);  
    M5.Lcd.drawString("WT",txtLeft, y2lines2,GFXFF); 
    M5.Lcd.setTextDatum(R_BASELINE);
    M5.Lcd.drawString("o",txtRight, y2lines1-50,GFXFF);  
    M5.Lcd.drawString("C",txtRight-15, y2lines2,GFXFF);      
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.drawString("o",txtRight, y2lines2-17,GFXFF);  
  }

  M5.Lcd.setTextDatum(R_BASELINE);
  format30(buffer, HDG);
  M5.Lcd.drawString(buffer,xpos, y2lines1,8);
  format21_30(buffer, Water);
  M5.Lcd.drawString(buffer,xpos, y2lines2,8);
}

void Page_5(void) {
  char buffer[20];
  int txtLeft = 2;
  int txtRight =  M5.Lcd.width() - 2;
  int y2lines1 = 112;
  int y2lines2 = 196;
  double TripLog = 0;
  double Log = 0;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);

  if (BoatData.TripLog >= 0) TripLog = BoatData.TripLog;
  else TripLog = 0.;
  if (BoatData.Log >= 0) Log = BoatData.Log;
  else Log = 0.;

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setTextDatum(L_BASELINE);
    M5.Lcd.setFreeFont(FSS12);
    M5.Lcd.drawString("[nm]",txtLeft, y2lines1-50,GFXFF);  
    M5.Lcd.setFreeFont(FSS18);
    M5.Lcd.drawString("Trip",txtLeft, y2lines1,GFXFF);  
    M5.Lcd.drawString("Log",txtLeft, y2lines2,GFXFF); 
  }

  M5.Lcd.setTextDatum(R_BASELINE);
  format21_30(buffer, TripLog);
  M5.Lcd.drawString(buffer,txtRight, y2lines1,8);
  format21_30(buffer, Log);
  M5.Lcd.drawString(buffer,txtRight, y2lines2,8);
}

void Page_4(void) {
  char buffer[20];
  int xpos = 280;
  int txtLeft = 2;
  int txtRight =  M5.Lcd.width() - 2;
  int y2lines1 = 112;
  int y2lines2 = 196;
  double WaterDepth = 0;
  double STW = 0;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);

  if (BoatData.WaterDepth >= 0 && BoatData.WaterDepth <= 360.) WaterDepth = BoatData.WaterDepth;
  else WaterDepth = 0.;
  if (BoatData.STW >= 0 && BoatData.STW < 100.) STW = BoatData.STW;
  else STW = 0.;

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setTextDatum(L_BASELINE);
    M5.Lcd.setFreeFont(FSS12);
    M5.Lcd.drawString("Depth",txtLeft, y2lines1,GFXFF);  
    M5.Lcd.setFreeFont(FSS18);
    M5.Lcd.drawString("STW",txtLeft, y2lines2,GFXFF); 
    M5.Lcd.setTextDatum(R_BASELINE);
    M5.Lcd.drawString("m",txtRight, y2lines1,GFXFF);  
    M5.Lcd.drawString("kn",txtRight, y2lines2,GFXFF);      
  }

  M5.Lcd.setTextDatum(R_BASELINE);
  format21_30(buffer, WaterDepth);
  M5.Lcd.drawString(buffer,xpos, y2lines1,8);
  format21_30(buffer, STW);
  M5.Lcd.drawString(buffer,xpos, y2lines2,8);
}

void Page_3(void) {
  char buffer[20];
  int xpos = 280;
  int txtLeft = 2;
  int txtRight =  M5.Lcd.width() - 2;
  int y2lines1 = 112;
  int y2lines2 = 196;
  double COG = 0;
  double SOG = 0;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);

  if (BoatData.COG >= 0 && BoatData.COG <= 360.) COG = BoatData.COG;
  else COG = 0.;
  if (BoatData.SOG >= 0 && BoatData.SOG < 100.) SOG = BoatData.SOG;
  else SOG = 0.;

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setTextDatum(L_BASELINE);
    M5.Lcd.setFreeFont(FSS18);
    M5.Lcd.drawString("COG",txtLeft, y2lines1,GFXFF);  
    M5.Lcd.drawString("SOG",txtLeft, y2lines2,GFXFF); 
    M5.Lcd.setTextDatum(R_BASELINE);
    M5.Lcd.drawString("o",txtRight, y2lines1-50,GFXFF);
    M5.Lcd.drawString("kn",txtRight, y2lines2,GFXFF);      
  }

  M5.Lcd.setTextDatum(R_BASELINE);
  format30(buffer, COG);
  M5.Lcd.drawString(buffer,xpos, y2lines1,8);
  format21_30(buffer, SOG);
  M5.Lcd.drawString(buffer,xpos, y2lines2,8);
}

void Page_2(void) {
  char buffer[20];
  double fractional, integer;
  int txtLeft = 2;
  int y2lines1up = 60;
  int y2lines1 = 102;
  int y2lines2up = 144;
  int y2lines2 = 186;
  int xchar = 26;
  int xpos2 = 25;
  int xpos1 = xpos2+xchar;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(L_BASELINE);

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setFreeFont(FSS12);
    M5.Lcd.drawString("LAT",txtLeft, y2lines1up,GFXFF);  
    M5.Lcd.drawString("LON",txtLeft, y2lines2up,GFXFF); 
  }
  //BoatData.Latitude = 55.123456789;
  if (abs(BoatData.Latitude)>90.) return;
  if (abs(BoatData.Longitude)>180.) return;

  fractional = modf(abs(BoatData.Latitude), &integer);
  fractional = fractional / 100 * 60;
  //minutes = abs(BoatData.Latitude) - abs(trunc(BoatData.Latitude));
  //minutes = minutes / 100 * 60;

  sprintf(buffer, "%02.0f", integer);
  if (strcmp(bakLat1, buffer) != 0) {
    strncpy(bakLat1, buffer, 4);
    M5.Lcd.setFreeFont(FSS24);
    M5.Lcd.drawString(buffer,xpos1, y2lines1,GFXFF);
    yield();
    M5.Lcd.setFreeFont(FSSB9);
    M5.Lcd.drawString("o",xpos1 +2*xchar, y2lines1-22,GFXFF);
    yield();
  }

  fractional = modf(fractional*100, &integer);
  sprintf(buffer, "%02.0f.", integer);
  if (strcmp(bakLat2, buffer) != 0) {
    strncpy(bakLat2, buffer, 4);
    M5.Lcd.setFreeFont(FSS24);
    M5.Lcd.drawString(buffer,xpos2 +(7*xchar)/2, y2lines1,GFXFF);
  }

  fractional = modf(fractional*1000, &integer);
  if (BoatData.Latitude > 0 )
    sprintf(buffer, ".%03.0f'N ", integer);
  else
    sprintf(buffer, ".%03.0f'S ", integer);
  M5.Lcd.setFreeFont(FSS24);
  delay(1);
  M5.Lcd.drawString(buffer,xpos2 +(11*xchar)/2, y2lines1,GFXFF);
  delay(2);
//#######
  fractional = modf(abs(BoatData.Longitude), &integer);
  fractional = fractional / 100 * 60;

  sprintf(buffer, "%03.0f", integer);
  if (strcmp(bakLon1, buffer) != 0) {
    strncpy(bakLon1, buffer, 4);
    M5.Lcd.setFreeFont(FSS24);
    M5.Lcd.drawString(buffer,xpos1 -xchar, y2lines2,GFXFF);
    yield();
    M5.Lcd.setFreeFont(FSSB9);
    M5.Lcd.drawString("o",xpos1 +2*xchar, y2lines2-22,GFXFF);
    yield();
  }

  fractional = modf(fractional*100, &integer);
  sprintf(buffer, "%02.0f.", integer);
  if (strcmp(bakLon2, buffer) != 0) {
    strncpy(bakLon2, buffer, 4);
    M5.Lcd.setFreeFont(FSS24);
    M5.Lcd.drawString(buffer,xpos2 +(7*xchar)/2, y2lines2,GFXFF);
  }

  fractional = modf(fractional*1000, &integer);
  if (BoatData.Longitude > 0 )
    sprintf(buffer, ".%03.0f'E ", integer);
  else
    sprintf(buffer, ".%03.0f'W ", integer);
  M5.Lcd.setFreeFont(FSS24);
  M5.Lcd.drawString(buffer,xpos2 +(11*xchar)/2, y2lines2,GFXFF);
}

void Page_1(void) {
  char buffer[20];
  int xpos = 285;
  int txtLeft = 2;
  int txtRight =  M5.Lcd.width() - 2;
  int y2lines1 = 112;
  int y2lines2 = 196;
  double AWS = 0;
  double AWA = 0;
    
  M5.Lcd.setTextColor(textColor, darkBackground);
  M5.Lcd.setTextSize(1);

  if (BoatData.AWS >= 0. && BoatData.AWS < 100.) AWS = BoatData.AWS;
  else AWS = 0.;
  if (BoatData.AWA >= -180. && BoatData.AWA <= 360.) AWA = BoatData.AWA;
  else AWA = 0.;

  if (firstTime) {
    firstTime = false;
    M5.Lcd.setTextDatum(L_BASELINE);
    M5.Lcd.setFreeFont(FSS12);
    M5.Lcd.drawString("AWS",txtLeft, y2lines1,GFXFF);  
    M5.Lcd.drawString("AWA",txtLeft, y2lines2,GFXFF); 
    M5.Lcd.setTextDatum(R_BASELINE);
    M5.Lcd.drawString("kn",txtRight, y2lines1,GFXFF);
    M5.Lcd.drawString("o",txtRight, y2lines2-50,GFXFF);
  }

  M5.Lcd.setTextDatum(R_BASELINE);
  format21_30(buffer, AWS);
  M5.Lcd.drawString(buffer,xpos, y2lines1,8);
  formatpm180(buffer, AWA);
  M5.Lcd.drawString(buffer,xpos, y2lines2,8);
}

void DiplayDateTime(void) {
  char buffer[12];

  int txtLeft = 2;
  int yHeadLine = 20;
  int txtRightBat =  M5.Lcd.width() - 6;

  if (MyTime != 0) {

    time_t rawtime = MyTime; // Create time from NMEA 2000 time (UTC)
    struct tm  ts;
    ts = *localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%d.%m.%Y", &ts);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(textColor, lightBackground);
    M5.Lcd.setTextDatum(L_BASELINE);
    if (strcmp(bakDate, buffer) != 0) {
      M5.Lcd.setFreeFont(FSS12);
      M5.Lcd.drawString(buffer,txtLeft, yHeadLine,GFXFF);
      strncpy(bakDate, buffer, 12);
      M5.Lcd.setFreeFont(FSSB9);
      M5.Lcd.drawString("UTC",245, yHeadLine,GFXFF);
    }
    M5.Lcd.setFreeFont(FSS12);
    strftime(buffer, sizeof(buffer), "%H:%M:%S ", &ts);
    M5.Lcd.drawString(buffer,145, yHeadLine,GFXFF);
    yield();

    bat = M5.Power.getBatteryLevel();
    if (bakBat != bat) {
      bakBat = bat;
      if (bat < 30)  { M5.Lcd.setTextColor(TFT_RED, lightBackground);}
      byte bBat = - bat/25 + 69;  //E = 69 leer A = 65 voll
      M5.Lcd.setTextDatum(R_BASELINE);
      M5.Lcd.setFreeFont(BAT12);
      buffer[0]=(char)bBat;
      buffer[1]=0x00;
      M5.Lcd.drawString(buffer,txtRightBat, yHeadLine,GFXFF);
    }
  }
}

void Display_Main (void)
{
  M5.Lcd.setTextFont(1);
  M5.Lcd.fillRect(0, 0, 320, 30, lightBackground);                      /* Upper dark blue area */
  M5.Lcd.fillRect(0, 30, 320, 180, darkBackground);                   /* Main light blue area */
  M5.Lcd.fillRect(0, 210, 320, 30, lightBackground);                    /* Lower dark blue area */
  M5.Lcd.drawFastHLine(0, 29, 320, TFT_WHITE);
  M5.Lcd.drawFastHLine(0, 210, 320, TFT_WHITE);
  firstTime = true;
  bakBat = 30;
  bakDate[0] = 0x00;
  bakLat1[0] = 0x00;
  bakLat2[0] = 0x00;
  bakLon1[0] = 0x00;
  bakLon2[0] = 0x00;
}
