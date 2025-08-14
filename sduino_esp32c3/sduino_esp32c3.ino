//#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <WebServer.h>
//#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <SPIFFS.h>
#include <sduino.h>

WiFiMulti wifiMulti;

//#define USE_IR_REC
#define LED_PIN 3
#define IR_PIN 18
#define DBG_PORT Serial
#define port Serial

#define FILESYSTEM SPIFFS

const char* host_name = "SduinoV4";

const IPAddress static_ip(192, 168, 1, 100);
const IPAddress static_gateway(192, 168, 1, 1);
const IPAddress static_subnet(255, 255, 255, 0);

struct wifiStation
{
  char* const ssid;
  char* const pasword;
};

wifiStation wifi_list[] = 
{
  {"Mindaugo", "zHSM6)LYVp1ppp"},
  {"MyndePC_AP", "ASUS123456"},
  {"MyndeAP", "SAMSUNG123456"},
};

// IPAddress client_ip_ap(uint32_t(0));
// IPAddress client_ip_wifi(uint32_t(0));
// IPAddress server_ip_ap(uint32_t(0));
// IPAddress server_ip_wifi(uint32_t(0));
// IPAddress router_ip(uint32_t(0));

WebServer server(80);
//WebSocketsServer ws_server(81);

//#define _connected 1
//#define _disconnected 0
//int socketState = _disconnected;

//holds the current upload
//File fsUploadFile;

// bool handleFileUploadSerial(String FileName, int TotalSize)
// {
//   if (fsUploadFile) {
//     DBG_PORT.printf("CMD:UPLOAD(FAIL)\n");
//   } return false;

//   if (!FileName.startsWith("/"))
//     FileName = "/" + FileName;
  
//   fsUploadFile = FILESYSTEM.open(FileName, "w");

//   uint8_t UploadBuffer[255];
//   int UploadedBytes = 0;
//   int UploadIndex = 0;
  
//   DBG_PORT.printf("CMD:UPLOAD(GET)\n");
//   while (UploadedBytes < TotalSize)
//   {
//     if (DBG_PORT.available()) {
//       int BytesRemaining = TotalSize - UploadedBytes;
//       int ReadBytes = DBG_PORT.readBytes(UploadBuffer + UploadIndex, BytesRemaining > (255 - UploadIndex) ? (255 - UploadIndex) : BytesRemaining);
//       UploadIndex += ReadBytes;
//       UploadedBytes += ReadBytes;
//       if (UploadIndex >= 255) {
//         fsUploadFile.write(UploadBuffer, UploadIndex);
//         UploadIndex = 0;
//       }
//     }
//   }

//   if (UploadIndex != 0) {
//     fsUploadFile.write(UploadBuffer, UploadIndex);
//     UploadIndex = 0;
//   }
  
//   //Upload complete, close file
//   fsUploadFile.close();
//   DBG_PORT.printf("CMD:UPLOAD(OK)\n");
//   DBG_PORT.printf("DBG:ESP32C3/File upload end: Size: %u\n", TotalSize);

//   return true;
// }

// bool handleFirmwareUpdateSerial(String FileName, int TotalSize)
// {
//   if (!Update.begin(TotalSize))
//   {
//     DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
//     DBG_PORT.printf("DBG:ESP32C3/Firmware update.begin error\n");
//     return false;
//   }
//   DBG_PORT.printf("DBG:ESP32C3/Firmware update start: %s\n", FileName.c_str());
    

//   uint8_t UploadBuffer[255];
//   int UploadedBytes = 0;
//   int UploadIndex = 0;
  
//   DBG_PORT.printf("CMD:UPDATE(GET)\n");
//   while (UploadedBytes < TotalSize)
//   {
//     if (DBG_PORT.available()) {
//       int BytesRemaining = TotalSize - UploadedBytes;
//       int ReadBytes = DBG_PORT.readBytes(UploadBuffer + UploadIndex, BytesRemaining > (255 - UploadIndex) ? (255 - UploadIndex) : BytesRemaining);
//       UploadIndex += ReadBytes;
//       UploadedBytes += ReadBytes;
//       if (UploadIndex >= 255) {
//         /* flashing firmware to ESP*/
//         if (Update.write(UploadBuffer, UploadIndex) != UploadIndex)
//         {
//           DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
//           DBG_PORT.printf("DBG:ESP32C3/Firmware update.write error\n");
//           return false;
//         }
//         UploadIndex = 0;
//       }
//     }
//   }

//   if (UploadIndex != 0) {
//     /* flashing firmware to ESP*/
//     if (Update.write(UploadBuffer, UploadIndex) != UploadIndex)
//     {
//       DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
//       DBG_PORT.printf("DBG:ESP32C3/Firmware update.write error\n");
//       return false;
//     }
//     UploadIndex = 0;
//   }
  
//   if (Update.end(true)) //true to set the size to the current progress
//   {
//     DBG_PORT.printf("CMD:UPDATE(OK)\n");
//     DBG_PORT.printf("DBG:ESP32C3/Firmware update success: %u Rebooting...\n", TotalSize);
//     return true;
//   }
//   else
//   {
//     DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
//     DBG_PORT.printf("DBG:ESP32C3/Firmware update.end error\n");
//     return false;
//   }

//   return false;
// }

// bool doUpload(String Message)
// {
//   if (Message.startsWith("(") && Message.endsWith(")"))
//   {
//     int SeparatorIndex = Message.indexOf(":");
//     if (SeparatorIndex < 0) return false;
//     String FileName = Message.substring(sizeof("(") - 1, SeparatorIndex);
//     int FileSize = Message.substring(SeparatorIndex + sizeof(":") - 1).toInt();

//     if (FileName == "" || FileSize <= 0) return false;

//     return handleFileUploadSerial(FileName, FileSize);
//   }
  
//   return false;
// }

// bool doUpdate(String Message)
// {
//   if (Message.startsWith("(") && Message.endsWith(")"))
//   {
//     int SeparatorIndex = Message.indexOf(":");
//     if (SeparatorIndex < 0) return false;
//     String FileName = Message.substring(sizeof("(") - 1, SeparatorIndex);
//     int FileSize = Message.substring(SeparatorIndex + sizeof(":") - 1).toInt();

//     if (FileName == "" || FileSize <= 0) return false;

//     return handleFirmwareUpdateSerial(FileName, FileSize);
//   }
  
//   return false;
// }

void setup(void) {
  DBG_PORT.begin(115200);
  DBG_PORT.setDebugOutput(false);
  pinMode(LED_PIN, OUTPUT);

  // if (FORMAT_FILESYSTEM) SPIFFS.format();
  // SPIFFS.begin();

  // //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // //WiFi.setHostname(host_name);  //define hostname
  // // Create ap
  // WiFi.softAP(ap_ssid, ap_password, 1, 0, 1);
  // server_ip_ap = WiFi.softAPIP();
  // // Connect to WiFi network
  // WiFi.begin(home_ssid, home_password);
  // server_ip_wifi = WiFi.localIP();
  // router_ip = WiFi.gatewayIP();
  // // Begin mdns
  // MDNS.begin(host_name);
  //irrecv.enableIRIn(); // Start the receiver
  if (!initFileSystem())
  {
    DBG_PORT.printf("DBG:ESP32C3/init failed [File System]\n");
    delay(2000);
    ESP.restart();
    while (true)
    {

    }
  }
  wifi_init();
  httpServerInit();
}

bool ledState = 0;
void loop(void)
{
  digitalWrite(LED_PIN, !ledState);
  if (DBG_PORT.available()) {
    String message = "";
    message += DBG_PORT.readStringUntil('\n');
    if (!parseMessage(message))
    {
      DBG_PORT.print(message);
    }
  }

  //ws_server.loop();
  server.handleClient();
  handleWifi();
  delay(2);
}

// bool parseMessage(String Message)
// {
  
//     if (Message.startsWith("CMD:"))
//         return parseCommand(Message.substring(sizeof("CMD:") - 1));
//     else if (Message.startsWith("DBG:"))
//         return false;
    
//     return false;
// }

// bool parseCommand(String Message)
// {
//     if (Message.startsWith("WS"))
//       return doWS(Message.substring(sizeof("WS") - 1));
//     else if (Message.startsWith("UPLOAD"))
//       return doUpload(Message.substring(sizeof("UPLOAD") - 1));
//     else if (Message.startsWith("UPDATE"))
//       return doUpdate(Message.substring(sizeof("UPDATE") - 1));

//     return false;
// }

// bool doWS(String Message)
// {
//   if (Message.startsWith("(") && Message.endsWith(")"))
//   {

//     if(socketState == _connected){
//       ws_server.broadcastTXT(Message);
//       return true;
//     }

//     return true;
//   }
//   return false;
// }


void wifi_init(void)
{
  //WIFI INIT
  WiFi.setHostname(host_name);  //define hostname
  WiFi.config(static_ip, static_gateway, static_subnet);
  WiFi.mode(WIFI_MODE_AP);

  for (int i = 0; i < sizeof(wifi_list) / sizeof(wifiStation); i++)
  {
    wifiMulti.addAP(wifi_list[i].ssid, wifi_list[i].pasword);
  }
  
  //DBG_PORT.printf("DBG:ESP32C3/Starting AP, %s\n", ap_ssid);
  //WiFi.softAP(ap_ssid, ap_password, 1, 0, 1);
  
  //DBG_PORT.printf("DBG:ESP32C3/Connecting to %s\n", pc_ssid);
  //WiFi.begin(home_ssid, home_password);
}

bool connectAcc = false;

void handleWifi(void)
{
  uint8_t st = wifiMulti.run();
  if (st == WL_CONNECTED && !connectAcc)
  {
    IPAddress myip = WiFi.localIP();
    DBG_PORT.printf("DBG:ESP32C3/Connected to %s, local ip: %s\n", WiFi.SSID(), myip.toString().c_str());
    connectAcc = true;
  }
  else if (st != WL_CONNECTED && connectAcc)
  {
    DBG_PORT.printf("DBG:ESP32C3/Disconnected from %s\n", WiFi.SSID());
    connectAcc = false;
  }
}

  // ws_server.begin();
  // ws_server.onEvent(webSocketEvent);
// void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
//   switch (type) {
//     case WStype_DISCONNECTED:             // if the websocket is disconnected
//       DBG_PORT.printf("DBG:[%u] WS Disconnected\n", num);
//       socketState = _disconnected;
//       break;
//     case WStype_CONNECTED: {        // if a new websocket connection is established
//         IPAddress ip = ws_server.remoteIP(num);
//         DBG_PORT.printf("DBG:[%u] WS Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
//         socketState = _connected;
//       }
//       break;
//     case WStype_TEXT:                     // if new text data is received
//       //webSocket.sendTXT(num, payload, lenght);
//       if(lenght != 0){
//         DBG_PORT.printf("WS:[%u](%s)\n", num, payload);
//       }
//       break;
//   }
// }









