//#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <SPIFFS.h>

WiFiMulti wifiMulti;

//#define USE_IR_REC
#define LED_PIN 3
#define IR_PIN 18
#define DBG_PORT Serial

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
WebSocketsServer ws_server(81);

#define _connected 1
#define _disconnected 0
int socketState = _disconnected;

//holds the current upload
File fsUploadFile;

bool handleFileUploadSerial(String FileName, int TotalSize)
{
  if (fsUploadFile) {
    DBG_PORT.printf("CMD:UPLOAD(FAIL)\n");
  } return false;

  if (!FileName.startsWith("/"))
    FileName = "/" + FileName;
  
  fsUploadFile = FILESYSTEM.open(FileName, "w");

  uint8_t UploadBuffer[255];
  int UploadedBytes = 0;
  int UploadIndex = 0;
  
  DBG_PORT.printf("CMD:UPLOAD(GET)\n");
  while (UploadedBytes < TotalSize)
  {
    if (DBG_PORT.available()) {
      int BytesRemaining = TotalSize - UploadedBytes;
      int ReadBytes = DBG_PORT.readBytes(UploadBuffer + UploadIndex, BytesRemaining > (255 - UploadIndex) ? (255 - UploadIndex) : BytesRemaining);
      UploadIndex += ReadBytes;
      UploadedBytes += ReadBytes;
      if (UploadIndex >= 255) {
        fsUploadFile.write(UploadBuffer, UploadIndex);
        UploadIndex = 0;
      }
    }
  }

  if (UploadIndex != 0) {
    fsUploadFile.write(UploadBuffer, UploadIndex);
    UploadIndex = 0;
  }
  
  //Upload complete, close file
  fsUploadFile.close();
  DBG_PORT.printf("CMD:UPLOAD(OK)\n");
  DBG_PORT.printf("DBG:ESP32C3/File upload end: Size: %u\n", TotalSize);

  return true;
}

bool handleFirmwareUpdateSerial(String FileName, int TotalSize)
{
  if (!Update.begin(TotalSize))
  {
    DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
    DBG_PORT.printf("DBG:ESP32C3/Firmware update.begin error\n");
    return false;
  }
  DBG_PORT.printf("DBG:ESP32C3/Firmware update start: %s\n", FileName.c_str());
    

  uint8_t UploadBuffer[255];
  int UploadedBytes = 0;
  int UploadIndex = 0;
  
  DBG_PORT.printf("CMD:UPDATE(GET)\n");
  while (UploadedBytes < TotalSize)
  {
    if (DBG_PORT.available()) {
      int BytesRemaining = TotalSize - UploadedBytes;
      int ReadBytes = DBG_PORT.readBytes(UploadBuffer + UploadIndex, BytesRemaining > (255 - UploadIndex) ? (255 - UploadIndex) : BytesRemaining);
      UploadIndex += ReadBytes;
      UploadedBytes += ReadBytes;
      if (UploadIndex >= 255) {
        /* flashing firmware to ESP*/
        if (Update.write(UploadBuffer, UploadIndex) != UploadIndex)
        {
          DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
          DBG_PORT.printf("DBG:ESP32C3/Firmware update.write error\n");
          return false;
        }
        UploadIndex = 0;
      }
    }
  }

  if (UploadIndex != 0) {
    /* flashing firmware to ESP*/
    if (Update.write(UploadBuffer, UploadIndex) != UploadIndex)
    {
      DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
      DBG_PORT.printf("DBG:ESP32C3/Firmware update.write error\n");
      return false;
    }
    UploadIndex = 0;
  }
  
  if (Update.end(true)) //true to set the size to the current progress
  {
    DBG_PORT.printf("CMD:UPDATE(OK)\n");
    DBG_PORT.printf("DBG:ESP32C3/Firmware update success: %u Rebooting...\n", TotalSize);
    return true;
  }
  else
  {
    DBG_PORT.printf("CMD:UPDATE(FAIL)\n");
    DBG_PORT.printf("DBG:ESP32C3/Firmware update.end error\n");
    return false;
  }

  return false;
}

bool doUpload(String Message)
{
  if (Message.startsWith("(") && Message.endsWith(")"))
  {
    int SeparatorIndex = Message.indexOf(":");
    if (SeparatorIndex < 0) return false;
    String FileName = Message.substring(sizeof("(") - 1, SeparatorIndex);
    int FileSize = Message.substring(SeparatorIndex + sizeof(":") - 1).toInt();

    if (FileName == "" || FileSize <= 0) return false;

    return handleFileUploadSerial(FileName, FileSize);
  }
  
  return false;
}

bool doUpdate(String Message)
{
  if (Message.startsWith("(") && Message.endsWith(")"))
  {
    int SeparatorIndex = Message.indexOf(":");
    if (SeparatorIndex < 0) return false;
    String FileName = Message.substring(sizeof("(") - 1, SeparatorIndex);
    int FileSize = Message.substring(SeparatorIndex + sizeof(":") - 1).toInt();

    if (FileName == "" || FileSize <= 0) return false;

    return handleFirmwareUpdateSerial(FileName, FileSize);
  }
  
  return false;
}

//#define DECODE_RC5
// #define DECODE_RC5
// #include <IRremote.h>
// IRrecv irrecv(IR_PIN);
// decode_results results;
// void irRemoteTask( void *pvParameters );
//BluetoothSerial SerialBT;
/*
 * setup function
 */
void setup(void) {
  DBG_PORT.begin(115200);
  DBG_PORT.setDebugOutput(false);
  //SerialBT.begin(host_name);
  ////SerialBT.setPin("1234");
// #ifdef USE_IR_REC
//   xTaskCreatePinnedToCore(
//     irRemoteTask
//     ,  "IrRemoteTask"   // A name just for humans
//     ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
//     ,  NULL
//     ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//     ,  NULL 
//     ,  ARDUINO_RUNNING_CORE);
// #endif
  // pinMode(IR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  // IrReceiver.begin(IR_PIN, false, LED_PIN);
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
  server_init();
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

  // if(Serial.available()>0){
  //     sizeReturn = Serial.readBytesUntil('\n',receiveBuffer,100);
  //     //Serial.write(receiveBuffer,sizeReturn);
  //     //Serial.write('\n');
  //     if(socketState == _connected){
  //       webSocket.broadcastTXT(receiveBuffer, sizeReturn);
  //     }
  // }

  // if (SerialBT.available()) {
  //   String message = "";
  //   message += SerialBT.readStringUntil('\n');
  //   DBG_PORT.print(message);
  // }

  // if (SerialBT.available()) {
  //   String message = "";
  //   message += SerialBT.readStringUntil('\n');
  //   DBG_PORT.print(message);
  // }
  // if (irrecv.decode(&results)) {
  //     //Serial.printf("%x, %x, %d\n", results.value, results.address, results.bits);
  //     DBG_PORT.printf("CMD:SET( var-11 : %d )\n", results.value);
  //     irrecv.resume(); // Receive the next value
  // }
  ws_server.loop();
  server.handleClient();
  handleWifi();
  delay(2);
}

bool parseMessage(String Message)
{
  
    if (Message.startsWith("CMD:"))
        return parseCommand(Message.substring(sizeof("CMD:") - 1));
    else if (Message.startsWith("DBG:"))
        return false;
    
    return false;
}

bool parseCommand(String Message)
{
    if (Message.startsWith("WS"))
      return doWS(Message.substring(sizeof("WS") - 1));
    else if (Message.startsWith("UPLOAD"))
      return doUpload(Message.substring(sizeof("UPLOAD") - 1));
    else if (Message.startsWith("UPDATE"))
      return doUpdate(Message.substring(sizeof("UPDATE") - 1));

    return false;
}

bool doWS(String Message)
{
  if (Message.startsWith("(") && Message.endsWith(")"))
  {

    if(socketState == _connected){
      ws_server.broadcastTXT(Message);
      return true;
    }

    return true;
  }
  return false;
}

// void irRemoteTask(void *pvParameters)  // This is a task.
// {
//   (void) pvParameters;

//   // DBG_PORT.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

//   // // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
//   pinMode(IR_PIN, INPUT);
//   pinMode(LED_PIN, OUTPUT);
//   //IrReceiver.begin(IR_PIN, false, LED_PIN);

//   // DBG_PORT.print(F("Ready to receive IR signals of protocols: "));
//   // printActiveIRProtocols(&DBG_PORT);
//   // //DBG_PORT.println(F("at pin " STR(IR_PIN)));

//   // pinMode(LED_PIN, OUTPUT);
//     /*
//      * Check if received data is available and if yes, try to decode it.
//      * Decoded result is in the IrReceiver.decodedIRData structure.
//      *
//      * E.g. command is in IrReceiver.decodedIRData.command
//      * address is in command is in IrReceiver.decodedIRData.address
//      * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
//      */
    
//     //pinMode(LED_PIN, OUTPUT);

//     for (;;)
//     {
//       // if (IrReceiver.decode()) {

//       //   // Print a short summary of received data
//       //   IrReceiver.printIRResultShort(&DBG_PORT);
//       //   IrReceiver.printIRSendUsage(&DBG_PORT);
//       //   if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
//       //       Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
//       //       // We have an unknown protocol here, print more info
//       //       IrReceiver.printIRResultRawFormatted(&DBG_PORT, true);
//       //   }
//       //   Serial.println();

//       //   /*
//       //    * !!!Important!!! Enable receiving of the next value,
//       //    * since receiving has stopped after the end of the current received data packet.
//       //    */
//       //   IrReceiver.resume(); // Enable receiving of the next value

//       //   /*
//       //    * Finally, check the received data and perform actions according to the received command
//       //    */
//       //   if (IrReceiver.decodedIRData.command == 0x10) {
//       //       // do something
//       //   } else if (IrReceiver.decodedIRData.command == 0x11) {
//       //       // do something else
//       //   }
//       // }
//       //digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//       //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
//       //digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
//       vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
//     }
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

void server_init(void)
{
  MDNS.begin(host_name);
  //SERVER INIT
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() {
    handleFileGet("/edit.htm");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //create file
  server.on("/editBackup", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST' action='/edit' enctype='multipart/form-data'>File upload:<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);

  server.on("/update", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'>File upload:<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, handleFirmwareUpdate);

  //called when the url is not defined here
  //use it to load content from FILESYSTEM
  server.onNotFound(handleFileGet);

  server.on("/sduino/data", HTTP_POST, []() {
    int totalArgs = server.args();
    for (int i = 0; i < totalArgs; i++) {
      DBG_PORT.printf("CMD:SET(%s:%s)\n", server.argName(i), server.arg(i));
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/sduino/data", HTTP_GET, []() {
    int totalArgs = server.args();
    for (int i = 0; i < totalArgs; i++) {
      DBG_PORT.printf("CMD:GET(%s)\n", server.argName(i));
    }
    String json = "{";
    json += DBG_PORT.readStringUntil('\n');
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  server.begin();
  DBG_PORT.printf("DBG:ESP32C3/Server started at http://%s.local/\n", host_name);

  ws_server.begin();
  ws_server.onEvent(webSocketEvent);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      DBG_PORT.printf("DBG:[%u] WS Disconnected\n", num);
      socketState = _disconnected;
      break;
    case WStype_CONNECTED: {        // if a new websocket connection is established
        IPAddress ip = ws_server.remoteIP(num);
        DBG_PORT.printf("DBG:[%u] WS Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        socketState = _connected;
      }
      break;
    case WStype_TEXT:                     // if new text data is received
      //webSocket.sendTXT(num, payload, lenght);
      if(lenght != 0){
        DBG_PORT.printf("WS:[%u](%s)\n", num, payload);
      }
      break;
  }
}





void handleFileUpload(void)
{
  if (server.uri() != "/edit") return;

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    if (fsUploadFile) return;
    String filename = upload.filename;
    if (!filename.startsWith("/"))
      filename = "/" + filename;
    
    DBG_PORT.printf("DBG:ESP32C3/File upload start: Name: %s\n", filename);
    fsUploadFile = FILESYSTEM.open(filename, "w");
    filename = String();
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
      fsUploadFile.close();
    DBG_PORT.printf("DBG:ESP32C3/File upload end: Size: %u\n", upload.totalSize);
  }
}

void handleFirmwareUpdate(void)
{
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    DBG_PORT.printf("DBG:ESP32C3/Firmware update start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) //start with max available size
      DBG_PORT.printf("DBG:ESP32C3/Firmware update.begin error\n");
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    /* flashing firmware to ESP*/
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
      DBG_PORT.printf("DBG:ESP32C3/Firmware update.write error\n");
  } 
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (Update.end(true)) //true to set the size to the current progress
      DBG_PORT.printf("DBG:ESP32C3/Firmware update success: %u Rebooting...\n", upload.totalSize);
    else
      DBG_PORT.printf("DBG:ESP32C3/Firmware update.end error\n");
  }
}









