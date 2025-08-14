
#include <sduino.h>

int fileUploadState = 0;

bool httpServerInit(String name)
{
  MDNS.begin(name);
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() { handleFileGet("/edit.htm"); });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    handleResponse(200, fileUploadState != 0 ? "FAIL" : "OK");
  }, handleFileUpload);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //create file
  server.on("/uploadFile", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST' action='/edit' enctype='multipart/form-data'>File upload:<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });
  //firmware update
  server.on("/update", HTTP_GET, []() {
    server.send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'>File upload:<input type='file' name='upload'><input type='submit' value='Upload'></form>");
  });
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    handleResponse(200, Update.hasError() ? "FAIL" : "OK");
    ESP.restart();
  }, handleUpdate);

  server.on("/sduino/data", HTTP_POST, []() {
    int totalArgs = server.args();
    for (int i = 0; i < totalArgs; i++) {
      DBG_PORT.printf("CMD:SET(%s:%s)\n", server.argName(i), server.arg(i));
    }
    handleResponse(200, "OK");
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

  server.onNotFound(handleFileGet);
  server.begin();
  return true;
}

void handleResponse(int code, String text)
{
  server.send(code, "text/plain", text);
}

void handleFileList(void)
{
  if (!server.hasArg("dir"))
    return handleResponse(500, "BAD ARGS");
  String path = server.arg("dir");
  String output = getFileList(path);
  server.send(200, "text/json", output);
}

void handleFileCreate(void)
{
  if (server.args() == 0)
    return handleResponse(500, "BAD ARGS");

  String path = server.arg(0);
  int result = createFile(path);

  if (result == FILE_SUCCESS) handleResponse(200, "");
  else if (result == FILE_ERROR_PATH) handleResponse(500, "BAD PATH");
  else if (result == FILE_ERROR_EXISTS) handleResponse(500, "FILE EXISTS");
  else if (result == FILE_ERROR_FAULT) handleResponse(500, "CREATE FAILED");
  else handleResponse(500, "CREATE FAILED");
}

void handleFileDelete(void)
{
  if (server.args() == 0)
    return handleResponse(500, "BAD ARGS");
  
  String path = server.arg(0);
  int result = deleteFile(path);

  if (result == FILE_SUCCESS) handleResponse(200, "");
  else if (result == FILE_ERROR_PATH) handleResponse(500, "BAD PATH");
  else if (result == FILE_ERROR_NOT_FOUND) handleResponse(500, "FileNotFound");
  else if (result == FILE_ERROR_FAULT) handleResponse(500, "DELETE FAILED");
  else handleResponse(500, "DELETE FAILED");
}

void handleFileGet(void)
{
  handleFileGet(server.uri());
}

void handleFileGet(const String &path)
{
  File file = getFile(path);

  if (file)
  {
    server.streamFile(file, getContentType(path));
    file.close();
    return true;
  }
  else handleResponse(404, "FileNotFound");
}

void handleFileUpload(void)
{
  int result = -10;
  if (server.uri() != "/edit") return;

  auto upload = server.upload();

  if (upload.status == UPLOAD_FILE_START)
    result = fileWriteBegin(upload.filename);
  else if (upload.status == UPLOAD_FILE_WRITE)
    result = fileWrite(upload.buf, upload.currentSize);
  else if (upload.status == UPLOAD_FILE_END)
    result = fileWriteEnd();
  fileUploadState = result;
}

void handleUpdate(void)
{
  bool error = true;
  if (server.uri() != "/update") return;

  auto upload = server.upload();

  if (upload.status == UPLOAD_FILE_START)
    error = !Update.begin(UPDATE_SIZE_UNKNOWN);//start with max available size
  else if (upload.status == UPLOAD_FILE_WRITE)
    error = Update.write(upload.buf, upload.currentSize) != upload.currentSize;
  else if (upload.status == UPLOAD_FILE_END)
    error = Update.end(true);
  if (error)
  {
  }
}
