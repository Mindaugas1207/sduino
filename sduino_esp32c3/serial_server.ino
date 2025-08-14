
void handleFileListSerial(void)
{
  if (!server.hasArg("dir"))
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg("dir");
  String output = getFileList(path);
  server.send(200, "text/json", output);
}

void handleFileCreate(void)
{
  if (server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");

  String path = server.arg(0);
  int result = createFile(path);

  if (result == FILE_SUCCESS) server.send(200, "text/plain", "");
  else if (result == FILE_ERROR_PATH) server.send(500, "text/plain", "BAD PATH");
  else if (result == FILE_ERROR_EXISTS) server.send(500, "text/plain", "FILE EXISTS");
  else if (result == FILE_ERROR_FAULT) server.send(500, "text/plain", "CREATE FAILED");
  else server.send(500, "text/plain", "CREATE FAILED");
}

void handleFileDelete(void)
{
  if (server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  
  String path = server.arg(0);
  int result = deleteFile(path);

  if (result == FILE_SUCCESS) server.send(200, "text/plain", "");
  else if (result == FILE_ERROR_PATH) server.send(500, "text/plain", "BAD PATH");
  else if (result == FILE_ERROR_NOT_FOUND) server.send(500, "text/plain", "FileNotFound");
  else if (result == FILE_ERROR_FAULT) server.send(500, "text/plain", "DELETE FAILED");
  else server.send(500, "text/plain", "DELETE FAILED");
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
  else server.send(404, "text/plain", "FileNotFound");
}