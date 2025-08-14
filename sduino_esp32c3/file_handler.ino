
#include <sduino.h>

File wfile;

bool initFileSystem(void)
{
  bool result = false;

  result = FILESYSTEM.begin();

  if (!result)
  {
    if (formatFileSystem())
      result = FILESYSTEM.begin();
  }



  // DBG_PORT.printf("DBG:ESP32C3/File system list start\n");
  // {
  //     File root = FILESYSTEM.open("/");
  //     File file = root.openNextFile();
  //     while(file)
  //     {
  //         String fileName = file.name();
  //         size_t fileSize = file.size();
  //         DBG_PORT.printf("DBG:ESP32C3/File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  //         file = root.openNextFile();
  //     }
  // }
  // DBG_PORT.printf("DBG:ESP32C3/File system list end\n");

  return result;
}

bool formatFileSystem(void)
{
  bool result = false;

  result = FILESYSTEM.format();

  return result;
}

String getFileList(const String &path)
{
  File root = FILESYSTEM.open(path);

  String output = "[";
  if(root.isDirectory())
  {
      File file = root.openNextFile();
      do
      {
          output += "{\"type\":\"";
          output += (file.isDirectory()) ? "dir" : "file";
          output += "\",\"name\":\"";
          output += String(file.path()).substring(1);
          output += "\"},";
          file = root.openNextFile();
      } while (file);
  }
  if (output != "[")
    output.pop_back();
  output += "]";

  return output;
}

int createFile(const String &path)
{
  if (path == "/")
    return FILE_ERROR_PATH;

  if (file_exists(path))
    return FILE_ERROR_EXISTS;

  File file = FILESYSTEM.open(path, FILE_WRITE);
  if (file)
  {
    file.close();
    return FILE_SUCCESS;
  }
  else
    return FILE_ERROR_FAULT;
}

int deleteFile(const String &path)
{
  if (path == "/")
    return FILE_ERROR_PATH;

  if (!file_exists(path))
    return FILE_ERROR_NOT_FOUND;

  if (SPIFFS.remove(path)) return FILE_SUCCESS;
  else return FILE_ERROR_FAULT;
}

File getFile(const String &path)
{
  File file;
  if (file_exists(path)
  {
    file = FILESYSTEM.open(path, FILE_READ);
  }
  else if (file_exists(path + ".gz")
  {
    file = FILESYSTEM.open(path + ".gz", FILE_READ);
  }

  if(!file) file.close();

  return file;
}

bool file_exists(const String &path)
{
  bool yes = false;
  File file = FILESYSTEM.open(path, FILE_READ);

  if(file)
  {
    if(!file.isDirectory()) yes = true;
    file.close();
  }

  return yes;
}

int fileWriteBegin(const String &path)
{
  if (path == "/")
    return FILE_ERROR_PATH;

  if (wfile)
    return FILE_ERROR_PATH;

  wfile = FILESYSTEM.open(path, FILE_WRITE);

  if (!wfile)
    return FILE_ERROR_FAULT;

  return FILE_SUCCESS;
}

int fileWrite(const uint8_t &buffer, const int size)
{
  if (!wfile)
    return FILE_ERROR_FAULT;

  if (wfile.write(buffer, size) != size)
    return FILE_ERROR_FAULT;

  return FILE_SUCCESS;
}

void fileWriteEnd()
{
  if (wfile)
    wfile.close();
}
