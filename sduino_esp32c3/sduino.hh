
#define FILE_SUCCESS (0)
#define FILE_ERROR_PATH (-1)
#define FILE_ERROR_EXISTS (-2)
#define FILE_ERROR_NOT_FOUND (-2)
#define FILE_ERROR_FAULT (-3)

String formatBytes(size_t bytes);
String getContentType(String filename);

bool initFileSystem(void);
bool formatFileSystem(void);
String getFileList(const String &path);
int createFile(const String &path);
int deleteFile(const String &path);
File getFile(const String &path);
bool file_exists(const String &path);
int fileWriteBegin(const String &path);
int fileWrite(const uint8_t &buffer, const int size);
void fileWriteEnd();

bool httpServerInit(String name);
void handleResponse(int code, String text);
void handleFileList(void);
void handleFileCreate(void);
void handleFileDelete(void);
void handleFileGet(void);
void handleFileGet(const String &path);
void handleFileUpload(void);
void handleUpdate(void);
