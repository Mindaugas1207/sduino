
#ifndef INC_INTERFACE_HPP_
#define INC_INTERFACE_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "string.h"
#include "functional"

#define INTERFACE_RECEIVE_BUFFER_LENGTH 256
#define INTERFACE_BULK_TRANSFER_SIZE 1024

#define INTERFACE_GET_OK 0
#define INTERFACE_GET_NOVAL 1
#define INTERFACE_GET_EOF 2
#define INTERFACE_GET_MAX_INDEX 0xFFFF

#define INTERFACE_SET_OK 0
#define INTERFACE_SET_NOVAL 1
#define INTERFACE_SET_EOF 2

class Interface_s
{
    char ReceiveBuffer[INTERFACE_RECEIVE_BUFFER_LENGTH];
    char *ReceiveBufferPointer = ReceiveBuffer;
    const char *ReceiveBufferEnd = &ReceiveBuffer[INTERFACE_RECEIVE_BUFFER_LENGTH-2]; // leave one for null termination

    std::function<std::tuple<int, float>(int)> getCallback;
    std::function<int(int, float)> setCallback;
public:
    void Init(std::function<std::tuple<int, float>(int)> _getCallback, std::function<int(int, float)> _setCallback);

    void Process();

    bool parseCommand(char *CmdStr);
    bool doGetCMDS();
    bool doGetAll();
    bool doGet(char *CmdStr);
    bool doSet(char *CmdStr);

    bool Parse();

};

// namespace Interface
// {

// class Range_s {
//     float get(void);
//     void set(float);
// };

// };



#endif
