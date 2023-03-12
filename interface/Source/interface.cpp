
#include "interface.hpp"

void Interface_s::Init(std::function<std::tuple<int, float>(int)> _getCallback, std::function<int(int, float)> _setCallback)
{
    getCallback = _getCallback;
    setCallback = _setCallback;
}

void Interface_s::Process()
{
    while (uart_is_readable(uart_internal))
    {
        *ReceiveBufferPointer++ = (char) uart_get_hw(uart_internal)->dr;
        if (*(ReceiveBufferPointer-1) == '\n' || ReceiveBufferPointer > ReceiveBufferEnd)
        {
            *ReceiveBufferPointer = '\0';
            Parse();
            ReceiveBufferPointer = ReceiveBuffer;
            return;
        }
    }
}

bool Interface_s::parseCommand(char *CmdStr)
{
    if (strncmp(CmdStr, "SET", sizeof("SET") - 1) == 0)
        return doSet(CmdStr + sizeof("SET") - 1);
    else if (strncmp(CmdStr, "GET( ALL )", sizeof("GET( ALL )") - 1) == 0)
        return doGetAll();
    else if (strncmp(CmdStr, "GET", sizeof("GET") - 1) == 0)
        return doGet(CmdStr + sizeof("GET") - 1);

    return false;
}

bool Interface_s::doGetAll()
{
    char buff[INTERFACE_BULK_TRANSFER_SIZE];
    int n = 0;
    bool StartFlag = true;
    bool EndFlag = false;
    for (int i = 0; i <= INTERFACE_GET_MAX_INDEX; i++)
    {
        auto [Status, Value] = getCallback(i);

        switch (Status)
        {
        case INTERFACE_GET_OK:
            n += sprintf(&buff[n], !StartFlag ? ",\"%X\":%.9g" : "\"%X\":%.9g", i, Value);
            StartFlag = false;
            if (n >= INTERFACE_BULK_TRANSFER_SIZE - 32)
            {
                buff[n] = '\0';
                uart_puts(uart_internal, buff);
                n = 0;

                #ifdef INTERFACE_DEBUG
                printf(buff);
                #endif
            }
            break;
        case INTERFACE_GET_NOVAL: break;
        case INTERFACE_GET_EOF: EndFlag = true; break;
        default: return false;
        }

        if (EndFlag) break;
    }

    buff[n++] = '\n';
    buff[n] = '\0';
    uart_puts(uart_internal, buff);
    #ifdef INTERFACE_DEBUG
    printf(buff);
    #endif

    #ifdef INTERFACE_DEBUG
    printf("CMD:GET( ALL )->OK\n");
    #endif

    return true;
}

bool Interface_s::doGet(char *CmdStr)
{
    int Address = 0;

    if (sscanf(CmdStr, "( %X )", &Address) != 1)
    {
        #ifdef INTERFACE_DEBUG
        printf("CMD:GET->PARSE_FAIL\n");
        #endif
        return false;
    }

    auto [Status, Value] = getCallback(Address);

    if (Status != INTERFACE_GET_OK)
    {
        #ifdef INTERFACE_DEBUG
        printf("CMD:GET( %X )->GET_FAIL\n");
        #endif
        return false;
    }

    char buff[64];
    sprintf(buff, "\"%X\":%.9g\n", Address, Value);
    uart_puts(uart_internal, buff);

    #ifdef INTERFACE_DEBUG
    printf("CMD:GET( %X )->OK(%g)\n", Address, Value);
    #endif

    return true;
}

bool Interface_s::doSet(char *CmdStr)
{
    int Address = 0;
    float Value = 0.0f;

    if (sscanf(CmdStr, "( %X : %g )", &Address, &Value) != 2)
    {
        #ifdef INTERFACE_DEBUG
        printf("CMD:SET->PARSE_FAIL\n");
        #endif
        return false;
    }
        

    if (setCallback(Address, Value) != INTERFACE_SET_OK)
    {
        #ifdef INTERFACE_DEBUG
        printf("CMD:SET( %X : %g )->SET_FAIL\n", Address, Value);
        #endif
        return false;
    }

    #ifdef INTERFACE_DEBUG
    printf("CMD:SET( %X : %g )->OK\n", Address, Value);
    #endif

    return true;
}

bool Interface_s::Parse()
{
    if (strncmp(ReceiveBuffer, "CMD:", sizeof("CMD:")-1) == 0)
        return parseCommand(ReceiveBuffer + sizeof("CMD:") - 1);
    else if (strncmp(ReceiveBuffer, "DBG:", sizeof("DBG:") - 1) == 0)
        return printf("%s", ReceiveBuffer) > 0 ? true : false;
    
    #ifdef INTERFACE_DEBUG
    printf("NDF:%s", ReceiveBuffer);
    #endif

    return false;
}
