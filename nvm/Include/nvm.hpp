
#ifndef INC_NVM_HPP_
#define INC_NVM_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "string.h"

class NVM_s {
    size_t Size;
    uint32_t AddressOffset;
    uint8_t *StartAddress;

    void* UserData;
    uint32_t UserDataSize;

    bool Initialised = false;

public:
    void init(size_t _Size, void* _UserData, size_t _UserDataSize);

    void erase(void);

    void load(void);

    void program(void);
};

#endif
