
#ifndef INC_NVM_HPP_
#define INC_NVM_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "string.h"

#define NVM_OK PICO_OK
#define NVM_ERROR PICO_ERROR_GENERIC

class NVM_s
{
    size_t Size;
    uint32_t AddressOffset;
    uint8_t *StartAddress;

    void* UserData;
    uint32_t UserDataSize;

    bool Initialised = false;

public:
    void init(size_t _Size, void* _UserData, size_t _UserDataSize)
    {
        Size = _Size;
        UserData = _UserData;
        UserDataSize = _UserDataSize;
        AddressOffset = (PICO_FLASH_SIZE_BYTES - Size);
        StartAddress = (uint8_t *) (XIP_BASE + AddressOffset);
        Initialised = true;
    }

    void erase(void)
    {
        if (!Initialised) return;
        multicore_lockout_start_blocking();
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(AddressOffset, Size);
        restore_interrupts(ints);
        multicore_lockout_end_blocking();
    }

    void load(void)
    {
        if (!Initialised) return;
        memcpy(( uint8_t *)UserData , StartAddress, UserDataSize);
    }

    void program(void)
    {
        if (!Initialised) return;
        multicore_lockout_start_blocking();
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(AddressOffset, Size);
        flash_range_program(AddressOffset, ( uint8_t *)UserData, UserDataSize);
        restore_interrupts(ints);
        multicore_lockout_end_blocking();
        load();
    }
};

#endif
