
#include "nvm.hpp"

void NVM_s::init(size_t _Size, void* _UserData, size_t _UserDataSize)
{
    Size = _Size;
    UserData = _UserData;
    UserDataSize = _UserDataSize;
    AddressOffset = (PICO_FLASH_SIZE_BYTES - Size);
    StartAddress = (uint8_t *) (XIP_BASE + AddressOffset);
    Initialised = true;
}

void NVM_s::erase(void)
{
    if (!Initialised) return;
    multicore_lockout_start_blocking();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(AddressOffset, Size);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
}

void NVM_s::load(void)
{
    if (!Initialised) return;
    memcpy(( uint8_t *)UserData , StartAddress, UserDataSize);
}

void NVM_s::program(void)
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

