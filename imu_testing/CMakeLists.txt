add_executable(imu_testing)

include_directories("${SDUINO_PROJECT_PATH}/imu_testing/Include")
include_directories("${SDUINO_PROJECT_PATH}/motion")
include_directories("${SDUINO_PROJECT_PATH}/sensors")
include_directories("${SDUINO_PROJECT_PATH}/VL53L0X/Include")
include_directories("${SDUINO_PROJECT_PATH}/BMI08x/Include")
include_directories("${SDUINO_PROJECT_PATH}/math")
include_directories("${SDUINO_PROJECT_PATH}/interface")
include_directories("${SDUINO_PROJECT_PATH}/hardware")

#target_compile_definitions(line_follower PUBLIC -DIMU_USE_MAG)
target_compile_definitions(imu_testing PUBLIC -DINTERFACE_DEBUG)


pico_generate_pio_header(imu_testing ${SDUINO_PROJECT_PATH}/hardware/pio/encoder.pio)
pico_generate_pio_header(imu_testing ${SDUINO_PROJECT_PATH}/hardware/pio/spi_adc.pio)

target_sources(imu_testing PRIVATE
    Source/main.cpp
    Source/interface.cpp
    ../BMI08x/Source/bmi08a.c
    ../BMI08x/Source/bmi08g.c
    ../VL53L0X/Source/vl53l0x_platform.c
    ../VL53L0X/Source/vl53l0x_platform_log.c
    ../VL53L0X/Source/vl53l0x_api.c
    ../VL53L0X/Source/vl53l0x_api_strings.c
    ../VL53L0X/Source/vl53l0x_api_ranging.c
    ../VL53L0X/Source/vl53l0x_api_core.c
    ../VL53L0X/Source/vl53l0x_api_calibration.c
    ../hardware/sensorx_vl53l0x.c
)

target_link_libraries(imu_testing 
                      pico_stdlib 
                      pico_multicore 
                      hardware_flash 
                      hardware_sync 
                      hardware_pio 
                      hardware_dma 
                      hardware_i2c 
                      hardware_spi 
                      hardware_pwm 
                      hardware_adc 
                      pico_bootsel_via_double_reset)

target_compile_definitions(imu_testing PRIVATE
    PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=0
)

# enable usb output
pico_enable_stdio_usb(imu_testing 1)
pico_enable_stdio_uart(imu_testing 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(imu_testing)
