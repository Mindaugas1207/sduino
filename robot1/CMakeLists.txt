add_executable(line_follower)

include_directories("${SDUINO_PROJECT_PATH}/eigen/Eigen")
include_directories("${SDUINO_PROJECT_PATH}/eigen/Eigen/src")
include_directories("${SDUINO_PROJECT_PATH}/robot1/Include")
include_directories("${SDUINO_PROJECT_PATH}/line_follower/Include")
include_directories("${SDUINO_PROJECT_PATH}/motors/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/sensorx/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/port/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/line_sensor/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/LED_DRIVER/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/padc/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/encoder/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/I2C_MUX/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/VL53L0X/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/BMI08x/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/MMC5603/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/imu/SensorFusion/Include")
include_directories("${SDUINO_PROJECT_PATH}/sensors/imu/Include")
include_directories("${SDUINO_PROJECT_PATH}/pid/Include")
include_directories("${SDUINO_PROJECT_PATH}/nvm/Include")
include_directories("${SDUINO_PROJECT_PATH}/interface/Include")

target_compile_definitions(line_follower PUBLIC -DMOTORDEF0)
target_compile_definitions(line_follower PUBLIC -DPORT_PI4MSD5V9540B)  # -D removed
#target_compile_definitions(line_follower PUBLIC -DIMU_USE_MAG)
target_compile_definitions(line_follower PUBLIC -DINTERFACE_DEBUG)

pico_generate_pio_header(line_follower ${SDUINO_PROJECT_PATH}/sensors/hardware/padc/Source/padc.pio)
pico_generate_pio_header(line_follower ${SDUINO_PROJECT_PATH}/sensors/hardware/encoder/Source/encoder.pio)

target_sources(line_follower PRIVATE
    Source/main.cpp
    ../nvm/Source/nvm.cpp
    ../interface/Source/interface.cpp
    ../line_follower/Source/line_follower.cpp
    ../motors/Source/motor_driver.cpp
    ../sensors/line_sensor/Source/line_sensor.cpp
    ../sensors/hardware/padc/Source/padc.c
    ../sensors/hardware/encoder/Source/encoder.cpp
    ../sensors/hardware/LED_DRIVER/Source/IS31FL3218.c
    ../sensors/hardware/I2C_MUX/Source/PI4MSD5V9540B.c
    ../sensors/port/Source/port.c
    ../sensors/hardware/BMI08x/Source/bmi08_spi.c
    ../sensors/hardware/BMI08x/Source/bmi08a.c
    ../sensors/hardware/BMI08x/Source/bmi08g.c
    ../sensors/hardware/MMC5603/Source/MMC5603.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_platform.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_platform_log.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_api.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_api_strings.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_api_ranging.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_api_core.c
    ../sensors/hardware/VL53L0X/Source/vl53l0x_api_calibration.c
    ../sensors/sensorx/Source/sensorx.c
    ../sensors/sensorx/Source/sensorx_vl53l0x.c
    ../sensors/imu/SensorFusion/Source/SensorFusion.cpp
    ../sensors/imu/Source/imu.cpp
)

target_link_libraries(line_follower 
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

target_compile_definitions(line_follower PRIVATE
    PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=0
)

# enable usb output
pico_enable_stdio_usb(line_follower 1)
pico_enable_stdio_uart(line_follower 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(line_follower)
