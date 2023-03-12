if (TARGET tinyusb_device)
    include_directories("${SDUINO_PROJECT_PATH}/robot2/Include")
    include_directories("${SDUINO_PROJECT_PATH}/mini_sumo/Include")
    include_directories("${SDUINO_PROJECT_PATH}/motors/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/sensorx/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/port/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/I2C_MUX/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/VL53L0X/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/BMI08x/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/hardware/MMC5603/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/imu/SensorFusion/Include")
    include_directories("${SDUINO_PROJECT_PATH}/sensors/imu/Include")
    include_directories("${SDUINO_PROJECT_PATH}/pid/Include")
    include_directories("${SDUINO_PROJECT_PATH}/nvm/Include")
    include_directories("${SDUINO_PROJECT_PATH}/interface/Include")

    add_executable(mini_sumo)

    target_compile_definitions(mini_sumo PUBLIC -DPORT_PCA9548A)  # -D removed
    #target_compile_definitions(line_follower PUBLIC -DIMU_USE_MAG)
    target_compile_definitions(mini_sumo PUBLIC -DINTERFACE_DEBUG)

    target_sources(mini_sumo PRIVATE
        Source/main.cpp
        ../nvm/Source/nvm.cpp
        ../mini_sumo/Source/mini_sumo.cpp
        ../interface/Source/interface.cpp
        ../motors/Source/motor_driver.cpp
        ../sensors/hardware/I2C_MUX/Source/PCA9548A.c
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

    target_link_libraries(mini_sumo pico_stdlib hardware_flash hardware_sync hardware_pio hardware_dma hardware_i2c hardware_spi hardware_pwm pico_bootsel_via_double_reset)

    target_compile_definitions(mini_sumo PRIVATE
	    PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
	)

    # enable usb output
    pico_enable_stdio_usb(mini_sumo 1)
    pico_enable_stdio_uart(mini_sumo 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(mini_sumo)

    # add url via pico_set_program_url
    example_auto_set_url(mini_sumo)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building sduino_self_test because TinyUSB submodule is not initialized in the SDK")
endif()
