cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD "sduino")

include(pico_sdk_import.cmake)

project(sduino C CXX ASM)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

set(SDUINO_PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(robot1)
