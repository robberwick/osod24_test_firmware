cmake_minimum_required(VERSION 3.26)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(osod_motor_2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(osod_motor_2040 main.cpp)

add_subdirectory(include/pico-cppm)
target_include_directories(${PROJECT_NAME} PRIVATE include/pico-cppm)
target_link_libraries(${PROJECT_NAME} pico_cppm)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create UF2 file from the binary
pico_add_extra_outputs(${PROJECT_NAME})

