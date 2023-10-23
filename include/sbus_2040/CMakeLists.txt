cmake_minimum_required(VERSION 3.26)

include(pico_sdk_import.cmake)

project(sbus_2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_library(sbus_2040
        src/sbus.c
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)


target_link_libraries(sbus_2040
        pico_stdlib
        hardware_uart
        hardware_irq
        pico_sync
)
