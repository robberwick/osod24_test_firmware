cmake_minimum_required(VERSION 3.26)
set(PIMORONI_PICO_PATH $ENV{PIMORONI_PICO_PATH})

include(pimoroni_pico_import.cmake)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(osod_motor_2040 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(osod_motor_2040
        main.cpp
        tank_driver_mixer.cpp
)
include(libraries/motor2040/motor2040)

add_subdirectory(include/pico-cppm)
target_include_directories(${PROJECT_NAME} PRIVATE include/pico-cppm)
target_link_libraries(${PROJECT_NAME}
        pico_cppm
        motor2040
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create UF2 file from the binary
pico_add_extra_outputs(${PROJECT_NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)