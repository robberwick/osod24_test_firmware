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
        receiver_utils.cpp
)

add_subdirectory(${PIMORONI_PICO_PATH} ${CMAKE_BINARY_DIR}/pimoroni-pico-build)

set(RX_PROTOCOL "CPPM" CACHE STRING "Set the RX protocol")  # "CPPM" is the default

if (RX_PROTOCOL STREQUAL "SBUS")
    message(STATUS "RX_PROTOCOL: SBUS")
    add_definitions(-DRX_PROTOCOL_SBUS)
    add_subdirectory(include/sbus_2040)
    target_sources(${PROJECT_NAME}
            PRIVATE
            receiver_sbus.cpp
    )
    target_include_directories(${PROJECT_NAME}
            PRIVATE include/sbus_2040
    )
    target_link_libraries(${PROJECT_NAME}
            sbus_2040)
else ()
    message(STATUS "RX_PROTOCOL: CPPM")
    add_definitions(-DRX_PROTOCOL_CPPM)
    add_subdirectory(include/pico-cppm)
    target_sources(${PROJECT_NAME}
            PRIVATE
            receiver_cppm.cpp
    )
    target_include_directories(${PROJECT_NAME}
            PRIVATE include/pico-cppm
    )
    target_link_libraries(${PROJECT_NAME}
            pico_cppm)
endif ()

#set the output name to have a suffix that includes the RX_PROTOCOL and a "_DEBUG suffix if debug build, and "_RELEASE" if release build
set(RELEASE_TYPE_SUFFIX ${CMAKE_BUILD_TYPE})
string(TOUPPER ${RELEASE_TYPE_SUFFIX} RELEASE_TYPE_SUFFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}_${RX_PROTOCOL}_${RELEASE_TYPE_SUFFIX}")
message(STATUS "Release type suffix: ${RELEASE_TYPE_SUFFIX}")
target_link_libraries(${PROJECT_NAME}
        motor2040
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_DEFAULT_UART=1
        PICO_DEFAULT_UART_TX_PIN=20
        PICO_DEFAULT_UART_RX_PIN=21
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