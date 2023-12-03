add_library(receiver STATIC
        src/receiver.cpp
        src/receiverSBUS.cpp
        src/receiverCPPM.cpp
)

set(RX_PROTOCOL "CPPM" CACHE STRING "Set the RX protocol")

if (${RX_PROTOCOL} STREQUAL "SBUS")
    message(STATUS "RX_PROTOCOL: SBUS")
    add_compile_definitions(-DRX_PROTOCOL_SBUS)
else ()
    message(STATUS "RX_PROTOCOL: CPPM")
    add_compile_definitions(-DRX_PROTOCOL_CPPM)
endif ()

target_include_directories(receiver PUBLIC
        PRIVATE include/sbus_2040
)

    include(FetchContent)

    FetchContent_Declare(
            pico_cppm
            GIT_REPOSITORY "https://github.com/cadouthat/pico-cppm"
            GIT_TAG "main"
    )

    FetchContent_GetProperties(pico-cppm)

    if (NOT pico-cppm_POPULATED)
        FetchContent_Populate(pico_cppm)
        message(STATUS "pico_cppm source dir: ${pico_cppm_SOURCE_DIR}")
        message(STATUS "pico_cppm binary dir: ${pico_cppm_BINARY_DIR}")
        add_subdirectory(
                ${pico_cppm_SOURCE_DIR}
                ${CMAKE_CURRENT_BINARY_DIR}/pico_cppm-bin
        )
    endif ()

target_link_libraries(receiver PUBLIC
        motor2040
        sbus_2040
        pico_cppm
)

target_include_directories(receiver PUBLIC include)