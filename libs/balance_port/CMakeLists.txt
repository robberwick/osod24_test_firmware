add_library(balance_port STATIC
        src/ads1x15.cpp
        src/balance_port.cpp
)


include(FetchContent)

    FetchContent_Declare(
            pico_ads1x15
            GIT_REPOSITORY "https://github.com/gavinlyonsrepo/ADS1x15_PICO"
            GIT_TAG "main"
    )

    FetchContent_GetProperties(pico_ads1x15)

    if (NOT pico_ads1x15_POPULATED)
        FetchContent_Populate(pico_ads1x15)
        message(STATUS "pico_ads1x15 source dir: ${pico_ads1x15_SOURCE_DIR}")
        message(STATUS "pico_ads1x15 binary dir: ${pico_ads1x15_BINARY_DIR}")
        add_subdirectory(
                ${pico_ads1x15_SOURCE_DIR}
                ${CMAKE_CURRENT_BINARY_DIR}/pico_ads1x15-bin
        )
    endif ()
 
target_link_libraries(balance_port PUBLIC
    pico_stdlib 
    hardware_i2c
    communicator
    config
    pico_ads1x15
)

target_include_directories(balance_port PUBLIC
        include
)