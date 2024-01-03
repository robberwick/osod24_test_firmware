add_library(common STATIC
        include/interfaces.h
        src/utils.cpp
)

target_link_libraries(common
        pico_stdlib 
        hardware_i2c
        config
)

target_include_directories(common PUBLIC
        include
)