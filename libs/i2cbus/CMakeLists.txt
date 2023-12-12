add_library(i2cbus STATIC
        src/i2cbus.cpp
)
target_link_libraries(i2cbus INTERFACE
        pico_stdlib 
        hardware_i2c
)
target_include_directories(i2cbus PUBLIC
        include
)