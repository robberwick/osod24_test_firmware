add_library(balance_port STATIC
        src/ads1x15.cpp
)
target_link_libraries(balance_port
        pico_stdlib 
        hardware_i2c
)
target_include_directories(balance_port PUBLIC
        include
)