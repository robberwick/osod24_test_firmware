add_library(bno080 STATIC
        src/bno080.cpp
)
target_link_libraries(i2cbus INTERFACE
        pico_stdlib 
        hardware_i2c
)
target_include_directories(bno080 PUBLIC
        include
)