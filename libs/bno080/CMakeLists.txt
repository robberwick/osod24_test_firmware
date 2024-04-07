add_library(bno080 STATIC
        src/bno080.cpp
        src/sh2.c
        src/shtp.c
        src/sh2_util.c
        src/sh2_SensorValue.c
)
target_link_libraries(bno080
        common
        pico_stdlib 
        hardware_i2c
)
target_include_directories(bno080 PUBLIC
        include
)