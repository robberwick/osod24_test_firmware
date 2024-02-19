add_library(tf_luna STATIC
        src/tf_luna.cpp
)
target_link_libraries(tf_luna
        pico_stdlib 
        hardware_i2c
)
target_include_directories(tf_luna PUBLIC
        include
)