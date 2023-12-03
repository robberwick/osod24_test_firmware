add_library(sbus_2040 STATIC
        src/sbus_2040.c
)
target_link_libraries(sbus_2040 PUBLIC
        receiver
)
target_include_directories(sbus_2040 PUBLIC
        include
)