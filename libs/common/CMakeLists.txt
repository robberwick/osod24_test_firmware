add_library(common STATIC
        include/interfaces.h
        src/utils.cpp
)

target_include_directories(common PUBLIC
        include
)