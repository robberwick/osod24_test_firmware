add_library(stoker STATIC
        src/stoker.cpp
)

target_link_libraries(stoker PUBLIC
        motor
)

target_include_directories(stoker PUBLIC
        include
)