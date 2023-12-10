add_library(navigator STATIC
        src/navigator.cpp
)
target_link_libraries(navigator PUBLIC
        pico_cppm
        receiver
        statemanager
        config
)
target_include_directories(navigator PUBLIC
        include
)