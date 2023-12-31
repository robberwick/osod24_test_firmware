add_library(statemanager STATIC
        src/statemanager.cpp
)

target_link_libraries(statemanager PUBLIC
        receiver
        stoker
        mixer
        servo
)

target_include_directories(statemanager PUBLIC
        include
        servo
        receiver
        stoker
        mixer
)