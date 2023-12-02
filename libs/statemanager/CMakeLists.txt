add_library(statemanager STATIC
        src/statemanager.cpp
)

target_link_libraries(statemanager PUBLIC
        receiver
)

target_include_directories(statemanager PUBLIC
        include
        receiver
)