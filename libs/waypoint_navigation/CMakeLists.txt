add_library(waypoint_navigation STATIC
        src/waypoint_navigation.cpp
)

target_link_libraries(waypoint_navigation PUBLIC
        common
        config
        motor
        pid
)

target_include_directories(waypoint_navigation PUBLIC
        include
)