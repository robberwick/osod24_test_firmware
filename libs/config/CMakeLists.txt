add_library(config STATIC
        src/waypoint_routes.cpp
)

target_link_libraries(config
        motor2040
        common
)

target_include_directories(config PUBLIC
        include
)