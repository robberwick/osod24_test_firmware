add_library(config INTERFACE)

target_link_libraries(config INTERFACE
        motor2040
)

target_include_directories(config INTERFACE
        include
)