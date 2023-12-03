add_library(mixer STATIC
        src/tank_steer_strategy.cpp
)

target_link_libraries(mixer PUBLIC
        motor
)

target_include_directories(mixer PUBLIC
        include
)