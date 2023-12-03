add_library(mixer STATIC
        src/tank_steer_strategy.cpp
        src/tank_steer_strategy_with_units.cpp
)

target_link_libraries(mixer PUBLIC
        motor
)

target_include_directories(mixer PUBLIC
        include
)