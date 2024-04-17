add_library(mixer STATIC
        src/tank_steer_strategy.cpp
        src/ackermann_strategy.cpp
        src/mecanum_strategy.cpp
)

target_link_libraries(mixer PUBLIC
        motor
        config
        common
)

target_include_directories(mixer PUBLIC
        include
)