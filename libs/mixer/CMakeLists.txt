add_library(mixer STATIC
        src/tank_steer_strategy.cpp
        src/ackermann_mixer.cpp
)

target_link_libraries(mixer PUBLIC
        motor
        config
)

target_include_directories(mixer PUBLIC
        include
)