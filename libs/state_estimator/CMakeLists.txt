add_library(state_estimator STATIC
        src/state_estimator.cpp
)

target_link_libraries(state_estimator PUBLIC
        config
        motor2040
        encoder
)

target_include_directories(state_estimator PUBLIC
        include
)