add_library(communicator STATIC
        src/communicator.cpp
        include/payloads.h
)
target_link_libraries(communicator PUBLIC
        serialtransfer
        common
)
target_include_directories(communicator PUBLIC
        include
)