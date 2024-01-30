add_library(serialtransfer STATIC
        src/Packet.cpp
        src/SerialTransfer.cpp
)

target_link_libraries(serialtransfer
        usb_serial
)

target_include_directories(serialtransfer PUBLIC
        include
        usb_serial
)