add_library(usb_serial STATIC
        src/USB_Serial.c
)

target_link_libraries(
        usb_serial
        pico_stdlib
        pico_stdio_usb
)

target_include_directories(usb_serial PUBLIC
        include
)
