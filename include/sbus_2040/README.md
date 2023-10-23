# SBUS 2040
Futaba SBUS protocol decoder for Raspberry Pi Pico.

# Description
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 0: channel 17 (0x01)
      * Bit 1: channel 18 (0x02)
      * Bit 2: frame lost (0x04)
      * Bit 3: failsafe activated (0x08)
   * Byte[24]: SBUS footer

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that many frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms, depending on the system configuration.

**Note on CH17 and CH18:** Channel 17 and channel 18 are digital on/off channels. These are not universally available on all SBUS receivers and servos.

No additional hardware is required to perform the serial inversion, as this library configures the UART hardware to do this automatically.

# Credits
Built on a fork of https://github.com/mmosca/pico-sbus, SBUS description in this README is from https://github.com/bolderflight/sbus
