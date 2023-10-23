#ifdef __cplusplus
extern "C" {
#endif

#ifndef __SBUS_H__
#define __SBUS_H__

#include <stdint.h>
#include <stdbool.h>

#include "hardware/uart.h"

#define SBUS_UART_ID uart0


#define SBUS_BAUD_RATE 100000
#define SBUS_DATA_BITS 8
#define SBUS_PARITY    UART_PARITY_EVEN
#define SBUS_STOP_BITS 2

#define SBUS_STARTBYTE	0x0F
#define SBUS_ENDBYTE	0X00
#define SBUS_MESSAGE_MAX_SIZE 25
#define SBUS_CHANNEL_BIT_MASK   0x7FF
#define SBUS_CHANNEL_BIT_MASK_NO_FIRST_BIT (SBUS_CHANNEL_BITS & ~1)

#define SBUS_CHANNEL_COUNT 18

#define SBUS_FIFO_SIZE	2

typedef struct {
    uint16_t ch[SBUS_CHANNEL_COUNT];
    bool framelost;
    bool failsafe;
} sbus_state_t;

bool hasSbusData();

bool readSbusData(uint8_t *data);

void sbus_on_uart_rx();

void decode_sbus_data(const uint8_t *data, sbus_state_t *decoded);

void sbus_init(uart_inst_t *uart, int rx_pin, int tx_pin);

#endif

#ifdef __cplusplus
}
#endif