#ifndef BMS_UART_H
#define BMS_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32l0xx_hal.h"
#include "bms_uart_channel.h"

#define BMS_UART_SOF0                         0xAAU
#define BMS_UART_SOF1                         0x55U
#define BMS_UART_PROTOCOL_VERSION             0x04U
#define BMS_UART_MAX_PAYLOAD_SIZE             96U

/*
 * Frame:
 *   SOF0 SOF1 CMD LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
 *
 * CRC16 is Modbus/IBM, init 0xFFFF, calculated over CMD, LEN and PAYLOAD.
 * Response CMD is request CMD OR 0x80. Response payload always starts with one
 * status byte followed by command data.
 */

typedef enum {
    BMS_UART_CMD_PING = 0x01U,
    BMS_UART_CMD_READ_SUMMARY = 0x10U,
    BMS_UART_CMD_READ_CELLS = 0x11U,
    BMS_UART_CMD_READ_FAULTS = 0x12U,
    BMS_UART_CMD_READ_LIMITS = 0x13U,
    BMS_UART_CMD_OTP_CHECK = 0x20U,
    BMS_UART_CMD_OTP_WRITE = 0x21U,
    BMS_UART_CMD_OTP_READ = 0x22U,
    BMS_UART_CMD_CALIBRATE_CURRENT = 0x30U
} bms_uart_command_t;

typedef enum {
    BMS_UART_STATUS_OK = 0x00U,
    BMS_UART_STATUS_BAD_LENGTH = 0x01U,
    BMS_UART_STATUS_BAD_COMMAND = 0x02U,
    BMS_UART_STATUS_BUSY = 0x03U,
    BMS_UART_STATUS_INTERNAL_ERROR = 0x04U,
    BMS_UART_STATUS_BAD_PAYLOAD = 0x05U
} bms_uart_status_t;

void bms_uart_init(UART_HandleTypeDef *uart);
void bms_uart_task(void);
bool bms_uart_is_enabled(void);

#endif
