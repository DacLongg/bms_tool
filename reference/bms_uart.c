#include "bms_uart.h"

#include <stddef.h>
#include <string.h>

#include "bms.h"
#include "power_manager.h"

#if BMS_UART_PROTOCOL_ENABLE

#define BMS_UART_RESPONSE_FLAG                 0x80U
#define BMS_UART_TX_TIMEOUT_MS                 50U
#define BMS_UART_RX_BUFFER_SIZE                192U
#define BMS_UART_TX_BUFFER_SIZE                (BMS_UART_MAX_PAYLOAD_SIZE + 6U)
#define BMS_UART_OTP_WRITE_MAGIC0              0x4FU
#define BMS_UART_OTP_WRITE_MAGIC1              0x54U
#define BMS_UART_OTP_WRITE_MAGIC2              0x50U
#define BMS_UART_OTP_WRITE_MAGIC3              0x21U

typedef enum {
    BMS_UART_PARSE_SOF0 = 0,
    BMS_UART_PARSE_SOF1,
    BMS_UART_PARSE_COMMAND,
    BMS_UART_PARSE_LENGTH,
    BMS_UART_PARSE_PAYLOAD,
    BMS_UART_PARSE_CRC_LO,
    BMS_UART_PARSE_CRC_HI
} bms_uart_parse_state_t;

typedef struct {
    bms_uart_parse_state_t state;
    uint8_t command;
    uint8_t length;
    uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    uint8_t payload_pos;
    uint8_t crc_lo;
} bms_uart_parser_t;

static UART_HandleTypeDef *g_bms_uart;
static uint8_t g_rx_byte;
static uint8_t g_rx_buffer[BMS_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_overflow_count;
static bms_uart_parser_t g_parser;

static void bms_uart_start_rx(void);
static void bms_uart_push_rx_byte(uint8_t byte);
static bool bms_uart_pop_rx_byte(uint8_t *byte);
static void bms_uart_parse_byte(uint8_t byte);
static void bms_uart_reset_parser(void);
static void bms_uart_handle_frame(uint8_t command,
                                  const uint8_t *payload,
                                  uint8_t length);
static void bms_uart_send_status(uint8_t command, bms_uart_status_t status);
static void bms_uart_send_response(uint8_t command,
                                   bms_uart_status_t status,
                                   const uint8_t *payload,
                                   uint8_t length);
static void bms_uart_send_frame(uint8_t command,
                                const uint8_t *payload,
                                uint8_t length);
static void bms_uart_handle_ping(uint8_t command,
                                 const uint8_t *payload,
                                 uint8_t length);
static void bms_uart_handle_read_summary(uint8_t command, uint8_t length);
static void bms_uart_handle_read_cells(uint8_t command, uint8_t length);
static void bms_uart_handle_read_faults(uint8_t command, uint8_t length);
static void bms_uart_handle_read_limits(uint8_t command, uint8_t length);
static void bms_uart_handle_otp_check(uint8_t command, uint8_t length);
static void bms_uart_handle_otp_write(uint8_t command,
                                      const uint8_t *payload,
                                      uint8_t length);
static void bms_uart_handle_otp_read(uint8_t command, uint8_t length);
static void bms_uart_handle_calibrate_current(uint8_t command,
                                              const uint8_t *payload,
                                              uint8_t length);
static void bms_uart_send_otp_status(uint8_t command,
                                     const bq76952_otp_status_t *status);
static void bms_uart_send_current_calibration_result(uint8_t command,
                                                     bms_uart_status_t status,
                                                     const BMS_CurrentCalibrationResult_t *result);
static uint16_t bms_uart_fault_bitmap(const BMS_Tracking_t *tracking);
static uint8_t bms_uart_fet_bitmap(const BMS_Tracking_t *tracking);
static uint8_t bms_uart_gate_signal_bitmap(const BMS_Tracking_t *tracking);
static uint16_t bms_uart_otp_flags(const bq76952_otp_status_t *status);
static uint16_t bms_uart_crc16(const uint8_t *data, uint16_t length);
static int32_t bms_uart_get_i32(const uint8_t *payload);
static bool bms_uart_put_u8(uint8_t *payload, uint8_t *length, uint8_t value);
static bool bms_uart_put_u16(uint8_t *payload, uint8_t *length, uint16_t value);
static bool bms_uart_put_i16(uint8_t *payload, uint8_t *length, int16_t value);
static bool bms_uart_put_u32(uint8_t *payload, uint8_t *length, uint32_t value);
static bool bms_uart_put_i32(uint8_t *payload, uint8_t *length, int32_t value);

void bms_uart_init(UART_HandleTypeDef *uart)
{
    g_bms_uart = uart;
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_overflow_count = 0UL;
    bms_uart_reset_parser();
    bms_uart_start_rx();
}

void bms_uart_task(void)
{
    uint8_t byte;

    while (bms_uart_pop_rx_byte(&byte)) {
        bms_uart_parse_byte(byte);
    }
}

void bms_uart_restart_rx(void)
{
    bms_uart_start_rx();
}

void bms_uart_send_protection_reason(uint8_t reason)
{
    bms_uart_send_frame(BMS_UART_CMD_PROTECTION_EVENT, &reason, 1U);
}

bool bms_uart_is_enabled(void)
{
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((g_bms_uart == NULL) || (huart != g_bms_uart)) {
        return;
    }

    bms_uart_push_rx_byte(g_rx_byte);
    power_manager_notify_uart_wakeup();
    bms_uart_start_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((g_bms_uart == NULL) || (huart != g_bms_uart)) {
        return;
    }

    power_manager_notify_uart_wakeup();
    bms_uart_start_rx();
}

static void bms_uart_start_rx(void)
{
    if (g_bms_uart == NULL) {
        return;
    }
    (void)HAL_UART_Receive_IT(g_bms_uart, &g_rx_byte, 1U);
}

static void bms_uart_push_rx_byte(uint8_t byte)
{
    uint16_t next_head = (uint16_t)((g_rx_head + 1U) % BMS_UART_RX_BUFFER_SIZE);

    if (next_head == g_rx_tail) {
        g_rx_overflow_count++;
        return;
    }

    g_rx_buffer[g_rx_head] = byte;
    g_rx_head = next_head;
}

static bool bms_uart_pop_rx_byte(uint8_t *byte)
{
    if ((byte == NULL) || (g_rx_tail == g_rx_head)) {
        return false;
    }

    *byte = g_rx_buffer[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % BMS_UART_RX_BUFFER_SIZE);
    return true;
}

static void bms_uart_parse_byte(uint8_t byte)
{
    switch (g_parser.state) {
    case BMS_UART_PARSE_SOF0:
        if (byte == BMS_UART_SOF0) {
            g_parser.state = BMS_UART_PARSE_SOF1;
        }
        break;

    case BMS_UART_PARSE_SOF1:
        if (byte == BMS_UART_SOF1) {
            g_parser.state = BMS_UART_PARSE_COMMAND;
        } else if (byte != BMS_UART_SOF0) {
            g_parser.state = BMS_UART_PARSE_SOF0;
        }
        break;

    case BMS_UART_PARSE_COMMAND:
        g_parser.command = byte;
        g_parser.state = BMS_UART_PARSE_LENGTH;
        break;

    case BMS_UART_PARSE_LENGTH:
        g_parser.length = byte;
        g_parser.payload_pos = 0U;
        if (g_parser.length > BMS_UART_MAX_PAYLOAD_SIZE) {
            bms_uart_reset_parser();
        } else if (g_parser.length == 0U) {
            g_parser.state = BMS_UART_PARSE_CRC_LO;
        } else {
            g_parser.state = BMS_UART_PARSE_PAYLOAD;
        }
        break;

    case BMS_UART_PARSE_PAYLOAD:
        g_parser.payload[g_parser.payload_pos++] = byte;
        if (g_parser.payload_pos >= g_parser.length) {
            g_parser.state = BMS_UART_PARSE_CRC_LO;
        }
        break;

    case BMS_UART_PARSE_CRC_LO:
        g_parser.crc_lo = byte;
        g_parser.state = BMS_UART_PARSE_CRC_HI;
        break;

    case BMS_UART_PARSE_CRC_HI: {
        uint8_t crc_input[BMS_UART_MAX_PAYLOAD_SIZE + 2U];
        uint16_t received_crc;
        uint16_t calculated_crc;

        crc_input[0] = g_parser.command;
        crc_input[1] = g_parser.length;
        if (g_parser.length > 0U) {
            memcpy(&crc_input[2], g_parser.payload, g_parser.length);
        }

        received_crc = (uint16_t)g_parser.crc_lo | ((uint16_t)byte << 8);
        calculated_crc = bms_uart_crc16(crc_input, (uint16_t)(g_parser.length + 2U));
        if (received_crc == calculated_crc) {
            bms_uart_handle_frame(g_parser.command,
                                  g_parser.payload,
                                  g_parser.length);
        }
        bms_uart_reset_parser();
        break;
    }

    default:
        bms_uart_reset_parser();
        break;
    }
}

static void bms_uart_reset_parser(void)
{
    g_parser.state = BMS_UART_PARSE_SOF0;
    g_parser.command = 0U;
    g_parser.length = 0U;
    g_parser.payload_pos = 0U;
    g_parser.crc_lo = 0U;
}

static void bms_uart_handle_frame(uint8_t command,
                                  const uint8_t *payload,
                                  uint8_t length)
{
    switch (command) {
    case BMS_UART_CMD_PING:
        bms_uart_handle_ping(command, payload, length);
        break;

    case BMS_UART_CMD_READ_SUMMARY:
        bms_uart_handle_read_summary(command, length);
        break;

    case BMS_UART_CMD_READ_CELLS:
        bms_uart_handle_read_cells(command, length);
        break;

    case BMS_UART_CMD_READ_FAULTS:
        bms_uart_handle_read_faults(command, length);
        break;

    case BMS_UART_CMD_READ_LIMITS:
        bms_uart_handle_read_limits(command, length);
        break;

    case BMS_UART_CMD_OTP_CHECK:
        bms_uart_handle_otp_check(command, length);
        break;

    case BMS_UART_CMD_OTP_WRITE:
        bms_uart_handle_otp_write(command, payload, length);
        break;

    case BMS_UART_CMD_OTP_READ:
        bms_uart_handle_otp_read(command, length);
        break;

    case BMS_UART_CMD_CALIBRATE_CURRENT:
        bms_uart_handle_calibrate_current(command, payload, length);
        break;

    default:
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_COMMAND);
        break;
    }
}

static void bms_uart_send_status(uint8_t command, bms_uart_status_t status)
{
    bms_uart_send_response(command, status, NULL, 0U);
}

static void bms_uart_send_response(uint8_t command,
                                   bms_uart_status_t status,
                                   const uint8_t *payload,
                                   uint8_t length)
{
    uint8_t response_payload[BMS_UART_MAX_PAYLOAD_SIZE];

    if ((uint16_t)length + 1U > BMS_UART_MAX_PAYLOAD_SIZE) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    response_payload[0] = (uint8_t)status;
    if ((payload != NULL) && (length > 0U)) {
        memcpy(&response_payload[1], payload, length);
    }

    bms_uart_send_frame((uint8_t)(command | BMS_UART_RESPONSE_FLAG),
                        response_payload,
                        (uint8_t)(length + 1U));
}

static void bms_uart_send_frame(uint8_t command,
                                const uint8_t *payload,
                                uint8_t length)
{
    uint8_t tx[BMS_UART_TX_BUFFER_SIZE];
    uint16_t crc;
    uint16_t frame_length;

    if ((g_bms_uart == NULL) || (length > BMS_UART_MAX_PAYLOAD_SIZE)) {
        return;
    }

    tx[0] = BMS_UART_SOF0;
    tx[1] = BMS_UART_SOF1;
    tx[2] = command;
    tx[3] = length;
    if ((payload != NULL) && (length > 0U)) {
        memcpy(&tx[4], payload, length);
    }

    crc = bms_uart_crc16(&tx[2], (uint16_t)(length + 2U));
    tx[4U + length] = (uint8_t)(crc & 0xFFU);
    tx[5U + length] = (uint8_t)(crc >> 8);
    frame_length = (uint16_t)(length + 6U);

    (void)HAL_UART_Transmit(g_bms_uart, tx, frame_length, BMS_UART_TX_TIMEOUT_MS);
}

static void bms_uart_handle_ping(uint8_t command, const uint8_t *payload, uint8_t length)
{
    if (length > (BMS_UART_MAX_PAYLOAD_SIZE - 1U)) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    bms_uart_send_response(command, BMS_UART_STATUS_OK, payload, length);
}

static void bms_uart_handle_read_summary(uint8_t command, uint8_t length)
{
    const BMS_Tracking_t *tracking;
    // uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    // uint8_t payload_len = 0U;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    tracking = BMS_GetTracking();
    if (tracking == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }


    // (void)bms_uart_put_u8(payload, &payload_len, BMS_UART_PROTOCOL_VERSION);
    // (void)bms_uart_put_u32(payload, &payload_len, HAL_GetTick());
    // (void)bms_uart_put_u8(payload, &payload_len, tracking->initialized ? 1U : 0U);
    // (void)bms_uart_put_u8(payload, &payload_len, tracking->connected ? 1U : 0U);
    // (void)bms_uart_put_u8(payload, &payload_len, (uint8_t)tracking->state);
    // (void)bms_uart_put_u8(payload, &payload_len, (uint8_t)tracking->currentDirection);
    // (void)bms_uart_put_u16(payload, &payload_len, bms_uart_fault_bitmap(tracking));
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->stackVoltage);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->packVoltage);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->batAdcEstimatedPack_mV);
    // (void)bms_uart_put_i32(payload, &payload_len, tracking->current_mA);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->minCellVoltage);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->maxCellVoltage);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->averageCellVoltage);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->deltaCellVoltage);
    // (void)bms_uart_put_i16(payload, &payload_len, tracking->temperature[0]);
    // (void)bms_uart_put_i16(payload, &payload_len, tracking->temperature[1]);
    // (void)bms_uart_put_u32(payload, &payload_len, tracking->chargeThroughput_mAh);
    // (void)bms_uart_put_u32(payload, &payload_len, tracking->dischargeThroughput_mAh);
    // (void)bms_uart_put_u32(payload, &payload_len, tracking->equivalentCycle_milliCycles);
    // (void)bms_uart_put_u8(payload, &payload_len, bms_uart_fet_bitmap(tracking));
    // (void)bms_uart_put_u8(payload, &payload_len, tracking->balanceRequired ? 1U : 0U);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->balanceMask);
    // (void)bms_uart_put_u32(payload, &payload_len, tracking->alertCounter);
    // (void)bms_uart_put_u16(payload, &payload_len, tracking->circle_counter);

    bms_uart_send_response(command, BMS_UART_STATUS_OK, (uint8_t *)tracking, sizeof(BMS_Tracking_t));
}

static void bms_uart_handle_read_cells(uint8_t command, uint8_t length)
{
    const BMS_Tracking_t *tracking;
    uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    uint8_t payload_len = 0U;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    tracking = BMS_GetTracking();
    if (tracking == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    (void)bms_uart_put_u8(payload, &payload_len, BMS_NUMBER_OF_CELLS);
    for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
        (void)bms_uart_put_u16(payload, &payload_len, tracking->cellVoltages.cellNum[i]);
    }

    bms_uart_send_response(command, BMS_UART_STATUS_OK, payload, payload_len);
}

static void bms_uart_handle_read_faults(uint8_t command, uint8_t length)
{
    const BMS_Tracking_t *tracking;
    uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    uint8_t payload_len = 0U;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    tracking = BMS_GetTracking();
    if (tracking == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    (void)bms_uart_put_u16(payload, &payload_len, bms_uart_fault_bitmap(tracking));
    (void)bms_uart_put_u8(payload, &payload_len, bms_uart_gate_signal_bitmap(tracking));
    (void)bms_uart_put_u8(payload, &payload_len, tracking->alertActive ? 1U : 0U);
    (void)bms_uart_put_u32(payload, &payload_len, tracking->alertCounter);

    bms_uart_send_response(command, BMS_UART_STATUS_OK, payload, payload_len);
}

static void bms_uart_handle_read_limits(uint8_t command, uint8_t length)
{
    uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    uint8_t payload_len = 0U;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    (void)bms_uart_put_u8(payload, &payload_len, BMS_NUMBER_OF_CELLS);
    (void)bms_uart_put_u8(payload, &payload_len, BMS_NUMBER_OF_THERMISTORS);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_CELL_OV_CUTOFF_MV_DEV);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_CELL_OV_RECOVER_MV);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_CELL_UV_CUTOFF_MV_DEV);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_CELL_UV_RECOVER_MV);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_BALANCE_DELTA_MV);
    (void)bms_uart_put_u16(payload, &payload_len, BMS_BALANCE_MIN_CELL_MV);
    (void)bms_uart_put_i32(payload, &payload_len, BMS_OVER_CURRENT_MA);
    (void)bms_uart_put_i32(payload, &payload_len, BMS_SHORT_CIRCUIT_MA);
    (void)bms_uart_put_i16(payload, &payload_len, BMS_CHARGE_OT_CUTOFF_C);
    (void)bms_uart_put_i16(payload, &payload_len, BMS_DISCHARGE_OT_CUTOFF_C);
    (void)bms_uart_put_i16(payload, &payload_len, BMS_UNDERTEMP_CUTOFF_C);
    (void)bms_uart_put_u32(payload, &payload_len, BMS_NOMINAL_CAPACITY_MAH);

    bms_uart_send_response(command, BMS_UART_STATUS_OK, payload, payload_len);
}

static void bms_uart_handle_otp_check(uint8_t command, uint8_t length)
{
    bq76952_otp_status_t otp_status;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    (void)bq76952_checkOTPWriteReady(&otp_status);
    bms_uart_send_otp_status(command, &otp_status);
}

static void bms_uart_handle_otp_write(uint8_t command,
                                      const uint8_t *payload,
                                      uint8_t length)
{
    bq76952_otp_status_t otp_status;

    if (length != 4U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }
    if ((payload == NULL) ||
        (payload[0] != BMS_UART_OTP_WRITE_MAGIC0) ||
        (payload[1] != BMS_UART_OTP_WRITE_MAGIC1) ||
        (payload[2] != BMS_UART_OTP_WRITE_MAGIC2) ||
        (payload[3] != BMS_UART_OTP_WRITE_MAGIC3)) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_PAYLOAD);
        return;
    }

    (void)bq76952_program_OTP_with_status(&otp_status);
    bms_uart_send_otp_status(command, &otp_status);
}

static void bms_uart_handle_otp_read(uint8_t command, uint8_t length)
{
    bq76952_otp_status_t otp_status;

    if (length != 0U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }

    if (!bq76952_readOTPStatus(&otp_status)) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    bms_uart_send_otp_status(command, &otp_status);
}

static void bms_uart_handle_calibrate_current(uint8_t command,
                                              const uint8_t *payload,
                                              uint8_t length)
{
    BMS_CurrentCalibrationResult_t result;
    bms_uart_status_t status;
    int32_t actual_mA;
    // BMS_CurrentCalibStatus_t status;

    if (length != 4U) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_LENGTH);
        return;
    }
    if (payload == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_BAD_PAYLOAD);
        return;
    }

    actual_mA = bms_uart_get_i32(payload);
    status = (bms_uart_status_t)BMS_CalibrateCurrent(actual_mA, &result);

    // switch (result.status) {
    // case BMS_CURRENT_CALIBRATION_OK:
    //     status = BMS_UART_STATUS_OK;
    //     break;
    // case BMS_CURRENT_CALIBRATION_WRITE_FAILED:
    //     status = BMS_UART_STATUS_INTERNAL_ERROR;
    //     break;
    // case BMS_CURRENT_CALIBRATION_BAD_INPUT:
    // case BMS_CURRENT_CALIBRATION_ZERO_READING:
    // case BMS_CURRENT_CALIBRATION_DEVIATION_TOO_HIGH:
    // default:
    //     status = BMS_UART_STATUS_BAD_PAYLOAD;
    //     break;
    // }

    bms_uart_send_current_calibration_result(command, status, &result);
}

static void bms_uart_send_otp_status(uint8_t command,
                                     const bq76952_otp_status_t *status)
{
    uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    uint8_t payload_len = 0U;

    if (status == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    (void)bms_uart_put_u16(payload, &payload_len, bms_uart_otp_flags(status));
    (void)bms_uart_put_u8(payload, &payload_len, status->securityState);
    (void)bms_uart_put_u8(payload, &payload_len, status->checkResult);
    (void)bms_uart_put_u16(payload, &payload_len, status->checkDataFailAddr);
    (void)bms_uart_put_u8(payload, &payload_len, status->writeResult);
    (void)bms_uart_put_u16(payload, &payload_len, status->writeDataFailAddr);
    (void)bms_uart_put_u16(payload, &payload_len, status->batteryStatusRaw);
    (void)bms_uart_put_u16(payload, &payload_len, status->staticConfigSignature);
    (void)bms_uart_put_u16(payload, &payload_len, status->stackVoltage_mV);
    (void)bms_uart_put_u16(payload, &payload_len, status->packVoltage_mV);
    (void)bms_uart_put_i16(payload, &payload_len, status->internalTemp_C);
    (void)bms_uart_put_u8(payload, &payload_len, status->reg0Config);
    (void)bms_uart_put_u8(payload, &payload_len, status->reg12Control);
    (void)bms_uart_put_u8(payload, &payload_len, status->daConfig);
    (void)bms_uart_put_u16(payload, &payload_len, status->vcellMode);
    (void)bms_uart_put_u8(payload, &payload_len, status->dchgPinConfig);
    (void)bms_uart_put_u8(payload, &payload_len, status->ddsgPinConfig);
    (void)bms_uart_put_u8(payload, &payload_len, status->dfetoffPinConfig);

    bms_uart_send_response(command, BMS_UART_STATUS_OK, payload, payload_len);
}

static void bms_uart_send_current_calibration_result(uint8_t command,
                                                     bms_uart_status_t status,
                                                     const BMS_CurrentCalibrationResult_t *result)
{
    // uint8_t payload[BMS_UART_MAX_PAYLOAD_SIZE];
    // uint8_t payload_len = 0U;

    if (result == NULL) {
        bms_uart_send_status(command, BMS_UART_STATUS_INTERNAL_ERROR);
        return;
    }

    // (void)bms_uart_put_u8(payload, &payload_len, (uint8_t)result->status);
    // (void)bms_uart_put_i32(payload, &payload_len, result->actual_mA);
    // (void)bms_uart_put_i32(payload, &payload_len, result->measured_mA);
    // (void)bms_uart_put_u32(payload, &payload_len, result->deviation_ppm);
    // (void)bms_uart_put_u32(payload, &payload_len, result->oldGain_ppm);
    // (void)bms_uart_put_u32(payload, &payload_len, result->newGain_ppm);

    bms_uart_send_response(command, status, (uint8_t *)result, sizeof(BMS_CurrentCalibrationResult_t));
}

static uint16_t bms_uart_fault_bitmap(const BMS_Tracking_t *tracking)
{
    uint16_t bitmap = 0U;

    if (tracking == NULL) {
        return 0U;
    }

    bitmap |= tracking->faults.cellOverVoltage ? (uint16_t)(1U << 0) : 0U;
    bitmap |= tracking->faults.cellUnderVoltage ? (uint16_t)(1U << 1) : 0U;
    bitmap |= tracking->faults.chargeOverTemperature ? (uint16_t)(1U << 2) : 0U;
    bitmap |= tracking->faults.dischargeOverTemperature ? (uint16_t)(1U << 3) : 0U;
    bitmap |= tracking->faults.underTemperature ? (uint16_t)(1U << 4) : 0U;
    bitmap |= tracking->faults.chargeOverCurrent ? (uint16_t)(1U << 5) : 0U;
    bitmap |= tracking->faults.dischargeOverCurrent ? (uint16_t)(1U << 6) : 0U;
    bitmap |= tracking->faults.shortCircuit ? (uint16_t)(1U << 7) : 0U;
    bitmap |= tracking->faults.bqSafetyFault ? (uint16_t)(1U << 8) : 0U;
    bitmap |= tracking->faults.communicationFault ? (uint16_t)(1U << 9) : 0U;
    return bitmap;
}

static uint8_t bms_uart_fet_bitmap(const BMS_Tracking_t *tracking)
{
    uint8_t bitmap = 0U;

    if (tracking == NULL) {
        return 0U;
    }

    bitmap |= tracking->fetsEnabled ? (uint8_t)(1U << 0) : 0U;
    bitmap |= tracking->chargeFetEnabled ? (uint8_t)(1U << 1) : 0U;
    bitmap |= tracking->dischargeFetEnabled ? (uint8_t)(1U << 2) : 0U;
    bitmap |= tracking->chargeDisabled ? (uint8_t)(1U << 3) : 0U;
    bitmap |= tracking->dischargeDisabled ? (uint8_t)(1U << 4) : 0U;
    bitmap |= tracking->fetOffAsserted ? (uint8_t)(1U << 5) : 0U;
    return bitmap;
}

static uint8_t bms_uart_gate_signal_bitmap(const BMS_Tracking_t *tracking)
{
    uint8_t bitmap = 0U;

    if (tracking == NULL) {
        return 0U;
    }

    bitmap |= tracking->chargeGateFaultSignal ? (uint8_t)(1U << 0) : 0U;
    bitmap |= tracking->dischargeGateFaultSignal ? (uint8_t)(1U << 1) : 0U;
    return bitmap;
}

static uint16_t bms_uart_otp_flags(const bq76952_otp_status_t *status)
{
    uint16_t flags = 0U;

    if (status == NULL) {
        return 0U;
    }

    flags |= status->fullAccessOk ? (uint16_t)(1U << 0) : 0U;
    flags |= status->configUpdateOk ? (uint16_t)(1U << 1) : 0U;
    flags |= status->checkOk ? (uint16_t)(1U << 2) : 0U;
    flags |= status->writeOk ? (uint16_t)(1U << 3) : 0U;
    flags |= status->otpBlocked ? (uint16_t)(1U << 4) : 0U;
    flags |= status->otpPending ? (uint16_t)(1U << 5) : 0U;
    flags |= (status->dchgPinConfig == BQ_PIN_CONFIG_DCHG_ACTIVE_HIGH) ? (uint16_t)(1U << 6) : 0U;
    flags |= (status->ddsgPinConfig == BQ_PIN_CONFIG_DDSG_ACTIVE_HIGH) ? (uint16_t)(1U << 7) : 0U;
    flags |= ((status->daConfig & BQ_DA_CONFIG_USER_VOLTS_CV) != 0U) ? (uint16_t)(1U << 8) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_LOCK) != 0U) ? (uint16_t)(1U << 9) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_NOSIG) != 0U) ? (uint16_t)(1U << 10) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_NODATA) != 0U) ? (uint16_t)(1U << 11) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_HT) != 0U) ? (uint16_t)(1U << 12) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_LV) != 0U) ? (uint16_t)(1U << 13) : 0U;
    flags |= ((status->checkResult & BQ_OTP_RESULT_HV) != 0U) ? (uint16_t)(1U << 14) : 0U;
    flags |= ((status->writeResult != 0U) && ((status->writeResult & BQ_OTP_RESULT_OK) == 0U)) ?
             (uint16_t)(1U << 15) : 0U;

    return flags;
}

static uint16_t bms_uart_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static int32_t bms_uart_get_i32(const uint8_t *payload)
{
    uint32_t value;

    if (payload == NULL) {
        return 0;
    }

    value = (uint32_t)payload[0] |
            ((uint32_t)payload[1] << 8) |
            ((uint32_t)payload[2] << 16) |
            ((uint32_t)payload[3] << 24);
    return (int32_t)value;
}

static bool bms_uart_put_u8(uint8_t *payload, uint8_t *length, uint8_t value)
{
    if ((payload == NULL) || (length == NULL) || (*length >= BMS_UART_MAX_PAYLOAD_SIZE)) {
        return false;
    }

    payload[*length] = value;
    (*length)++;
    return true;
}

static bool bms_uart_put_u16(uint8_t *payload, uint8_t *length, uint16_t value)
{
    if ((payload == NULL) || (length == NULL) ||
        ((uint16_t)(*length) + 2U > BMS_UART_MAX_PAYLOAD_SIZE)) {
        return false;
    }

    payload[*length] = (uint8_t)(value & 0xFFU);
    payload[(uint8_t)(*length + 1U)] = (uint8_t)(value >> 8);
    *length = (uint8_t)(*length + 2U);
    return true;
}

static bool bms_uart_put_i16(uint8_t *payload, uint8_t *length, int16_t value)
{
    return bms_uart_put_u16(payload, length, (uint16_t)value);
}

static bool bms_uart_put_u32(uint8_t *payload, uint8_t *length, uint32_t value)
{
    if ((payload == NULL) || (length == NULL) ||
        ((uint16_t)(*length) + 4U > BMS_UART_MAX_PAYLOAD_SIZE)) {
        return false;
    }

    payload[*length] = (uint8_t)(value & 0xFFUL);
    payload[(uint8_t)(*length + 1U)] = (uint8_t)((value >> 8) & 0xFFUL);
    payload[(uint8_t)(*length + 2U)] = (uint8_t)((value >> 16) & 0xFFUL);
    payload[(uint8_t)(*length + 3U)] = (uint8_t)((value >> 24) & 0xFFUL);
    *length = (uint8_t)(*length + 4U);
    return true;
}

static bool bms_uart_put_i32(uint8_t *payload, uint8_t *length, int32_t value)
{
    return bms_uart_put_u32(payload, length, (uint32_t)value);
}

#else

void bms_uart_init(UART_HandleTypeDef *uart)
{
    (void)uart;
}

void bms_uart_task(void)
{
}

void bms_uart_restart_rx(void)
{
}

void bms_uart_send_protection_reason(uint8_t reason)
{
    (void)reason;
}

bool bms_uart_is_enabled(void)
{
    return false;
}

#endif
