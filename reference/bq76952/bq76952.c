#include "bq76952.h"



/* Vùng thanh ghi command/response trực tiếp của BQ76952.
 * 0x3E/0x3F: ghi subcommand hoặc data-memory address.
 * 0x40..   : vùng phản hồi/subcommand data.
 * 0x60..   : checksum + độ dài khi ghi data memory.
 */
#define CMD_DIR_SUBCMD_LOW          0x3EU
#define CMD_DIR_RESP_START          0x40U
#define CMD_DIR_RESP_CHKSUM         0x60U

/* Direct commands đọc dữ liệu tức thời. */
#define CMD_READ_VOLTAGE_STACK      0x34U
#define CMD_READ_VOLTAGE_PACK       0x36U

#define CMD_DIR_SAFETY_STATUS_A     0x03U
#define CMD_DIR_SAFETY_STATUS_B     0x05U
#define CMD_DIR_SAFETY_ALERT_C      0x06U
#define CMD_DIR_SAFETY_STATUS_C     0x07U
#define CMD_DIR_CONTROL_STATUS      0x00U
#define CMD_DIR_BATTERY_STATUS      0x12U
#define CMD_DIR_CC2_CUR             0x3AU
#define CMD_DIR_ALARM_STATUS        0x62U
#define CMD_DIR_ALARM_RAW_STATUS    0x64U
#define CMD_DIR_ALARM_ENABLE        0x66U
#define CMD_DIR_INT_TEMP            0x68U
#define CMD_DIR_FET_STAT            0x7FU
#define BQ_FET_STAT_CHG_FET         0x01U
#define BQ_FET_STAT_DSG_FET         0x04U
#define BQ_CHG_FET_PROTECTION_A_OCC 0x10U
#define BQ_ALARM_MASK_WITH_WAKE     0xF801U
#define BQ_ALERT_PIN_CONFIG_OPEN_DRAIN 0x22U
#define BQ_SLEEP_WAKE_COMPARATOR_CURRENT_MA 500
#define BQ_POWER_CONFIG_WK_SPD_MASK 0x0003U
#define BQ_POWER_CONFIG_DPSLP_PD    0x0800U
#define BQ_POWER_CONFIG_DPSLP_LDO   0x0400U
#define BQ_CONTROL_STATUS_DEEPSLEEP 0x0004U
#define BQ_LOW_V_SHUTDOWN_DELAY     0x9243U
#define BQ_SHUTDOWN_FET_OFF_DELAY   0x9252U
#define BQ_SHUTDOWN_COMMAND_DELAY   0x9253U
#define BQ_SHUTDOWN_AUTO_TIME       0x9254U
#define BQ_SHUTDOWN_FET_OFF_DELAY_NONE  0U
#define BQ_SHUTDOWN_COMMAND_DELAY_1S    4U
#define BQ_SHUTDOWN_AUTO_TIME_DISABLED  0U
#define BQ_SHUTDOWN_SUBCOMMAND_GAP_MS   5U
#define BQ_SHUTDOWN_SEQUENCE_TIMEOUT_MS 1500U
#define BQ_SHUTDOWN_BOOT_TIMEOUT_MS     500U
#define BQ_SHUTDOWN_BOOT_POLL_MS        25U
#define BQ_SLEEP_ENTRY_TRIES            10U
#define BQ_SLEEP_ENTRY_POLL_MS          100U
#define BQ_SECURITY_STATE_SEALED        3U
#define BQ_CURRENT_CALIBRATION_DEFAULT_PPM 1000000UL
#define BQ_CURRENT_CALIBRATION_PPM_DEN 1000000ULL
#define BQ_CC_GAIN_DEFAULT_RAW      0x413F67F5UL
#define BQ_CAPACITY_GAIN_DEFAULT_RAW 0x4A59C710UL
#define BQ_IEEE754_SIGN_MASK        0x80000000UL
#define BQ_IEEE754_EXP_MASK         0x7F800000UL
#define BQ_IEEE754_MANT_MASK        0x007FFFFFUL
#define BQ_IEEE754_MANT_HIDDEN_BIT  0x00800000UL
#define BQ_IEEE754_MANT_OVERFLOW_BIT 0x01000000UL
#define BQ_OC_THRESHOLD_STEP_MV     2U
#define BQ_OCC_THRESHOLD_MIN_CODE   2U
#define BQ_OCC_THRESHOLD_MAX_CODE   62U
#define BQ_OCD_THRESHOLD_MIN_CODE   2U
#define BQ_OCD_THRESHOLD_MAX_CODE   100U
#define BQ_CB_CONFIG_CHARGE         0x01U
#define BQ_CB_CONFIG_RELAX          0x02U
#define BQ_CB_CONFIG_SLEEP          0x04U

#define CMD_DEVICE_NUMBER           0x0001U
#define CMD_HW_VERSION              0x0003U
#define CMD_STATIC_CFG_SIG          0x0005U
#define CMD_COV_SNAPSHOT            0x0081U
#define SUBCMD_CB_ACTIVE_CELLS      0x0083U
#define SUBCMD_CBSTATUS1            0x0085U
#define SUBCMD_DEEPSLEEP            0x000FU
#define SUBCMD_EXIT_DEEPSLEEP       0x000EU
#define SUBCMD_SHUTDOWN             0x0010U
#define SUBCMD_SLEEP_ENABLE         0x0099U
#define SUBCMD_SLEEP_DISABLE        0x009AU
#define SUBCMD_REG12_CONTROL        0x0098U
/* Các subcommand phục vụ chuỗi ghi OTP. */
#define SUBCMD_OTP_WR_CHECK         0x00A0U
#define SUBCMD_OTP_WRITE            0x00A1U

/* Vị trí bit trong Safety Status A. */
#define BIT_SA_SC_DCHG              7U
#define BIT_SA_OC2_DCHG             6U
#define BIT_SA_OC1_DCHG             5U
#define BIT_SA_OC_CHG               4U
#define BIT_SA_CELL_OV              3U
#define BIT_SA_CELL_UV              2U

/* Vị trí bit trong Safety Status B. */
#define BIT_SB_OTF                  7U
#define BIT_SB_OTINT                6U
#define BIT_SB_OTD                  5U
#define BIT_SB_OTC                  4U
#define BIT_SB_UTINT                2U
#define BIT_SB_UTD                  1U
#define BIT_SB_UTC                  0U

/* Mỗi điện áp cell chiếm 2 byte, bắt đầu từ direct command 0x14. */
#define CELL_NO_TO_ADDR(cell_no) ((byte)(0x14U + ((cell_no) * 2U)))
#define LOW_BYTE(data) ((byte)((data) & 0x00FFU))
#define HIGH_BYTE(data) ((byte)(((data) >> 8) & 0x00FFU))
#define REG12_CONFIG(reg1v, reg1_en, reg2v, reg2_en) \
    ((byte)((((reg2v) & 0x07U) << 5U) | ((reg2_en) ? 0x10U : 0x00U) | \
            (((reg1v) & 0x07U) << 1U) | ((reg1_en) ? 0x01U : 0x00U)))
#define BQ_READ_STATUS_BIT(value, bit) (((value) >> (bit)) & 0x01U)
#define BQ_CONFIG_UPDATE_TIMEOUT_MS 100U
#define BQ_USER_CV_TO_MV            10U
#define BQ_OTP_CHECK_WAIT_MS        1000U
#define BQ_OTP_WRITE_WAIT_MS        1000U

#define UTC_THRESHOLD_CONFIG        0x92A6U
#define UTC_DELAY_CONFIG            0x92A7U
#define UTC_RECOVERY_CONFIG         0x92A8U
#define UTD_THRESHOLD_CONFIG        0x92A9U
#define UTD_DELAY_CONFIG            0x92AAU
#define UTD_RECOVERY_CONFIG         0x92ABU


static bq76952_protection_t     g_protection_status;
static bq76952_safety_alert_c_t g_safety_alert_c;
static uint16_t g_unseal_key_step_1;
static uint16_t g_unseal_key_step_2;
static uint16_t g_full_access_key_step_1;
static uint16_t g_full_access_key_step_2;
static bq76952_write_verify_t g_last_write_verify;

BQ76952_RawInfo_t BQ_RawInfo;

// static bool bq76952_write_register(byte reg, const byte *data, uint16_t len);
// static bool bq76952_read_register(byte reg, byte *data, uint16_t len);
static unsigned int bq76952_directCommand(byte command);
static bool bq76952_writeDirectCommand(byte command, uint16_t data);
static void bq76952_subCommand(unsigned int data);
static void bq76952_subCommandWithU8Data(unsigned int command, byte data);
static void bq76952_subCommandWithU16Data(unsigned int command, uint16_t data);
static int16_t bq76952_subCommandResponseInt(byte offset);
static byte bq76952_calculateChecksum(const byte *data, uint16_t len);
static byte bq76952_makeReg12Control(bool enable_reg1, bool enable_reg2);
static void bq76952_applyReg12Control(bool enable_reg1, bool enable_reg2);
static byte bq76952_make_pin_config(byte pin_fxn, bool active_low);
static byte bq76952_ocMvToThresholdCode(unsigned int mv, byte min_code, byte max_code);
static uint32_t bq76952_scaleIeee754RawByPpm(uint32_t raw_value, uint32_t gain_ppm);
static bool bq76952_writeU32DataMemory(unsigned int addr, uint32_t raw_value);
static unsigned int bq76952_userVoltageCommandToMv(byte command);
static uint16_t bq76952_dataMemoryExpectedValue(int16_t data, byte noOfBytes);
static bool bq76952_readDataMemoryBytes(unsigned int addr, byte *data, byte noOfBytes);
static bool bq76952_waitConfigUpdateMode(bool expected, uint32_t timeout_ms);
static bool bq76952_shutdownRequiresDoubleCommand(void);
static bool bq76952_waitShutdownSequenceStarted(uint32_t timeout_ms);
static bool bq76952_writeDataMemoryPayload(unsigned int addr, int16_t data, byte noOfBytes);
static bool bq76952_writeDataMemory(unsigned int addr, int16_t data, byte noOfBytes);
static int16_t bq76952_clampTemperatureLimit(int temp, int fallback);
static int16_t bq76952_deciKelvinToCelsius(uint16_t raw_deci_kelvin);
static void bq76952_fillOTPStatusSnapshot(bq76952_otp_status_t *status);
static bool bq76952_otpResultIsOk(uint8_t result);

// /* Driver hiện tại dùng lớp I2C software riêng thay vì HAL I2C trực tiếp. */
// void bq76952_begin(void)
// {
//     I2C_Soft_Init();
// }

// /* Ghi liên tiếp len byte vào một thanh ghi bắt đầu tại reg. */
// static bool bq76952_write_register(byte reg, const byte *data, uint16_t len)
// {
//     return I2C_Soft_WriteDataFromAddress(BQ_I2C_ADDR, reg, (uint8_t *)data, len) == E_OK;
// }

/* Đọc liên tiếp len byte từ một thanh ghi bắt đầu tại reg. */
// static bool bq76952_read_register(byte reg, byte *data, uint16_t len)
// {
//     return I2C_Soft_ReadDataFromAddress(BQ_I2C_ADDR, reg, data, len) == E_OK;
// }

/* Gửi direct command loại 2 byte rồi ghép little-endian thành unsigned int. */
static unsigned int bq76952_directCommand(byte command)
{
    byte data[2] = {0};
    if (!bq76952_read_register(command, data, 2U)) {
        return 0U;
    }
    return (unsigned int)(((unsigned int)data[1] << 8) | data[0]);
}

static bool bq76952_writeDirectCommand(byte command, uint16_t data)
{
    byte payload[2];

    payload[0] = LOW_BYTE(data);
    payload[1] = HIGH_BYTE(data);
    return bq76952_write_register(command, payload, 2U);
}

/* Gửi subcommand 16-bit vào cặp thanh ghi 0x3E/0x3F. */
static void bq76952_subCommand(unsigned int data)
{
    byte payload[2];

    payload[0] = LOW_BYTE(data);
    payload[1] = HIGH_BYTE(data);
    (void)bq76952_write_register(CMD_DIR_SUBCMD_LOW, payload, 2U);
}

static void bq76952_subCommandWithU8Data(unsigned int command, byte data)
{
    byte payload[3];
    byte footer[2];

    payload[0] = LOW_BYTE(command);
    payload[1] = HIGH_BYTE(command);
    payload[2] = data;

    footer[0] = bq76952_calculateChecksum(payload, 3U);
    footer[1] = 0x05U;

    (void)bq76952_write_register(CMD_DIR_SUBCMD_LOW, payload, 3U);
    (void)bq76952_write_register(CMD_DIR_RESP_CHKSUM, footer, 2U);
}

/* Gửi subcommand có payload 16-bit. Dùng cho các lệnh runtime như manual cell balance. */
static void bq76952_subCommandWithU16Data(unsigned int command, uint16_t data)
{
    byte payload[4];
    byte footer[2];

    payload[0] = LOW_BYTE(command);
    payload[1] = HIGH_BYTE(command);
    payload[2] = LOW_BYTE(data);
    payload[3] = HIGH_BYTE(data);

    footer[0] = bq76952_calculateChecksum(payload, 4U);
    footer[1] = 0x06U;

    (void)bq76952_write_register(CMD_DIR_SUBCMD_LOW, payload, 4U);
    (void)bq76952_write_register(CMD_DIR_RESP_CHKSUM, footer, 2U);
}

/* Đọc phản hồi 16-bit trong vùng response với offset byte tương ứng. */
static int16_t bq76952_subCommandResponseInt(byte offset)
{
    byte data[2] = {0};

    if (!bq76952_read_register((byte)(CMD_DIR_RESP_START + offset), data, 2U)) {
        return 0;
    }

    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

/* 0x0090 là subcommand chuẩn để vào Config Update mode. */
void bq76952_enterConfigUpdate(void)
{
    bq76952_subCommand(0x0090U);
    HAL_Delay(2U);
}

/* 0x0092 yêu cầu IC thoát Config Update mode và commit cấu hình. */
void bq76952_exitConfigUpdate(void)
{
    bq76952_subCommand(0x0092U);
    HAL_Delay(1U);
}

/* Checksum cua BQ76952 la one's-complement cua tong cac byte payload. */
static byte bq76952_calculateChecksum(const byte *data, uint16_t len)
{
    byte sum = 0U;

    if (data == NULL) {
        return 0xFFU;
    }

    for (uint16_t i = 0U; i < len; ++i) {
        sum = (byte)(sum + data[i]);
    }

    return (byte)(~sum);
}

static byte bq76952_makeReg12Control(bool enable_reg1, bool enable_reg2)
{
    return REG12_CONFIG(BQ_REG1_VOLTAGE_CODE,
                        enable_reg1,
                        BQ_REG2_VOLTAGE_CODE,
                        enable_reg2);
}

static void bq76952_applyReg12Control(bool enable_reg1, bool enable_reg2)
{
    bq76952_subCommandWithU8Data(SUBCMD_REG12_CONTROL,
                                 bq76952_makeReg12Control(enable_reg1, enable_reg2));
    HAL_Delay(1U);
}

static byte bq76952_make_pin_config(byte pin_fxn, bool active_low)
{
    byte cfg = (byte)(pin_fxn & 0x03U);

    /* OPT3=1 + OPT1=1: dùng REG1 làm mức logic cao và bật active drive.
     * OPT5 quyết định cực tính active-high/active-low của tín hiệu chức năng ALT.
     */
    cfg |= 0x28U;
    if (active_low) {
        cfg |= 0x80U;
    }
    return cfg;
}

static byte bq76952_ocMvToThresholdCode(unsigned int mv, byte min_code, byte max_code)
{
    unsigned int code = (mv + (BQ_OC_THRESHOLD_STEP_MV - 1U)) / BQ_OC_THRESHOLD_STEP_MV;

    if (code < min_code) {
        code = min_code;
    } else if (code > max_code) {
        code = max_code;
    }
    return (byte)code;
}

static uint32_t bq76952_scaleIeee754RawByPpm(uint32_t raw_value, uint32_t gain_ppm)
{
    uint32_t sign;
    uint32_t exponent;
    uint64_t mantissa;

    if (gain_ppm == 0UL) {
        gain_ppm = BQ_CURRENT_CALIBRATION_DEFAULT_PPM;
    }

    exponent = (raw_value & BQ_IEEE754_EXP_MASK) >> 23U;
    if ((exponent == 0UL) || (exponent == 0xFFUL)) {
        return raw_value;
    }

    sign = raw_value & BQ_IEEE754_SIGN_MASK;
    mantissa = (uint64_t)BQ_IEEE754_MANT_HIDDEN_BIT |
               (uint64_t)(raw_value & BQ_IEEE754_MANT_MASK);
    mantissa = ((mantissa * (uint64_t)gain_ppm) +
                (BQ_CURRENT_CALIBRATION_PPM_DEN / 2ULL)) /
               BQ_CURRENT_CALIBRATION_PPM_DEN;

    if (mantissa == 0ULL) {
        return sign;
    }

    while (mantissa >= (uint64_t)BQ_IEEE754_MANT_OVERFLOW_BIT) {
        mantissa = (mantissa + 1ULL) >> 1U;
        exponent++;
        if (exponent >= 0xFFUL) {
            return sign | BQ_IEEE754_EXP_MASK;
        }
    }

    while ((mantissa < (uint64_t)BQ_IEEE754_MANT_HIDDEN_BIT) && (exponent > 1UL)) {
        mantissa <<= 1U;
        exponent--;
    }

    if (mantissa < (uint64_t)BQ_IEEE754_MANT_HIDDEN_BIT) {
        return sign;
    }

    return sign |
           (uint32_t)(exponent << 23U) |
           (uint32_t)(mantissa & BQ_IEEE754_MANT_MASK);
}

static bool bq76952_writeU32DataMemory(unsigned int addr, uint32_t raw_value)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(addr,
                                               (int16_t)(uint16_t)(raw_value & 0xFFFFUL),
                                               2U);
    status &= (uint8_t)bq76952_writeDataMemory(addr + 2U,
                                               (int16_t)(uint16_t)((raw_value >> 16) & 0xFFFFUL),
                                               2U);
    return status != 0U;
}

static unsigned int bq76952_userVoltageCommandToMv(byte command)
{
    int16_t raw_user_voltage = (int16_t)bq76952_directCommand(command);

    if (raw_user_voltage <= 0) {
        return 0U;
    }

    return (unsigned int)raw_user_voltage * BQ_USER_CV_TO_MV;
}

static uint16_t bq76952_dataMemoryExpectedValue(int16_t data, byte noOfBytes)
{
    if (noOfBytes == 1U) {
        return (uint16_t)LOW_BYTE(data);
    }

    return (uint16_t)data;
}

static bool bq76952_readDataMemoryBytes(unsigned int addr, byte *data, byte noOfBytes)
{
    byte request[2];

    if ((data == NULL) || ((noOfBytes != 1U) && (noOfBytes != 2U))) {
        return false;
    }

    request[0] = LOW_BYTE(addr);
    request[1] = HIGH_BYTE(addr);
    if (!bq76952_write_register(CMD_DIR_SUBCMD_LOW, request, 2U)) {
        return false;
    }

    HAL_Delay(2U);
    return bq76952_read_register(CMD_DIR_RESP_START, data, noOfBytes);
}

static bool bq76952_waitConfigUpdateMode(bool expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do {
        bq76952_battery_status_t batt_status = bq76952_getBatteryStatusRegister();
        bool in_config_update = batt_status.bits.CONFIG_UPDATE_MODE != 0U;

        if (in_config_update == expected) {
            return true;
        }

        HAL_Delay(1U);
    } while ((HAL_GetTick() - start) < timeout_ms);

    return false;
}

static bool bq76952_shutdownRequiresDoubleCommand(void)
{
    unsigned int batt_status_raw = bq76952_getBatteryStatusRaw();
    uint16_t security_state = (uint16_t)((batt_status_raw >> 8U) & 0x03U);

    return (batt_status_raw == 0U) ||
           (security_state == BQ_SECURITY_STATE_SEALED);
}

static bool bq76952_waitShutdownSequenceStarted(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do {
        bq76952_battery_status_t batt_status;

        if (!bq76952_isConnected()) {
            return true;
        }

        batt_status = bq76952_getBatteryStatusRegister();
        if (batt_status.bits.SHUTDOWN_PENDING != 0U) {
            return true;
        }

        HAL_Delay(BQ_SHUTDOWN_BOOT_POLL_MS);
    } while ((HAL_GetTick() - start) < timeout_ms);

    return !bq76952_isConnected();
}

static bool bq76952_writeDataMemoryPayload(unsigned int addr, int16_t data, byte noOfBytes)
{
    byte payload[4];
    byte footer[2];
    uint16_t payload_len;

    if ((noOfBytes != 1U) && (noOfBytes != 2U)) {
        return false;
    }

    payload_len = (noOfBytes == 1U) ? 3U : 4U;
    payload[0] = LOW_BYTE(addr);
    payload[1] = HIGH_BYTE(addr);
    payload[2] = LOW_BYTE(data);
    payload[3] = HIGH_BYTE(data);

    footer[0] = bq76952_calculateChecksum(payload, payload_len);
    footer[1] = (byte)(payload_len + 2U);

    return bq76952_write_register(CMD_DIR_SUBCMD_LOW, payload, payload_len) &&
           bq76952_write_register(CMD_DIR_RESP_CHKSUM, footer, 2U);
}

/* Ghi một mục data memory.
 * noOfBytes = 1 hoặc 2, footer[1] là tổng độ dài khung theo giao thức BQ76952.
 * Hàm này tự vào/ra Config Update mode cho từng lần ghi.
 */
static bool bq76952_writeDataMemory(unsigned int addr, int16_t data, byte noOfBytes)
{
    byte readback[2] = {0};
    uint16_t actual = 0U;
    uint16_t expected = bq76952_dataMemoryExpectedValue(data, noOfBytes);
    bool Status;

    if ((noOfBytes != 1U) && (noOfBytes != 2U)) {
        return false;
    }

    bq76952_enterConfigUpdate();
    Status = bq76952_waitConfigUpdateMode(true, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    if (Status == true) {
        Status = bq76952_writeDataMemoryPayload(addr, data, noOfBytes);
        if (Status == true) {
            HAL_Delay(2U);
            Status = bq76952_readDataMemoryBytes(addr, readback, noOfBytes);
            if (Status == true) {
                actual = (noOfBytes == 1U) ?
                         (uint16_t)readback[0] :
                         (uint16_t)(((uint16_t)readback[1] << 8) | readback[0]);
                Status = (actual == expected);
            }
        }
    }

    bq76952_exitConfigUpdate();
    (void)bq76952_waitConfigUpdateMode(false, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    return Status;
}

/* Temperature protection limits are I1 values in degrees Celsius. */
static int16_t bq76952_clampTemperatureLimit(int temp, int fallback)
{
    if (temp < -40 || temp > 120) {
        temp = fallback;
    }
    return (int16_t)temp;
}

static int16_t bq76952_deciKelvinToCelsius(uint16_t raw_deci_kelvin)
{
    if (raw_deci_kelvin >= 2732U) {
        return (int16_t)((raw_deci_kelvin - 2732U) / 10U);
    }
    return (int16_t)(-((int16_t)((2732U - raw_deci_kelvin + 9U) / 10U)));
}

/* Đọc 1 hoặc 2 byte data memory tại địa chỉ addr.
 * size nên là 1 hoặc 2; nếu 2 thì kết quả được ghép little-endian.
 */
unsigned int bq76952_readDataMemory(unsigned int addr, int size)
{
    byte data[2] = {0};

    if (!bq76952_readDataMemoryBytes(addr, data, (byte)size)) {
        return 0U;
    }

    if (size == 1) {
        return data[0];
    }

    return (unsigned int)(((unsigned int)data[1] << 8) | data[0]);
}

bool bq76952_getLastWriteVerify(bq76952_write_verify_t *status)
{
    if (status == NULL) {
        return false;
    }

    *status = g_last_write_verify;
    return g_last_write_verify.attempted;
}

bool bq76952_isConnected(void)
{
    return I2C_Soft_WriteData(BQ_I2C_ADDR, NULL, 0U) == E_OK;
}

/* 0x0012 là reset mềm thiết bị. */
void bq76952_reset(void)
{
    bq76952_subCommand(0x0012U);
}

byte bq76952_getMfgStatusInitRegister(void)
{
    return (byte)bq76952_readDataMemory(0x9343U, 1);
}

int bq76952_getCellVoltage(byte cellNumber)
{
    return (int)(int16_t)bq76952_directCommand(CELL_NO_TO_ADDR(cellNumber));
}

/* Đọc toàn bộ 16 kênh cell của IC, kể cả các kênh không dùng. */
void bq76952_getAllCellVoltages(uint16_t *cellArray, uint8_t numCells)
{
    if (cellArray == NULL) {
        return;
    }
    int CellV = 0;

    for (byte index = 0U; index < numCells; ++index) {
        CellV = bq76952_getCellVoltage(index);
        cellArray[index] = CellV < 0 ? 0 : CellV;
    }
}

/* Mapping này phản ánh cách pack hiện tại nối 10 cell vào 16 kênh đo của BQ76952.
 * Các phần tử bị bỏ qua là các cell sense không được dùng trong phần cứng này.
 */
void bq76952_getOnlyConnectedCellVoltages(uint16_t *cellArray)
{
    if (cellArray == NULL) {
        return;
    }

    bq76952_getAllCellVoltages(cellArray, 10U);
    // for (byte cell = 0U; cell < 10U; ++cell) {
    //     cellArray[cell] = allcells[cell];
    // }
}

int bq76952_getCurrent(void)
{
    return (int)(int16_t)bq76952_directCommand(CMD_DIR_CC2_CUR);
}

int bq76952_getCurrentNow(void)
{
    return bq76952_getCurrent();
}

int bq76952_getCurrentAvg(void)
{
    /* 0x0075 offset 20 là CC3 Current, tức dòng trung bình theo cấu hình CC3 Samples. */
    bq76952_subCommand(0x0075U);
    HAL_Delay(1U);
    return bq76952_subCommandResponseInt(20U);
}

unsigned int bq76952_getManufacturingStatus(void)
{
    /* 0x0057 đọc Manufacturing Status, chứa nhiều cờ vận hành như FET_EN, fuse, sleep. */
    bq76952_subCommand(0x0057U);
    HAL_Delay(1U);
    return (unsigned int)bq76952_subCommandResponseInt(0U);
}

unsigned int bq76952_getDeviceNumber(void)
{
    bq76952_subCommand(CMD_DEVICE_NUMBER);
    HAL_Delay(1U);
    return (unsigned int)bq76952_subCommandResponseInt(0U);
}

unsigned int bq76952_getHWVersion(void)
{
    bq76952_subCommand(CMD_HW_VERSION);
    HAL_Delay(1U);
    return (unsigned int)bq76952_subCommandResponseInt(0U);
}

bool bq76952_areFETs_Enabled(void)
{
    return (bq76952_getManufacturingStatus() & 0x10U) != 0U;
}

byte bq76952_getFetStatusRaw(void)
{
    return (byte)bq76952_directCommand(CMD_DIR_FET_STAT);
}

unsigned int bq76952_getStackVoltage(void)
{
    return bq76952_userVoltageCommandToMv(CMD_READ_VOLTAGE_STACK);
}

unsigned int bq76952_getPackVoltage(void)
{
    return bq76952_userVoltageCommandToMv(CMD_READ_VOLTAGE_PACK);
}


unsigned int bq76952_getCOVSnapshot(byte cell)
{
    /* Sau lỗi COV, IC lưu lại snapshot để biết cell nào đạt bao nhiêu điện áp tại thời điểm fault. */
    bq76952_subCommand(CMD_COV_SNAPSHOT);
    HAL_Delay(1U);
    return (unsigned int)bq76952_subCommandResponseInt(cell);
}

bool bq76952_is_OTP_already_programmed(void)
{
    bq76952_otp_status_t status;

    if (!bq76952_checkOTPWriteReady(&status)) {
        return (status.checkResult & (BQ_OTP_RESULT_LOCK |
                                      BQ_OTP_RESULT_NOSIG |
                                      BQ_OTP_RESULT_NODATA)) != 0U;
    }

    return false;
}

bool bq76952_checkSecurityKeys(void)
{
    /* 0x0035 đọc liên tiếp 4 word key: unseal step1/2 và full-access step1/2. */
    bq76952_subCommand(0x0035U);
    HAL_Delay(1U);
    g_unseal_key_step_1 = (uint16_t)bq76952_subCommandResponseInt(0U);

    bq76952_subCommand(0x0035U);
    HAL_Delay(1U);
    g_unseal_key_step_2 = (uint16_t)bq76952_subCommandResponseInt(2U);

    bq76952_subCommand(0x0035U);
    HAL_Delay(1U);
    g_full_access_key_step_1 = (uint16_t)bq76952_subCommandResponseInt(4U);

    bq76952_subCommand(0x0035U);
    HAL_Delay(1U);
    g_full_access_key_step_2 = (uint16_t)bq76952_subCommandResponseInt(6U);

    return (g_full_access_key_step_1 == FULL_ACCESS_KEY_STEP_1) &&
           (g_full_access_key_step_2 == FULL_ACCESS_KEY_STEP_2);
}

unsigned int bq76952_getBatteryStatusRaw(void)
{
    return bq76952_directCommand(CMD_DIR_BATTERY_STATUS);
}

bq76952_battery_status_t bq76952_getBatteryStatusRegister(void)
{
    bq76952_battery_status_t batt_stat = {0};
    unsigned int regData = bq76952_getBatteryStatusRaw();

    batt_stat.bits.SLEEP_MODE = BQ_READ_STATUS_BIT(regData, 15U);
    batt_stat.bits.SHUTDOWN_PENDING = BQ_READ_STATUS_BIT(regData, 13U);
    batt_stat.bits.PERMANENT_FAULT = BQ_READ_STATUS_BIT(regData, 12U);
    batt_stat.bits.SAFETY_FAULT = BQ_READ_STATUS_BIT(regData, 11U);
    batt_stat.bits.FUSE_PIN = BQ_READ_STATUS_BIT(regData, 10U);
    batt_stat.bits.SECURITY_STATE = (uint16_t)((regData >> 8U) & 0x03U);
    batt_stat.bits.WR_TO_OTP_BLOCKED = BQ_READ_STATUS_BIT(regData, 7U);
    batt_stat.bits.WR_TO_OTP_PENDING = BQ_READ_STATUS_BIT(regData, 6U);
    batt_stat.bits.OPEN_WIRE_CHECK = BQ_READ_STATUS_BIT(regData, 5U);
    batt_stat.bits.WD_WAS_TRIGGERED = BQ_READ_STATUS_BIT(regData, 4U);
    batt_stat.bits.FULL_RESET_OCCURED = BQ_READ_STATUS_BIT(regData, 3U);
    batt_stat.bits.SLEEP_EN_ALLOWED = BQ_READ_STATUS_BIT(regData, 2U);
    batt_stat.bits.PRECHARGE_MODE = BQ_READ_STATUS_BIT(regData, 1U);
    batt_stat.bits.CONFIG_UPDATE_MODE = BQ_READ_STATUS_BIT(regData, 0U);

    return batt_stat;
}

bool bq76952_Enter_FullAccessMode(void)
{
    bq76952_battery_status_t batt_st;

    g_unseal_key_step_1 = UNSEAL_KEY_STEP_1;
    g_unseal_key_step_2 = UNSEAL_KEY_STEP_2;
    g_full_access_key_step_1 = FULL_ACCESS_KEY_STEP_1;
    g_full_access_key_step_2 = FULL_ACCESS_KEY_STEP_2;

    batt_st = bq76952_getBatteryStatusRegister();
    if (batt_st.bits.SECURITY_STATE == 1U) {
        return true;
    }

    if (batt_st.bits.SECURITY_STATE == 3U) {
        /* Sealed: cần unseal rồi mới vào full access. */
        bq76952_subCommand(g_unseal_key_step_1);
        HAL_Delay(2U);
        bq76952_subCommand(g_unseal_key_step_2);
        HAL_Delay(2U);
        batt_st = bq76952_getBatteryStatusRegister();
    }

    if (batt_st.bits.SECURITY_STATE == 2U) {
        /* Unsealed: gửi full-access key. */
        bq76952_subCommand(g_full_access_key_step_1);
        HAL_Delay(2U);
        bq76952_subCommand(g_full_access_key_step_2);
        HAL_Delay(2U);
    }

    batt_st = bq76952_getBatteryStatusRegister();
    return batt_st.bits.SECURITY_STATE == 1U;
}

bool bq76952_configure_before_OTP_write(void)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_setEnablePreRegulator();
    status &= (uint8_t)bq76952_setEnableRegulator(true, true);
    return status != 0U;
}

static bool bq76952_otpResultIsOk(uint8_t result)
{
    return ((result & BQ_OTP_RESULT_OK) != 0U) &&
           ((result & (BQ_OTP_RESULT_LOCK |
                       BQ_OTP_RESULT_NOSIG |
                       BQ_OTP_RESULT_NODATA |
                       BQ_OTP_RESULT_HT |
                       BQ_OTP_RESULT_LV |
                       BQ_OTP_RESULT_HV)) == 0U);
}

static void bq76952_fillOTPStatusSnapshot(bq76952_otp_status_t *status)
{
    uint16_t battery_status_raw;

    if (status == NULL) {
        return;
    }

    battery_status_raw = (uint16_t)bq76952_getBatteryStatusRaw();
    status->batteryStatusRaw = battery_status_raw;
    status->securityState = (uint8_t)((battery_status_raw >> 8U) & 0x03U);
    status->otpBlocked = BQ_READ_STATUS_BIT(battery_status_raw, 7U) != 0U;
    status->otpPending = BQ_READ_STATUS_BIT(battery_status_raw, 6U) != 0U;
    status->stackVoltage_mV = (uint16_t)bq76952_getStackVoltage();
    status->packVoltage_mV = (uint16_t)bq76952_getPackVoltage();
    status->internalTemp_C = bq76952_getInternalTemp();

    bq76952_subCommand(CMD_STATIC_CFG_SIG);
    HAL_Delay(1U);
    status->staticConfigSignature = (uint16_t)bq76952_subCommandResponseInt(0U);

    status->reg0Config = (uint8_t)bq76952_readDataMemory(REG0_CONFIG, 1);
    status->reg12Control = (uint8_t)bq76952_readDataMemory(REG12_CONTROL, 1);
    status->daConfig = (uint8_t)bq76952_readDataMemory(DA_CONFIGURATION, 1);
    status->vcellMode = (uint16_t)bq76952_readDataMemory(VCELL_MODE, 2);
    status->dchgPinConfig = (uint8_t)bq76952_readDataMemory(DCHG_PIN_CONFIG, 1);
    status->ddsgPinConfig = (uint8_t)bq76952_readDataMemory(DDSG_PIN_CONFIG, 1);
    status->dfetoffPinConfig = (uint8_t)bq76952_readDataMemory(DFETOFF_PIN_CONFIG, 1);
}

bool bq76952_readOTPStatus(bq76952_otp_status_t *status)
{
    if (status == NULL) {
        return false;
    }

    *status = (bq76952_otp_status_t){0};
    bq76952_fillOTPStatusSnapshot(status);
    return true;
}

bool bq76952_checkOTPWriteReady(bq76952_otp_status_t *status)
{
    uint16_t check_battery_status_raw = 0U;
    bool blocked_during_check = false;

    if (status == NULL) {
        return false;
    }

    *status = (bq76952_otp_status_t){0};
    status->fullAccessOk = bq76952_Enter_FullAccessMode();
    if (!status->fullAccessOk) {
        bq76952_fillOTPStatusSnapshot(status);
        return false;
    }

    if (!bq76952_configure_before_OTP_write()) {
        bq76952_fillOTPStatusSnapshot(status);
        return false;
    }

    bq76952_enterConfigUpdate();
    status->configUpdateOk = bq76952_waitConfigUpdateMode(true, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    if (status->configUpdateOk) {
        bq76952_subCommand(SUBCMD_OTP_WR_CHECK);
        HAL_Delay(BQ_OTP_CHECK_WAIT_MS);
        status->checkResult = (uint8_t)bq76952_subCommandResponseInt(0U);
        status->checkDataFailAddr = (uint16_t)bq76952_subCommandResponseInt(1U);
        check_battery_status_raw = (uint16_t)bq76952_getBatteryStatusRaw();
        blocked_during_check = BQ_READ_STATUS_BIT(check_battery_status_raw, 7U) != 0U;
    }
    bq76952_exitConfigUpdate();
    (void)bq76952_waitConfigUpdateMode(false, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    bq76952_fillOTPStatusSnapshot(status);
    status->otpBlocked = status->otpBlocked || blocked_during_check;

    status->checkOk = status->configUpdateOk &&
                      bq76952_otpResultIsOk(status->checkResult) &&
                      !blocked_during_check;
    return status->checkOk;
}

bool bq76952_program_OTP_with_status(bq76952_otp_status_t *status)
{
    bq76952_otp_status_t local_status;
    bq76952_otp_status_t *result = (status != NULL) ? status : &local_status;
    bool allow_write = false;

    *result = (bq76952_otp_status_t){0};
    result->fullAccessOk = bq76952_Enter_FullAccessMode();
    if (!result->fullAccessOk) {
        bq76952_fillOTPStatusSnapshot(result);
        return false;
    }

    if (!bq76952_configure_before_OTP_write()) {
        bq76952_fillOTPStatusSnapshot(result);
        return false;
    }

    bq76952_enterConfigUpdate();
    result->configUpdateOk = bq76952_waitConfigUpdateMode(true, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    if (result->configUpdateOk) {
        uint16_t battery_status_raw;

        bq76952_subCommand(SUBCMD_OTP_WR_CHECK);
        HAL_Delay(BQ_OTP_CHECK_WAIT_MS);
        result->checkResult = (uint8_t)bq76952_subCommandResponseInt(0U);
        result->checkDataFailAddr = (uint16_t)bq76952_subCommandResponseInt(1U);
        battery_status_raw = (uint16_t)bq76952_getBatteryStatusRaw();
        allow_write = bq76952_otpResultIsOk(result->checkResult) &&
                      (BQ_READ_STATUS_BIT(battery_status_raw, 7U) == 0U);

        if (allow_write) {
            bq76952_subCommand(SUBCMD_OTP_WRITE);
            HAL_Delay(BQ_OTP_WRITE_WAIT_MS);
            result->writeResult = (uint8_t)bq76952_subCommandResponseInt(0U);
            result->writeDataFailAddr = (uint16_t)bq76952_subCommandResponseInt(1U);
        }
    }

    bq76952_exitConfigUpdate();
    (void)bq76952_waitConfigUpdateMode(false, BQ_CONFIG_UPDATE_TIMEOUT_MS);
    bq76952_fillOTPStatusSnapshot(result);

    result->checkOk = result->configUpdateOk &&
                      bq76952_otpResultIsOk(result->checkResult) &&
                      !result->otpBlocked;
    result->writeOk = allow_write && bq76952_otpResultIsOk(result->writeResult);
    return result->writeOk;
}

bool bq76952_program_OTP(void)
{
    bq76952_otp_status_t status;

    return bq76952_program_OTP_with_status(&status);
}

int16_t bq76952_getInternalTemp(void)
{
    /* BQ76952 trả nhiệt độ theo 0.1 Kelvin, cần đổi sang Celsius. */
    return bq76952_deciKelvinToCelsius((uint16_t)bq76952_directCommand(CMD_DIR_INT_TEMP));
}

int16_t bq76952_getThermistorTemp(bq76952_thermistor_t thermistor)
{
    byte command = 0x70U;

    /* Mỗi nguồn nhiệt độ có một direct command riêng, cách nhau 2 byte. */
    switch (thermistor) {
    case TS1:
        command = 0x70U;
        break;
    case TS2:
        command = 0x72U;
        break;
    case TS3:
        command = 0x74U;
        break;
    case HDQ:
        command = 0x76U;
        break;
    case DCHG:
        command = 0x78U;
        break;
    case DDSG:
        command = 0x7AU;
        break;
    default:
        break;
    }

    return bq76952_deciKelvinToCelsius((uint16_t)bq76952_directCommand(command));
}

bq76952_protection_t bq76952_getProtectionStatus(void)
{
    byte regData = (byte)bq76952_directCommand(CMD_DIR_SAFETY_STATUS_A);

    g_protection_status.bits.SC_DCHG = BQ_READ_STATUS_BIT(regData, BIT_SA_SC_DCHG);
    g_protection_status.bits.OC2_DCHG = BQ_READ_STATUS_BIT(regData, BIT_SA_OC2_DCHG);
    g_protection_status.bits.OC1_DCHG = BQ_READ_STATUS_BIT(regData, BIT_SA_OC1_DCHG);
    g_protection_status.bits.OC_CHG = BQ_READ_STATUS_BIT(regData, BIT_SA_OC_CHG);
    g_protection_status.bits.CELL_OV = BQ_READ_STATUS_BIT(regData, BIT_SA_CELL_OV);
    g_protection_status.bits.CELL_UV = BQ_READ_STATUS_BIT(regData, BIT_SA_CELL_UV);

    return g_protection_status;
}

bq76952_safety_alert_c_t bq76952_getSafetyAlert_C(void)
{
    byte regData = (byte)bq76952_directCommand(CMD_DIR_SAFETY_ALERT_C);

    g_safety_alert_c.bits.OCD3 = BQ_READ_STATUS_BIT(regData, 7U);
    g_safety_alert_c.bits.SCDL = BQ_READ_STATUS_BIT(regData, 6U);
    g_safety_alert_c.bits.OCDL = BQ_READ_STATUS_BIT(regData, 5U);
    g_safety_alert_c.bits.COVL = BQ_READ_STATUS_BIT(regData, 4U);
    g_safety_alert_c.bits.PTOS = BQ_READ_STATUS_BIT(regData, 3U);

    return g_safety_alert_c;
}

BQ76952_SafetyStatusC_t bq76952_getSafetyStatus_C(void)
{
    BQ_RawInfo.statusC.all = (byte)bq76952_directCommand(CMD_DIR_SAFETY_STATUS_C);
    return BQ_RawInfo.statusC;
}

bq76952_temp_t bq76952_getTemperatureStatus(void)
{
    bq76952_temp_t status = {0};
    byte regData = (byte)bq76952_directCommand(CMD_DIR_SAFETY_STATUS_B);

    /* Safety Status B is a protection bitmap, not an absolute temperature value. */
    status.bits.OVERTEMP_FET = BQ_READ_STATUS_BIT(regData, BIT_SB_OTF);
    status.bits.OVERTEMP_INTERNAL = BQ_READ_STATUS_BIT(regData, BIT_SB_OTINT);
    status.bits.OVERTEMP_DCHG = BQ_READ_STATUS_BIT(regData, BIT_SB_OTD);
    status.bits.OVERTEMP_CHG = BQ_READ_STATUS_BIT(regData, BIT_SB_OTC);
    status.bits.UNDERTEMP_INTERNAL = BQ_READ_STATUS_BIT(regData, BIT_SB_UTINT);
    status.bits.UNDERTEMP_DCHG = BQ_READ_STATUS_BIT(regData, BIT_SB_UTD);
    status.bits.UNDERTEMP_CHG = BQ_READ_STATUS_BIT(regData, BIT_SB_UTC);

    return status;
}

void bq76952_setFET(bq76952_fet_t fet, bq76952_fet_state_t state)
{
    /* Mặc định 0x0096 là ALL_FETS_ON; các nhánh OFF dùng subcommand riêng cho CHG/DSG/ALL. */
    unsigned int subcmd = 0x0096U;

    if (state == OFF) {
        switch (fet) {
        case DCH:
            subcmd = 0x0093U;
            break;
        case CHG:
            subcmd = 0x0094U;
            break;
        case ALL:
        default:
            subcmd = 0x0095U;
            break;
        }
    }

    bq76952_subCommand(subcmd);
}

void bq76952_setFET_ENABLE(void)
{
    bq76952_subCommand(0x0022U);
}

bool bq76952_isCharging(void)
{
    return bq76952_isChargeFetOn();
}

bool bq76952_isDischarging(void)
{
    return bq76952_isDischargeFetOn();
}

bool bq76952_isChargeFetOn(void)
{
    BQ_RawInfo.FetStatus.all = bq76952_getFetStatusRaw();
    return BQ_RawInfo.FetStatus.bit.CHG_FET;
}

bool bq76952_isDischargeFetOn(void)
{
    BQ_RawInfo.FetStatus.all = bq76952_getFetStatusRaw();
    return BQ_RawInfo.FetStatus.bit.DSG_FET;
}

bool bq76952_ConfigManualCellBalancing(int8_t min_cell_temp_c,
                                              int8_t max_cell_temp_c,
                                              int8_t max_internal_temp_c,
                                              uint8_t interval_s,
                                              uint8_t max_cells)
{
    byte cfg = 0x08;
    bq76952_writeDataMemory(BALANCING_CONFIGURATION, cfg, 1U);
    bq76952_writeDataMemory(CELL_BALANCE_MIN_CELL_TEMP, min_cell_temp_c, 1U);
    bq76952_writeDataMemory(CELL_BALANCE_MAX_CELL_TEMP, max_cell_temp_c, 1U);
    bq76952_writeDataMemory(CELL_BALANCE_MAX_INTERNAL_TEMP, max_internal_temp_c, 1U);
    bq76952_writeDataMemory(CELL_BALANCE_INTERVAL, interval_s, 1U);
    return bq76952_writeDataMemory(CELL_BALANCE_MAX_CELLS, max_cells, 1U);

}

bool bq76952_configureAutonomousCellBalancing(uint16_t min_cell_mv,
                                              uint8_t start_delta_mv,
                                              uint8_t stop_delta_mv,
                                              int8_t min_cell_temp_c,
                                              int8_t max_cell_temp_c,
                                              int8_t max_internal_temp_c,
                                              uint8_t interval_s,
                                              uint8_t max_cells)
{
    uint8_t status = 1U;
    byte cfg = BQ_CB_CONFIG_CHARGE | BQ_CB_CONFIG_RELAX | BQ_CB_CONFIG_SLEEP;

    if ((interval_s == 0U) || (max_cells > 16U) || (stop_delta_mv > start_delta_mv)) {
        return false;
    }

    status &= (uint8_t)bq76952_writeDataMemory(BALANCING_CONFIGURATION, cfg, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MIN_CELL_TEMP, min_cell_temp_c, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MAX_CELL_TEMP, max_cell_temp_c, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MAX_INTERNAL_TEMP, max_internal_temp_c, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_INTERVAL, interval_s, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MAX_CELLS, max_cells, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MIN_CELL_V_CHARGE,
                                               (int16_t)min_cell_mv,
                                               2U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MIN_DELTA_CHARGE, start_delta_mv, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_STOP_DELTA_CHARGE, stop_delta_mv, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MIN_CELL_V_RELAX,
                                               (int16_t)min_cell_mv,
                                               2U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_MIN_DELTA_RELAX, start_delta_mv, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CELL_BALANCE_STOP_DELTA_RELAX, stop_delta_mv, 1U);

    return status != 0U;
}

void bq76952_setCellBalanceMask(uint16_t logical_cell_mask)
{
    uint16_t bq_vc_mask = 0U;

    for (byte cell = 0U; cell < 10U; ++cell) {
        if ((logical_cell_mask & (uint16_t)(1U << cell)) != 0U) {
            bq_vc_mask |= (uint16_t)(1U << cell);
        }
    }

    bq76952_subCommandWithU16Data(SUBCMD_CB_ACTIVE_CELLS, bq_vc_mask);
}

uint16_t bq76952_getCellBalanceActiveMask(void)
{
    bq76952_subCommand(SUBCMD_CB_ACTIVE_CELLS);
    HAL_Delay(1U);
    return (uint16_t)bq76952_subCommandResponseInt(0U);
}

uint16_t bq76952_getCellBalanceActiveSeconds(void)
{
    bq76952_subCommand(SUBCMD_CBSTATUS1);
    HAL_Delay(1U);
    return (uint16_t)bq76952_subCommandResponseInt(0U);
}

bool bq76952_setCellOvervoltageProtection(unsigned int mv, unsigned int ms)
{
    /* Cell OV threshold trong data memory dùng bước ~50.6 mV/LSB.
     * Delay dùng bước ~3.3 ms và mã hóa theo công thức datasheet: code = time/3.3 - 2.
     */
    byte thresh = (byte)(((uint32_t)mv * 10U) / 506U);
    uint16_t dly = (uint16_t)(((uint32_t)ms * 10U) / 33U) - 2U;
    uint8_t status = 1U;

    /* 86 ~ 4.35 V, 74 ~ khoảng 250 ms: đây là fallback an toàn của thư viện. */
    if (thresh < 20U || thresh > 110U) {
        thresh = 86U;
    }
    if (dly < 1U || dly > 2047U) {
        dly = 74U;
    }

    status &= (uint8_t)bq76952_writeDataMemory(0x9278U, thresh, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9279U, (int16_t)dly, 2U);
    return status != 0U;
}

bool bq76952_setCellUndervoltageProtection(unsigned int mv, unsigned int ms)
{
    /* UV cũng dùng bước ~50.6 mV/LSB và delay cùng công thức với OV. */
    byte thresh = (byte)(((uint32_t)mv * 10U) / 506U);
    uint16_t dly = (uint16_t)(((uint32_t)ms * 10U) / 33U) - 2U;
    uint8_t status = 1U;

    if (thresh < 20U || thresh > 90U) {
        thresh = 50U;
    }
    if (dly < 1U || dly > 2047U) {
        dly = 74U;
    }

    status &= (uint8_t)bq76952_writeDataMemory(0x9275U, thresh, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9276U, (int16_t)dly, 2U);
    return status != 0U;
}

bool bq76952_setShortCircuitThreshold(void)
{
    /* Preset hiện tại:
     * threshold code = 2 và delay code = 30.
     * Ý nghĩa vật lý phụ thuộc Rsense và bảng mã SCD trong datasheet.
     */
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(SCD_THRESHOLD_CONFIG, 2, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(SCD_DELAY_CONFIG, 30, 1U);
    return status != 0U;
}

bool bq76952_setProtectionConfiguration(void)
{
    /* 0x0600 là bitmask cấu hình cách protection phản ứng/latch theo lựa chọn của dự án. */
    return bq76952_writeDataMemory(PROTECTION_CONFIGURATION, 0x0600, 2U);
}

bool bq76952_setShutdownStackVoltage(unsigned int voltage)
{
    return bq76952_writeDataMemory(SHUTDOWN_STACK_VOLTAGE, (int16_t)voltage, 2U);
}

bool bq76952_setChargingOvercurrentProtection(unsigned int mv, byte ms)
{
    /* COC dùng bước 2 mV/LSB trên shunt; delay cũng dùng khoảng 3.3 ms/LSB. */
    byte thresh = bq76952_ocMvToThresholdCode(mv,
                                              BQ_OCC_THRESHOLD_MIN_CODE,
                                              BQ_OCC_THRESHOLD_MAX_CODE);
    byte dly = (byte)(((uint16_t)ms * 10U) / 33U) - 2U;

    if (dly < 1U || dly > 127U) {
        dly = 4U;
    }

    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(0x9280U, thresh, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9281U, dly, 1U);
    return status != 0U;
}

bool bq76952_setChargingOvercurrentProtection_Recovery(int16_t mA)
{
    return bq76952_writeDataMemory(OCC_RECOVERY_THRESHOLD_CONFIG, mA, 2U);
}

bool bq76952_setProtectionRecoveryTime(byte sec)
{
    return bq76952_writeDataMemory(PROTECTION_RECOVERY_TIME_CONFIG, sec, 1U);
}

bool bq76952_setDischargingOvercurrentProtection(unsigned int mv, byte ms)
{
    /* Ghi cùng ngưỡng cho OCD1 và OCD2, nhưng delay được ghi ở vùng OCD1 delay. */
    byte thresh = bq76952_ocMvToThresholdCode(mv,
                                              BQ_OCD_THRESHOLD_MIN_CODE,
                                              BQ_OCD_THRESHOLD_MAX_CODE);
    byte dly = (byte)(((uint16_t)ms * 10U) / 33U) - 2U;

    if (dly < 1U || dly > 127U) {
        dly = 1U;
    }

    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(0x9282U, thresh, 1U);
    HAL_Delay(2U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9284U, thresh, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9283U, dly, 1U);
    return status != 0U;
}

bool bq76952_setDischargingOvercurrentProtection_OCD3(int16_t mA)
{
    /* OCD3 dùng giá trị dòng theo đơn vị và dấu được BQ76952 định nghĩa trong data memory. */
    return bq76952_writeDataMemory(0x928AU, mA, 2U);
}

bool bq76952_setDischargingOvercurrentProtection_Recovery(int16_t mA)
{
    return bq76952_writeDataMemory(0x928DU, mA, 2U);
}

bool bq76952_setDischargingShortcircuitProtection(bq76952_scd_thresh_t thresh, unsigned int us)
{
    /* Delay SCD xấp xỉ 15 us/LSB, thư viện cộng 1 để tránh code 0. */
    byte dly = (byte)(us / 15U) + 1U;

    if (dly < 1U || dly > 31U) {
        dly = 2U;
    }

    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(0x9286U, (int16_t)thresh, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x9287U, dly, 1U);
    return status != 0U;
}

bool bq76952_setChargingTemperatureMaxLimit(int temp, byte sec)
{
    /* Giới hạn temp lưu trực tiếp theo độ C nguyên; sec là thời gian xác nhận lỗi. */
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(0x929AU, bq76952_clampTemperatureLimit(temp, 55), 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x929BU, sec, 1U);
    return status != 0U;
}

bool bq76952_setDischargingTemperatureMaxLimit(int temp, byte sec)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(0x929DU, bq76952_clampTemperatureLimit(temp, 60), 1U);
    status &= (uint8_t)bq76952_writeDataMemory(0x929EU, sec, 1U);
    return status != 0U;
}

bool bq76952_setChargingTemperatureMinLimit(int threshold, int recovery, byte sec)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(UTC_THRESHOLD_CONFIG,
                                               bq76952_clampTemperatureLimit(threshold, 0),
                                               1U);
    status &= (uint8_t)bq76952_writeDataMemory(UTC_DELAY_CONFIG, sec, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(UTC_RECOVERY_CONFIG,
                                               bq76952_clampTemperatureLimit(recovery, 5),
                                               1U);
    return status != 0U;
}

bool bq76952_setDischargingTemperatureMinLimit(int threshold, int recovery, byte sec)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(UTD_THRESHOLD_CONFIG,
                                               bq76952_clampTemperatureLimit(threshold, 0),
                                               1U);
    status &= (uint8_t)bq76952_writeDataMemory(UTD_DELAY_CONFIG, sec, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(UTD_RECOVERY_CONFIG,
                                               bq76952_clampTemperatureLimit(recovery, 5),
                                               1U);
    return status != 0U;
}

bool bq76952_setEnablePreRegulator(void)
{
    /* REG0 bit0 = bật pre-regulator. */
    return bq76952_writeDataMemory(REG0_CONFIG, 0x01, 1U);
}

bool bq76952_configurePowerOutputs(void)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_setEnablePreRegulator();
    status &= (uint8_t)bq76952_setEnableRegulator(true, true);
    return status != 0U;
}
#if BQ76952_LOW_POWER_MODE == BQ76952_LOW_POWER_MODE_SLEEP
bool bq76952_prepareSleepWithReg2(void)
{
    bq76952_battery_status_t batt_status = {0};

    bq76952_applyReg12Control(true, true);

    for (uint8_t count = 0U; count < BQ_SLEEP_ENTRY_TRIES; count++) {
        bq76952_subCommand(SUBCMD_SLEEP_ENABLE);
        HAL_Delay(BQ_SLEEP_ENTRY_POLL_MS);
        
        batt_status = bq76952_getBatteryStatusRegister();
        if (batt_status.bits.SLEEP_MODE != 0U) {
            break;
        }
    }
    return batt_status.bits.SLEEP_MODE != 0U;
}

void bq76952_resumeFromSleep(void)
{
    bq76952_subCommand(SUBCMD_SLEEP_DISABLE);
    HAL_Delay(1U);
    bq76952_applyReg12Control(true, true);
}

bool bq76952_configureSleepWake(void)
{
    uint8_t status = 1U;
    uint16_t power_config;

    power_config = (uint16_t)bq76952_readDataMemory(POWER_CONFIG, 2);
    if (power_config == 0U) {
        return false;
    }

    /* Wake comparator chay cham nhat de giam nhieu; 500 mA la nguong min cua BQ76952. */
    power_config &= (uint16_t)~BQ_POWER_CONFIG_WK_SPD_MASK;
    status &= (uint8_t)bq76952_writeDataMemory(POWER_CONFIG, (int16_t)power_config, 2U);
    status &= (uint8_t)bq76952_writeDataMemory(SLEEP_WAKE_COMPARATOR_CURRENT,
                                               BQ_SLEEP_WAKE_COMPARATOR_CURRENT_MA,
                                               2U);
    return status != 0U;
}
#elif BQ76952_LOW_POWER_MODE == BQ76952_LOW_POWER_MODE_DEEPSLEEP
bool bq76952_prepareSleepWithReg2(void)
{
    unsigned int control_status = 0U;

    bq76952_applyReg12Control(true, true);
    bq76952_subCommand(SUBCMD_DEEPSLEEP);
    HAL_Delay(5U);
    bq76952_subCommand(SUBCMD_DEEPSLEEP);
    for (uint8_t count = 0U; count < 10U; count++) {
        HAL_Delay(100U);
        control_status = bq76952_directCommand(CMD_DIR_CONTROL_STATUS);
        if ((control_status & BQ_CONTROL_STATUS_DEEPSLEEP) != 0U) {
            break;
        }
    }
    return (control_status & BQ_CONTROL_STATUS_DEEPSLEEP) != 0U;
}

void bq76952_resumeFromSleep(void)
{
    bq76952_subCommand(SUBCMD_EXIT_DEEPSLEEP);
    HAL_Delay(300U);
    bq76952_applyReg12Control(true, true);
}

bool bq76952_configureSleepWake(void)
{
    uint8_t status = 1U;
    uint16_t power_config;

    power_config = (uint16_t)bq76952_readDataMemory(POWER_CONFIG, 2);
    if (power_config == 0U) {
        return false;
    }

    /* Wake comparator chay cham nhat de giam nhieu; 500 mA la nguong min cua BQ76952. */
    power_config &= (uint16_t)~BQ_POWER_CONFIG_WK_SPD_MASK;
    power_config |= BQ_POWER_CONFIG_DPSLP_PD;
    power_config |= BQ_POWER_CONFIG_DPSLP_LDO;
    status &= (uint8_t)bq76952_writeDataMemory(POWER_CONFIG, (int16_t)power_config, 2U);
    status &= (uint8_t)bq76952_writeDataMemory(BQ_LOW_V_SHUTDOWN_DELAY, 1, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(SLEEP_WAKE_COMPARATOR_CURRENT,
                                               BQ_SLEEP_WAKE_COMPARATOR_CURRENT_MA,
                                               2U);
    return status != 0U;
}
#elif BQ76952_LOW_POWER_MODE == BQ76952_LOW_POWER_MODE_SHUTDOWN
bool bq76952_prepareSleepWithReg2(void)
{
    bool send_second_shutdown = bq76952_shutdownRequiresDoubleCommand();

    bq76952_setFET(ALL, OFF);
    HAL_Delay(2U);

    bq76952_subCommand(SUBCMD_SHUTDOWN);
    HAL_Delay(BQ_SHUTDOWN_SUBCOMMAND_GAP_MS);

    if (send_second_shutdown) {
        bq76952_subCommand(SUBCMD_SHUTDOWN);
        HAL_Delay(BQ_SHUTDOWN_SUBCOMMAND_GAP_MS);
    }

    return bq76952_waitShutdownSequenceStarted(BQ_SHUTDOWN_SEQUENCE_TIMEOUT_MS);
}

void bq76952_resumeFromSleep(void)
{
    uint32_t start = HAL_GetTick();

    bq76952_begin();
    do {
        if (bq76952_isConnected()) {
            bq76952_applyReg12Control(true, true);
            return;
        }
        HAL_Delay(BQ_SHUTDOWN_BOOT_POLL_MS);
    } while ((HAL_GetTick() - start) < BQ_SHUTDOWN_BOOT_TIMEOUT_MS);
}

bool bq76952_configureSleepWake(void)
{
    uint8_t status = 1U;

    /* In SHUTDOWN the BQ loses RAM settings and I2C stops. Wake must be done by
     * TS2, LD/charger, or RST_SHUT hardware; firmware can only start the sequence.
     * This board schematic shows TS2 connected to RT2, which can prevent true
     * lowest-current shutdown if that NTC pulls TS2 below the wake threshold.
     */
    status &= (uint8_t)bq76952_writeDataMemory(BQ_SHUTDOWN_FET_OFF_DELAY,
                                               BQ_SHUTDOWN_FET_OFF_DELAY_NONE,
                                               1U);
    status &= (uint8_t)bq76952_writeDataMemory(BQ_SHUTDOWN_COMMAND_DELAY,
                                               BQ_SHUTDOWN_COMMAND_DELAY_1S,
                                               1U);
    status &= (uint8_t)bq76952_writeDataMemory(BQ_LOW_V_SHUTDOWN_DELAY, 1, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(BQ_SHUTDOWN_AUTO_TIME,
                                               BQ_SHUTDOWN_AUTO_TIME_DISABLED,
                                               1U);
    return status != 0U;
}
#endif

bool bq76952_setDA_Config(void)
{
    /* USER_VOLTS_CV=1 để Stack/PACK/LD không tràn signed 16-bit trên pack 10S;
     * USER_AMPS=1 giữ đơn vị dòng là mA.
     */
    return bq76952_writeDataMemory(DA_CONFIGURATION, BQ_DA_CONFIG_DEFAULT, 1U);
}

bool bq76952_setCurrentSenseCalibration(uint32_t gain_ppm)
{
    uint8_t status = 1U;
    uint32_t cc_gain_raw;
    uint32_t capacity_gain_raw;

    cc_gain_raw = bq76952_scaleIeee754RawByPpm(BQ_CC_GAIN_DEFAULT_RAW, gain_ppm);
    capacity_gain_raw = bq76952_scaleIeee754RawByPpm(BQ_CAPACITY_GAIN_DEFAULT_RAW, gain_ppm);

    status &= (uint8_t)bq76952_writeU32DataMemory(CC_GAIN, cc_gain_raw);
    status &= (uint8_t)bq76952_writeU32DataMemory(CAPACITY_GAIN, capacity_gain_raw);
    return status != 0U;
}

bool bq76952_setSF_AlertMask_A(void)
{
    return bq76952_writeDataMemory(SF_ALERT_MASK_A, 0x00, 1U);
}

bool bq76952_setSF_AlertMask_B(void)
{
    return bq76952_writeDataMemory(SF_ALERT_MASK_B, 0x00, 1U);
}

bool bq76952_setSF_AlertMask_C(void)
{
    return bq76952_writeDataMemory(SF_ALERT_MASK_C, 0x00, 1U);
}

bool bq76952_setEnableRegulator(bool enable_reg1, bool enable_reg2)
{
    byte reg12 = bq76952_makeReg12Control(enable_reg1, enable_reg2);
    bool status = bq76952_writeDataMemory(REG12_CONTROL, reg12, 1U);

    bq76952_applyReg12Control(enable_reg1, enable_reg2);
    return status;
}

bool bq76952_setAlertPinConfig(void)
{
    /* Board dùng pull-up ALERT, nên cấu hình open-drain để BQ kéo thấp khi có alarm. */
    return bq76952_writeDataMemory(ALERT_PIN_CONFIG, BQ_ALERT_PIN_CONFIG_OPEN_DRAIN, 1U);
}

bool bq76952_setDFETOFFPinConfig(bool both_off_mode, bool active_low)
{
    byte cfg = bq76952_make_pin_config(0x02U, active_low);

    if (both_off_mode) {
        cfg |= 0x10U;
    }
    return bq76952_writeDataMemory(DFETOFF_PIN_CONFIG, cfg, 1U);
}

bool bq76952_setDCHGPinConfig(bool active_low)
{
    return bq76952_writeDataMemory(DCHG_PIN_CONFIG, bq76952_make_pin_config(0x02U, active_low), 1U);
}

bool bq76952_setDDSGPinConfig(bool active_low)
{
    return bq76952_writeDataMemory(DDSG_PIN_CONFIG, bq76952_make_pin_config(0x02U, active_low), 1U);
}

bool bq76952_setDefaultAlarmMaskConfig(void)
{
    bool status;

    /* Keep default safety/PF alarm bits and add WAKE so ALERT toggles when BQ exits SLEEP. */
    status = bq76952_writeDataMemory(DEFAULT_ALARM_MASK_CONFIG,
                                     (int16_t)BQ_ALARM_MASK_WITH_WAKE,
                                     2U);
    if (status) {
        status = bq76952_setAlarmEnableRegister(BQ_ALARM_MASK_WITH_WAKE);
    }
    return status;
}



bool bq76952_setVcellMode(uint16_t vcell_mode)
{
    /* Mỗi bit tương ứng một kênh VCx; bit = 1 nghĩa là tham gia đo điện áp cell. */
    return bq76952_writeDataMemory(VCELL_MODE, (int16_t)vcell_mode, 2U);
}

bool bq76952_setEnableCHG_FET_Protection(void)
{
    uint8_t status = 1U;

    status &= (uint8_t)bq76952_writeDataMemory(CHG_FET_PROTECTION_A,
                                               BQ_CHG_FET_PROTECTION_A_OCC,
                                               1U);
    status &= (uint8_t)bq76952_writeDataMemory(CHG_FET_PROTECTION_B, 0x00, 1U);
    status &= (uint8_t)bq76952_writeDataMemory(CHG_FET_PROTECTION_C, 0x00, 1U);
    return status != 0U;
}

bool bq76952_setEnableProtectionsA(void)
{
    /* 0xFC bật hầu hết protection nhóm A, trừ các bit thấp không dùng. */
    return bq76952_writeDataMemory(ENABLE_PROTECTIONS_A, 0xFC, 1U);
}

bool bq76952_setEnableProtectionsB(void)
{
    return bq76952_writeDataMemory(ENABLE_PROTECTIONS_B, 0xF7, 1U);
}

bool bq76952_setEnableProtectionsC(void)
{
    /* 0x80 hiện chỉ bật protection bit cao của nhóm C. */
    return bq76952_writeDataMemory(ENABLE_PROTECTIONS_C, 0x80, 1U);
}

bool bq76952_setCHGFETProtectionsA(byte val)
{
    return bq76952_writeDataMemory(CHG_FET_PROTECTIONS_A, val, 1U);
}

bool bq76952_setCellInterconnectResistances(void)
{
    /* Ghi tuần tự 16 mục vì BQ76952 cho phép hiệu chỉnh riêng từng liên kết cell sense. */
    for (byte cell = 0U; cell < 16U; ++cell) {
        if (!bq76952_writeDataMemory(CELL_INTERCONNECT_RESISTANCE + ((unsigned int)cell * 2U),
                                     CELL_INTERCONNECT_RESISTANCE_MOHM,
                                     2U)) {
            return false;
        }
    }
    return true;
}

bool bq76952_setDSGFETProtectionsA(void)
{
    /* 0xA4 là preset mapping fault -> DSG FET action của dự án. */
    return bq76952_writeDataMemory(DSG_FET_PROTECTION_A, 0xA4, 1U);
}

bool bq76952_setDSGFETProtectionsB(void)
{
    return bq76952_writeDataMemory(DSG_FET_PROTECTION_B, 0x00, 1U);
}

bool bq76952_setDSGFETProtectionsC(void)
{
    return bq76952_writeDataMemory(DSG_FET_PROTECTION_C, 0x80, 1U);
}

bool bq76952_setFET_Options(void)
{
    /* 0x1D cấu hình hành vi FET/precharge/predischarge theo thiết kế board hiện tại. */
    return bq76952_writeDataMemory(FET_OPTIONS, 0x1F, 1U);
}

bool bq76952_setFET_PredischargeTimeout(void)
{
    return bq76952_writeDataMemory(FET_PREDISCHARGE_TIMEOUT, 0x00, 1U);
}

bool bq76952_setFET_PredischargeStopDelta(void)
{
    /* 100 là delta điện áp kết thúc predischarge theo đơn vị mã hóa của BQ76952. */
    return bq76952_writeDataMemory(FET_PREDISCHARGE_STOP_DELTA, 100, 1U);
}

bool bq76952_setEnableTS1(void)
{
    return bq76952_writeDataMemory(TS1_CONFIG, 0x07, 1U);
}

bool bq76952_setEnableTS2(void)
{
    return bq76952_writeDataMemory(TS2_CONFIG, 0x07, 1U);
}

bool bq76952_setEnableTS3(void)
{
    return bq76952_writeDataMemory(TS3_CONFIG, 0x07, 1U);
}

unsigned int bq76952_getAlertStatusRegister(void)
{
    return bq76952_directCommand(CMD_DIR_ALARM_STATUS);
}

unsigned int bq76952_getAlertRawStatusRegister(void)
{
    return bq76952_directCommand(CMD_DIR_ALARM_RAW_STATUS);
}

unsigned int bq76952_getAlarmEnableRegister(void)
{
    return bq76952_directCommand(CMD_DIR_ALARM_ENABLE);
}

bool bq76952_setAlarmEnableRegister(uint16_t mask)
{
    return bq76952_writeDirectCommand(CMD_DIR_ALARM_ENABLE, mask);
}

bool bq76952_clearAlertStatusRegister(uint16_t mask)
{
    if (mask == 0U) {
        return true;
    }
    return bq76952_writeDirectCommand(CMD_DIR_ALARM_STATUS, mask);
}

byte bq76952_HandleAlarm(void)
{
    return (byte)bq76952_getAlertStatusRegister();
}

void bq76952_init(void)
{
    unsigned int hwVersion;
    unsigned int devNumber;

    bq76952_begin();

    /* Chờ IC hoàn tất power-up trước khi đọc metadata và ghi cấu hình. */
    HAL_Delay(1000U);

    devNumber = bq76952_getDeviceNumber();
    hwVersion = bq76952_getHWVersion();
    (void)devNumber;
    (void)hwVersion;

    /* Đọc raw alert một lần để đồng bộ trạng thái ban đầu. */
    (void)bq76952_getAlertRawStatusRegister();
}

void bq76952_handle_alarm(void)
{
    (void)bq76952_HandleAlarm();
}

// void bq76952_check_batt_status(void)
// {
//     int cellArray[16];
//     bool isDischarging;
//     bool isCharging;
//     bq76952_battery_status_t batt_status;
//     unsigned int alarmStatus;
//     unsigned int cov[16] = {0};
//     unsigned int manufacturing_status;
//     int current_now;
//     bq76952_temp_t temperature_status;
//     int16_t internal_temp;
//     unsigned int stack_voltage;
//     int cell_voltage;
//     unsigned int alert_raw;
//     bq76952_protection_t status;
//     bq76952_safety_alert_c_t safety_alert_c;

//     /* Hàm này chủ yếu phục vụ debug/runtime inspection, chưa có logic xử lý fault hoàn chỉnh. */
//     HAL_Delay(50U);
//     bq76952_getAllCellVoltages(cellArray, 10);
//     alarmStatus = bq76952_getAlertStatusRegister();
//     isDischarging = bq76952_isDischarging();
//     isCharging = bq76952_isCharging();
//     batt_status = bq76952_getBatteryStatusRegister();

//     if (batt_status.bits.FULL_RESET_OCCURED) {
//     }

//     (void)bq76952_readDataMemory(0x9236U, 1);
//     (void)bq76952_readDataMemory(0x9286U, 1);

//     manufacturing_status = bq76952_getManufacturingStatus();
//     current_now = bq76952_getCurrentNow();
//     temperature_status = bq76952_getTemperatureStatus();
//     internal_temp = bq76952_getInternalTemp();
//     stack_voltage = bq76952_getStackVoltage();
//     cell_voltage = bq76952_getCellVoltage(15U);
//     alert_raw = bq76952_getAlertRawStatusRegister();
//     status = bq76952_getProtectionStatus();
//     safety_alert_c = bq76952_getSafetyAlert_C();

//     (void)alarmStatus;
//     (void)isDischarging;
//     (void)isCharging;
//     (void)manufacturing_status;
//     (void)current_now;
//     (void)temperature_status;
//     (void)internal_temp;
//     (void)stack_voltage;
//     (void)cell_voltage;
//     (void)alert_raw;
//     (void)safety_alert_c;

//     if (status.bits.CELL_OV) {
//         for (int i = 0; i < 16; ++i) {
//             cov[i] = bq76952_getCOVSnapshot((byte)i);
//         }
//     }

//     if (status.bits.SC_DCHG) {
//     }

//     (void)cov;
// }
