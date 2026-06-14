#include "bms.h"
#include "main.h"
#include "adc.h"
#include "MyDrivers/power/power_manager.h"

#include <limits.h>
#include <stddef.h>

#include "debug_log.h"
#include "bms_uart.h"
#include "storage_flash.h"

#define BMS_ALERT_POLL_FALLBACK_MS              1000U
#define BMS_SHUT_HOLD_MS                        1100U
#define BMS_BAT_ADC_SAMPLE_MS                   1000U
#define BMS_BAT_ADC_SETTLE_MS                   5U
#define BMS_FET_STATUS_SETTLE_MS                2U
#define BMS_BQ_FET_STAT_CHG_FET                 0x01U
#define BMS_BQ_FET_STAT_DSG_FET                 0x04U
#define BMS_BQ_ALARM_RAW_XCHG                   0x0040U
#define BMS_BQ_ALARM_RAW_XDSG                   0x0020U
#define BMS_ALERT_ACTIVE_LEVEL                  GPIO_PIN_RESET
#define BMS_BQ_OCC_RECOVERY_THRESHOLD_MA        0
#define BMS_BQ_RECOVERY_TIME_SEC ((BMS_OVER_CURRENT_RECOVERY_DELAY_MS + 999UL) / 1000UL)
#define BMS_BAT_ADC_REF_MV                      3300UL
#define BMS_BAT_ADC_COUNTS                      4095UL
#define BMS_BAT_ADC_DIVIDER_NUM                 678300UL
#define BMS_BAT_ADC_DIVIDER_DEN                 13300UL
/* Calibration for BAT_ADC divider tolerance.
 * Latest log: cell-sum pack ~= 37985 mV while ADC estimate ~= 39780 mV.
 */
#define BMS_BAT_ADC_CAL_NUM                     955UL
#define BMS_BAT_ADC_CAL_DEN                     1000UL

#define BMS_BQ_CONFIG_STEP(ok_var, expr)        ((ok_var) &= (uint8_t)(expr))
#define BMS_CURRENT_TO_SENSE_MV(current_mA)     ((((uint32_t)(current_mA) * BMS_BQ_SENSE_RESISTOR_UOHM) + 999999UL) / 1000000UL)
#define BMS_BQ_OC_COMPARATOR_MIN_MV             4UL
#define BMS_CLAMP_BQ_OC_MV(mv)                  (((mv) < BMS_BQ_OC_COMPARATOR_MIN_MV) ? BMS_BQ_OC_COMPARATOR_MIN_MV : (mv))


static BMS_Tracking_t g_bms_tracking;
static uint32_t g_last_update_tick;


static volatile bool g_alert_irq_pending;
static volatile uint32_t g_alert_irq_counter;
// static volatile bool g_dchg_signal_active;
// static volatile bool g_ddsg_signal_active;


static bool g_shutdown_pulse_active;
static uint32_t g_shutdown_pulse_tick;



static void BMS_ResetTracking(void);
static void BMS_ConfigureMonitor(void);
static void BMS_ConfigureHardwarePins(void);
static void BMS_ReadMeasurements(BMS_Tracking_t *tracking, uint32_t Now);
static void BMS_HandleHardwareSignals(BMS_Tracking_t *tracking, uint32_t now);
static void BMS_UpdateBatteryAdc(BMS_Tracking_t *tracking, uint32_t now);
static void BMS_SetFetoff(bool asserted);
static void BMS_SetBatSenseEnable(bool enabled);
static void BMS_UpdateCellStatistics(BMS_Tracking_t *tracking);
static void BMS_UpdateCurrentDirection(BMS_Tracking_t *tracking);
static void BMS_UpdateFetStatus(BMS_Tracking_t *tracking);
static void BMS_UpdateBqAlarmRawStatus(BMS_Tracking_t *tracking);
static void BMS_UpdateFaultFlags(BMS_Tracking_t *tracking, uint32_t now);
static void BMS_MergeBQFaultFlags(BMS_Tracking_t *tracking);
static void BMS_UpdateState(BMS_Tracking_t *tracking);
static void BMS_ApplyFetPolicy(BMS_Tracking_t *tracking);
static void BMS_SyncStateWithFetAvailability(BMS_Tracking_t *tracking);
static void BMS_UpdateCoulombCounter(BMS_Tracking_t *tracking, uint32_t dt_ms);
static void BMS_UpdateBalancing(BMS_Tracking_t *tracking, uint32_t now);
static void BMS_LoadPersistedData(BMS_Tracking_t *tracking);
static bool BMS_SaveCurrentCalibration(uint32_t gain_ppm);
static void BMS_SavePersistedDataIfNeeded(const BMS_Tracking_t *tracking, uint32_t now);
static bool BMS_IsAlertPinActive(void);
#if BMS_DEBUG_LOG_ENABLE
const char *BMS_StateName(BMS_State_t state);
#endif
static bool BMS_AllCellsAtOrBelow(const BMS_Tracking_t *tracking, uint16_t threshold_mV);
static bool BMS_AllCellsAtOrAbove(const BMS_Tracking_t *tracking, uint16_t threshold_mV);
static bool BMS_AllTemperaturesAtOrBelow(const BMS_Tracking_t *tracking, int16_t threshold_C);
static bool BMS_AllTemperaturesAtOrAbove(const BMS_Tracking_t *tracking, int16_t threshold_C);
static int32_t BMS_AbsCurrent(int32_t current_mA);
static uint32_t BMS_AbsCurrentU32(int32_t current_mA);

void BMS_Init(void)
{
    BMS_ResetTracking();
    BMS_ConfigureHardwarePins();
    BMS_LOG_INFO("bms init");
    bq76952_init();

    g_bms_tracking.connected = bq76952_isConnected();
    g_bms_tracking.initialized = g_bms_tracking.connected;

    if (!g_bms_tracking.connected) {
        BMS_LOG_ERROR("bq76952 not connected");
        BMS_Error_Handler();
        return;
    }

    BMS_LoadPersistedData(&g_bms_tracking);
    BMS_ConfigureMonitor();
    g_last_update_tick = HAL_GetTick();
    // g_last_alert_service_tick = g_last_update_tick;
    // g_last_bat_adc_sample_tick = g_last_update_tick;

    BMS_Update();
}

void BMS_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - g_last_update_tick;

    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    g_last_update_tick = now;
    // BMS_UpdateShutdownPulse(now); // tạm thời k dùng chế độ shutdown

    g_bms_tracking.connected = bq76952_isConnected();
    if (!g_bms_tracking.connected) {
        if (!g_bms_tracking.faults.communicationFault) {
            bms_uart_send_protection_reason(BMS_UART_PROTECT_COMMUNICATION);
        }
        g_bms_tracking.faults.communicationFault = true;
        BMS_LOG_ERROR("bq76952 communication fault");
        BMS_Error_Handler();
        return;
    }

    BMS_ReadMeasurements(&g_bms_tracking, now);
    BMS_HandleHardwareSignals(&g_bms_tracking, now);
    BMS_UpdateBatteryAdc(&g_bms_tracking, now);
    BMS_UpdateCellStatistics(&g_bms_tracking); // Cập nhật min/max/average/delta cell voltage và stack voltage
    BMS_UpdateCurrentDirection(&g_bms_tracking); // Cập nhật chiều dòng điện dựa trên giá trị current_mA
    BMS_UpdateFaultFlags(&g_bms_tracking, now); // Cập nhật các cờ lỗi dựa trên ngưỡng điện áp, nhiệt độ, dòng điện
    BMS_MergeBQFaultFlags(&g_bms_tracking); // Kết hợp các cờ lỗi từ BQ76952 vào tracking
    BMS_UpdateCoulombCounter(&g_bms_tracking, dt_ms); // Cập nhật tích trữ mAs và throughput mAh dựa trên current và dt
    BMS_UpdateState(&g_bms_tracking); // Cập nhật trạng thái BMS dựa trên các cờ lỗi và điều kiện hoạt động
    BMS_ApplyFetPolicy(&g_bms_tracking); // Điều khiển FET sạc/xả dựa trên trạng thái và cờ lỗi
    BMS_UpdateBalancing(&g_bms_tracking, now); // Cập nhật trạng thái cân bằng cell và mask dựa trên delta cell voltage và ngưỡng
    BMS_SavePersistedDataIfNeeded(&g_bms_tracking, now);

    g_bms_tracking.circle_counter++;
    g_bms_tracking.initialized = true;
}

const BMS_Tracking_t *BMS_GetTracking(void)
{
    return &g_bms_tracking;
}

bool BMS_IsFaultActive(void)
{
    return g_bms_tracking.faults.cellOverVoltage ||
           g_bms_tracking.faults.cellUnderVoltage ||
           g_bms_tracking.faults.chargeOverTemperature ||
           g_bms_tracking.faults.dischargeOverTemperature ||
           g_bms_tracking.faults.underTemperature ||
           g_bms_tracking.faults.chargeOverCurrent ||
           g_bms_tracking.faults.dischargeOverCurrent ||
           g_bms_tracking.faults.shortCircuit ||
           g_bms_tracking.faults.bqSafetyFault ||
           g_bms_tracking.faults.communicationFault;
}

uint8_t BMS_CalibrateCurrent(int32_t actual_mA, BMS_CurrentCalibrationResult_t *result)
{
    BMS_CurrentCalibStatus_t Status;
    BMS_CurrentCalibrationResult_t local = {0};
    uint32_t actual_abs;
    uint32_t measured_abs;
    uint32_t diff_abs;
    uint64_t numerator;

    measured_abs = (int32_t)bq76952_getCurrentAvg();
    if (g_bms_tracking.currentCalibrationGainPpm == 0UL) {
        g_bms_tracking.currentCalibrationGainPpm = BMS_CURRENT_CALIBRATION_DEFAULT_PPM;
    }
    local.newGain_ppm = g_bms_tracking.currentCalibrationGainPpm;

    actual_abs = BMS_AbsCurrent(actual_mA);
    measured_abs = BMS_AbsCurrent(measured_abs);

    if (actual_abs == 0UL) {
        Status = BMS_CURRENT_CALIBRATION_BAD_INPUT;
    } else if (measured_abs == 0UL) {
        Status = BMS_CURRENT_CALIBRATION_ZERO_READING;
    } else {
        diff_abs = (actual_abs > measured_abs) ?
                   (actual_abs - measured_abs) :
                   (measured_abs - actual_abs);
        local.deviation_ppm = (uint32_t)((((uint64_t)diff_abs * 1000000ULL) +
                                          ((uint64_t)actual_abs / 2ULL)) /
                                         (uint64_t)actual_abs);

        if (local.deviation_ppm > BMS_CURRENT_CALIBRATION_MAX_DEVIATION_PPM) {
            Status = BMS_CURRENT_CALIBRATION_DEVIATION_TOO_HIGH;
        } else {
            numerator = ((uint64_t)g_bms_tracking.currentCalibrationGainPpm * (uint64_t)actual_abs) +
                        ((uint64_t)measured_abs / 2ULL);
            local.newGain_ppm = (uint32_t)(numerator / (uint64_t)measured_abs);
            if ((local.newGain_ppm == 0UL) ||
                !bq76952_setCurrentSenseCalibration(local.newGain_ppm) ||
                !BMS_SaveCurrentCalibration(local.newGain_ppm)) {
                Status = BMS_CURRENT_CALIBRATION_WRITE_FAILED;
            } else {
                Status = BMS_CURRENT_CALIBRATION_OK;
            }
        }
    }

    if (result != NULL) {
        *result = local;
    }

    BMS_LOG_INFO("current cal status=%u actual=%ld measured=%ld dev=%lu old=%lu new=%lu",
                 (unsigned int)Status,
                 (long)actual_abs,
                 (long)measured_abs,
                 (unsigned long)local.deviation_ppm,
                 (unsigned long)g_bms_tracking.currentCalibrationGainPpm,
                 (unsigned long)local.newGain_ppm);

    return (uint8_t)Status;
}

void BMS_Error_Handler(void)
{
    BMS_LOG_ERROR("bms error handler");
    BMS_ResetTracking();
    // g_bms_tracking.connected = false;
    // g_bms_tracking.fetsEnabled = false;
    // g_bms_tracking.chargeFetEnabled = false;
    // g_bms_tracking.dischargeFetEnabled = false;
    // g_bms_tracking.charging = false;
    // g_bms_tracking.discharging = false;
    // g_bms_tracking.chargeDisabled = true;
    // g_bms_tracking.dischargeDisabled = true;
    // g_bms_tracking.state = BMS_STATE_FAULT;
    // g_bms_tracking.bqChargeFetBlocked = false;
    // g_bms_tracking.bqDischargeFetBlocked = false;
    // g_bms_tracking.bqAlarmRawStatus.raw = 0U;
    // g_bms_tracking.balanceMask = 0U;
    // g_bms_tracking.balanceRequired = false;
    // g_bms_tracking.fetOffAsserted = true;
    // g_bms_tracking.batSenseEnabled = false;
    bq76952_setCellBalanceMask(0U);
    bq76952_setFET(ALL, OFF);
    BMS_SetFetoff(true);
    BMS_SetBatSenseEnable(false);
}

void BMS_NotifyAlertInterrupt(void)
{
    g_alert_irq_pending = true;
    g_alert_irq_counter++;
}

void BMS_RequestShutdown(void)
{
    if (g_shutdown_pulse_active) {
        return;
    }
    HAL_GPIO_WritePin(SHUT_GPIO_Port, SHUT_Pin, GPIO_PIN_SET);
    g_shutdown_pulse_tick = HAL_GetTick();
    g_shutdown_pulse_active = true;
    BMS_LOG_WARN("shutdown pulse start");
}

static void BMS_ResetTracking(void)
{
    /* Reset all cell voltages */
    uint8_t *Tracking_Ptr = (uint8_t *)&g_bms_tracking;
    for(uint8_t i = 0; i < sizeof(BMS_Tracking_t); i ++)
    {
        Tracking_Ptr[i] = 0;
    }
    g_bms_tracking.chargeDisabled           = true;
    g_bms_tracking.dischargeDisabled        = true;
    g_bms_tracking.currentCalibrationGainPpm    = BMS_CURRENT_CALIBRATION_DEFAULT_PPM;

    // for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
    //     g_bms_tracking.cellVoltages.cellNum[i] = 0U;
    //     g_bms_tracking.cellVoltages.RealTimeAccumulated[i] = 0;
    //     g_bms_tracking.cellVoltages.IndexAccumulated[i] = 0;
    // }

    // for (uint8_t i = 0U; i < BMS_NUMBER_OF_THERMISTORS; ++i) {
    //     g_bms_tracking.temperature[i] = 0;
    // }
    
    // g_bms_tracking.initialized          = false;
    // g_bms_tracking.connected            = false;
    // g_bms_tracking.state                = BMS_STATE_INIT;
    // g_bms_tracking.currentDirection     = BMS_CURRENT_IDLE;
    // g_bms_tracking.circle_counter       = 0U;
    // g_bms_tracking.stackVoltage         = 0U;
    // g_bms_tracking.packVoltage          = 0U;
    // g_bms_tracking.cellVoltages.minCellVoltage       = 0U;
    // g_bms_tracking.cellVoltages.maxCellVoltage       = 0U;
    // g_bms_tracking.cellVoltages.averageCellVoltage   = 0U;
    // g_bms_tracking.cellVoltages.deltaCellVoltage     = 0U;
    // g_bms_tracking.current_mA           = 0;
    // g_bms_tracking.charging             = false;
    // g_bms_tracking.discharging          = false;
    // g_bms_tracking.chargeFetEnabled     = false;
    // g_bms_tracking.dischargeFetEnabled  = false;
    // g_bms_tracking.fetsEnabled          = false;
    // g_bms_tracking.bqChargeFetBlocked   = false;
    // g_bms_tracking.bqDischargeFetBlocked    = false;
    // g_bms_tracking.bqAlarmRawStatus.raw         = 0U;
    // g_bms_tracking.faults                   = (BMS_FaultFlags_t){0};
    // g_bms_tracking.chargeDisabled           = true;
    // g_bms_tracking.dischargeDisabled        = true;
    // g_bms_tracking.chargeGateFaultSignal    = false;
    // g_bms_tracking.dischargeGateFaultSignal = false;
    // g_bms_tracking.fetOffAsserted           = false;
    // g_bms_tracking.alertActive              = false;
    // g_bms_tracking.alertCounter             = 0UL;
    // g_bms_tracking.bqSleepMode              = false;
    // g_bms_tracking.bqSleepAllowed           = false;
    // g_bms_tracking.batSenseEnabled          = false;
    // g_bms_tracking.batAdcEstimatedPack_mV   = 0U;
    // g_bms_tracking.balanceRequired          = false;
    // g_bms_tracking.balanceMask              = 0U;
    // g_bms_tracking.chargeAccumulated_mAs    = 0ULL;
    // g_bms_tracking.dischargeAccumulated_mAs = 0ULL;
    // g_bms_tracking.chargeThroughput_mAh     = 0UL;
    // g_bms_tracking.equivalentCycle_milliCycles  = 0UL;
    // g_bms_tracking.currentCalibrationGainPpm    = BMS_CURRENT_CALIBRATION_DEFAULT_PPM;
}

static void BMS_ConfigureMonitor(void)
{
    uint32_t over_current_sense_mV;
    uint32_t over_current_chargr_mV;
    int16_t discharge_ocd3_threshold_mA;
    uint8_t config_ok = 1U;

    BMS_LOG_INFO("configure bq76952");
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_Enter_FullAccessMode());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_configurePowerOutputs());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setVcellMode(BMS_BQ_VCELL_MODE_10S));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDA_Config());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setCurrentSenseCalibration(
                                  g_bms_tracking.currentCalibrationGainPpm));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableProtectionsA());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableProtectionsB());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableProtectionsC());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setProtectionConfiguration());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableCHG_FET_Protection());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDSGFETProtectionsA());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDSGFETProtectionsB());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDSGFETProtectionsC());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setFET_Options());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setFET_PredischargeTimeout());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setFET_PredischargeStopDelta());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setCellInterconnectResistances());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableTS1());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setEnableTS3());

    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setAlertPinConfig());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDFETOFFPinConfig(true, false));
    // BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDCHGPinConfig(false));
    // BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDDSGPinConfig(false));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDefaultAlarmMaskConfig());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_configureSleepWake());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setSF_AlertMask_A());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setSF_AlertMask_B());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setSF_AlertMask_C());
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setCellOvervoltageProtection(BMS_CELL_OV_CUTOFF_MV_BQ,
                                                                       BMS_BQ_PROTECTION_DELAY_MS));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setCellUndervoltageProtection(BMS_CELL_UV_CUTOFF_MV_BQ,
                                                                        BMS_BQ_PROTECTION_DELAY_MS));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setChargingTemperatureMaxLimit(
                                      BMS_CHARGE_OT_CUTOFF_C,
                                      BMS_BQ_TEMPERATURE_PROTECTION_DELAY_SEC));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingTemperatureMaxLimit(
                                      BMS_DISCHARGE_OT_CUTOFF_C,
                                      BMS_BQ_TEMPERATURE_PROTECTION_DELAY_SEC));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setChargingTemperatureMinLimit(
                                      BMS_UNDERTEMP_CUTOFF_C,
                                      BMS_UNDERTEMP_RECOVER_C,
                                      BMS_BQ_TEMPERATURE_PROTECTION_DELAY_SEC));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingTemperatureMinLimit(
                                      BMS_UNDERTEMP_CUTOFF_C,
                                      BMS_UNDERTEMP_RECOVER_C,
                                      BMS_BQ_TEMPERATURE_PROTECTION_DELAY_SEC));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_ConfigManualCellBalancing(BMS_BALANCE_MIN_TEMP_C,
                                      BMS_BALANCE_MAX_TEMP_C,
                                      BMS_BALANCE_MAX_INTERNAL_TEMP_C,
                                      BMS_BALANCE_INTERVAL_SEC,
                                      BMS_BALANCE_MAX_ACTIVE_CELLS));

    over_current_sense_mV = BMS_CLAMP_BQ_OC_MV(BMS_CURRENT_TO_SENSE_MV(BMS_OVER_CURRENT_DISCHARGE));
    over_current_chargr_mV = BMS_CLAMP_BQ_OC_MV(BMS_CURRENT_TO_SENSE_MV(BMS_OVER_CURRENT_CHARGE));
#if BMS_CURRENT_CHARGE_IS_POSITIVE
    discharge_ocd3_threshold_mA = (BMS_OVER_CURRENT_DISCHARGE > 32767L) ?
                                  -32767 :
                                  (int16_t)(-BMS_OVER_CURRENT_DISCHARGE);
#else
    discharge_ocd3_threshold_mA = (BMS_OVER_CURRENT_DISCHARGE > 32767L) ?
                                  32767 :
                                  (int16_t)BMS_OVER_CURRENT_DISCHARGE;
#endif
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setChargingOvercurrentProtection(
                                      (unsigned int)over_current_chargr_mV,
                                      50U));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setChargingOvercurrentProtection_Recovery(
                                      BMS_BQ_OCC_RECOVERY_THRESHOLD_MA));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setProtectionRecoveryTime(
                                      (byte)BMS_BQ_RECOVERY_TIME_SEC));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingOvercurrentProtection(
                                      (unsigned int)over_current_sense_mV,
                                      50U));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingOvercurrentProtection_OCD3(
                                      discharge_ocd3_threshold_mA));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingOvercurrentProtection_Recovery(0));
    BMS_BQ_CONFIG_STEP(config_ok, bq76952_setDischargingShortcircuitProtection(SCD_60, 30U));

    if (!bq76952_areFETs_Enabled()) {
        bq76952_setFET_ENABLE();
    }
    bq76952_setFET(ALL, ON);
    g_bms_tracking.faults.communicationFault = (config_ok == 0U);
    BMS_LOG_INFO("bq configured occ=%lu mV ocd=%lu mV ocd3=%ld mA ok=%u",
                 (unsigned long)over_current_chargr_mV,
                 (unsigned long)over_current_sense_mV,
                 (long)discharge_ocd3_threshold_mA,
                 (unsigned int)config_ok);
}

static void BMS_ConfigureHardwarePins(void)
{
    HAL_GPIO_WritePin(FETOFF_GPIO_Port, FETOFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHUT_GPIO_Port, SHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BATS_EN_GPIO_Port, BATS_EN_Pin, GPIO_PIN_RESET);
    g_alert_irq_pending = false;
    g_alert_irq_counter = 0UL;
    // g_dchg_signal_active = (HAL_GPIO_ReadPin(DCHG_GPIO_Port, DCHG_Pin) == GPIO_PIN_SET);
    // g_ddsg_signal_active = (HAL_GPIO_ReadPin(DDSG_GPIO_Port, DDSG_Pin) == GPIO_PIN_SET);
    g_shutdown_pulse_active = false;
    g_shutdown_pulse_tick = 0UL;
    // g_charge_oc_recovery_pending = false;
    // g_discharge_oc_recovery_pending = false;
    // g_charge_oc_recovery_tick = 0UL;
    // g_discharge_oc_recovery_tick = 0UL;
}

static void BMS_SetFetoff(bool asserted)
{
    HAL_GPIO_WritePin(FETOFF_GPIO_Port, FETOFF_Pin, asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_bms_tracking.fetOffAsserted = asserted;
}

static void BMS_SetBatSenseEnable(bool enabled)
{
    HAL_GPIO_WritePin(BATS_EN_GPIO_Port, BATS_EN_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_bms_tracking.batSenseEnabled = enabled;
}

static void BMS_HandleHardwareSignals(BMS_Tracking_t *tracking, uint32_t now)
{
    bool should_service_alert;
    bool alert_signal_active;
    unsigned int alarm_status;
    static uint32_t g_last_alert_service_tick = 0;

    if (tracking == NULL) {
        return;
    }

    // g_dchg_signal_active = (HAL_GPIO_ReadPin(DCHG_GPIO_Port, DCHG_Pin) == GPIO_PIN_SET);
    // g_ddsg_signal_active = (HAL_GPIO_ReadPin(DDSG_GPIO_Port, DDSG_Pin) == GPIO_PIN_SET);
    alert_signal_active = BMS_IsAlertPinActive();

    /* DCHG/DDSG high means the corresponding BQ FET output is disabled.
     * It is a status/wakeup signal, not a standalone pack fault.
     */
    // tracking->chargeGateFaultSignal = g_dchg_signal_active;
    // tracking->dischargeGateFaultSignal = g_ddsg_signal_active;

    should_service_alert = g_alert_irq_pending ||
                           alert_signal_active ||
                           ((now - g_last_alert_service_tick) >= BMS_ALERT_POLL_FALLBACK_MS);
    if (!should_service_alert) {
        tracking->alertActive = false;
        return;
    }

    g_alert_irq_pending = false;
    g_last_alert_service_tick = now;
    tracking->alertCounter = g_alert_irq_counter;
    alarm_status = bq76952_getAlertStatusRegister();
    tracking->alertActive = alert_signal_active || (alarm_status != 0U);
    (void)bq76952_clearAlertStatusRegister((uint16_t)alarm_status);
    BMS_UpdateBqAlarmRawStatus(tracking);
}

static bool BMS_IsAlertPinActive(void)
{
    return HAL_GPIO_ReadPin(ALERT_GPIO_Port, ALERT_Pin) == BMS_ALERT_ACTIVE_LEVEL;
}

static void BMS_UpdateBatteryAdc(BMS_Tracking_t *tracking, uint32_t now)
{
    uint32_t pack_mv;
    uint64_t pack_num;
    uint64_t pack_den;
    uint16_t batAdcRaw;
    static uint32_t g_last_bat_adc_sample_tick = 0;

    if (tracking == NULL) {
        return;
    }
    if ((now - g_last_bat_adc_sample_tick) < BMS_BAT_ADC_SAMPLE_MS) {
        return;
    }
    g_last_bat_adc_sample_tick = now;

    BMS_SetBatSenseEnable(true);
    HAL_Delay(BMS_BAT_ADC_SETTLE_MS);

    if (HAL_ADC_Start(&hadc) != HAL_OK) {
        BMS_SetBatSenseEnable(false);
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc, 5U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc);
        BMS_SetBatSenseEnable(false);
        return;
    }

    batAdcRaw = (uint16_t)HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);
    BMS_SetBatSenseEnable(false);

    pack_num = (uint64_t)batAdcRaw *
               BMS_BAT_ADC_REF_MV *
               BMS_BAT_ADC_DIVIDER_NUM *
               BMS_BAT_ADC_CAL_NUM;
    pack_den = (uint64_t)BMS_BAT_ADC_COUNTS *
               BMS_BAT_ADC_DIVIDER_DEN *
               BMS_BAT_ADC_CAL_DEN;
    pack_mv = (uint32_t)((pack_num + (pack_den / 2ULL)) / pack_den);
    if (pack_mv > UINT16_MAX) {
        pack_mv = UINT16_MAX;
    }

    tracking->batAdcEstimatedPack_mV = (uint16_t)pack_mv;
}

static void BMS_ReadMeasurements(BMS_Tracking_t *tracking, uint32_t Now)
{
    uint16_t raw_cell_voltage[BMS_NUMBER_OF_CELLS];
    bq76952_battery_status_t batt_status;

    if (tracking == NULL) {
        return;
    }

    bq76952_getOnlyConnectedCellVoltages(raw_cell_voltage);
    for(uint8_t count = 0; count < BMS_NUMBER_OF_CELLS; count ++)
    {
        tracking->cellVoltages.RealTimeAccumulated[count] += raw_cell_voltage[count];
        if(++ tracking->cellVoltages.IndexAccumulated[count] >= BMS_CELL_TIMES_ACUMMULATE)
        {
            tracking->cellVoltages.cellNum[count] = tracking->cellVoltages.RealTimeAccumulated[count] / BMS_CELL_TIMES_ACUMMULATE;
            tracking->cellVoltages.RealTimeAccumulated[count] = 0;
            tracking->cellVoltages.IndexAccumulated[count] = 0;
        }
    }
    
    tracking->stackVoltage = 0U;
    tracking->current_mA = (int32_t)bq76952_getCurrentAvg();
    tracking->temperature[0] = bq76952_getThermistorTemp(TS1);
    tracking->temperature[1] = bq76952_getThermistorTemp(TS3);
    BMS_UpdateFetStatus(tracking);
    batt_status = bq76952_getBatteryStatusRegister();
    tracking->bqSleepMode = batt_status.bits.SLEEP_MODE != 0U;
    tracking->bqSleepAllowed = batt_status.bits.SLEEP_EN_ALLOWED != 0U;
}

static void BMS_UpdateCellStatistics(BMS_Tracking_t *tracking)
{
    uint32_t voltage_sum = 0U;
    uint16_t min_voltage = UINT16_MAX;
    uint16_t max_voltage = 0U;

    if (tracking == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
        uint16_t voltage = tracking->cellVoltages.cellNum[i];
        if(voltage == 0)
        {
            return;
        }

        voltage_sum += voltage;
        if ((voltage > 0U) && (voltage < min_voltage)) {
            min_voltage = voltage;
        }
        if (voltage > max_voltage) {
            max_voltage = voltage;
        }
    }

    tracking->cellVoltages.minCellVoltage = (min_voltage == UINT16_MAX) ? 0U : min_voltage;
    tracking->cellVoltages.maxCellVoltage = max_voltage;
    tracking->cellVoltages.averageCellVoltage = (uint16_t)(voltage_sum / BMS_NUMBER_OF_CELLS);
    tracking->cellVoltages.deltaCellVoltage = tracking->cellVoltages.maxCellVoltage - tracking->cellVoltages.minCellVoltage;
    tracking->packVoltage = (voltage_sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)voltage_sum;
}

static void BMS_UpdateCurrentDirection(BMS_Tracking_t *tracking)
{
    if (tracking == NULL) {
        return;
    }

#if BMS_CURRENT_CHARGE_IS_POSITIVE
    if (tracking->current_mA > BMS_CURRENT_DEADBAND_MA) {
        tracking->currentDirection = BMS_CURRENT_CHARGE;
    } else if (tracking->current_mA < -BMS_CURRENT_DEADBAND_MA) {
        tracking->currentDirection = BMS_CURRENT_DISCHARGE;
    } else
#else
    if (tracking->current_mA < -BMS_CURRENT_DEADBAND_MA) {
        tracking->currentDirection = BMS_CURRENT_CHARGE;
    } else if (tracking->current_mA > BMS_CURRENT_DEADBAND_MA) {
        tracking->currentDirection = BMS_CURRENT_DISCHARGE;
    } else
#endif
    {
        tracking->currentDirection = BMS_CURRENT_IDLE;
    }

    tracking->charging = (tracking->currentDirection == BMS_CURRENT_CHARGE);
    tracking->discharging = (tracking->currentDirection == BMS_CURRENT_DISCHARGE);
}

static void BMS_UpdateFetStatus(BMS_Tracking_t *tracking)
{
    byte fet_status;

    if (tracking == NULL) {
        return;
    }

    fet_status = bq76952_getFetStatusRaw();
    tracking->chargeFetEnabled = (fet_status & BMS_BQ_FET_STAT_CHG_FET) != 0U;
    tracking->dischargeFetEnabled = (fet_status & BMS_BQ_FET_STAT_DSG_FET) != 0U;
    tracking->fetsEnabled = bq76952_areFETs_Enabled();
}

static void BMS_UpdateBqAlarmRawStatus(BMS_Tracking_t *tracking)
{
    uint16_t alarm_raw_status;

    if (tracking == NULL) {
        return;
    }

    alarm_raw_status = (uint16_t)bq76952_getAlertRawStatusRegister();
    tracking->bqAlarmRawStatus.raw = alarm_raw_status;
    tracking->bqChargeFetBlocked = tracking->bqAlarmRawStatus.bit.XCHG;
    tracking->bqDischargeFetBlocked = tracking->bqAlarmRawStatus.bit.XDSG;
}

static void BMS_UpdateFaultFlags(BMS_Tracking_t *tracking, uint32_t now)
{
    int32_t abs_current;
    // static bool charge_oc_recovery_pending = false;
    // static bool discharge_oc_recovery_pending = false;
    static uint32_t g_charge_oc_recovery_tick = 0;
    static uint32_t g_discharge_oc_recovery_tick = 0;

    if (tracking == NULL) {
        return;
    }

    if (tracking->cellVoltages.averageCellVoltage >= BMS_CELL_OV_CUTOFF_MV_DEV) {
        if (!tracking->faults.cellOverVoltage) {
            bms_uart_send_protection_reason(BMS_UART_PROTECT_CELL_OV);
        }
        tracking->faults.cellOverVoltage = true;
    }
    if (tracking->cellVoltages.averageCellVoltage <= BMS_CELL_UV_CUTOFF_MV_DEV) {
        if (!tracking->faults.cellUnderVoltage) {
            bms_uart_send_protection_reason(BMS_UART_PROTECT_CELL_UV);
        }
        tracking->faults.cellUnderVoltage = true;
    }

    if (tracking->faults.cellOverVoltage &&
        BMS_AllCellsAtOrBelow(tracking, BMS_CELL_OV_RECOVER_MV)) {
        tracking->faults.cellOverVoltage = false;
    }
    if (tracking->faults.cellUnderVoltage &&
        BMS_AllCellsAtOrAbove(tracking, BMS_CELL_UV_RECOVER_MV)) {
        tracking->faults.cellUnderVoltage = false;
    }

    for (uint8_t i = 0U; i < BMS_NUMBER_OF_THERMISTORS; ++i) {
        if (tracking->temperature[i] >= BMS_CHARGE_OT_CUTOFF_C) {
            if (!tracking->faults.chargeOverTemperature) {
                bms_uart_send_protection_reason(BMS_UART_PROTECT_CHARGE_OT);
            }
            tracking->faults.chargeOverTemperature = true;
        }
        if (tracking->temperature[i] >= BMS_DISCHARGE_OT_CUTOFF_C) {
            if (!tracking->faults.dischargeOverTemperature) {
                bms_uart_send_protection_reason(BMS_UART_PROTECT_DISCHARGE_OT);
            }
            tracking->faults.dischargeOverTemperature = true;
        }
        if (tracking->temperature[i] <= BMS_UNDERTEMP_CUTOFF_C) {
            if (!tracking->faults.underTemperature) {
                bms_uart_send_protection_reason(BMS_UART_PROTECT_UNDERTEMP);
            }
            tracking->faults.underTemperature = true;
        }
    }

    if (tracking->faults.chargeOverTemperature &&
        BMS_AllTemperaturesAtOrBelow(tracking, BMS_CHARGE_OT_RECOVER_C)) {
        tracking->faults.chargeOverTemperature = false;
    }
    if (tracking->faults.dischargeOverTemperature &&
        BMS_AllTemperaturesAtOrBelow(tracking, BMS_DISCHARGE_OT_RECOVER_C)) {
        tracking->faults.dischargeOverTemperature = false;
    }
    if (tracking->faults.underTemperature &&
        BMS_AllTemperaturesAtOrAbove(tracking, BMS_UNDERTEMP_RECOVER_C)) {
        tracking->faults.underTemperature = false;
    }

    abs_current = BMS_AbsCurrent(tracking->current_mA);
    if (abs_current >= BMS_SHORT_CIRCUIT_MA) {
        if (!tracking->faults.shortCircuit) {
            bms_uart_send_protection_reason(BMS_UART_PROTECT_SHORT_CIRCUIT);
        }
        tracking->faults.shortCircuit = true;
    }
    if(abs_current >= BMS_OVER_CURRENT_CHARGE)
    {
        if (tracking->currentDirection == BMS_CURRENT_CHARGE) {
            if (!tracking->faults.chargeOverCurrent) {
                bms_uart_send_protection_reason(BMS_UART_PROTECT_CHARGE_OC);
            }
            tracking->faults.chargeOverCurrent = true;
            // charge_oc_recovery_pending = false;
            g_charge_oc_recovery_tick = 0;
        }
    }
    if (abs_current >= BMS_OVER_CURRENT_DISCHARGE) {
        if (tracking->currentDirection == BMS_CURRENT_DISCHARGE) {
            if (!tracking->faults.dischargeOverCurrent) {
                bms_uart_send_protection_reason(BMS_UART_PROTECT_DISCHARGE_OC);
            }
            // tracking->faults.dischargeOverCurrent = true;
            // discharge_oc_recovery_pending = false;
            g_discharge_oc_recovery_tick = 0; 
        }
    } else if (abs_current <= BMS_CURRENT_DEADBAND_MA) {
        if (tracking->faults.chargeOverCurrent) {
            if (g_charge_oc_recovery_tick == 0) {
                // charge_oc_recovery_pending = true;
                g_charge_oc_recovery_tick = now;
            } else if ((now - g_charge_oc_recovery_tick) >= BMS_OVER_CURRENT_RECOVERY_DELAY_MS) {
                tracking->faults.chargeOverCurrent = false;
                g_charge_oc_recovery_tick = 0;
                // charge_oc_recovery_pending = false;
        }
        } else {
            // charge_oc_recovery_pending = false;
            g_charge_oc_recovery_tick = 0;
        }

        if (tracking->faults.dischargeOverCurrent) {
            if (g_discharge_oc_recovery_tick == 0) {
                // discharge_oc_recovery_pending = true;
                g_discharge_oc_recovery_tick = now;
            } else if ((now - g_discharge_oc_recovery_tick) >= BMS_OVER_CURRENT_RECOVERY_DELAY_MS) {
                tracking->faults.dischargeOverCurrent = false;
                g_discharge_oc_recovery_tick = 0;
                // discharge_oc_recovery_pending = false;
            }
        } else {
            // discharge_oc_recovery_pending = false;
            g_discharge_oc_recovery_tick = 0;
        }
    } else {
        // charge_oc_recovery_pending = false;
        // discharge_oc_recovery_pending = false;
        g_charge_oc_recovery_tick = 0;
        g_discharge_oc_recovery_tick = 0;
    }
}

static void BMS_MergeBQFaultFlags(BMS_Tracking_t *tracking)
{
    bq76952_protection_t protection;
    BQ76952_SafetyStatusC_t safety_status_c;
    bq76952_temp_t temperature_status;

    if (tracking == NULL) {
        return;
    }

    protection = bq76952_getProtectionStatus();
    safety_status_c = bq76952_getSafetyStatus_C();
    temperature_status = bq76952_getTemperatureStatus();

    if (protection.bits.CELL_OV && !tracking->faults.cellOverVoltage) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_CELL_OV);
    }
    tracking->faults.cellOverVoltage = tracking->faults.cellOverVoltage || protection.bits.CELL_OV;

    if (protection.bits.CELL_UV && !tracking->faults.cellUnderVoltage) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_CELL_UV);
    }
    tracking->faults.cellUnderVoltage = tracking->faults.cellUnderVoltage || protection.bits.CELL_UV;

    if (protection.bits.OC_CHG && !tracking->faults.chargeOverCurrent) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_CHARGE_OC);
    }
    tracking->faults.chargeOverCurrent = tracking->faults.chargeOverCurrent || protection.bits.OC_CHG;

    if ((protection.bits.OC1_DCHG || protection.bits.OC2_DCHG || safety_status_c.bit.OCD3) &&
        !tracking->faults.dischargeOverCurrent) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_DISCHARGE_OC);
    }
    tracking->faults.dischargeOverCurrent = tracking->faults.dischargeOverCurrent ||
                                            protection.bits.OC1_DCHG ||
                                            protection.bits.OC2_DCHG ||
                                            safety_status_c.bit.OCD3;

    if (protection.bits.SC_DCHG && !tracking->faults.shortCircuit) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_SHORT_CIRCUIT);
    }
    tracking->faults.shortCircuit = tracking->faults.shortCircuit || protection.bits.SC_DCHG;

    if (temperature_status.bits.OVERTEMP_CHG && !tracking->faults.chargeOverTemperature) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_CHARGE_OT);
    }
    tracking->faults.chargeOverTemperature = tracking->faults.chargeOverTemperature ||
                                             temperature_status.bits.OVERTEMP_CHG;

    if (temperature_status.bits.OVERTEMP_DCHG && !tracking->faults.dischargeOverTemperature) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_DISCHARGE_OT);
    }
    tracking->faults.dischargeOverTemperature = tracking->faults.dischargeOverTemperature ||
                                                temperature_status.bits.OVERTEMP_DCHG;

    if ((temperature_status.bits.UNDERTEMP_CHG || temperature_status.bits.UNDERTEMP_DCHG) &&
        !tracking->faults.underTemperature) {
        bms_uart_send_protection_reason(BMS_UART_PROTECT_UNDERTEMP);
    }
    tracking->faults.underTemperature = tracking->faults.underTemperature ||
                                        temperature_status.bits.UNDERTEMP_CHG ||
                                        temperature_status.bits.UNDERTEMP_DCHG;
    tracking->faults.bqSafetyFault = tracking->faults.shortCircuit;
    if(tracking->faults.chargeOverCurrent > 0)
    {
        BMS_LOG_INFO("protection : OCC");
    }
}

static void BMS_UpdateState(BMS_Tracking_t *tracking)
{
    if (tracking == NULL) {
        return;
    }

    tracking->chargeDisabled = tracking->faults.cellOverVoltage ||
                               tracking->faults.chargeOverTemperature ||
                               tracking->faults.underTemperature ||
                               tracking->faults.chargeOverCurrent ||
                               tracking->faults.shortCircuit ||
                               tracking->faults.bqSafetyFault ||
                               tracking->faults.communicationFault;

    tracking->dischargeDisabled = tracking->faults.cellUnderVoltage ||
                                  tracking->faults.dischargeOverTemperature ||
                                  tracking->faults.underTemperature ||
                                  tracking->faults.dischargeOverCurrent ||
                                  tracking->faults.shortCircuit ||
                                  tracking->faults.bqSafetyFault ||
                                  tracking->faults.communicationFault;

    if (tracking->faults.shortCircuit || tracking->faults.communicationFault) {
        tracking->state = BMS_STATE_FAULT;
    } else if (tracking->chargeDisabled && tracking->dischargeDisabled) {
        tracking->state = BMS_STATE_FAULT;
    } else if (tracking->chargeDisabled) {
        tracking->state = BMS_STATE_CHARGE_PROTECT;
    } else if (tracking->dischargeDisabled) {
        tracking->state = BMS_STATE_DISCHARGE_PROTECT;
    } else {
        tracking->state = BMS_STATE_NORMAL;
    }

}

static void BMS_ApplyFetPolicy(BMS_Tracking_t *tracking)
{
    if (tracking == NULL) {
        return;
    }

    if (tracking->chargeDisabled && tracking->dischargeDisabled) {
        BMS_SetFetoff(true);
        bq76952_setFET(ALL, OFF);
        HAL_Delay(BMS_FET_STATUS_SETTLE_MS);
        BMS_UpdateFetStatus(tracking);
        BMS_SyncStateWithFetAvailability(tracking);
        return;
    }

    if (tracking->chargeDisabled) {
        BMS_SetFetoff(false);
        bq76952_setFET(CHG, OFF);
        HAL_Delay(BMS_FET_STATUS_SETTLE_MS);
        BMS_UpdateFetStatus(tracking);
        BMS_SyncStateWithFetAvailability(tracking);
        return;
    }

    if (tracking->dischargeDisabled) {
        BMS_SetFetoff(false);
        bq76952_setFET(DCH, OFF);
        HAL_Delay(BMS_FET_STATUS_SETTLE_MS);
        BMS_UpdateFetStatus(tracking);
        BMS_SyncStateWithFetAvailability(tracking);
        return;
    }

    if(tracking->state == BMS_STATE_NORMAL && (tracking->bqAlarmRawStatus.bit.XCHG || tracking->bqAlarmRawStatus.bit.XDSG) \
    && !tracking->bqAlarmRawStatus.bit.SSA && !tracking->bqAlarmRawStatus.bit.SSBC)
    {
        BMS_SetFetoff(false);
        bq76952_setFET(ALL, ON);
        HAL_Delay(BMS_FET_STATUS_SETTLE_MS);
        BMS_UpdateFetStatus(tracking);
        BMS_SyncStateWithFetAvailability(tracking);
    }
    
}

static void BMS_SyncStateWithFetAvailability(BMS_Tracking_t *tracking)
{
    uint16_t alarm_raw_status;
    bool charge_blocked;
    bool discharge_blocked;

    if (tracking == NULL) {
        return;
    }

    alarm_raw_status = (uint16_t)bq76952_getAlertRawStatusRegister();
    tracking->bqAlarmRawStatus.raw = alarm_raw_status;
    tracking->bqChargeFetBlocked = (alarm_raw_status & BMS_BQ_ALARM_RAW_XCHG) != 0U;
    tracking->bqDischargeFetBlocked = (alarm_raw_status & BMS_BQ_ALARM_RAW_XDSG) != 0U;

    if (tracking->state != BMS_STATE_NORMAL) {
        return;
    }

    charge_blocked = tracking->bqChargeFetBlocked || !tracking->chargeFetEnabled;
    discharge_blocked = tracking->bqDischargeFetBlocked || !tracking->dischargeFetEnabled;

    if (!charge_blocked && !discharge_blocked) {
        return;
    }

    tracking->chargeDisabled = charge_blocked;
    tracking->dischargeDisabled = discharge_blocked;

    if (charge_blocked && discharge_blocked) {
        tracking->state = BMS_STATE_FAULT;
    } else if (charge_blocked) {
        tracking->state = BMS_STATE_CHARGE_PROTECT;
    } else {
        tracking->state = BMS_STATE_DISCHARGE_PROTECT;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ALERT_Pin) {
        BMS_NotifyAlertInterrupt();
        power_manager_notify_gpio_wakeup();
    } 
}

static void BMS_UpdateCoulombCounter(BMS_Tracking_t *tracking, uint32_t dt_ms)
{
    uint64_t sample_mAs;
    int32_t abs_current;

    if (tracking == NULL) {
        return;
    }

    abs_current = BMS_AbsCurrent(tracking->current_mA);
    if (abs_current <= BMS_CURRENT_DEADBAND_MA) {
        return;
    }

    sample_mAs = ((uint64_t)abs_current * (uint64_t)dt_ms) / 1000ULL;
    if (tracking->currentDirection == BMS_CURRENT_CHARGE) {
        tracking->chargeAccumulated_mAs += sample_mAs;
    }

    tracking->chargeThroughput_mAh = (uint32_t)(tracking->chargeAccumulated_mAs / 3600ULL);
    tracking->equivalentCycle_milliCycles =
        (uint32_t)((tracking->chargeThroughput_mAh * 1000ULL) / BMS_NOMINAL_CAPACITY_MAH);
        
}

static void BMS_UpdateBalancing(BMS_Tracking_t *tracking, uint32_t now)
{
    static uint32_t g_last_balance_tick = 0;
    bool balance_allowed;
    static uint16_t previous_mask = 0;
    uint16_t selected_mask = 0U;
    uint16_t delta[BMS_NUMBER_OF_CELLS] = {0U};
    bool candidate[BMS_NUMBER_OF_CELLS] = {false};

    if (tracking == NULL) {
        return;
    }

    if (tracking->balanceRequired) {
        if (tracking->cellVoltages.deltaCellVoltage < BMS_BALANCE_DELTA_MV_RECOVERY) {
            tracking->balanceRequired = false;
        }
    } else if (tracking->cellVoltages.deltaCellVoltage >= BMS_BALANCE_DELTA_MV) {
        tracking->balanceRequired = true;
    }

    balance_allowed = (tracking->state == BMS_STATE_NORMAL) &&
                      tracking->balanceRequired &&
                      (tracking->currentDirection != BMS_CURRENT_DISCHARGE);
    if (!balance_allowed) {
        if ((tracking->balanceMask != 0U)) {
            bq76952_setCellBalanceMask(0U);
            BMS_LOG_INFO("balance off");
        }
        tracking->balanceMask = 0U;
        return;
    }

    if ((now - g_last_balance_tick) < BMS_BALANCE_REFRESH_MS) {
        return;
    }
    g_last_balance_tick = now;

    for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
        // bool was_selected = (previous_mask & (uint16_t)(1U << i)) != 0U;

        if (tracking->cellVoltages.cellNum[i] >= tracking->cellVoltages.minCellVoltage) {
            delta[i] = tracking->cellVoltages.cellNum[i] - tracking->cellVoltages.minCellVoltage;
        }

        if (tracking->cellVoltages.cellNum[i] < BMS_BALANCE_MIN_CELL_MV) {
            continue;
        }

        candidate[i] = delta[i] >= BMS_BALANCE_DELTA_MV_RECOVERY;
    }

    /* New cells enter above 30 mV; existing cells are held until below 20 mV. */
    for (;;) {
        int best = -1;

        for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
            bool current_was_selected;
            bool best_was_selected;

            if (!candidate[i]) {
                continue;
            }

            if (best < 0) {
                best = (int)i;
                continue;
            }

            if (delta[i] > delta[best]) {
                best = (int)i;
                continue;
            }

            if (delta[i] < delta[best]) {
                continue;
            }

            current_was_selected = (previous_mask & (uint16_t)(1U << i)) != 0U;
            best_was_selected = (previous_mask & (uint16_t)(1U << best)) != 0U;
            if (current_was_selected && !best_was_selected) {
                best = (int)i;
            }
        }

        if (best < 0) {
            break;
        }

        if (((best == 0) ||
             ((selected_mask & (uint16_t)(1U << (uint8_t)(best - 1))) == 0U)) &&
            (((uint8_t)best == (BMS_NUMBER_OF_CELLS - 1U)) ||
             ((selected_mask & (uint16_t)(1U << (uint8_t)(best + 1))) == 0U))) {
            selected_mask |= (uint16_t)(1U << (uint8_t)best);
        }
        candidate[best] = false;
    }

    tracking->balanceMask = selected_mask;
    bq76952_setCellBalanceMask(selected_mask);
}

static void BMS_LoadPersistedData(BMS_Tracking_t *tracking)
{
    storage_flash_record_t record;

    if (tracking == NULL) {
        return;
    }
    /* Load persisted data from flash */
    if (!storage_flash_load(&record)) {
        BMS_LOG_WARN("flash record invalid, use defaults");
        storage_flash_make_default(&record);
    }

    tracking->chargeThroughput_mAh = record.chargeThroughput_mAh;   // Note: discharge throughput and equivalent cycle may be inconsistent with charge throughput, but it's acceptable for estimation purpose
    tracking->equivalentCycle_milliCycles = record.equivalentCycle_milliCycles; // chu kì sạc xả
    tracking->currentCalibrationGainPpm = (record.currentCalibrationGainPpm == 0UL) ?
                                          BMS_CURRENT_CALIBRATION_DEFAULT_PPM :
                                          record.currentCalibrationGainPpm;
    tracking->chargeAccumulated_mAs = (uint64_t)record.chargeThroughput_mAh * 3600ULL; // tích trữ mAs dựa trên charge throughput đã lưu, vì discharge throughput có thể không chính xác nếu có lỗi ghi flash trước đó

    BMS_LOG_INFO("flash load chg=%lu cyc=%lu cal=%lu",
                 (unsigned long)tracking->chargeThroughput_mAh,
                 (unsigned long)tracking->equivalentCycle_milliCycles,
                 (unsigned long)tracking->currentCalibrationGainPpm);
}

static bool BMS_SaveCurrentCalibration(uint32_t gain_ppm)
{
    storage_flash_record_t record;
    storage_flash_record_t old_record;

    if (gain_ppm == 0UL) {
        return false;
    }

    storage_flash_make_default(&record);
    if (storage_flash_load(&old_record)) {
        record.writeCounter = old_record.writeCounter + 1U;
    } else {
        record.writeCounter = 1U;
    }

    record.chargeThroughput_mAh = g_bms_tracking.chargeThroughput_mAh;
    record.equivalentCycle_milliCycles = g_bms_tracking.equivalentCycle_milliCycles;
    record.nominalCapacity_mAh = BMS_NOMINAL_CAPACITY_MAH;
    record.currentCalibrationGainPpm = gain_ppm;

    if (!storage_flash_save(&record)) {
        BMS_LOG_ERROR("flash save current cal failed");
        return false;
    }

    g_bms_tracking.currentCalibrationGainPpm = gain_ppm;
    return true;
}

static void BMS_SavePersistedDataIfNeeded(const BMS_Tracking_t *tracking, uint32_t now)
{
    storage_flash_record_t record;
    storage_flash_record_t old_record;
    uint32_t charge_delta;
    static uint32_t g_last_flash_save_tick = 0;
    static uint32_t g_last_saved_charge_mAh = 0;;

    if (tracking == NULL) {
        return;
    }

    if ((now - g_last_flash_save_tick) < BMS_FLASH_SAVE_INTERVAL_MS) {
        return;
    }

    charge_delta = (tracking->chargeThroughput_mAh >= g_last_saved_charge_mAh) ?
                   (tracking->chargeThroughput_mAh - g_last_saved_charge_mAh) : 0U;

    if ((charge_delta < BMS_FLASH_SAVE_DELTA_MAH)) {
        g_last_flash_save_tick = now;
        return;
    }

    storage_flash_make_default(&record);
    if (storage_flash_load(&old_record)) {
        record.writeCounter = old_record.writeCounter + 1U;
    } else {
        record.writeCounter = 1U;
    }
    record.chargeThroughput_mAh = tracking->chargeThroughput_mAh;
    record.equivalentCycle_milliCycles = tracking->equivalentCycle_milliCycles;
    record.nominalCapacity_mAh = BMS_NOMINAL_CAPACITY_MAH;
    record.currentCalibrationGainPpm = (tracking->currentCalibrationGainPpm == 0UL) ?
                                       BMS_CURRENT_CALIBRATION_DEFAULT_PPM :
                                       tracking->currentCalibrationGainPpm;

    if (storage_flash_save(&record)) {
        g_last_flash_save_tick = now;
        g_last_saved_charge_mAh = tracking->chargeThroughput_mAh;
        BMS_LOG_INFO("flash save chg=%lu cyc=%lu",
                     (unsigned long)record.chargeThroughput_mAh,
                     (unsigned long)record.equivalentCycle_milliCycles);
    } else {
        BMS_LOG_ERROR("flash save failed");
    }
}

#if BMS_DEBUG_LOG_ENABLE
const char *BMS_StateName(BMS_State_t state)
{
    switch (state) {
    case BMS_STATE_INIT:
        return "INIT";
    case BMS_STATE_NORMAL:
        return "NORMAL";
    case BMS_STATE_CHARGE_PROTECT:
        return "CHG_PROTECT";
    case BMS_STATE_DISCHARGE_PROTECT:
        return "DCH_PROTECT";
    case BMS_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
#endif

/* Helper functions to check if all cells  are above/below certain thresholds for fault recovery conditions */
static bool BMS_AllCellsAtOrBelow(const BMS_Tracking_t *tracking, uint16_t threshold_mV)
{
    for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
        if (tracking->cellVoltages.cellNum[i] > threshold_mV) {
            return false;
        }
    }
    return true;
}

static bool BMS_AllCellsAtOrAbove(const BMS_Tracking_t *tracking, uint16_t threshold_mV)
{
    for (uint8_t i = 0U; i < BMS_NUMBER_OF_CELLS; ++i) {
        if ((tracking->cellVoltages.cellNum[i] > 0U) && (tracking->cellVoltages.cellNum[i] < threshold_mV)) {
            return false;
        }
    }
    return true;
}

static bool BMS_AllTemperaturesAtOrBelow(const BMS_Tracking_t *tracking, int16_t threshold_C)
{
    for (uint8_t i = 0U; i < BMS_NUMBER_OF_THERMISTORS; ++i) {
        if (tracking->temperature[i] > threshold_C) {
            return false;
        }
    }
    return true;
}

static bool BMS_AllTemperaturesAtOrAbove(const BMS_Tracking_t *tracking, int16_t threshold_C)
{
    for (uint8_t i = 0U; i < BMS_NUMBER_OF_THERMISTORS; ++i) {
        if (tracking->temperature[i] < threshold_C) {
            return false;
        }
    }
    return true;
}

static int32_t BMS_AbsCurrent(int32_t current_mA)
{
    if (current_mA < 0) {
        return -current_mA;
    }
    return current_mA;
}

static uint32_t BMS_AbsCurrentU32(int32_t current_mA)
{
    if (current_mA < 0) {
        return (uint32_t)(-(current_mA + 1)) + 1UL;
    }
    return (uint32_t)current_mA;
}

bool BMS_Set_5V_Output(bool enabled)
{
    bool ok;

    if (enabled) {
        ok = bq76952_setEnableRegulator(true, true); // enable 5V regulator in auto mode (enabled when either charge or discharge FET is on)
    } else {
        ok = bq76952_setEnableRegulator(false, false); // disable 5V regulator
    }

    if (!ok) {
        g_bms_tracking.faults.communicationFault = true;
    }
    return ok;
}
