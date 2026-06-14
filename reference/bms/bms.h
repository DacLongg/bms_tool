#ifndef BMS_H
#define BMS_H

#include <stdbool.h>
#include <stdint.h>

#include "bq76952.h"

#define BMS_NUMBER_OF_CELLS                         10U
#define BMS_NUMBER_OF_THERMISTORS                   2U

#define BMS_CELL_TIMES_ACUMMULATE                   10
#define BMS_CELL_OV_CUTOFF_MV_BQ                    4200U
#define BMS_CELL_OV_CUTOFF_MV_DEV                   4150U
#define BMS_CELL_OV_RECOVER_MV                      4100U
#define BMS_CELL_UV_CUTOFF_MV_BQ                    3500U
#define BMS_CELL_UV_CUTOFF_MV_DEV                   3550U
#define BMS_CELL_UV_RECOVER_MV                      3650U

#define BMS_BALANCE_DELTA_MV                        30U
#define BMS_BALANCE_DELTA_MV_RECOVERY               20U
#define BMS_BALANCE_MIN_CELL_MV                     3800U
#define BMS_BALANCE_REFRESH_MS                      10000U
#define BMS_BALANCE_MIN_TEMP_C                      5
#define BMS_BALANCE_MAX_TEMP_C                      45
#define BMS_BALANCE_MAX_INTERNAL_TEMP_C             70
#define BMS_BALANCE_INTERVAL_SEC                    20U
#define BMS_BALANCE_MAX_ACTIVE_CELLS                9U

#define BMS_CURRENT_DEADBAND_MA                     300L
#define BMS_OVER_CURRENT_CHARGE                     1000
#define BMS_OVER_CURRENT_DISCHARGE                  2000L
#define BMS_OVER_CURRENT_MA                         BMS_OVER_CURRENT_DISCHARGE
#define BMS_OVER_CURRENT_RECOVERY_DELAY_MS          10000UL
#define BMS_SHORT_CIRCUIT_MA                        120000L
#define BMS_NOMINAL_CAPACITY_MAH                    1000UL
#define BMS_CURRENT_CALIBRATION_DEFAULT_PPM         1000000UL
#define BMS_CURRENT_CALIBRATION_MAX_DEVIATION_PPM   300000UL

#define BMS_CHARGE_OT_CUTOFF_C                      45
#define BMS_CHARGE_OT_RECOVER_C                     40
#define BMS_DISCHARGE_OT_CUTOFF_C                   60
#define BMS_DISCHARGE_OT_RECOVER_C                  55
#define BMS_UNDERTEMP_CUTOFF_C                      0
#define BMS_UNDERTEMP_RECOVER_C                     5
#define BMS_BQ_TEMPERATURE_PROTECTION_DELAY_SEC     2U

/* Hardware 10S uses consecutive BQ cell channels VC0..VC10. */
#define BMS_BQ_VCELL_MODE_10S                       0x03FFU
#define BMS_BQ_SENSE_RESISTOR_UOHM                  500UL
#define BMS_BQ_PROTECTION_DELAY_MS                  3000U
#define BMS_CURRENT_CHARGE_IS_POSITIVE              1
#define BMS_FLASH_SAVE_INTERVAL_MS                  600000UL
#define BMS_FLASH_SAVE_DELTA_MAH                    500UL

#define NUMBER_OF_CELLS                             BMS_NUMBER_OF_CELLS
#define NUMBER_OF_THERMISTORS                       BMS_NUMBER_OF_THERMISTORS

typedef enum {
    BMS_STATE_INIT = 0,
    BMS_STATE_NORMAL,
    BMS_STATE_CHARGE_PROTECT,
    BMS_STATE_DISCHARGE_PROTECT,
    BMS_STATE_FAULT
} BMS_State_t;

typedef enum {
    BMS_CURRENT_IDLE = 0,
    BMS_CURRENT_CHARGE,
    BMS_CURRENT_DISCHARGE
} BMS_CurrentDirection_t;

typedef enum {
    BMS_CURRENT_CALIBRATION_OK = 0,
    BMS_CURRENT_CALIBRATION_BAD_INPUT,
    BMS_CURRENT_CALIBRATION_ZERO_READING,
    BMS_CURRENT_CALIBRATION_DEVIATION_TOO_HIGH,
    BMS_CURRENT_CALIBRATION_WRITE_FAILED
} BMS_CurrentCalibStatus_t;

typedef struct {
    int32_t measured_mA;
    uint32_t deviation_ppm;
    // uint32_t oldGain_ppm;
    uint32_t newGain_ppm;
} BMS_CurrentCalibrationResult_t;

typedef struct {
    bool cellOverVoltage;
    bool cellUnderVoltage;
    bool chargeOverTemperature;
    bool dischargeOverTemperature;
    bool underTemperature;
    bool chargeOverCurrent;
    bool dischargeOverCurrent;
    bool shortCircuit;
    bool bqSafetyFault;
    bool communicationFault;
} BMS_FaultFlags_t;

typedef struct // 58B
{
    /* data */
    uint8_t  IndexAccumulated[BMS_NUMBER_OF_CELLS];     // 10B
    uint16_t RealTimeAccumulated[BMS_NUMBER_OF_CELLS];  // 20B
    uint16_t cellNum[BMS_NUMBER_OF_CELLS];         // 20B
    uint16_t minCellVoltage;
    uint16_t maxCellVoltage;
    uint16_t averageCellVoltage;
    uint16_t deltaCellVoltage;
}cellVoltages_t;


typedef struct {
    bool                    initialized;
    bool                    connected;
    BMS_State_t             state;
    BMS_CurrentDirection_t  currentDirection;
    cellVoltages_t          cellVoltages;

    // uint16_t cellVoltages[BMS_NUMBER_OF_CELLS];
    // uint16_t minCellVoltage;
    // uint16_t maxCellVoltage;
    // uint16_t averageCellVoltage;
    // uint16_t deltaCellVoltage;
    uint16_t stackVoltage;
    uint16_t packVoltage;
    uint16_t circle_counter;

    int32_t  current_mA;
    int16_t  temperature[BMS_NUMBER_OF_THERMISTORS];

    bool     charging;
    bool     discharging;
    bool     chargeFetEnabled;
    bool     dischargeFetEnabled;
    bool     fetsEnabled;
    bool     bqChargeFetBlocked;
    bool     bqDischargeFetBlocked;
    bq76952_AlamStatus_t        bqAlarmRawStatus;

    BMS_FaultFlags_t faults;
    bool     chargeDisabled;
    bool     dischargeDisabled;
    /* Legacy field names: these store raw DCHG/DDSG pin levels.
     * High means the corresponding BQ FET output is disabled, not necessarily faulted.
     */
    bool     chargeGateFaultSignal;
    bool     dischargeGateFaultSignal;
    bool     fetOffAsserted;
    bool     alertActive;
    uint32_t alertCounter;
    bool     bqSleepMode;
    bool     bqSleepAllowed;
    bool     batSenseEnabled;
    uint16_t batAdcEstimatedPack_mV;
    bool     balanceRequired;
    uint16_t balanceMask;

    uint64_t chargeAccumulated_mAs;
    uint64_t dischargeAccumulated_mAs;
    uint32_t chargeThroughput_mAh;
    uint32_t equivalentCycle_milliCycles;
    uint32_t currentCalibrationGainPpm;
} BMS_Tracking_t;

void BMS_Init(void);
void BMS_Update(void);
const BMS_Tracking_t *BMS_GetTracking(void);
bool BMS_IsFaultActive(void);
BMS_CurrentCalibStatus_t BMS_CalibrateCurrent(int32_t actual_mA, BMS_CurrentCalibrationResult_t *result);
void BMS_Error_Handler(void);
void BMS_NotifyAlertInterrupt(void);
void BMS_RequestShutdown(void);
bool BMS_Set_5V_Output(bool enabled);

const char *BMS_StateName(BMS_State_t state);

#endif
