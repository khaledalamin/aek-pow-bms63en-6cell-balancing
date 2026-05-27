#include "AEK_POW_BMS63CHAIN_app_mng.h"

#include "components.h"
#include "AEK_POW_BMS63CHAIN_chain_mng.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * Some embedded toolchains do not expose strtoul() prototype correctly,
 * even when <stdlib.h> is included. Declare it explicitly to avoid
 * implicit-declaration warnings.
 */
extern unsigned long strtoul(const char *nptr, char **endptr, int base);
/*
 * Active-cell balancing mask for the current 6-cell physical configuration.
 *
 * Active cells:
 *   CELL1, CELL2, CELL3, CELL4, CELL13, CELL14
 *
 * Unused/bridged cells:
 *   CELL5 to CELL12
 *
 * Bit mapping:
 *   CELL1  -> bit 0  -> 0x0001
 *   CELL2  -> bit 1  -> 0x0002
 *   CELL3  -> bit 2  -> 0x0004
 *   CELL4  -> bit 3  -> 0x0008
 *   CELL13 -> bit 12 -> 0x1000
 *   CELL14 -> bit 13 -> 0x2000
 *
 * Therefore:
 *   allowed mask = 0x300F
 */
#define AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK      ((uint16_t)0x300FU)
#define AEK_POW_BMS63CHAIN_ALL_CELL_MASK         ((uint16_t)0x3FFFU)

/*
 * Automatic balancing thresholds.
 *
 * Start balancing when the spread between highest and lowest active cell
 * is above 30 mV.
 *
 * Stop balancing when the spread becomes smaller than 15 mV.
 */
#define AEK_POW_BMS63CHAIN_BAL_START_DELTA_V     (0.030F)
#define AEK_POW_BMS63CHAIN_BAL_STOP_DELTA_V      (0.015F)

#define AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM          39.0F
#define AEK_POW_BMS63CHAIN_BAL_CELL_CAPACITY_MAH     3500.0F

/*
 * First AUTO version:
 * balance only one cell at a time.
 */
#define AEK_POW_BMS63CHAIN_BAL_ONE_CELL_ONLY     1U

/*
 * Local app-level alias.
 * Some older parts of this file still use BAL_ALLOWED_MASK.
 * Force it to mean the real active-cell mask for the 6-cell setup.
 */
#ifdef AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK
#undef AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK
#endif

#define AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK      AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK


#define AEK_POW_BMS63CHAIN_SERIAL_RX_BUFFER_SIZE 64U
#define AEK_POW_BMS63CHAIN_SERIAL_CMD_BUFFER_SIZE 64U

/*
 * AUTO balancing v2.
 *
 * Strategy:
 * - Balance only during near-rest condition.
 * - Start when max-min spread is large.
 * - Stop when max-min spread is small.
 * - Balance only one cell at a time.
 * - Use hysteresis so the selected cell does not jump between almost equal cells.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_START_DELTA_V          0.030F  /* 30 mV: start balancing */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_STOP_DELTA_V           0.012F  /* 12 mV: stop balancing */

/*
 * Tie / switching margin.
 * If the current selected cell is within 6 mV of the highest cell,
 * keep balancing the same cell instead of jumping to another cell.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_RESELECT_MARGIN_V      0.006F  /* 6 mV */

/*
 * Energy-aware tie group.
 * Cells within this voltage margin from the maximum are treated as tied.
 * Among tied cells, AUTO prefers the one with the least removed mAh.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_TIE_MARGIN_V          0.006F  /* 6 mV */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_STICKY_MAH_MARGIN     1.0F    /* 1 mAh */

/*
 * Safety voltage window.
 * For your current bench tests, 3.30 V is acceptable.
 * For production-style top balancing, this would usually be higher.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE       3.30F
#define AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CELL_VOLTAGE       4.18F

/*
 * Balance only when pack current is very small.
 * Your idle current is around 0.0003-0.0008 A, so 0.05 A is generous.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CURRENT_A          0.05F

/*
 * Pulse/cooldown.
 * You tested 60 s ON / 60 s OFF without heating, so we keep it.
 */
#define AEK_POW_BMS63CHAIN_BAL_AUTO_PULSE_MS               60000U
#define AEK_POW_BMS63CHAIN_BAL_AUTO_COOLDOWN_MS            60000U

#define AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL           0xFFU

/*
 * AUTO balancing strategy modes.
 * FAIR_SINGLE is the safest default: one cell at a time.
 * MULTI2 is experimental/professional characterization mode:
 *   - at most two cells simultaneously
 *   - only cells in the high-voltage tie group
 *   - never adjacent channels
 *   - total estimated balancing power is capped
 */
#define AEK_POW_BMS63CHAIN_BAL_STRATEGY_FAIR_SINGLE        0U
#define AEK_POW_BMS63CHAIN_BAL_STRATEGY_MULTI2             1U
#define AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_CELLS             2U
#define AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_POWER_W           0.75F



static RT_MODEL rtMdev1_;
static RT_MODEL *const rtMPtrdev1 = &rtMdev1_; /* Real-time model */
static DW rtDWdev1;
static uint8_t AEK_POW_BMS63CHAIN_serialRxBuffer[AEK_POW_BMS63CHAIN_SERIAL_RX_BUFFER_SIZE];
static char AEK_POW_BMS63CHAIN_serialCmdBuffer[AEK_POW_BMS63CHAIN_SERIAL_CMD_BUFFER_SIZE];
static uint8_t AEK_POW_BMS63CHAIN_serialCmdIdx = 0U;
static volatile AEK_POW_BMS63CHAIN_balCtrlMode_t AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
static volatile uint16_t AEK_POW_BMS63CHAIN_balManualMask = 0U;
static volatile uint8_t AEK_POW_BMS63CHAIN_balClearRequest = 1U;
static volatile uint8_t AEK_POW_BMS63CHAIN_balStrategyMode = AEK_POW_BMS63CHAIN_BAL_STRATEGY_FAIR_SINGLE;

static volatile uint8_t AEK_POW_BMS63CHAIN_serialStreamEnabled = 1U;

static uint16_t AEK_POW_BMS63CHAIN_balAutoMask = 0U;
static uint8_t AEK_POW_BMS63CHAIN_balAutoActive = 0U;
static uint8_t AEK_POW_BMS63CHAIN_balAutoCooldown = 0U;
static uint32_t AEK_POW_BMS63CHAIN_balAutoTimerMs = 0U;

/*
 * AUTO v2 diagnostic/state variables.
 * These are useful for debugging and for avoiding cell-selection chatter.
 */
static uint8_t AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
static uint8_t AEK_POW_BMS63CHAIN_balAutoLastMinCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
static uint8_t AEK_POW_BMS63CHAIN_balAutoLastMaxCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;

static float AEK_POW_BMS63CHAIN_balAutoLastMinV = 0.0F;
static float AEK_POW_BMS63CHAIN_balAutoLastMaxV = 0.0F;
static float AEK_POW_BMS63CHAIN_balAutoLastDeltaV = 0.0F;

static float AEK_POW_BMS63CHAIN_balRemoved_mAh[14] = {0.0F};
static float AEK_POW_BMS63CHAIN_balRemoved_Wh[14]  = {0.0F};
static uint32_t AEK_POW_BMS63CHAIN_balOnTimeMs[14] = {0U};

static uint32_t AEK_POW_BMS63CHAIN_balAccountingLastMs = 0U;

static uint16_t AEK_POW_BMS63CHAIN_app_getActualBalMask(void);
static const char *AEK_POW_BMS63CHAIN_app_balModeText(void);
static const char *AEK_POW_BMS63CHAIN_app_autoStateText(void);
static const char *AEK_POW_BMS63CHAIN_app_balStrategyText(void);
static void AEK_POW_BMS63CHAIN_app_sendBalancingStrategyReport(void);
static uint8_t AEK_POW_BMS63CHAIN_app_balancingSafetyOk(void);
static void AEK_POW_BMS63CHAIN_app_sendBalancingSafetyReport(void);

static uint16_t AEK_POW_BMS63CHAIN_app_sanitizeBalMask(uint16_t requestedMask);
static uint8_t AEK_POW_BMS63CHAIN_app_balMaskTouchesUnusedCells(uint16_t requestedMask);

/*
 * Forward declarations.
 * These functions are defined later in this file, but some of them are used
 * earlier by the serial command parser.
 */
static float AEK_POW_BMS63CHAIN_app_absf(float value);
static void AEK_POW_BMS63CHAIN_app_resetAutoBalancing(void);
static uint16_t AEK_POW_BMS63CHAIN_app_computeAutoBalancingMask(AEK_POW_BMS63CHAIN_nodeData_t *nodeData);
static uint16_t AEK_POW_BMS63CHAIN_app_buildMultiHighCellMask(AEK_POW_BMS63CHAIN_nodeData_t *nodeData, float maxVoltage, uint8_t firstCellIdx);
static void AEK_POW_BMS63CHAIN_app_printFaultFlag(uint16_t value, char *name, uint16_t *activeCount);
static void AEK_POW_BMS63CHAIN_app_sendFaultReport(void);

static void AEK_POW_BMS63CHAIN_app_sendTrimReport(void);

static void AEK_POW_BMS63CHAIN_app_sendAutoDebug(void);

static uint16_t AEK_POW_BMS63CHAIN_app_sanitizeBalMask(uint16_t requestedMask)
{
    /*
     * Keep only the cells physically present in the pack.
     * This prevents accidental balancing of CELL5-CELL12.
     */
    return (uint16_t)(requestedMask & AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK);
}

static uint8_t AEK_POW_BMS63CHAIN_app_balMaskTouchesUnusedCells(uint16_t requestedMask)
{
    uint16_t unusedMask;

    unusedMask = (uint16_t)(
        AEK_POW_BMS63CHAIN_ALL_CELL_MASK &
        ((uint16_t)(~AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK))
    );

    return ((requestedMask & unusedMask) != 0U) ? 1U : 0U;
}

static SerialConfig AEK_POW_BMS63CHAIN_serialBufferedConfig = {
	115200,
	SD_MODE_8BITS_PARITY_NONE,
	SPC5_LIN_API_MODE_BUFFERED_IO,
	NULL,
	NULL,
	FALSE,
	NULL,
	AEK_POW_BMS63CHAIN_SERIAL_RX_BUFFER_SIZE,
	AEK_POW_BMS63CHAIN_serialRxBuffer
};

static void AEK_POW_BMS63CHAIN_app_updateBalancingAccounting(void);
static void AEK_POW_BMS63CHAIN_app_sendBalancingEnergyReport(void);

static void AEK_POW_BMS63CHAIN_app_sendBalancingSummary(void);


AEK_POW_BMS63CHAIN_app_dataChain_t AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN_NUM];
extern AEK_POW_BMS63CHAIN_chain_t AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_CHAIN_NUM];


static void sendMessage(char *outputMessage);




static uint8_t AEK_POW_BMS63CHAIN_app_balancingSafetyOk(void)
{
    uint8_t i;
    uint16_t cellMask;
    float vcell;

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    AEK_POW_BMS63CHAIN_fastDiag_t *diag =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastDiag[devidx - 1U];

    AEK_POW_BMS63CHAIN_fastMeas_t *meas =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastMeas[devidx - 1U];

    /* VREF sanity. Normal in this bench setup is around 4.97 V. */
    if((nodeData->AEK_POW_BMS63CHAIN_Vref < 4.5F) ||
       (nodeData->AEK_POW_BMS63CHAIN_Vref > 5.2F)){
        return 0U;
    }

    if((meas->AEK_POW_BMS63CHAIN_VTrefMeas < 4.5F) ||
       (meas->AEK_POW_BMS63CHAIN_VTrefMeas > 5.2F)){
        return 0U;
    }

    /* Trim / EEPROM / RAM integrity. */
    if(diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalOff != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalRam != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_eepromCrcErrSect0 != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_RamCrcErr != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_trimmCalOK == 0U){ return 0U; }
    if(meas->AEK_POW_BMS63CHAIN_trimmCalOK == 0U){ return 0U; }

    /* Critical hardware/reference faults. Do not block on wuIsoLine. */
    if(diag->AEK_POW_BMS63CHAIN_OTchip != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_lossAgnd != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_lossDgnd != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_lossCgnd != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_lossGndRef != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_vtRefUV != 0U){ return 0U; }
    if(diag->AEK_POW_BMS63CHAIN_vtRefOV != 0U){ return 0U; }

    /* Balance only near rest. */
    if(AEK_POW_BMS63CHAIN_app_absf(nodeData->AEK_POW_BMS63CHAIN_Pack_Current) >
       AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CURRENT_A){
        return 0U;
    }

    /* Active-cell voltage sanity. This blocks impossible 5.5 V measurement states. */
    for(i = 0U; i < 14U; i++){
        cellMask = (uint16_t)(1U << i);

        if(((AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK & cellMask) != 0U) &&
           (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[i] != 0U)){

            vcell = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[i];

            if((vcell < AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE) ||
               (vcell > AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CELL_VOLTAGE)){
                return 0U;
            }
        }
    }

    return 1U;
}

static const char *AEK_POW_BMS63CHAIN_app_autoStateText(void)
{
    if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_OFF){
        return "OFF";
    }

    if(AEK_POW_BMS63CHAIN_app_balancingSafetyOk() == 0U){
        return "BLOCKED_SAFETY";
    }

    if(AEK_POW_BMS63CHAIN_balAutoActive != 0U){
        return "BALANCING";
    }

    if(AEK_POW_BMS63CHAIN_balAutoCooldown != 0U){
        return "COOLDOWN";
    }

    if((AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO) &&
       (AEK_POW_BMS63CHAIN_balAutoLastDeltaV <= AEK_POW_BMS63CHAIN_BAL_AUTO_STOP_DELTA_V) &&
       (AEK_POW_BMS63CHAIN_balAutoLastDeltaV > 0.0F)){
        return "STOPPED_DELTA_SMALL";
    }

    if((AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO) &&
       (AEK_POW_BMS63CHAIN_balAutoLastDeltaV < AEK_POW_BMS63CHAIN_BAL_AUTO_START_DELTA_V)){
        return "WAIT_DELTA";
    }

    return "READY";
}

static void AEK_POW_BMS63CHAIN_app_sendBalancingSafetyReport(void)
{
    char message[128];

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0]
            .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1U];

    sprintf(message,
            "OK;BAL_SAFETY;STATUS;%s;STATE;%s;VREF;%.4f;CURR;%.4f;\r\n",
            (AEK_POW_BMS63CHAIN_app_balancingSafetyOk() != 0U) ? "OK" : "BLOCKED",
            AEK_POW_BMS63CHAIN_app_autoStateText(),
            nodeData->AEK_POW_BMS63CHAIN_Vref,
            nodeData->AEK_POW_BMS63CHAIN_Pack_Current);

    sendMessage(message);
}

static void AEK_POW_BMS63CHAIN_app_sendBalancingSummary(void)
{
    /*
     * Compact, safe, multi-line summary.
     *
     * IMPORTANT:
     * We intentionally avoid one very long sprintf() line here.
     * On embedded targets, long floating-point sprintf() calls can consume
     * stack and make debugging harder. These shorter lines are safer.
     */
    char message[192];

    uint8_t i;
    uint8_t minCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    uint8_t maxCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    uint8_t actualCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    uint8_t selectedCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;

    uint16_t cellMask;
    uint16_t actualMask;

    float vcell;
    float minV = 100.0F;
    float maxV = 0.0F;
    float deltaMv = 0.0F;

    float selectedV = 0.0F;
    float selectedIbal_mA = 0.0F;
    float selectedOnTime_s = 0.0F;
    float selectedRemoved_mAh = 0.0F;
    float selectedRemoved_Wh = 0.0F;
    float selectedSocRemoved_pct = 0.0F;

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    actualMask = AEK_POW_BMS63CHAIN_app_getActualBalMask();

    for(i = 0U; i < 14U; i++){
        cellMask = (uint16_t)(1U << i);

        if(((AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK & cellMask) != 0U) &&
           (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[i] != 0U)){

            vcell = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[i];

            if(vcell < minV){
                minV = vcell;
                minCellIdx = i;
            }

            if(vcell > maxV){
                maxV = vcell;
                maxCellIdx = i;
            }
        }

        if((actualMask & cellMask) != 0U){
            actualCellIdx = i;
        }
    }

    if((minCellIdx != AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) &&
       (maxCellIdx != AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL)){
        deltaMv = (maxV - minV) * 1000.0F;
    }

    if(actualCellIdx != AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL){
        selectedCellIdx = actualCellIdx;
    }
    else{
        selectedCellIdx = AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx;
    }

    if(selectedCellIdx != AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL){
        selectedV = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[selectedCellIdx];

        if((selectedV > AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE) &&
           (selectedV < AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CELL_VOLTAGE)){
            selectedIbal_mA = (selectedV / AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM) * 1000.0F;
        }

        selectedOnTime_s =
            ((float)AEK_POW_BMS63CHAIN_balOnTimeMs[selectedCellIdx]) / 1000.0F;

        selectedRemoved_mAh =
            AEK_POW_BMS63CHAIN_balRemoved_mAh[selectedCellIdx];

        selectedRemoved_Wh =
            AEK_POW_BMS63CHAIN_balRemoved_Wh[selectedCellIdx];

        selectedSocRemoved_pct =
            (selectedRemoved_mAh / AEK_POW_BMS63CHAIN_BAL_CELL_CAPACITY_MAH) * 100.0F;
    }

    sendMessage("OK;BAL_SUMMARY;BEGIN;\r\n");

    sprintf(message,
            "BAL_SUMMARY;MODE;%s;ACTUAL;0x%04X;ACTIVE;%u;COOLDOWN;%u;\r\n",
            AEK_POW_BMS63CHAIN_app_balModeText(),
            (unsigned int)actualMask,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoActive,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoCooldown);
    sendMessage(message);

    sprintf(message,
            "BAL_SUMMARY;MIN_CELL;%u;MIN_V;%.4f;MAX_CELL;%u;MAX_V;%.4f;DELTA_MV;%.1f;\r\n",
            (minCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(minCellIdx + 1U),
            minV,
            (maxCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(maxCellIdx + 1U),
            maxV,
            deltaMv);
    sendMessage(message);

    sprintf(message,
            "BAL_SUMMARY;SELECTED;%u;VSEL;%.4f;IBAL_mA;%.1f;ON_TIME_S;%.1f;\r\n",
            (selectedCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(selectedCellIdx + 1U),
            selectedV,
            selectedIbal_mA,
            selectedOnTime_s);
    sendMessage(message);

    sprintf(message,
            "BAL_SUMMARY;REMOVED_mAh;%.3f;REMOVED_Wh;%.5f;SOC_REMOVED_PCT;%.4f;\r\n",
            selectedRemoved_mAh,
            selectedRemoved_Wh,
            selectedSocRemoved_pct);
    sendMessage(message);

    sendMessage("OK;BAL_SUMMARY;END;\r\n");
}

static void AEK_POW_BMS63CHAIN_app_sendBalancingEnergyReport(void)
{
    char message[192];
    uint8_t i;
    float socRemovedPct;

    sendMessage("OK;BAL_ENERGY;BEGIN;\r\n");

    for(i = 0U; i < 14U; i++){

        if((AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK & ((uint16_t)(1U << i))) != 0U){

            socRemovedPct =
                (AEK_POW_BMS63CHAIN_balRemoved_mAh[i] /
                 AEK_POW_BMS63CHAIN_BAL_CELL_CAPACITY_MAH) * 100.0F;

            sprintf(message,
                    "BAL_ENERGY;CELL;%u;ON_TIME_S;%.1f;REMOVED_mAh;%.3f;REMOVED_Wh;%.5f;SOC_REMOVED_PCT;%.4f;\r\n",
                    (unsigned int)(i + 1U),
                    ((float)AEK_POW_BMS63CHAIN_balOnTimeMs[i]) / 1000.0F,
                    AEK_POW_BMS63CHAIN_balRemoved_mAh[i],
                    AEK_POW_BMS63CHAIN_balRemoved_Wh[i],
                    socRemovedPct);

            sendMessage(message);
        }
    }

    sendMessage("OK;BAL_ENERGY;END;\r\n");
}

static void AEK_POW_BMS63CHAIN_app_updateBalancingAccounting(void)
{
    uint8_t i;
    uint32_t nowMs;
    uint32_t dtMs;
    float dtHours;
    float vcell;
    float ibal;
    float removedAh;
    float removedWh;

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    nowMs = osalThreadGetMilliseconds();

    if(AEK_POW_BMS63CHAIN_balAccountingLastMs == 0U){
        AEK_POW_BMS63CHAIN_balAccountingLastMs = nowMs;
        return;
    }

    dtMs = nowMs - AEK_POW_BMS63CHAIN_balAccountingLastMs;
    AEK_POW_BMS63CHAIN_balAccountingLastMs = nowMs;

    if(dtMs == 0U){
        return;
    }

    dtHours = ((float)dtMs) / 3600000.0F;

    for(i = 0U; i < 14U; i++){

        if(nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[i] != 0U){

            vcell = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[i];

            if((vcell > AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE) &&
               (vcell < AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CELL_VOLTAGE)){

                ibal = vcell / AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM;

                removedAh = ibal * dtHours;
                removedWh = vcell * ibal * dtHours;

                AEK_POW_BMS63CHAIN_balRemoved_mAh[i] += removedAh * 1000.0F;
                AEK_POW_BMS63CHAIN_balRemoved_Wh[i]  += removedWh;
                AEK_POW_BMS63CHAIN_balOnTimeMs[i]    += dtMs;
            }
        }
    }
}

static uint16_t AEK_POW_BMS63CHAIN_app_getActualBalMask(void)
{
    uint16_t mask = 0U;
    uint8_t i;

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    for(i = 0U; i < 14U; i++){
        if(nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[i] != 0U){
            mask |= (uint16_t)(1U << i);
        }
    }

    return AEK_POW_BMS63CHAIN_app_sanitizeBalMask(mask);
}

static char *AEK_POW_BMS63CHAIN_app_skipSpaces(char *text){
	while(*text == ' ' || *text == '\t'){
		text++;
	}
	return text;
}

static const char *AEK_POW_BMS63CHAIN_app_balModeText(void){
	if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO){
		return "AUTO";
	}
	if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL){
		return "MANUAL";
	}
	return "OFF";
}

static const char *AEK_POW_BMS63CHAIN_app_balStrategyText(void)
{
    if(AEK_POW_BMS63CHAIN_balStrategyMode == AEK_POW_BMS63CHAIN_BAL_STRATEGY_MULTI2){
        return "MULTI2";
    }

    return "FAIR_SINGLE";
}

static void AEK_POW_BMS63CHAIN_app_sendBalancingStrategyReport(void)
{
    char message[128];

    sprintf(message,
            "OK;BAL_STRATEGY;MODE;%s;MAX_CELLS;%u;PWR_CAP_W;%.2f;TIE_MV;%.1f;\r\n",
            AEK_POW_BMS63CHAIN_app_balStrategyText(),
            (AEK_POW_BMS63CHAIN_balStrategyMode == AEK_POW_BMS63CHAIN_BAL_STRATEGY_MULTI2) ?
                (unsigned int)AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_CELLS : 1U,
            AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_POWER_W,
            AEK_POW_BMS63CHAIN_BAL_AUTO_TIE_MARGIN_V * 1000.0F);

    sendMessage(message);
}

static void AEK_POW_BMS63CHAIN_app_sendBalStatus(char *prefix)
{
    char message[160];
    uint16_t effectiveMask = 0U;
    uint16_t actualMask = 0U;

    actualMask = AEK_POW_BMS63CHAIN_app_getActualBalMask();

    if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO){
        /*
         * In AUTO mode, the most useful mask is the actual applied mask.
         * balAutoMask may be cleared/reset during pulse/cooldown timing.
         */
        effectiveMask = actualMask;
    }
    else if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL){
        effectiveMask = AEK_POW_BMS63CHAIN_balManualMask;
    }
    else{
        effectiveMask = 0U;
    }

    effectiveMask = AEK_POW_BMS63CHAIN_app_sanitizeBalMask(effectiveMask);

    sprintf(message,
            "%s;BAL;MODE;%s;STRATEGY;%s;MASK;0x%04X;MAN;0x%04X;AUTO_REQ;0x%04X;ACTUAL;0x%04X;ACTIVE;%u;COOLDOWN;%u;\r\n",
            prefix,
            AEK_POW_BMS63CHAIN_app_balModeText(),
            AEK_POW_BMS63CHAIN_app_balStrategyText(),
            (unsigned int)effectiveMask,
            (unsigned int)AEK_POW_BMS63CHAIN_balManualMask,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoMask,
            (unsigned int)actualMask,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoActive,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoCooldown);

    sendMessage(message);
}

static void AEK_POW_BMS63CHAIN_app_printFaultFlag(uint16_t value, char *name, uint16_t *activeCount)
{
    char message[96];

    if(value != 0U){
        sprintf(message, "FAULT;ACTIVE;%s;%u;\r\n", name, (unsigned int)value);
        sendMessage(message);

        if(activeCount != NULL){
            *activeCount = (uint16_t)(*activeCount + 1U);
        }
    }
}

static void AEK_POW_BMS63CHAIN_app_sendAutoDebug(void)
{
    char message[192];

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    sprintf(message,
            "AUTO_DBG;MODE;%s;NODE_VREF;%.4f;CURR;%.4f;MASK;0x%04X;ACTIVE;%u;COOLDOWN;%u;\r\n",
            AEK_POW_BMS63CHAIN_app_balModeText(),
            nodeData->AEK_POW_BMS63CHAIN_Vref,
            nodeData->AEK_POW_BMS63CHAIN_Pack_Current,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoMask,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoActive,
            (unsigned int)AEK_POW_BMS63CHAIN_balAutoCooldown);
    sendMessage(message);

    sprintf(message,
            "AUTO_DBG;MIN_CELL;%u;MIN_V;%.4f;MAX_CELL;%u;MAX_V;%.4f;DELTA_MV;%.1f;SEL_CELL;%u;\r\n",
            (AEK_POW_BMS63CHAIN_balAutoLastMinCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(AEK_POW_BMS63CHAIN_balAutoLastMinCellIdx + 1U),
            AEK_POW_BMS63CHAIN_balAutoLastMinV,
            (AEK_POW_BMS63CHAIN_balAutoLastMaxCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(AEK_POW_BMS63CHAIN_balAutoLastMaxCellIdx + 1U),
            AEK_POW_BMS63CHAIN_balAutoLastMaxV,
            AEK_POW_BMS63CHAIN_balAutoLastDeltaV * 1000.0F,
            (AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL) ?
                0U : (unsigned int)(AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx + 1U));
    sendMessage(message);

    sprintf(message,
            "AUTO_DBG;VC1;%.4f;VC2;%.4f;VC3;%.4f;VC4;%.4f;VC13;%.4f;VC14;%.4f;\r\n",
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[0],
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[1],
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[2],
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[3],
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[12],
            nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[13]);
    sendMessage(message);

    sprintf(message,
            "AUTO_DBG;EN1;%u;EN2;%u;EN3;%u;EN4;%u;EN13;%u;EN14;%u;\r\n",
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[0],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[1],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[2],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[3],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[12],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[13]);
    sendMessage(message);

    sprintf(message,
            "AUTO_DBG;CMD1;%u;CMD2;%u;CMD3;%u;CMD4;%u;CMD13;%u;CMD14;%u;EN_STS;%u;\r\n",
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[0],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[1],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[2],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[3],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[12],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[13],
            (unsigned int)nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts);
    sendMessage(message);
}


static void AEK_POW_BMS63CHAIN_app_sendFaultReport(void)
{
    char message[128];

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
    uint16_t activeCount = 0U;

    AEK_POW_BMS63CHAIN_fastDiag_t *diag =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastDiag[devidx - 1U];

    AEK_POW_BMS63CHAIN_fastMeas_t *meas =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastMeas[devidx - 1U];

    AEK_POW_BMS63CHAIN_nodeData_t *nodeData =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_nodeData[devidx - 1U];

    sendMessage("OK;FAULT_REPORT;BEGIN;\r\n");
    uint16_t effectiveBalMask = 0U;

    if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO){
        effectiveBalMask = AEK_POW_BMS63CHAIN_app_getActualBalMask();
    }
    else if(AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL){
        effectiveBalMask = AEK_POW_BMS63CHAIN_balManualMask;
    }
    else{
        effectiveBalMask = 0U;
    }

    effectiveBalMask = AEK_POW_BMS63CHAIN_app_sanitizeBalMask(effectiveBalMask);
    sprintf(message,
            "MEAS;VREF;%.3f;VTOT;%.2f;CURR;%.4f;BALMODE;%s;BALMASK;0x%04X;\r\n",
            meas->AEK_POW_BMS63CHAIN_VTrefMeas,
            meas->AEK_POW_BMS63CHAIN_vPackSum,
            nodeData->AEK_POW_BMS63CHAIN_Pack_Current,
            AEK_POW_BMS63CHAIN_app_balModeText(),
            (unsigned int)effectiveBalMask);
    sendMessage(message);

    sprintf(message,
            "SPI;TXRX;%d;FRAME;%d;GSW;%d;\r\n",
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_txRxSts,
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_frameErrSts,
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_GSWErrSts);
    sendMessage(message);

    /*
     * Critical/general diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_ovrLatch, "ovrLatch", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_tCycleOvf, "tCycleOvf", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_sensePlusOpen, "sensePlusOpen", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_senseMinusOpen, "senseMinusOpen", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_OTchip, "OTchip", &activeCount);

    /*
     * Internal supply/reference diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vAnaOV, "vAnaOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vDigOV, "vDigOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vtRefUV, "vtRefUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vtRefOV, "vtRefOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vtRegUV, "vtRegUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vtRegOV, "vtRegOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vComUV, "vComUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vtComOV, "vtComOV", &activeCount);

    /*
     * Wake-up and fault-line indicators.
     * Some of these may be persistent status flags rather than dangerous faults.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_wuGpio7, "wuGpio7", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_wuSpi, "wuSpi", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_wuIsoLine, "wuIsoLine", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_wufaultH, "wufaultH", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_wuCycWup, "wuCycWup", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_faultLlineSts, "faultLlineSts", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_faultHlineSts, "faultHlineSts", &activeCount);

    /*
     * Ground/reference diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_lossAgnd, "lossAgnd", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_lossDgnd, "lossDgnd", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_lossCgnd, "lossCgnd", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_lossGndRef, "lossGndRef", &activeCount);

    /*
     * EEPROM / CRC / memory diagnostics.
     * These are the likely source of your persistent GUI fault.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalOff, "eepromCrcErrCalOff", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_eepromCrcErrSect0, "eepromCrcErrSect0", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalRam, "eepromCrcErrCalRam", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_RamCrcErr, "RamCrcErr", &activeCount);

    /*
     * Cell open diagnostics for your active cells.
     * Current 6-cell setup: CELL1, CELL2, CELL3, CELL4, CELL13, CELL14.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel1Open, "cell1Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel2Open, "cell2Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel3Open, "cell3Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel4Open, "cell4Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel13Open, "cell13Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_cel14Open, "cell14Open", &activeCount);

    /*
     * Active-cell under/over-voltage diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell1UV, "cell1UV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell2UV, "cell2UV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell3UV, "cell3UV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell4UV, "cell4UV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell13UV, "cell13UV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell14UV, "cell14UV", &activeCount);

    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell1OV, "cell1OV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell2OV, "cell2OV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell3OV, "cell3OV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell4OV, "cell4OV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell13OV, "cell13OV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell14OV, "cell14OV", &activeCount);

    /*
     * Pack-level voltage diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vBattWrnOV, "vBattWrnOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vBattWrnUV, "vBattWrnUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vBattCritOV, "vBattCritOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vBattCritUV, "vBattCritUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vSumOV, "vSumOV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vSumUV, "vSumUV", &activeCount);

    /*
     * Balancing diagnostics for active cells.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal1Open, "bal1Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal2Open, "bal2Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal3Open, "bal3Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal4Open, "bal4Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal13Open, "bal13Open", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal14Open, "bal14Open", &activeCount);

    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal1Short, "bal1Short", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal2Short, "bal2Short", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal3Short, "bal3Short", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal4Short, "bal4Short", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal13Short, "bal13Short", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bal14Short, "bal14Short", &activeCount);

    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell1BalUV, "cell1BalUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell2BalUV, "cell2BalUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell3BalUV, "cell3BalUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell4BalUV, "cell4BalUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell13BalUV, "cell13BalUV", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vCell14BalUV, "cell14BalUV", &activeCount);

    /*
     * GPIO/temperature diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio9FastChgOT, "gpio9FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio8FastChgOT, "gpio8FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio7FastChgOT, "gpio7FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio6FastChgOT, "gpio6FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio5FastChgOT, "gpio5FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio4FastChgOT, "gpio4FastChgOT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio3FastChgOT, "gpio3FastChgOT", &activeCount);

    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio9OT, "gpio9OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio8OT, "gpio8OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio7OT, "gpio7OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio6OT, "gpio6OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio5OT, "gpio5OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio4OT, "gpio4OT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio3OT, "gpio3OT", &activeCount);

    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio9UT, "gpio9UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio8UT, "gpio8UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio7UT, "gpio7UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio6UT, "gpio6UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio5UT, "gpio5UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio4UT, "gpio4UT", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpio3UT, "gpio3UT", &activeCount);

    /*
     * BIST / monitor / current-sense diagnostics.
     */
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vBatCompBistFail, "vBatCompBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vRegCompBistFail, "vRegCompBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vComCompBistFail, "vComCompBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_vTrefCompBistFail, "vTrefCompBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_gpioBistFail, "gpioBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_muxBistFail, "muxBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bistBalCompHsFail, "bistBalCompHsFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_bistBalCompLsFail, "bistBalCompLsFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_openBistFail, "openBistFail", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_curSenseOvcSleep, "curSenseOvcSleep", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_curSenseOvcNorm, "curSenseOvcNorm", &activeCount);
    AEK_POW_BMS63CHAIN_app_printFaultFlag(diag->AEK_POW_BMS63CHAIN_oscFail, "oscFail", &activeCount);

    sprintf(message, "OK;FAULT_REPORT;ACTIVE_COUNT;%u;\r\n", (unsigned int)activeCount);
    sendMessage(message);

    if(activeCount == 0U){
        sendMessage("OK;FAULT_REPORT;NO_ACTIVE_FLAGS;\r\n");
    }

    sendMessage("OK;FAULT_REPORT;END;\r\n");
}

static void AEK_POW_BMS63CHAIN_app_sendTrimReport(void)
{
    char message[160];

    uint8_t chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
    uint8_t devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;

    AEK_POW_BMS63CHAIN_fastDiag_t *diag =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastDiag[devidx - 1U];

    AEK_POW_BMS63CHAIN_fastMeas_t *meas =
        &AEK_POW_BMS63CHAIN_app_dataChain[chainidx]
            .AEK_POW_BMS63CHAIN_fastMeas[devidx - 1U];

    sendMessage("OK;TRIM_REPORT;BEGIN;\r\n");

    /*
     * Cached diagnostic values only.
     * No direct SPI/register reads here.
     */
    sprintf(message,
            "TRIM;CACHED;EEPROM_DONE;%u;TRIM_CAL_OK_DIAG;%u;TRIM_CAL_OK_MEAS;%u;DRDY_VTREF;%u;\r\n",
            (unsigned int)diag->AEK_POW_BMS63CHAIN_eepromDwnldDone,
            (unsigned int)diag->AEK_POW_BMS63CHAIN_trimmCalOK,
            (unsigned int)meas->AEK_POW_BMS63CHAIN_trimmCalOK,
            (unsigned int)meas->AEK_POW_BMS63CHAIN_drdyVtref);
    sendMessage(message);

    sprintf(message,
            "TRIM;CRC;EEPROM_CALOFF;%u;EEPROM_CALRAM;%u;EEPROM_SECT0;%u;RAMCRC;%u;\r\n",
            (unsigned int)diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalOff,
            (unsigned int)diag->AEK_POW_BMS63CHAIN_eepromCrcErrCalRam,
            (unsigned int)diag->AEK_POW_BMS63CHAIN_eepromCrcErrSect0,
            (unsigned int)diag->AEK_POW_BMS63CHAIN_RamCrcErr);
    sendMessage(message);

    sprintf(message,
            "RAWMEAS;VC1;%.4f;VC2;%.4f;VC3;%.4f;VC4;%.4f;VC13;%.4f;VC14;%.4f;VPACKSUM;%.4f;VTREF;%.4f;\r\n",
            meas->AEK_POW_BMS63CHAIN_vCell1,
            meas->AEK_POW_BMS63CHAIN_vCell2,
            meas->AEK_POW_BMS63CHAIN_vCell3,
            meas->AEK_POW_BMS63CHAIN_vCell4,
            meas->AEK_POW_BMS63CHAIN_vCell13,
            meas->AEK_POW_BMS63CHAIN_vCell14,
            meas->AEK_POW_BMS63CHAIN_vPackSum,
            meas->AEK_POW_BMS63CHAIN_VTrefMeas);
    sendMessage(message);

    sprintf(message,
            "SPI;TXRX;%d;FRAME;%d;GSW;%d;\r\n",
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_txRxSts,
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_frameErrSts,
            (int)AEK_POW_BMS63CHAIN_chain[chainidx]
                .AEK_POW_BMS63CHAIN_spi_chainConf
                .AEK_POW_BMS63CHAIN_spi_GSWErrSts);
    sendMessage(message);

    sendMessage("OK;TRIM_REPORT;END;\r\n");
}

static void AEK_POW_BMS63CHAIN_app_uppercase(char *text){
	while(*text != '\0'){
		if(*text >= 'a' && *text <= 'z'){
			*text = (char)(*text - ('a' - 'A'));
		}
		text++;
	}
}

static void AEK_POW_BMS63CHAIN_app_processSerialCommand(char *command){
	char *arg;
	char *end;
	unsigned long value;
	uint16_t bitMask;

	AEK_POW_BMS63CHAIN_app_uppercase(command);
	command = AEK_POW_BMS63CHAIN_app_skipSpaces(command);
    if(*command == '\0'){
    return;
    }
	else if((strcmp(command, "STREAM OFF") == 0) ||
   (strcmp(command, "MON OFF") == 0) ||
   (strcmp(command, "S0") == 0)){

    AEK_POW_BMS63CHAIN_serialStreamEnabled = 0U;
    sendMessage("OK;STREAM;OFF;\r\n");
	}

    else if((strcmp(command, "STREAM ON") == 0) ||
            (strcmp(command, "MON ON") == 0) ||
            (strcmp(command, "S1") == 0)){

        AEK_POW_BMS63CHAIN_serialStreamEnabled = 1U;
        sendMessage("OK;STREAM;ON;\r\n");
        }

    else if((strcmp(command, "STREAM?") == 0) ||
            (strcmp(command, "MON?") == 0) ||
            (strcmp(command, "S?") == 0)){

        if(AEK_POW_BMS63CHAIN_serialStreamEnabled){
            sendMessage("OK;STREAM;ON;\r\n");
            }
        else{
            sendMessage("OK;STREAM;OFF;\r\n");
            }
        }

    else if((strcmp(command, "FAULT?") == 0) ||
            (strcmp(command, "FAULT REPORT") == 0) ||
            (strcmp(command, "FLT?") == 0)){

        AEK_POW_BMS63CHAIN_app_sendFaultReport();
        }

    else if((strcmp(command, "TRIM?") == 0) ||
            (strcmp(command, "EEPROM?") == 0) ||
            (strcmp(command, "CALFAULT?") == 0)){

        AEK_POW_BMS63CHAIN_app_sendTrimReport();
    }

    else if((strcmp(command, "BAL SAFETY?") == 0) ||
            (strcmp(command, "BSAFE?") == 0)){

        AEK_POW_BMS63CHAIN_app_sendBalancingSafetyReport();
    }
    /* Shortcut: balance CELL13 only */
    else if((strcmp(command, "B13 ON") == 0) ||
            (strcmp(command, "BAL13 ON") == 0) ||
            (strcmp(command, "CELL13 ON") == 0)){

        AEK_POW_BMS63CHAIN_balManualMask = AEK_POW_BMS63CHAIN_app_sanitizeBalMask((uint16_t)0x1000U);	
        AEK_POW_BMS63CHAIN_balClearRequest = 0U;
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL;
        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
        }

    /* Shortcut: stop all balancing */
    else if((strcmp(command, "B13 OFF") == 0) ||
            (strcmp(command, "BAL13 OFF") == 0) ||
            (strcmp(command, "CELL13 OFF") == 0)){

        AEK_POW_BMS63CHAIN_balManualMask = 0U;
        AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
        AEK_POW_BMS63CHAIN_balClearRequest = 1U;
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
        }

    else if(strcmp(command, "BAL?") == 0 || strcmp(command, "BAL STATUS") == 0){
        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
        }

    else if(strcmp(command, "BAL OFF") == 0){
        AEK_POW_BMS63CHAIN_balManualMask = 0U;
        AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
        AEK_POW_BMS63CHAIN_balClearRequest = 1U;
        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
        }

    else if((strcmp(command, "BAL STRATEGY?") == 0) ||
            (strcmp(command, "BSTRAT?") == 0))
    {
        AEK_POW_BMS63CHAIN_app_sendBalancingStrategyReport();
    }

    else if((strcmp(command, "BAL STRATEGY SINGLE") == 0) ||
            (strcmp(command, "BAL STRATEGY FAIR") == 0) ||
            (strcmp(command, "BAL STRATEGY FAIR_SINGLE") == 0) ||
            (strcmp(command, "BSTRAT SINGLE") == 0))
    {
        AEK_POW_BMS63CHAIN_balStrategyMode = AEK_POW_BMS63CHAIN_BAL_STRATEGY_FAIR_SINGLE;
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        AEK_POW_BMS63CHAIN_app_sendBalancingStrategyReport();
    }

    else if((strcmp(command, "BAL STRATEGY MULTI2") == 0) ||
            (strcmp(command, "BSTRAT MULTI2") == 0))
    {
        AEK_POW_BMS63CHAIN_balStrategyMode = AEK_POW_BMS63CHAIN_BAL_STRATEGY_MULTI2;
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        AEK_POW_BMS63CHAIN_app_sendBalancingStrategyReport();
    }

    else if(strcmp(command, "BAL AUTO") == 0)
    {
        /*
        * AUTO mode:
        * - manual mask cleared
        * - auto state reset
        * - actual auto mask is computed later in applyBalancingControl()
        */
        AEK_POW_BMS63CHAIN_balManualMask = 0U;
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();

        AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO;
        AEK_POW_BMS63CHAIN_balClearRequest = 0U;

        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
    }

    else if(strncmp(command, "BAL MAN ", 8) == 0)
    {
        uint32_t requestedMask32;
        uint16_t requestedMask;
        uint16_t safeMask;
        char message[96];

        requestedMask32 = strtoul(&command[8], NULL, 0);
        requestedMask = (uint16_t)(requestedMask32 & AEK_POW_BMS63CHAIN_ALL_CELL_MASK);

        safeMask = AEK_POW_BMS63CHAIN_app_sanitizeBalMask(requestedMask);

        /*
        * If the user requested unused cells, warn but do not allow them.
        */
        if(AEK_POW_BMS63CHAIN_app_balMaskTouchesUnusedCells(requestedMask) != 0U)
        {
            sprintf(message,
                    "WARN;BAL;UNUSED_CELLS_REMOVED;REQ;0x%04X;SAFE;0x%04X;\r\n",
                    requestedMask,
                    safeMask);
            sendMessage(message);
        }

        AEK_POW_BMS63CHAIN_balManualMask = safeMask;

        if(safeMask == 0U)
        {
            AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
        }
        else
        {
            AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL;
        }

        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
    }

    else if(strncmp(command, "BAL CELL ", 9) == 0)
    {
        arg = AEK_POW_BMS63CHAIN_app_skipSpaces(&command[9]);
        value = strtoul(arg, &end, 10);

        if(end == arg || value < 1UL || value > 14UL){
            sendMessage("ERR;BAL;BAD_CELL;\r\n");
            return;
            }

        bitMask = (uint16_t)(1U << ((uint8_t)value - 1U));

        /*
        * Reject unused cells.
        * Current physical setup allows only:
        * CELL1, CELL2, CELL3, CELL4, CELL13, CELL14.
        */
        if((bitMask & AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK) == 0U){
            sendMessage("ERR;BAL;CELL_NOT_ALLOWED;\r\n");
            return;
            }

        end = AEK_POW_BMS63CHAIN_app_skipSpaces(end);

        if(strcmp(end, "ON") == 0){

            AEK_POW_BMS63CHAIN_balManualMask =
                AEK_POW_BMS63CHAIN_app_sanitizeBalMask(
                    (uint16_t)(AEK_POW_BMS63CHAIN_balManualMask | bitMask)
                );

            AEK_POW_BMS63CHAIN_balClearRequest = 0U;
            AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
            AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL;
            AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
            }
        else if(strcmp(end, "OFF") == 0){

            AEK_POW_BMS63CHAIN_balManualMask =
                AEK_POW_BMS63CHAIN_app_sanitizeBalMask(
                    (uint16_t)(AEK_POW_BMS63CHAIN_balManualMask & (uint16_t)(~bitMask))
                );

            AEK_POW_BMS63CHAIN_app_resetAutoBalancing();

            if(AEK_POW_BMS63CHAIN_balManualMask == 0U){
                AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
                AEK_POW_BMS63CHAIN_balClearRequest = 1U;
                }
            else{
                AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL;
                AEK_POW_BMS63CHAIN_balClearRequest = 0U;
                }

            AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
            }
        else{
            sendMessage("ERR;BAL;BAD_CELL_CMD;\r\n");
            }
        }
    
    else if((strcmp(command, "AUTO?") == 0) ||
        (strcmp(command, "AUTO DBG") == 0) ||
        (strcmp(command, "AUTO DEBUG") == 0))
        {
        AEK_POW_BMS63CHAIN_app_sendAutoDebug();
        }


    else if((strcmp(command, "BAL STATE?") == 0) ||
            (strcmp(command, "BSTATE?") == 0))
        {
        AEK_POW_BMS63CHAIN_app_sendBalancingSafetyReport();
        AEK_POW_BMS63CHAIN_app_sendBalStatus("OK");
        }

    else if((strcmp(command, "BAL ENERGY?") == 0) ||
        (strcmp(command, "BAL EN?") == 0))
        {
        AEK_POW_BMS63CHAIN_app_sendBalancingEnergyReport();
        }

    else if(strcmp(command, "BAL ENERGY RESET") == 0)
        {
        uint8_t i;

        for(i = 0U; i < 14U; i++){
            AEK_POW_BMS63CHAIN_balRemoved_mAh[i] = 0.0F;
            AEK_POW_BMS63CHAIN_balRemoved_Wh[i] = 0.0F;
            AEK_POW_BMS63CHAIN_balOnTimeMs[i] = 0U;
        }

        AEK_POW_BMS63CHAIN_balAccountingLastMs = osalThreadGetMilliseconds();

        sendMessage("OK;BAL_ENERGY;RESET;\r\n");
        }
    else if((strcmp(command, "BAL SUMMARY?") == 0) ||
            (strcmp(command, "BAL SUM?") == 0) ||
            (strcmp(command, "BS?") == 0))
        {
            AEK_POW_BMS63CHAIN_app_sendBalancingSummary();
        }
    else{
            sendMessage("ERR;UNKNOWN_CMD;\r\n");
        }
    }

void AEK_POW_BMS63CHAIN_app_serialPollCommands(void){
	uint8_t rxByte;

	while(SD5.rx_read_ptr != SD5.rx_write_ptr){
		(void)sd_lld_read(&SD5, &rxByte, 1U);
		if(rxByte == '\r' || rxByte == '\n' || rxByte == ';'){
			if(AEK_POW_BMS63CHAIN_serialCmdIdx > 0U){
				AEK_POW_BMS63CHAIN_serialCmdBuffer[AEK_POW_BMS63CHAIN_serialCmdIdx] = '\0';
				AEK_POW_BMS63CHAIN_app_processSerialCommand(AEK_POW_BMS63CHAIN_serialCmdBuffer);
				AEK_POW_BMS63CHAIN_serialCmdIdx = 0U;
			}
		}
		else if(AEK_POW_BMS63CHAIN_serialCmdIdx < (AEK_POW_BMS63CHAIN_SERIAL_CMD_BUFFER_SIZE - 1U)){
			AEK_POW_BMS63CHAIN_serialCmdBuffer[AEK_POW_BMS63CHAIN_serialCmdIdx] = (char)rxByte;
			AEK_POW_BMS63CHAIN_serialCmdIdx++;
		}
		else{
			AEK_POW_BMS63CHAIN_serialCmdIdx = 0U;
			sendMessage("ERR;CMD_TOO_LONG;\r\n");
		}
	}
}

void AEK_POW_BMS63CHAIN_app_initModels(void){
	//INIT MODEL FOR CHAIN0 DEV1
	RT_MODEL *const rtMdev1 = rtMPtrdev1;
	rtMdev1->dwork = &rtDWdev1;
	BMS_Controller_initialize(rtMdev1);
}

void AEK_POW_BMS63CHAIN_app_modelExec(void){
	RT_MODEL *const rtMdev1 = rtMPtrdev1;
	rtMdev1->dwork = &rtDWdev1;
	//Model SOC Estimation FOR CHAIN0 DEV1
	BMS_Controller_step(rtMdev1,                     							\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC,\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage, 		\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_Current,			\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_Enable_Bal,		\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_Enable_Discharge,	\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Cell_Enabled,		    \
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_SOC,				\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_SOH,				\
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_Bal_cmd,			\
			&(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_CHAIN0].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_NODE_DEV1 - 1].AEK_POW_BMS63CHAIN_Pack_Bal_en_sts));
}


float AEK_POW_BMS63CHAIN_app_getNTCResistor(float AEK_POW_BMS63CHAIN_app_gpioVoltage, float AEK_POW_BMS63CHAIN_app_Vtref){
	return (AEK_POW_BMS63CHAIN_app_gpioVoltage * AEK_POW_BMS_RP_NTC) / (AEK_POW_BMS63CHAIN_app_Vtref - AEK_POW_BMS63CHAIN_app_gpioVoltage);
}

void AEK_POW_BMS63CHAIN_app_parserData(void){
	uint8_t AEK_POW_BMS63CHAIN_chainidx = 0;
	uint8_t AEK_POW_BMS63CHAIN_devidx = 0;
	  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0; AEK_POW_BMS63CHAIN_chainidx<AEK_POW_BMS63CHAIN_chain_geNum(); AEK_POW_BMS63CHAIN_chainidx++){
		  for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1; AEK_POW_BMS63CHAIN_devidx<=AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx); AEK_POW_BMS63CHAIN_devidx++){
			//Temperature
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL1 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL2 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL3 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL4 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL5 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL6 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL7 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL8 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL9 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL10 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL11 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL12 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL13 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_CELL14 - 1] = AEK_POW_BMS63CHAIN_app_getNTCResistor(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9Meas, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
			//Voltage
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL1 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell1;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL2 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell2;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL3 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell3;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL4 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell4;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL5 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell5;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL6 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell6;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL7 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell7;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL8 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell8;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL9 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell9;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL10 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell10;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL11 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell11;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL12 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell12;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL13 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell13;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_CELL14 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell14;
			//Current
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Current = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_curInstSync;
            // Vref
            AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Vref = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas;
			//Enable BAL
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Enable_Bal = (AEK_POW_BMS63CHAIN_balCtrlMode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO) ? 1U : 0U;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Enable_Discharge = 0;
			//Enable CEL
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL1 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell1_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL2 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell2_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL3 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell3_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL4 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell4_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL5 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell5_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL6 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell6_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL7 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell7_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL8 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell8_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL9 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell9_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL10 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell10_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL11 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell11_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL12 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell12_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL13 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell13_En;
			AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_CELL14 - 1] = AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell14_En;
		  }
	  }
}

static float AEK_POW_BMS63CHAIN_app_absf(float value)
{
    if(value < 0.0F){
        return -value;
    }
    return value;
}

static void AEK_POW_BMS63CHAIN_app_resetAutoBalancing(void)
{
    AEK_POW_BMS63CHAIN_balAutoMask = 0U;
    AEK_POW_BMS63CHAIN_balAutoActive = 0U;
    AEK_POW_BMS63CHAIN_balAutoCooldown = 0U;
    AEK_POW_BMS63CHAIN_balAutoTimerMs = osalThreadGetMilliseconds();

    AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    AEK_POW_BMS63CHAIN_balAutoLastMinCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    AEK_POW_BMS63CHAIN_balAutoLastMaxCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;

    AEK_POW_BMS63CHAIN_balAutoLastMinV = 0.0F;
    AEK_POW_BMS63CHAIN_balAutoLastMaxV = 0.0F;
    AEK_POW_BMS63CHAIN_balAutoLastDeltaV = 0.0F;
}

static uint16_t AEK_POW_BMS63CHAIN_app_buildMultiHighCellMask(AEK_POW_BMS63CHAIN_nodeData_t *nodeData, float maxVoltage, uint8_t firstCellIdx)
{
    uint8_t i;
    uint8_t selectedCount = 0U;
    uint8_t bestCellIdx;
    uint16_t mask = 0U;
    uint16_t bit;
    float voltage;
    float cellPowerW;
    float totalPowerW = 0.0F;
    float bestRemoved_mAh;

    if(nodeData == NULL){
        return 0U;
    }

    if(firstCellIdx >= 14U){
        return 0U;
    }

    bit = (uint16_t)(1U << firstCellIdx);

    if(((AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK & bit) == 0U) ||
       (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[firstCellIdx] == 0U)){
        return 0U;
    }

    voltage = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[firstCellIdx];
    cellPowerW = (voltage * voltage) / AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM;

    if(cellPowerW > AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_POWER_W){
        return 0U;
    }

    mask = bit;
    totalPowerW = cellPowerW;
    selectedCount = 1U;

    while(selectedCount < AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_CELLS){
        bestCellIdx = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
        bestRemoved_mAh = 1000000.0F;

        for(i = 0U; i < 14U; i++){
            bit = (uint16_t)(1U << i);

            if((mask & bit) != 0U){
                continue;
            }

            if(((AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK & bit) == 0U) ||
               (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[i] == 0U)){
                continue;
            }

            voltage = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[i];

            if(voltage < (maxVoltage - AEK_POW_BMS63CHAIN_BAL_AUTO_TIE_MARGIN_V)){
                continue;
            }

            /*
             * Conservative board-level rule:
             * do not balance adjacent channels at the same time.
             * This reduces local thermal stress and avoids measurement disturbance.
             */
            if((i > 0U) && ((mask & (uint16_t)(1U << (i - 1U))) != 0U)){
                continue;
            }
            if((i < 13U) && ((mask & (uint16_t)(1U << (i + 1U))) != 0U)){
                continue;
            }

            cellPowerW = (voltage * voltage) / AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM;

            if((totalPowerW + cellPowerW) > AEK_POW_BMS63CHAIN_BAL_MULTI_MAX_POWER_W){
                continue;
            }

            if(AEK_POW_BMS63CHAIN_balRemoved_mAh[i] < bestRemoved_mAh){
                bestRemoved_mAh = AEK_POW_BMS63CHAIN_balRemoved_mAh[i];
                bestCellIdx = i;
            }
        }

        if(bestCellIdx == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL){
            break;
        }

        bit = (uint16_t)(1U << bestCellIdx);
        voltage = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[bestCellIdx];
        cellPowerW = (voltage * voltage) / AEK_POW_BMS63CHAIN_BAL_RESISTOR_OHM;

        mask |= bit;
        totalPowerW += cellPowerW;
        selectedCount++;
    }

    return AEK_POW_BMS63CHAIN_app_sanitizeBalMask(mask);
}

static uint16_t AEK_POW_BMS63CHAIN_app_computeAutoBalancingMask(AEK_POW_BMS63CHAIN_nodeData_t *nodeData)
{
    uint8_t cellidx = 0U;
    uint8_t enabledCount = 0U;

    uint8_t minCellIndex = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    uint8_t maxCellIndex = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    uint8_t candidateCellIndex = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;

    uint16_t cellMask = 0U;
    uint16_t selectedMask = 0U;

    uint32_t nowMs = osalThreadGetMilliseconds();

    float voltage = 0.0F;
    float minVoltage = 100.0F;
    float maxVoltage = 0.0F;
    float deltaVoltage = 0.0F;

    float previousSelectedVoltage = 0.0F;
    uint8_t previousSelectedStillValid = 0U;

    uint8_t tieCandidateFound = 0U;
    uint8_t bestTieCellIndex = AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL;
    float bestTieRemoved_mAh = 1000000.0F;
    float previousSelectedRemoved_mAh = 0.0F;

    if(nodeData == NULL){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    /*
     * 1) Measurement sanity: VREF must be realistic.
     * If VREF is wrong, cell measurements cannot be trusted.
     */
    if((nodeData->AEK_POW_BMS63CHAIN_Vref < 4.5F) ||
       (nodeData->AEK_POW_BMS63CHAIN_Vref > 5.2F)){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    /*
     * 2) Balance only near rest.
     * During real current flow, voltage differences include IR drop.
     */
    if(AEK_POW_BMS63CHAIN_app_absf(nodeData->AEK_POW_BMS63CHAIN_Pack_Current) >
       AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CURRENT_A){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    /*
     * 3) Scan only allowed and enabled cells:
     * CELL1, CELL2, CELL3, CELL4, CELL13, CELL14.
     */
    for(cellidx = 0U; cellidx < 14U; cellidx++){
        cellMask = (uint16_t)(1U << cellidx);

        if(((AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK & cellMask) != 0U) &&
           (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[cellidx] != 0U)){

            voltage = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[cellidx];

            /*
             * Per-cell safety window.
             * If any active cell is outside the allowed range, do not balance.
             */
            if((voltage < AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE) ||
               (voltage > AEK_POW_BMS63CHAIN_BAL_AUTO_MAX_CELL_VOLTAGE)){
                AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
                return 0U;
            }

            if(voltage < minVoltage){
                minVoltage = voltage;
                minCellIndex = cellidx;
            }

            if(voltage > maxVoltage){
                maxVoltage = voltage;
                maxCellIndex = cellidx;
            }

            if(cellidx == AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx){
                previousSelectedVoltage = voltage;
                previousSelectedStillValid = 1U;
            }

            enabledCount++;
        }
    }

    if(enabledCount < 2U){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    deltaVoltage = maxVoltage - minVoltage;

    /*
     * Store diagnostics.
     */
    AEK_POW_BMS63CHAIN_balAutoLastMinV = minVoltage;
    AEK_POW_BMS63CHAIN_balAutoLastMaxV = maxVoltage;
    AEK_POW_BMS63CHAIN_balAutoLastDeltaV = deltaVoltage;
    AEK_POW_BMS63CHAIN_balAutoLastMinCellIdx = minCellIndex;
    AEK_POW_BMS63CHAIN_balAutoLastMaxCellIdx = maxCellIndex;

    /*
     * 4) Stop condition.
     * If cells are close enough, stop completely and clear selection.
     */
    if(deltaVoltage <= AEK_POW_BMS63CHAIN_BAL_AUTO_STOP_DELTA_V){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    /*
     * 5) If a pulse is active, keep it active until:
     *    - pulse time expires, or
     *    - selected cell is no longer meaningfully high.
     *
     * The hysteresis prevents jumping between two nearly equal cells.
     */
    if(AEK_POW_BMS63CHAIN_balAutoActive != 0U){

        if((uint32_t)(nowMs - AEK_POW_BMS63CHAIN_balAutoTimerMs) >=
           AEK_POW_BMS63CHAIN_BAL_AUTO_PULSE_MS){

            AEK_POW_BMS63CHAIN_balAutoActive = 0U;
            AEK_POW_BMS63CHAIN_balAutoCooldown = 1U;
            AEK_POW_BMS63CHAIN_balAutoTimerMs = nowMs;
            AEK_POW_BMS63CHAIN_balAutoMask = 0U;

            /*
             * Keep selectedCellIdx during cooldown.
             * This helps the next pulse continue the same cell if it is still high.
             */
            return 0U;
        }

        /*
         * If the selected cell is still within the reselect margin of the maximum,
         * keep balancing it. Example:
         *   CELL4 = 3.5415 V
         *   CELL13 = 3.5407 V
         * Difference is only 0.8 mV, so do not jump.
         */
        if((previousSelectedStillValid != 0U) &&
           (previousSelectedVoltage >=
            (maxVoltage - AEK_POW_BMS63CHAIN_BAL_AUTO_RESELECT_MARGIN_V))){

            return AEK_POW_BMS63CHAIN_app_sanitizeBalMask(AEK_POW_BMS63CHAIN_balAutoMask);
        }

        /*
         * Selected cell is no longer the right choice.
         * Stop and cooldown rather than switching instantly.
         */
        AEK_POW_BMS63CHAIN_balAutoActive = 0U;
        AEK_POW_BMS63CHAIN_balAutoCooldown = 1U;
        AEK_POW_BMS63CHAIN_balAutoTimerMs = nowMs;
        AEK_POW_BMS63CHAIN_balAutoMask = 0U;
        return 0U;
    }

    /*
     * 6) Cooldown phase.
     */
    if(AEK_POW_BMS63CHAIN_balAutoCooldown != 0U){

        if((uint32_t)(nowMs - AEK_POW_BMS63CHAIN_balAutoTimerMs) <
           AEK_POW_BMS63CHAIN_BAL_AUTO_COOLDOWN_MS){
            return 0U;
        }

        AEK_POW_BMS63CHAIN_balAutoCooldown = 0U;
    }

    /*
     * 7) Start condition.
     */
    if(deltaVoltage < AEK_POW_BMS63CHAIN_BAL_AUTO_START_DELTA_V){
        AEK_POW_BMS63CHAIN_balAutoMask = 0U;
        return 0U;
    }

    /*
     * 8) Energy-aware tie selection.
     *
     * If several cells are very close to the highest voltage, they are treated
     * as a tie group. Among tied high cells, choose the one that has received
     * the least balancing so far. This avoids over-working one cell when
     * CELL4 / CELL13 / CELL14 are almost equal.
     */
    bestTieCellIndex = maxCellIndex;
    bestTieRemoved_mAh = AEK_POW_BMS63CHAIN_balRemoved_mAh[maxCellIndex];
    tieCandidateFound = 0U;

    for(cellidx = 0U; cellidx < 14U; cellidx++){
        cellMask = (uint16_t)(1U << cellidx);

        if(((AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK & cellMask) != 0U) &&
           (nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[cellidx] != 0U)){

            voltage = nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[cellidx];

            if(voltage >= (maxVoltage - AEK_POW_BMS63CHAIN_BAL_AUTO_TIE_MARGIN_V)){
                tieCandidateFound = 1U;

                if(AEK_POW_BMS63CHAIN_balRemoved_mAh[cellidx] < bestTieRemoved_mAh){
                    bestTieRemoved_mAh = AEK_POW_BMS63CHAIN_balRemoved_mAh[cellidx];
                    bestTieCellIndex = cellidx;
                }
            }
        }
    }

    if(tieCandidateFound != 0U){
        candidateCellIndex = bestTieCellIndex;
    }
    else{
        candidateCellIndex = maxCellIndex;
    }

    /*
     * 9) Sticky protection.
     * If the previous selected cell is still in the high-voltage tie group and
     * has not received much more balancing than the best fair candidate, keep it.
     */
    if((previousSelectedStillValid != 0U) &&
       (previousSelectedVoltage >=
        (maxVoltage - AEK_POW_BMS63CHAIN_BAL_AUTO_TIE_MARGIN_V))){

        previousSelectedRemoved_mAh =
            AEK_POW_BMS63CHAIN_balRemoved_mAh[AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx];

        if(previousSelectedRemoved_mAh <=
           (bestTieRemoved_mAh + AEK_POW_BMS63CHAIN_BAL_AUTO_STICKY_MAH_MARGIN)){
            candidateCellIndex = AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx;
        }
    }

    if(candidateCellIndex == AEK_POW_BMS63CHAIN_BAL_AUTO_INVALID_CELL){
        AEK_POW_BMS63CHAIN_balAutoMask = 0U;
        return 0U;
    }

    selectedMask = (uint16_t)(1U << candidateCellIndex);

    if(AEK_POW_BMS63CHAIN_balStrategyMode == AEK_POW_BMS63CHAIN_BAL_STRATEGY_MULTI2){
        selectedMask = AEK_POW_BMS63CHAIN_app_buildMultiHighCellMask(nodeData, maxVoltage, candidateCellIndex);
    }

    selectedMask = AEK_POW_BMS63CHAIN_app_sanitizeBalMask(selectedMask);

    if(selectedMask == 0U){
        AEK_POW_BMS63CHAIN_app_resetAutoBalancing();
        return 0U;
    }

    /*
     * 10) Start a new balancing pulse.
     */
    AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx = candidateCellIndex;
    AEK_POW_BMS63CHAIN_balAutoMask = selectedMask;
    AEK_POW_BMS63CHAIN_balAutoActive = 1U;
    AEK_POW_BMS63CHAIN_balAutoCooldown = 0U;
    AEK_POW_BMS63CHAIN_balAutoTimerMs = nowMs;

    return AEK_POW_BMS63CHAIN_balAutoMask;
}

static void AEK_POW_BMS63CHAIN_app_applyBalancingControl(void)
{
    uint8_t AEK_POW_BMS63CHAIN_chainidx = 0U;
    uint8_t AEK_POW_BMS63CHAIN_devidx = 0U;
    uint8_t AEK_POW_BMS63CHAIN_cellidx = 0U;
    uint16_t AEK_POW_BMS63CHAIN_cellMask = 0U;
    uint16_t AEK_POW_BMS63CHAIN_controlMask = 0U;

    uint16_t AEK_POW_BMS63CHAIN_manualMask = AEK_POW_BMS63CHAIN_balManualMask;
    AEK_POW_BMS63CHAIN_balCtrlMode_t AEK_POW_BMS63CHAIN_mode = AEK_POW_BMS63CHAIN_balCtrlMode;
    AEK_POW_BMS63CHAIN_nodeData_t *AEK_POW_BMS63CHAIN_nodeData = NULL;

    for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
        AEK_POW_BMS63CHAIN_chainidx < AEK_POW_BMS63CHAIN_chain_geNum();
        AEK_POW_BMS63CHAIN_chainidx++){

        for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
            AEK_POW_BMS63CHAIN_devidx <= AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx);
            AEK_POW_BMS63CHAIN_devidx++){

            AEK_POW_BMS63CHAIN_nodeData =
                &AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                    .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U];

            /*
             * Final hard safety gate.
             * If measurements or critical diagnostics are not trustworthy,
             * force all balancing commands OFF before they reach the BMS.
             */
            if((AEK_POW_BMS63CHAIN_mode != AEK_POW_BMS63CHAIN_BAL_CTRL_OFF) &&
               (AEK_POW_BMS63CHAIN_app_balancingSafetyOk() == 0U)){

                AEK_POW_BMS63CHAIN_balManualMask = 0U;
                AEK_POW_BMS63CHAIN_balAutoMask = 0U;
                AEK_POW_BMS63CHAIN_balClearRequest = 1U;
                AEK_POW_BMS63CHAIN_balAutoActive = 0U;
                AEK_POW_BMS63CHAIN_balAutoCooldown = 0U;

                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;
                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

                for(AEK_POW_BMS63CHAIN_cellidx = 0U;
                    AEK_POW_BMS63CHAIN_cellidx < 14U;
                    AEK_POW_BMS63CHAIN_cellidx++){
                    AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 0U;
                }

                continue;
            }

            /*
             * OFF mode:
             * force all balancing commands to zero.
             */
            if(AEK_POW_BMS63CHAIN_mode == AEK_POW_BMS63CHAIN_BAL_CTRL_OFF){

                AEK_POW_BMS63CHAIN_app_resetAutoBalancing();

                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;
                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

                for(AEK_POW_BMS63CHAIN_cellidx = 0U; AEK_POW_BMS63CHAIN_cellidx < 14U; AEK_POW_BMS63CHAIN_cellidx++){
                    AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 0U;
                }
            }

            /*
             * MANUAL mode:
             * user selects cells through BAL MAN 0x....
             * Still protected by allowed mask, enabled-cell flag, and minimum voltage.
             */
            else if(AEK_POW_BMS63CHAIN_mode == AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL){

                AEK_POW_BMS63CHAIN_app_resetAutoBalancing();

                AEK_POW_BMS63CHAIN_controlMask =
                                AEK_POW_BMS63CHAIN_app_sanitizeBalMask(AEK_POW_BMS63CHAIN_manualMask);

                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;
                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

                for(AEK_POW_BMS63CHAIN_cellidx = 0U; AEK_POW_BMS63CHAIN_cellidx < 14U; AEK_POW_BMS63CHAIN_cellidx++){
                    AEK_POW_BMS63CHAIN_cellMask = (uint16_t)(1U << AEK_POW_BMS63CHAIN_cellidx);

                    if(((AEK_POW_BMS63CHAIN_controlMask & AEK_POW_BMS63CHAIN_cellMask) != 0U) &&
                        ((AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK & AEK_POW_BMS63CHAIN_cellMask) != 0U) &&
                        (AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_cellidx] != 0U) &&
                        (AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_cellidx] >
                        AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE)){

                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 1U;
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 1U;
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 1U;
                    }
                    else{
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 0U;
                    }
                }
            }

            /*
             * AUTO mode:
             * safe voltage-based automatic balancing.
             * This does NOT trust the generated model to select cells yet.
             * It computes its own mask after the model has run.
             */
            else if(AEK_POW_BMS63CHAIN_mode == AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO){

                /*
                * AUTO mode:
                * Compute the requested balancing mask from the measured cell voltages.
                * Then force the result through the active-cell safety mask.
                *
                * This guarantees that AUTO can only balance:
                * CELL1, CELL2, CELL3, CELL4, CELL13, CELL14.
                */
                AEK_POW_BMS63CHAIN_controlMask =
                    AEK_POW_BMS63CHAIN_app_computeAutoBalancingMask(AEK_POW_BMS63CHAIN_nodeData);

                AEK_POW_BMS63CHAIN_controlMask =
                    AEK_POW_BMS63CHAIN_app_sanitizeBalMask(AEK_POW_BMS63CHAIN_controlMask);

                AEK_POW_BMS63CHAIN_balAutoMask = AEK_POW_BMS63CHAIN_controlMask;

                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;
                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

                for(AEK_POW_BMS63CHAIN_cellidx = 0U;
                    AEK_POW_BMS63CHAIN_cellidx < 14U;
                    AEK_POW_BMS63CHAIN_cellidx++){

                    AEK_POW_BMS63CHAIN_cellMask =
                        (uint16_t)(1U << AEK_POW_BMS63CHAIN_cellidx);

                    /*
                    * Final per-cell safety check:
                    * 1. requested by AUTO
                    * 2. physically allowed cell
                    * 3. enabled cell
                    * 4. voltage above minimum balancing voltage
                    */
                    if(((AEK_POW_BMS63CHAIN_controlMask & AEK_POW_BMS63CHAIN_cellMask) != 0U) &&
                    ((AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK & AEK_POW_BMS63CHAIN_cellMask) != 0U) &&
                    (AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Cell_Enabled[AEK_POW_BMS63CHAIN_cellidx] != 0U) &&
                    (AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_cellidx] >
                        AEK_POW_BMS63CHAIN_BAL_AUTO_MIN_CELL_VOLTAGE)){

                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 1U;
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 1U;
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 1U;
                    }
                    else{
                        AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 0U;
                    }
                }
            }

            /*
             * Unknown state: fail safe.
             */
            else{
                AEK_POW_BMS63CHAIN_app_resetAutoBalancing();

                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;
                AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

                for(AEK_POW_BMS63CHAIN_cellidx = 0U; AEK_POW_BMS63CHAIN_cellidx < 14U; AEK_POW_BMS63CHAIN_cellidx++){
                    AEK_POW_BMS63CHAIN_nodeData->AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx] = 0U;
                }
            }
        }
    }
}

static void AEK_POW_BMS63CHAIN_app_forceBalancingOffOnChip(void)
{
  uint8_t AEK_POW_BMS63CHAIN_chainidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_devidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_cellidx = 0U;

  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
      AEK_POW_BMS63CHAIN_chainidx < AEK_POW_BMS63CHAIN_chain_geNum();
      AEK_POW_BMS63CHAIN_chainidx++){

    for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
        AEK_POW_BMS63CHAIN_devidx <= AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx);
        AEK_POW_BMS63CHAIN_devidx++){

      AEK_POW_BMS63CHAIN_node_setStopBal(AEK_POW_BMS63CHAIN_chainidx, AEK_POW_BMS63CHAIN_devidx);

      AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
        .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1]
        .AEK_POW_BMS63CHAIN_Pack_Enable_Bal = 0U;

      AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
        .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1]
        .AEK_POW_BMS63CHAIN_Pack_Bal_en_sts = 0U;

      for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1;
          AEK_POW_BMS63CHAIN_cellidx <= AEK_POW_BMS63CHAIN_CELL14;
          AEK_POW_BMS63CHAIN_cellidx++){

        AEK_POW_BMS63CHAIN_node_disableBalCell(
          AEK_POW_BMS63CHAIN_chainidx,
          AEK_POW_BMS63CHAIN_devidx,
          AEK_POW_BMS63CHAIN_cellidx
        );

        AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
          .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1]
          .AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx - 1] = 0U;

        AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
          .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1]
          .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1] = 0U;
      }
    }
  }
}


void AEK_POW_BMS63CHAIN_app_init(void){
  uint8_t AEK_POW_BMS63CHAIN_chainidx = 0;
  uint8_t AEK_POW_BMS63CHAIN_devidx = 0;
  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0; AEK_POW_BMS63CHAIN_chainidx<AEK_POW_BMS63CHAIN_chain_geNum(); AEK_POW_BMS63CHAIN_chainidx++){
	  AEK_POWBMS63CHAIN_chain_init(AEK_POW_BMS63CHAIN_chainidx);
	  for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1; AEK_POW_BMS63CHAIN_devidx<=AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx); AEK_POW_BMS63CHAIN_devidx++){
		  AEK_POW_BMS63CHAIN_node_cyclicMeasurementConversionRequest(AEK_POW_BMS63CHAIN_chainidx, AEK_POW_BMS63CHAIN_devidx, AEK_POW_BMS63CHAIN_TCYCLE_TRIGGERED);
	  }
  }
  AEK_POW_BMS63CHAIN_app_initModels();

  AEK_POW_BMS63CHAIN_balManualMask = 0U;
  AEK_POW_BMS63CHAIN_balCtrlMode = AEK_POW_BMS63CHAIN_BAL_CTRL_OFF;
  AEK_POW_BMS63CHAIN_balClearRequest = 1U;

  AEK_POW_BMS63CHAIN_app_forceBalancingOffOnChip();
}

void AEK_POW_BMS63CHAIN_app_step(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp)
{
  static uint32_t lastStepMs = 0U;

  uint32_t nowMs = osalThreadGetMilliseconds();
  uint8_t AEK_POW_BMS63CHAIN_chainidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_devidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_cellidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_clearBalReq = 0U;

  /*
   * Safety against division by zero and timing instability.
   * The old code used:
   *   osalThreadGetMilliseconds() % timestamp == 0
   * That can be true many times inside the same millisecond.
   */
  if(AEK_POW_BMS63CHAIN_app_timeStamp == 0U){
      return;
  }

  if((uint32_t)(nowMs - lastStepMs) < (uint32_t)AEK_POW_BMS63CHAIN_app_timeStamp){
      return;
  }

  lastStepMs = nowMs;

  /*
   * 1) Acquisition
   * This asks the L9963E/BMS library for the latest measurement and diagnostic data.
   */
  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
      AEK_POW_BMS63CHAIN_chainidx < AEK_POW_BMS63CHAIN_chain_geNum();
      AEK_POW_BMS63CHAIN_chainidx++){

      for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
          AEK_POW_BMS63CHAIN_devidx <= AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx);
          AEK_POW_BMS63CHAIN_devidx++){

          AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
              .AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1U] =
                  AEK_POW_BMS63CHAIN_node_getFastMeas(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx);

          AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
              .AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1U] =
                  AEK_POW_BMS63CHAIN_node_getFastDiag(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx);

          /*
           * If the diagnostic says there is an over-latch/current-sense condition,
           * request a fresh conversion only when the node conversion routine is idle.
           */
          if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                 .AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1U]
                 .AEK_POW_BMS63CHAIN_ovrLatch ||
             AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                 .AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1U]
                 .AEK_POW_BMS63CHAIN_curSenseOvcNorm){

              if(AEK_POW_BMS63CHAIN_node_getVoltageConvRoutineSts(
                    AEK_POW_BMS63CHAIN_chainidx,
                    AEK_POW_BMS63CHAIN_devidx) == AEK_POW_BMS63CHAIN_NODE_IDLE){

                  AEK_POW_BMS63CHAIN_node_onDemandVoltageConversionRequest(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx,
                      AEK_POW_BMS63CHAIN_GPIOCONV_ENABLE,
                      AEK_POW_BMS63CHAIN_GPIOTERMCONV_ENABLE,
                      AEK_POW_BMS63CHAIN_CELLTERMCONV_ENABLE,
                      AEK_POW_BMS63CHAIN_BALTERMCONV_ENABLE,
                      AEK_POW_BMS63CHAIN_HWSCCONV_ENABLE);

                  AEK_POW_BMS63CHAIN_node_cyclicMeasurementConversionRequest(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx,
                      AEK_POW_BMS63CHAIN_TCYCLE_TRIGGERED);
              }
          }
      }
  }

  /*
   * 2) Parse raw fast measurements into application data:
   * cell voltages, NTC values, current, Vref, enabled-cell flags, etc.
   */
  AEK_POW_BMS63CHAIN_app_parserData();

  /*
   * 3) Run the model.
   * Important: AUTO balancing is disabled in the command parser.
   * Then app_applyBalancingControl() overrides balance commands according to:
   * OFF / MANUAL / AUTO.
   */
  AEK_POW_BMS63CHAIN_app_modelExec();
  AEK_POW_BMS63CHAIN_app_applyBalancingControl();
  AEK_POW_BMS63CHAIN_app_updateBalancingAccounting();

  AEK_POW_BMS63CHAIN_clearBalReq = AEK_POW_BMS63CHAIN_balClearRequest;

  /*
   * 4) Apply balancing commands to the L9963E.
   * This part is close to the original ST logic, but now it is reached only once
   * per requested time step, not many times inside one millisecond.
   */
  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
      AEK_POW_BMS63CHAIN_chainidx < AEK_POW_BMS63CHAIN_chain_geNum();
      AEK_POW_BMS63CHAIN_chainidx++){

      for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
          AEK_POW_BMS63CHAIN_devidx <= AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx);
          AEK_POW_BMS63CHAIN_devidx++){

          if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                 .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                 .AEK_POW_BMS63CHAIN_Pack_Bal_en_sts){

              /*
               * If balancing is already ongoing and the requested command changed,
               * stop first. The next cycle will reconfigure and restart cleanly.
               */
              if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                     .AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1U]
                     .AEK_POW_BMS63CHAIN_balOn &&
                 !AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                     .AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1U]
                     .AEK_POW_BMS63CHAIN_eofBal){

                  for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1;
                      AEK_POW_BMS63CHAIN_cellidx <= AEK_POW_BMS63CHAIN_CELL14;
                      AEK_POW_BMS63CHAIN_cellidx++){

                      if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                             .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                             .AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx - 1U] !=
                         AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                             .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                             .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1U]){

                          AEK_POW_BMS63CHAIN_node_setStopBal(
                              AEK_POW_BMS63CHAIN_chainidx,
                              AEK_POW_BMS63CHAIN_devidx);
                          break;
                      }
                  }
              }
              else{
                  /*
                   * Balancing is not ongoing: program the requested cell switches,
                   * then start balancing.
                   */
                  for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1;
                      AEK_POW_BMS63CHAIN_cellidx <= AEK_POW_BMS63CHAIN_CELL14;
                      AEK_POW_BMS63CHAIN_cellidx++){

                      if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                             .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                             .AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx - 1U] == 1U){

                          AEK_POW_BMS63CHAIN_node_enableBalCell(
                              AEK_POW_BMS63CHAIN_chainidx,
                              AEK_POW_BMS63CHAIN_devidx,
                              AEK_POW_BMS63CHAIN_cellidx);

                          AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                              .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                              .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1U] = 1U;
                      }
                      else{
                          AEK_POW_BMS63CHAIN_node_disableBalCell(
                              AEK_POW_BMS63CHAIN_chainidx,
                              AEK_POW_BMS63CHAIN_devidx,
                              AEK_POW_BMS63CHAIN_cellidx);

                          AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                              .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                              .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1U] = 0U;
                      }
                  }

                  AEK_POW_BMS63CHAIN_node_setStartBal(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx);
              }
          }
          else{
              /*
               * Balancing is not enabled.
               * If the chip says balancing is active, stop it.
               * If clear request is active, clear all software/hardware balance cells.
               */
              if(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                     .AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1U]
                     .AEK_POW_BMS63CHAIN_balOn &&
                 !AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                     .AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1U]
                     .AEK_POW_BMS63CHAIN_eofBal){

                  AEK_POW_BMS63CHAIN_node_setStopBal(
                      AEK_POW_BMS63CHAIN_chainidx,
                      AEK_POW_BMS63CHAIN_devidx);
              }

              for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1;
                  AEK_POW_BMS63CHAIN_cellidx <= AEK_POW_BMS63CHAIN_CELL14;
                  AEK_POW_BMS63CHAIN_cellidx++){

                  if(AEK_POW_BMS63CHAIN_clearBalReq ||
                     AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                         .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                         .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1U]){

                      AEK_POW_BMS63CHAIN_node_disableBalCell(
                          AEK_POW_BMS63CHAIN_chainidx,
                          AEK_POW_BMS63CHAIN_devidx,
                          AEK_POW_BMS63CHAIN_cellidx);

                      AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx]
                          .AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1U]
                          .AEK_POW_BMS63CHAIN_Pack_Bal_sts[AEK_POW_BMS63CHAIN_cellidx - 1U] = 0U;
                  }
              }
          }
      }
  }

  if(AEK_POW_BMS63CHAIN_clearBalReq){
      AEK_POW_BMS63CHAIN_balClearRequest = 0U;
  }
}


static int stringLen(char *str){
	int n=0;
	while(str[n]!='\0')
	{
		n++;
	}
	return n;

}


static void sendMessage(char *outputMessage)
{
	sd_lld_write(&SD5,(uint8_t *)outputMessage, (uint16_t)(stringLen(outputMessage)));
}

void AEK_POW_BMS63CHAIN_app_serialInit(void){
	  sd_lld_start(&SD5,&AEK_POW_BMS63CHAIN_serialBufferedConfig);
}

void AEK_POW_BMS63CHAIN_app_serialStep_GUI(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp){ //AEK_POW_BMS63CHAIN_app_serialStep_new
  static uint32_t lastSerialGuiMs = 0U;
  uint32_t nowMs = osalThreadGetMilliseconds();

  char message[128];
  char msg[16];

  uint8_t AEK_POW_BMS63CHAIN_chainidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_devidx = 0U;
  uint8_t AEK_POW_BMS63CHAIN_cellidx = 1U;
  uint8_t TOTAL_BMS = 0U;

  AEK_POW_BMS63CHAIN_app_serialPollCommands();

  if(AEK_POW_BMS63CHAIN_serialStreamEnabled == 0U){
    return;
}

  if(AEK_POW_BMS63CHAIN_app_timeStamp == 0U){
      return;
  }

  if((uint32_t)(nowMs - lastSerialGuiMs) >= (uint32_t)AEK_POW_BMS63CHAIN_app_timeStamp){
      lastSerialGuiMs = nowMs;
	  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0; AEK_POW_BMS63CHAIN_chainidx<AEK_POW_BMS63CHAIN_CHAIN_NUM; AEK_POW_BMS63CHAIN_chainidx++){
	 		  for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1; AEK_POW_BMS63CHAIN_devidx<=AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx); AEK_POW_BMS63CHAIN_devidx++){
	 			 TOTAL_BMS=TOTAL_BMS+1;
	 		  }
	  }
	  for(AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0; AEK_POW_BMS63CHAIN_chainidx<AEK_POW_BMS63CHAIN_chain_geNum(); AEK_POW_BMS63CHAIN_chainidx++){
		  for(AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1; AEK_POW_BMS63CHAIN_devidx<=AEK_POW_BMS63CHAIN_chain_getDevNum(AEK_POW_BMS63CHAIN_chainidx); AEK_POW_BMS63CHAIN_devidx++){
			  	  	sprintf(message, "TOTDEV;%d; ",TOTAL_BMS); sendMessage(message);
		  		    //Printing DEV
		  		    sprintf(message, "CHAIN;%d;DEV;%d; ",(int)(AEK_POW_BMS63CHAIN_chainidx), (int)(AEK_POW_BMS63CHAIN_devidx));
		  		    sendMessage(message);
		  		    sendMessage("SOC;");
		  		    //Printing SoC
		  			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		  		     sprintf(message, "%.1d;",(int)(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_SOC[AEK_POW_BMS63CHAIN_cellidx - 1] * 100));
		  		     sendMessage(message);   }
		  		     sendMessage("Vcell:;");
		  		    //Printing Voltage
		  			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		  		     sprintf(message, "%.2f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_cellidx - 1]);
		  		     sendMessage(message);    }
		  			 sendMessage("TEMP:;");
		  		    //Printing Temperature
		  			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		  		     sprintf(message, "%.2f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_cellidx - 1]);
		  		     sendMessage(message);    }
		  		     sendMessage("BAL:;");
		  		    //Printing Bal_cmd
		  			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		  		     sprintf(message, "%d;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx - 1]);
		  		     sendMessage(message);    }
		  		    //Printing Current
		  		    sprintf(message, "Curr:;%.4f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Current);
		  		    sendMessage(message);
		  		    //Printing PackVoltage
		  		    sprintf(message, "totV:;%.2f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vPackSum);
		  		    sendMessage(message);
		  		    //Printing Vref
		  		    sprintf(message, "Vref:;%.2f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas);
		  		    sendMessage(message);
		  		    //Printing Vcell UV/OV
		  			sprintf(message, "VUV:;%.2f;",AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_node[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_node_vCellUV_TH); sendMessage(message);
		  			sprintf(message, "VOV:;%.2f;",AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_node[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_node_vCellOV_TH); sendMessage(message);
		  		    //Printing Vgpio UT/OT
		  			sprintf(message, "GPUT:;%.2f;",AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_node[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_node_GPIO3_UT_TH); sendMessage(message);
		  			sprintf(message, "GPOT:;%.2f;",AEK_POW_BMS63CHAIN_chain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_node[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_node_GPIO3_OT_TH); sendMessage(message);

		  		    sendMessage("FAULTS:;");
		  			//Printing FAULTS
		  		  	sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_ovrLatch				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_tCycleOvf			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_sensePlusOpen		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_senseMinusOpen		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_OTchip				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vAnaOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vDigOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vtRefUV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vtRefOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vtRegUV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vtRegOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vComUV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vtComOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_wuGpio7				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_wuSpi				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_wuIsoLine			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_wufaultH				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_wuCycWup				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_lossAgnd				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_lossDgnd				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_lossCgnd				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_lossGndRef			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_trimmCalOK			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_coCouOvf				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eobTimerError		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3FastChgOT		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eepromDwnldDone		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal14Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal13Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal12Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal11Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal10Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal9Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal8Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal7Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal6Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal5Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal4Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal3Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal2Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal1Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBatCompBistFail	          ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vRegCompBistFail	          ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vComCompBistFail	          ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vTrefCompBistFail	      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal14Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal13Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal12Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal11Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal10Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal9Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal8Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal7Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal6Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal5Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal4Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal3Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal2Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bal1Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eepromCrcErrCalOff	      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_hwscDone				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBattOpen			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel14Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel13Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel12Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel11Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel10Open			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel9Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel8Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel7Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel6Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel5Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel4Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel3Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel2Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel1Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_cel0Open				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eepromCrcErrSect0	      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_commtimeoutFlt		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eepromCrcErrCalRam	      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_RamCrcErr			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell14UV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell13UV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell12UV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell11UV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell10UV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell9UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell8UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell7UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell6UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell5UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell4UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell3UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell2UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell1UV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBattWrnOV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBattWrnUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBattCritUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vSumUV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell14OV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell13OV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell12OV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell11OV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell10OV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell9OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell8OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell7OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell6OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell5OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell4OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell3OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell2OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell1OV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_balOn				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_eofBal				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vBattCritOV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vSumOV				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3OT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3UT				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell14BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell13BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell12BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell11BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell10BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell9BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell8BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell7BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell6BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell5BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell4BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell3BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell2BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_vCell1BalUV			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_faultLlineSts		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7On				      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio9Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio8Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio7Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio6Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio5Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio4Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpio3Short			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_gpioBistFail			      ); sendMessage(msg);
		  			sprintf(msg, "%d;",AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_heartBeatFault		      ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_faultHlineSts		  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_faultHen			  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_heartBeatEn			  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_muxBistFail			  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_curSenseOvcSleep	  ); sendMessage(msg);
		  		//sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_heartBeatCycle		  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bistBalCompHsFail	  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_bistBalCompLsFail	  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_curSenseOvcNorm		  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_oscFail				  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_clkMonEn			  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_clkMonInitDone		  ); sendMessage(msg);
		  			sprintf(msg, "%d;",(int)AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastDiag[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_openBistFail	      ); sendMessage(msg);
		  		    //Printing DEV
		  		    sprintf(message, "VTREF;%f;", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_fastMeas[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_VTrefMeas		);sendMessage(message);
		  			sendMessage("ENDData");
		  	  }
	  	  }
	  osalThreadDelayMilliseconds(1);
	  }
  }


void AEK_POW_BMS63CHAIN_app_serialStep(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp){
  static uint32_t lastSerialMs = 0U;
  uint32_t nowMs = osalThreadGetMilliseconds();

  char message[128];

  uint8_t AEK_POW_BMS63CHAIN_chainidx = AEK_POW_BMS63CHAIN_CHAIN0;
  uint8_t AEK_POW_BMS63CHAIN_devidx = AEK_POW_BMS63CHAIN_NODE_DEV1;
  uint8_t AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1;

  AEK_POW_BMS63CHAIN_app_serialPollCommands();
  if(AEK_POW_BMS63CHAIN_serialStreamEnabled == 0U){
    return;
 }

  if(AEK_POW_BMS63CHAIN_app_timeStamp == 0U){
      return;
  }

  if((uint32_t)(nowMs - lastSerialMs) >= (uint32_t)AEK_POW_BMS63CHAIN_app_timeStamp){
      lastSerialMs = nowMs;

		    //Printing DEV
		    sprintf(message, "DEV,%d, ", (int)(AEK_POW_BMS63CHAIN_devidx));
		    sendMessage(message);
		    sendMessage("\n");

		    //Printing SoC
			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		    sprintf(message, "S%d,%.3d, ",AEK_POW_BMS63CHAIN_cellidx, (int)(AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_SOC[AEK_POW_BMS63CHAIN_cellidx - 1] * 100));
		    sendMessage(message);
		    }
		    sendMessage("\n");

		    //Printing Voltage
			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		    sprintf(message, "V%d,%.3f, ",AEK_POW_BMS63CHAIN_cellidx, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellVoltage[AEK_POW_BMS63CHAIN_cellidx - 1]);
		    sendMessage(message);
		    }
		    sendMessage("\n");

		    //Printing Current
		    sprintf(message, "C,%.4f, ", AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Current);
		    sendMessage(message);
		    sendMessage("\n");

		    //Printing Temperature
			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		    sprintf(message, "T%d,%.3f, ", AEK_POW_BMS63CHAIN_cellidx, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[AEK_POW_BMS63CHAIN_cellidx - 1]);
		    sendMessage(message);
		    }
		    sendMessage("\n");

		    //Printing Bal_cmd
			for(AEK_POW_BMS63CHAIN_cellidx = AEK_POW_BMS63CHAIN_CELL1; AEK_POW_BMS63CHAIN_cellidx<= AEK_POW_BMS63CHAIN_CELL14; AEK_POW_BMS63CHAIN_cellidx++){
		    sprintf(message, "B%d,%d, ", AEK_POW_BMS63CHAIN_cellidx, AEK_POW_BMS63CHAIN_app_dataChain[AEK_POW_BMS63CHAIN_chainidx].AEK_POW_BMS63CHAIN_nodeData[AEK_POW_BMS63CHAIN_devidx - 1].AEK_POW_BMS63CHAIN_Pack_Bal_cmd[AEK_POW_BMS63CHAIN_cellidx - 1]);
		    sendMessage(message);
		    }
		    sendMessage("\n");

		}
}


void *sbrk(size_t incr)
{
	extern uint8_t __heap_base__;
	extern uint8_t __heap_end__;
	static uint8_t *p=&__heap_base__;
	static uint8_t *newp;

	newp = p+ incr;
	if(newp> &__heap_end__)
	{
		return (void*)-1;
	}
	return p =newp;
}

