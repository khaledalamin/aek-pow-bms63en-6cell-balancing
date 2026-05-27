/*
 * AEK_POW_BMSCHAIN_App.h
 *
 *  Created on: May 10, 2024
 *      Author: luca russotti3
 */

#ifndef AEK_POW_BMS63CHAIN_APP_MNG_H_
#define AEK_POW_BMS63CHAIN_APP_MNG_H_

#include "components.h"
#include "AEK_POW_BMS63CHAIN_chain_mng.h"
#include "BMS_Controller.h"
#include "serial_lld_cfg.h"
#include <stdio.h>
#include <string.h>

#define AEK_POW_BMS_RP_NTC 10000 // ohm
#define AEK_POW_BMS63CHAIN_BAL_MIN_CELL_VOLTAGE 2.5F
#define AEK_POW_BMS63CHAIN_BAL_ALLOWED_MASK 0x300FU

typedef struct{
		float AEK_POW_BMS63CHAIN_Pack_CellTemperatureNTC[14];
		float AEK_POW_BMS63CHAIN_Pack_CellVoltage[14];
		float AEK_POW_BMS63CHAIN_Pack_Current;
		uint8_t AEK_POW_BMS63CHAIN_Pack_Enable_Bal;
		float AEK_POW_BMS63CHAIN_Pack_SOC[14];
		float AEK_POW_BMS63CHAIN_Pack_SOH[14];
		uint8_t AEK_POW_BMS63CHAIN_Pack_Bal_cmd[14];
		uint8_t AEK_POW_BMS63CHAIN_Pack_Bal_sts[14];
		uint8_t AEK_POW_BMS63CHAIN_Pack_Bal_en_sts;
		float AEK_POW_BMS63CHAIN_Pack_Vbatt;
		float AEK_POW_BMS63CHAIN_Vref;
		uint8_t AEK_POW_BMS63CHAIN_Cell_Enabled[14];
		uint8_t AEK_POW_BMS63CHAIN_Pack_Enable_Discharge;
		uint32_t AEK_POW_BMS63CHAIN_Fault1_data;
		uint32_t AEK_POW_BMS63CHAIN_Fault2_data;
		uint32_t AEK_POW_BMS63CHAIN_VcellOV_Fault_data;
		uint32_t AEK_POW_BMS63CHAIN_VcellUV_Fault_data;
		uint32_t AEK_POW_BMS63CHAIN_GPIOOTUT_Fault_data;
		uint8_t AEK_POW_BMS63CHAIN_OverLatch;
}AEK_POW_BMS63CHAIN_nodeData_t;

typedef struct{
	AEK_POW_BMS63CHAIN_fastMeas_t AEK_POW_BMS63CHAIN_fastMeas[31];
	AEK_POW_BMS63CHAIN_fastDiag_t AEK_POW_BMS63CHAIN_fastDiag[31];
	AEK_POW_BMS63CHAIN_nodeData_t AEK_POW_BMS63CHAIN_nodeData[31];
}AEK_POW_BMS63CHAIN_app_dataChain_t;

typedef enum{
	AEK_POW_BMS63CHAIN_BAL_CTRL_OFF = 0,
	AEK_POW_BMS63CHAIN_BAL_CTRL_AUTO = 1,
	AEK_POW_BMS63CHAIN_BAL_CTRL_MANUAL = 2
}AEK_POW_BMS63CHAIN_balCtrlMode_t;

void AEK_POW_BMS63CHAIN_app_init(void);
void AEK_POW_BMS63CHAIN_app_serialInit(void);
void AEK_POW_BMS63CHAIN_app_step(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp);
void AEK_POW_BMS63CHAIN_app_serialStep(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp);
void AEK_POW_BMS63CHAIN_app_serialStep_GUI(uint16_t AEK_POW_BMS63CHAIN_app_timeStamp);
void AEK_POW_BMS63CHAIN_app_serialPollCommands(void);

#endif /* AEK_POW_BMS63CHAIN_APP_MNG_H_ */
