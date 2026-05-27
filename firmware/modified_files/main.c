/****************************************************************************
*
* Copyright © 2015-2024 STMicroelectronics - All Rights Reserved
*
* This software is licensed under SLA0098 terms that can be found in the
* DM00779817_1_0.pdf file in the licenses directory of this software product.
* 
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED, 
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*****************************************************************************/

/* Inclusion of the main header files of all the imported components in the
   order specified in the application wizard. The file is generated
   automatically.*/
#include "AEK_POW_BMS63CHAIN_app_mng.h"

#define AEK_POW_BMS63CHAIN_TIMESTEP 1000 //msec
#define AEK_POW_BMS63CHAIN_SERIAL_TIMESTEP 1000 //msec

void main_core0(void) {
  irqIsrEnable();
  AEK_POW_BMS63CHAIN_app_serialInit();
  for(;;){
	  AEK_POW_BMS63CHAIN_app_serialStep_GUI(AEK_POW_BMS63CHAIN_SERIAL_TIMESTEP);
  }
}

int main(void) {
  componentsInit();
  irqIsrEnable();
  /* Application init */
  AEK_POW_BMS63CHAIN_app_init();
  runCore0();
  /* Application main loop.*/
  for ( ; ; ) {
	  AEK_POW_BMS63CHAIN_app_step(AEK_POW_BMS63CHAIN_TIMESTEP);
  }
}
