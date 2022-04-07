#ifndef DRV8305_H
#define DRV8305_H

#include "main.h"

extern HAL_StatusTypeDef DRV8305_ReadWarnings();
extern HAL_StatusTypeDef DRV8305_ReadFaultsOVVDS();
extern HAL_StatusTypeDef DRV8305_ReadFaultsIC();
extern HAL_StatusTypeDef DRV8305_ReadFaultsVGS();
extern HAL_StatusTypeDef DRV8305_ReadGateControlHS();
extern HAL_StatusTypeDef DRV8305_ReadGateControlLS(); 
extern HAL_StatusTypeDef DRV8305_ReadCurentGains();

extern HAL_StatusTypeDef DRV8305_WriteGateControl(uint8_t drvBmISink, uint8_t drvBmISource, uint8_t drvBmTdrive);
extern HAL_StatusTypeDef DRV8305_WriteCurrentGains(uint8_t gain);

extern uint8_t DRV8305_Warnings[2];
extern uint8_t DRV8305_FaultsOVVDS[2];
extern uint8_t DRV8305_FaultsIC[2];
extern uint8_t DRV8305_FaultsVGS[2];
extern uint8_t DRV8305_GateControlLS[2];
extern uint8_t DRV8305_GateControlHS[2];
extern uint8_t DRV8305_CurrentGains[2];

#endif