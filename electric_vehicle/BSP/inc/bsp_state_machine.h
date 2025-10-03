#ifndef BSP_STATE_MACHINE_H
#define BSP_STATE_MACHINE_H

#include "main.h"
#include "bsp_global.h"

void STM_Init(STM_Handle_t *pHandle);
bool STM_NextState(STM_Handle_t *pHandle, State_t bState);
State_t STM_FaultProcessing(STM_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors);
State_t STM_GetState(STM_Handle_t *pHandle);
bool STM_FaultAcknowledged(STM_Handle_t *pHandle);
uint32_t STM_GetFaultState(STM_Handle_t *pHandle);

#endif /* BSP_STATE_MACHINE_H */
