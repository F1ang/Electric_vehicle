#ifndef BSP_RAMP_H
#define BSP_RAMP_H

#include "main.h"
#include "bsp_global.h"

extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;

void REMNG_Init(RampExtMngr_Handle_t *pHandle);
int32_t REMNG_Calc(RampExtMngr_Handle_t *pHandle);
bool REMNG_ExecRamp(RampExtMngr_Handle_t *pHandle, int32_t TargetFinal, uint32_t Durationms);
int32_t REMNG_GetValue(RampExtMngr_Handle_t *pHandle);
bool REMNG_RampCompleted(RampExtMngr_Handle_t *pHandle);
void REMNG_StopRamp(RampExtMngr_Handle_t *pHandle);

#endif /* BSP_RAMP_H */

