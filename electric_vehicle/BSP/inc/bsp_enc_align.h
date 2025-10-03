#ifndef BSP_ENC_ALIGN_H
#define BSP_ENC_ALIGN_H

#include "main.h"
#include "bsp_global.h"

void EAC_Init(EncAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, ENCODER_Handle_t *pENC);
void EAC_StartAlignment(EncAlign_Handle_t *pHandle);
bool EAC_Exec(EncAlign_Handle_t *pHandle);
bool EAC_IsAligned(EncAlign_Handle_t *pHandle);
void EAC_SetRestartState(EncAlign_Handle_t *pHandle, bool restart);
bool EAC_GetRestartState(EncAlign_Handle_t *pHandle);

#endif /* BSP_ENC_ALIGN_H */
