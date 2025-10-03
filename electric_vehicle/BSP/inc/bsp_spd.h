#ifndef BSP_SPD_H
#define BSP_SPD_H

#include "main.h"
#include "bsp_global.h"

/* 位置与转速信息 */
int16_t SPD_GetElAngle(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetMecAngle(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetAvrgMecSpeed01Hz(SpeednPosFdbk_Handle_t *pHandle);
int16_t SPD_GetElSpeedDpp(SpeednPosFdbk_Handle_t *pHandle);
bool SPD_Check(SpeednPosFdbk_Handle_t *pHandle);
bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, int16_t *pMecSpeed01Hz);
int16_t SPD_GetS16Speed(SpeednPosFdbk_Handle_t *pHandle);
uint8_t SPD_GetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle);
void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP);

#endif /* BSP_SPD_H */
