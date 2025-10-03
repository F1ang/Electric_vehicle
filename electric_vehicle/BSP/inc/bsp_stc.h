#ifndef BSP_STC_H
#define BSP_STC_H

#include "main.h"
#include "bsp_global.h"

/* 速度转矩控制函数 */
void STC_Init(SpeednTorqCtrl_Handle_t *pHandle, PID_Handle_t *pPI, SpeednPosFdbk_Handle_t *SPD_Handle);
void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *SPD_Handle);
SpeednPosFdbk_Handle_t *STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle);
void STC_Clear(SpeednTorqCtrl_Handle_t *pHandle);
int16_t STC_GetMecSpeedRef01Hz(SpeednTorqCtrl_Handle_t *pHandle);
int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle);
void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, STC_Modality_t bMode);
STC_Modality_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle);
bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms);
void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle);
int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle);
int16_t STC_GetMecSpeedRef01HzDefault(SpeednTorqCtrl_Handle_t *pHandle);
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle);
int16_t STC_GetMinAppNegativeMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle);
bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle);
bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle);
Curr_Components STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle);
void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent);
void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle);

#endif /* BSP_STC_H */
