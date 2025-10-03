#ifndef BSP_VSS_H
#define BSP_VSS_H

#include "main.h"
#include "bsp_global.h"

/* 虚拟速度传感器对象 */
void VSS_Init(VirtualSpeedSensor_Handle_t *pHandle);
int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, void *pInputVars_str);
bool VSS_CalcAvrgMecSpeed01Hz(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeed01Hz);
void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle);
void VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t hFinalMecSpeed01Hz, uint16_t hDurationms);
bool VSS_RampCompleted(VirtualSpeedSensor_Handle_t *pHandle);
int16_t VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle);
bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand);
bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle);
void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle);
void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle);

#endif /* BSP_VSS_H */
