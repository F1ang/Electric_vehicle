#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "main.h"
#include "bsp_global.h"

extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern ENCODER_Handle_t ENCODER_M1;
extern EncAlign_Handle_t EncAlignCtrlM1;

/* ±àÂëÆ÷¶ÔÏó */
void ENC_Init(ENCODER_Handle_t *pHandle);
void ENC_Clear(ENCODER_Handle_t *pHandle);
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle);
bool ENC_CalcAvrgMecSpeed01Hz(ENCODER_Handle_t *pHandle, int16_t *pMecSpeed01Hz);
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle);
void *ENC_IRQHandler(void *pHandleVoid);

#endif /* BSP_ENCODER_H */
