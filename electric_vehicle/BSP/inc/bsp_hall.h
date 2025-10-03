/******************************************************************************
 *@brief  HALLsensor driver header file.
 *@author By Spotted Owl
 *@date     2025.06.24
 ******************************************************************************/
#ifndef BSP_HALL_H
#define BSP_HALL_H

#include "main.h"
#include "bsp_global.h"

#define HALL_LPF_FACTOR 0.8

#define LOW_RES_THRESHOLD ((uint16_t)0x5500u)

#define HALL_COUNTER_RESET ((uint16_t)0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536 / 3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536 / 6)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE      (int8_t)-1
#define POSITIVE      (int8_t)1
#define NEGATIVE_SWAP (int8_t)-2
#define POSITIVE_SWAP (int8_t)2

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED ((int16_t)0x7FFF)

#define CCER_CC1E_Set   ((uint16_t)0x0001)
#define CCER_CC1E_Reset ((uint16_t)0xFFFE)

extern HALL_Handle_t HALL_M1;

void HALL_Init(HALL_Handle_t *pHandle);
void HALL_Clear(HALL_Handle_t *pHandle);
static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle);
int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle);
static int16_t HALL_CalcAvrgElSpeedDpp(HALL_Handle_t *pHandle);
bool HALL_CalcAvrgMecSpeed01Hz(HALL_Handle_t *pHandle, int16_t *hMecSpeed01Hz);

void *HALL_TIMx_CC_IRQHandler(void *pHandleVoid);
void *HALL_TIMx_UP_IRQHandler(void *pHandleVoid);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* BSP_HALL_H */
