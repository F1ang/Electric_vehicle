#ifndef BSP_STO_CORDIC_H
#define BSP_STO_CORDIC_H

#include "main.h"
#include "bsp_global.h"

#define ATAN1DIV1    (int16_t)8192
#define ATAN1DIV2    (int16_t)4836
#define ATAN1DIV4    (int16_t)2555
#define ATAN1DIV8    (int16_t)1297
#define ATAN1DIV16   (int16_t)651
#define ATAN1DIV32   (int16_t)326
#define ATAN1DIV64   (int16_t)163
#define ATAN1DIV128  (int16_t)81
#define ATAN1DIV256  (int16_t)41
#define ATAN1DIV512  (int16_t)20
#define ATAN1DIV1024 (int16_t)10
#define ATAN1DIV2048 (int16_t)5
#define ATAN1DIV4096 (int16_t)3
#define ATAN1DIV8192 (int16_t)1

/*********************** 状态观测器在旋转坐标系下的参数 *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1) * RS) / (LS * TF_REGULATION_RATE))
#define CORD_C2 (int32_t)CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1) * MAX_BEMF_VOLTAGE) / (LS * MAX_CURRENT * TF_REGULATION_RATE))
#define CORD_C4 (int32_t)CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1) * MAX_VOLTAGE) / (LS * MAX_CURRENT * \
                                                                TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR (uint16_t)(CORD_VARIANCE_THRESHOLD * 128u)
// #define CORD_MINIMUM_SPEED     (uint16_t)(CORD_MINIMUM_SPEED_RPM / 6u)

extern STO_CR_Handle_t STO_CR_M1;

void STO_CR_Init(STO_CR_Handle_t *pHandle);
void STO_CR_Return(STO_CR_Handle_t *pHandle, uint8_t flag);
int16_t STO_CR_CalcElAngle(STO_CR_Handle_t *pHandle, Observer_Inputs_t *pInputVars_str);
bool STO_CR_CalcAvrgMecSpeed01Hz(STO_CR_Handle_t *pHandle, int16_t *pMecSpeed01Hz);
void STO_CR_Clear(STO_CR_Handle_t *pHandle);

void STO_CR_SetMecAngle(STO_CR_Handle_t *pHandle, int16_t hMecAngle);
static void STO_CR_Store_Rotor_Speed(STO_CR_Handle_t *pHandle, int16_t hRotor_Speed, int16_t hOrRotor_Speed);
static int16_t STO_CR_ExecuteCORDIC(STO_CR_Handle_t *pHandle, int32_t wBemf_alfa_est, int32_t wBemf_beta_est);
static void STO_CR_InitSpeedBuffer(STO_CR_Handle_t *pHandle);

bool STO_CR_IsObserverConverged(STO_CR_Handle_t *pHandle, int16_t hForcedMecSpeed01Hz);
Volt_Components STO_CR_GetEstimatedBemf(STO_CR_Handle_t *pHandle);
Curr_Components STO_CR_GetEstimatedCurrent(STO_CR_Handle_t *pHandle);
void STO_CR_GetObserverGains(STO_CR_Handle_t *pHandle, int16_t *pC2, int16_t *pC4);
void STO_CR_SetObserverGains(STO_CR_Handle_t *pHandle, int16_t hC1, int16_t hC2);
void STO_CR_CalcAvrgElSpeedDpp(STO_CR_Handle_t *pHandle);
int32_t STO_CR_GetEstimatedBemfLevel(STO_CR_Handle_t *pHandle);
int32_t STO_CR_GetObservedBemfLevel(STO_CR_Handle_t *pHandle);

void STO_CR_BemfConsistencyCheckSwitch(STO_CR_Handle_t *pHandle, bool bSel);
bool STO_CR_IsBemfConsistent(STO_CR_Handle_t *pHandle);
bool STO_CR_IsSpeedReliable(STO_Handle_t *pHandle);
void STO_CR_ForceConvergency1(STO_Handle_t *pHandle);
void STO_CR_ForceConvergency2(STO_Handle_t *pHandle);

#endif /* BSP_STO_CORDIC_H */
