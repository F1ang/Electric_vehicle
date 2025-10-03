/******************************************************************************
 *@brief  PID controller header file.
 *@author By Spotted Owl
 *@date     2025.06.04
 ******************************************************************************/
#ifndef BSP_PID_H
#define BSP_PID_H

#include "main.h"
#include "mc_math.h"
#include "bsp_global.h"

/* TODO:HALL角度补偿后,速度环/电流环需要调参 */
/* Gains values for torque and flux control loops */
#if HALL_ENABLE
#define PID_TORQUE_KP_DEFAULT 2172
#define PID_TORQUE_KI_DEFAULT 2200
#define PID_TORQUE_KD_DEFAULT 100
#define PID_FLUX_KP_DEFAULT   2172
#define PID_FLUX_KI_DEFAULT   2277
#define PID_FLUX_KD_DEFAULT   100
#else
#define PID_TORQUE_KP_DEFAULT 2172
#define PID_TORQUE_KI_DEFAULT 2277
#define PID_TORQUE_KD_DEFAULT 100
#define PID_FLUX_KP_DEFAULT   2172
#define PID_FLUX_KI_DEFAULT   2277
#define PID_FLUX_KD_DEFAULT   100
#endif

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                     1024
#define TF_KIDIV                     16384
#define TF_KDDIV                     8192
#define TFDIFFERENTIAL_TERM_ENABLING DISABLE

/* Speed control loop */
#if ENCODER_ENABLE
#define PID_SPEED_KP_DEFAULT 50
#define PID_SPEED_KI_DEFAULT 20
#define PID_SPEED_KD_DEFAULT 0

#elif HALL_ENABLE
#define PID_SPEED_KP_DEFAULT 100
#define PID_SPEED_KI_DEFAULT 18
#define PID_SPEED_KD_DEFAULT 0

#elif SENSORLESS_ENABLE
#define PID_SPEED_KP_DEFAULT 100
#define PID_SPEED_KI_DEFAULT 18
#define PID_SPEED_KD_DEFAULT 0

#else
#define PID_SPEED_KP_DEFAULT 50
#define PID_SPEED_KI_DEFAULT 20
#define PID_SPEED_KD_DEFAULT 0

#endif

/* Speed PID parameter dividers */
#define SP_KPDIV 16
#define SP_KIDIV 256
#define SP_KDDIV 16

#define PID_SPEED_INTEGRAL_INIT_DIV    1
#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE

/*  Maximum Torque Per Ampere strategy parameters */
#define IQMAX  4997
#define SEGDIV 0
#define ANGC   { 0, 0, 0, 0, 0, 0, 0, 0 }
#define OFST   { 0, 0, 0, 0, 0, 0, 0, 0 }

/*************** PI divisor  ***************/
#define SP_KPDIV_LOG            LOG2(16)
#define SP_KIDIV_LOG            LOG2(256)
#define SP_KDDIV_LOG            LOG2(16)
#define TF_KPDIV_LOG            LOG2(1024)
#define TF_KIDIV_LOG            LOG2(16384)
#define TF_KDDIV_LOG            LOG2(8192)
#define FW_KPDIV_LOG            LOG2(32768)
#define FW_KIDIV_LOG            LOG2(32768)
#define PLL_KPDIV               16384
#define PLL_KPDIV_LOG           LOG2(PLL_KPDIV)
#define PLL_KIDIV               65535
#define PLL_KIDIV_LOG           LOG2(PLL_KIDIV)
#define F1_LOG                  LOG2(16384)
#define F2_LOG                  LOG2(2048)
#define STO_FIFO_DEPTH_DPP_LOG  LOG2(64)
#define CORD_FIFO_DEPTH_DPP_LOG LOG2(64)
#define HFI_PID_KPDIV_LOG       LOG2(16384)
#define HFI_PID_KIDIV_LOG       LOG2(32768)

extern PID_Handle_t PIDSpeedHandle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;

void PID_HandleInit(PID_Handle_t *pHandle);
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain);
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain);
int16_t PID_GetKP(PID_Handle_t *pHandle);
int16_t PID_GetKI(PID_Handle_t *pHandle);
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle);
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle);
void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue);
uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle);
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2);
uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle);
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2);
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit);
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit);
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit);
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit);
void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError);
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain);
int16_t PID_GetKD(PID_Handle_t *pHandle);
uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle);
void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2);
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError);

#endif /* BSP_PID_H */
