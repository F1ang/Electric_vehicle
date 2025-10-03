/******************************************************************************
 *@brief  滑膜观测器
 *@author By Spotted Owl
 *@date     2025.06.28
 ******************************************************************************/
#ifndef BSP_STO_PLL_H
#define BSP_STO_PLL_H

#include "main.h"
#include "bsp_global.h"

/****** HFI ******/
#define HFI_FREQUENCY 400
#define HFI_AMPLITUDE 25

#define HFI_PID_KP_DEFAULT 800
#define HFI_PID_KI_DEFAULT 400
#define HFI_PID_KPDIV      16384
#define HFI_PID_KIDIV      32768

#define HFI_IDH_DELAY 32400

#define HFI_PLL_KP_DEFAULT 0.00060
#define HFI_PLL_KI_DEFAULT 0.00400

#define HFI_NOTCH_0_COEFF 0.969619
#define HFI_NOTCH_1_COEFF -1.923946
#define HFI_NOTCH_2_COEFF 0.969619
#define HFI_NOTCH_3_COEFF 1.923946
#define HFI_NOTCH_4_COEFF -0.939237

#define HFI_LP_0_COEFF 0.00597
#define HFI_LP_1_COEFF 0.011941
#define HFI_LP_2_COEFF 0.00597
#define HFI_LP_3_COEFF 1.769837
#define HFI_LP_4_COEFF -0.793719

#define HFI_HP_0_COEFF 0.939693
#define HFI_HP_1_COEFF -1.879386
#define HFI_HP_2_COEFF 0.939693
#define HFI_HP_3_COEFF 1.875746
#define HFI_HP_4_COEFF -0.883026

#define HFI_DC_0_COEFF 0.00146
#define HFI_DC_1_COEFF 0.002921
#define HFI_DC_2_COEFF 0.00146
#define HFI_DC_3_COEFF 1.889033
#define HFI_DC_4_COEFF -0.894874

#define HFI_MINIMUM_SPEED_RPM     402
#define HFI_SPD_BUFFER_DEPTH_01HZ 64
#define HFI_LOCKFREQ              33
#define HFI_SCANROTATIONSNO       3
#define HFI_WAITBEFORESN          6
#define HFI_WAITAFTERNS           4
#define HFI_HIFRAMPLSCAN          25
#define HFI_NSMAXDETPOINTS        20
#define HFI_NSDETPOINTSSKIP       10
#define HFI_DEBUG_MODE            false

#define HFI_STO_RPM_TH      OBS_MINIMUM_SPEED_RPM
#define STO_HFI_RPM_TH      460
#define HFI_RESTART_RPM_TH  (((HFI_STO_RPM_TH) + (STO_HFI_RPM_TH)) / 2)
#define HFI_NS_MIN_SAT_DIFF 0

#define HFI_REVERT_DIRECTION true
#define HFI_WAITTRACK        20
#define HFI_WAITSYNCH        20
#define HFI_STEPANGLE        3640
#define HFI_MAXANGLEDIFF     3640
#define HFI_RESTARTTIMESEC   0.1

/* PLL PI参数 */
#define PLL_KP_GAIN             269
#define PLL_KI_GAIN             8
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

/* 观测器参数 */
/* e^(-p)=1-p+p^2/2!-p^3/3!...泰勒展开 */
#define C1 (int32_t)((((int16_t)F1) * RS) / (LS * TF_REGULATION_RATE))                             /* Rs/(Ls*Ts) */
#define C2 (int32_t)GAIN1                                                                          /* I_est的增益->K1*Z */
#define C3 (int32_t)((((int16_t)F1) * MAX_BEMF_VOLTAGE) / (LS * MAX_CURRENT * TF_REGULATION_RATE)) /* Rs_bemf/(Ls*Ts) */
#define C4 (int32_t)GAIN2                                                                          /* E_est的增益->K2*Z */
#define C5 (int32_t)((((int16_t)F1) * MAX_VOLTAGE) / (LS * MAX_CURRENT * TF_REGULATION_RATE))      /* Rs_vs/(Ls*Ts) */

#define PERCENTAGE_FACTOR (uint16_t)(VARIANCE_THRESHOLD * 128u)
#define OBS_MINIMUM_SPEED (uint16_t)(OBS_MINIMUM_SPEED_RPM / 6u)
#define HFI_MINIMUM_SPEED (uint16_t)(HFI_MINIMUM_SPEED_RPM / 6u)

#define OBS_MEAS_ERRORS_BEFORE_FAULTS 16 /* 报告速度反馈错误之前进行方差测试时出现的连续错误次数 */
#define STO_FIFO_DEPTH_DPP            64 /* 速度平滑缓存大小 */
#define STO_FIFO_DEPTH_01HZ           64 /* 速度平滑缓存大小 */

extern STO_PLL_Handle_t STO_PLL_M1;
extern STO_PLL_Handle_t *pSTO_PLL_M1;
extern STO_Handle_t STO_M1;

void STO_PLL_Init(STO_PLL_Handle_t *pHandle);
void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag);
int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputVars_str);
bool STO_PLL_CalcAvrgMecSpeed01Hz(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeed01Hz);
void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle);
void STO_PLL_Clear(STO_PLL_Handle_t *pHandle);

inline static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed);
inline static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
static void STO_InitSpeedBuffer(STO_PLL_Handle_t *pHandle);

bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t hForcedMecSpeed01Hz);
Volt_Components STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle);
Curr_Components STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle);
void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *pC2, int16_t *pC4);
void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hC1, int16_t hC2);
void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain);
void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain);

void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle);
void STO_OTF_ResetPLL(STO_Handle_t *pHandle);
void STO_ResetPLL(STO_PLL_Handle_t *pHandle);
void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle);
int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle);
void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel);
bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle);
bool STO_PLL_IsVarianceTight(STO_Handle_t *pHandle);
void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle);
void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle);

#endif /* BSP_STO_PLL_H */
