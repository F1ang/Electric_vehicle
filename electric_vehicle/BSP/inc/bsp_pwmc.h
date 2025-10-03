#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"
#include "bsp_global.h"

/* »Øµ÷º¯Êý×¢²á */
void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle, Curr_Components *pStator_Currents);
uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, Volt_Components Valfa_beta);
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle);
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle);
bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle, CRCAction_t action);
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle);
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle);
void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle, uint16_t hDACVref);
void PWMC_RegisterGetPhaseCurrentsCallBack(PWMC_GetPhaseCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSwitchOffPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSwitchonPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterReadingCalibrationCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterTurnOnLowSidesCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect1CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect2CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect3CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect4CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect5CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterSampPointSect6CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterIsOverCurrentOccurredCallBack(PWMC_OverCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle);
void PWMC_RegisterOCPSetRefVoltageCallBack(PWMC_SetOcpRefVolt_Cb_t pCallBack, PWMC_Handle_t *pHandle);

#endif /* BSP_PWM_H */
