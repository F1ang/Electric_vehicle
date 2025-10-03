#ifndef BSP_R3F4_H
#define BSP_R3F4_H

#include "main.h"
#include "bsp_global.h"

/* FOC»Øµ÷ */
void R3F4XX_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents);
void R3F4XX_SwitchOffPWM(PWMC_Handle_t *pHdl);
void R3F4XX_SwitchOnPWM(PWMC_Handle_t *pHdl);
void R3F4XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl);
void R3F4XX_TurnOnLowSides(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect1(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect2(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect3(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect4(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect5(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_SetADCSampPointSect6(PWMC_Handle_t *pHdl);
uint16_t R3F4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl);
void R3F4XX_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents);
void R3F4XX_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents);

#endif /* BSP_R3F4_H */
