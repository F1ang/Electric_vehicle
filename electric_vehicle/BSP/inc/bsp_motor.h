/******************************************************************************
 *@brief  Motor control driver
 *@author By Spotted Owl
 *@date     2025.05.29
 ******************************************************************************/
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "main.h"
#include "circle_limitation.h"
#include "bsp_global.h"
#include "bsp_r3f4.h"
#include "bsp_dac_comm.h"

extern PWMC_R3_F4_Handle_t PWM_Handle_M1;
extern PWMC_Handle_t *pwmcHandle;
extern SpeednTorqCtrl_Handle_t *pSTC;
extern PID_Handle_t *pPIDSpeed;
extern FOCVars_t FOCVars;
extern PID_Handle_t *pPIDIq;
extern PID_Handle_t *pPIDId;
extern MCI_Handle_t *oMCInterface;
extern STM_Handle_t STM;
extern CircleLimitation_Handle_t *pCLM;
extern DAC_UI_Handle_t *pDAC;
extern volatile uint16_t cnt_500ms;
extern RampExtMngr_Handle_t *pREMNG;

void MX_MotorControl_Init(void);
void MCboot(void);
void R3F4XX_Init(PWMC_R3_F4_Handle_t *pHandle);
void MC_RunMotorControlTasks(void);
void MC_Scheduler(void);

#endif /* BSP_MOTOR_H */
