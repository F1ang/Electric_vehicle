#ifndef BSP_TASK_H
#define BSP_TASK_H

#include "main.h"
#include "bsp_global.h"

extern volatile uint16_t hBootCapDelayCounterM1;
extern volatile uint16_t hStopPermanencyCounterM1;

/* Tasks */
void TSK_MediumFrequencyTaskM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
uint8_t TSK_HighFrequencyTask(void);
inline uint16_t FOC_CurrController(uint8_t bMotor);
void TSK_SafetyTask(void);
void TSK_SafetyTask_PWMOFF(uint8_t bMotor);
void TSK_HardwareFaultTask(void);

#endif /* BSP_TASK_H */
