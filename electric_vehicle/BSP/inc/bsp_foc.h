#ifndef BSP_FOC_H
#define BSP_FOC_H

#include "main.h"
#include "bsp_global.h"

/* FOC´¦Àíº¯Êý */
void FOC_Clear(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
uint16_t FOC_CurrController(uint8_t bMotor);

#endif /* BSP_FOC_H */
