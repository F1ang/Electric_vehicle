/******************************************************************************
 *@brief  bsp²ãÇý¶¯
 *@author By Spotted Owl
 *@date     2025.10.07
 ******************************************************************************/
#ifndef __HARDWARE_INIT_H
#define __HARDWARE_INIT_H

#include "global_variable.h"

#define LED1(i)                          \
    if (i == 1)                          \
        GPIO_SetBits(GPIO0, GPIO_Pin_6); \
    else                                 \
        GPIO_ResetBits(GPIO0, GPIO_Pin_6);

#define LED2(i)                          \
    if (i == 1)                          \
        GPIO_SetBits(GPIO0, GPIO_Pin_7); \
    else                                 \
        GPIO_ResetBits(GPIO0, GPIO_Pin_7);

#define LED3(i)                          \
    if (i == 1)                          \
        GPIO_SetBits(GPIO0, GPIO_Pin_3); \
    else                                 \
        GPIO_ResetBits(GPIO0, GPIO_Pin_3);

void SystemInit(void);
void SoftDelay(u32 cnt);
void Clock_Init(void);

void Hardware_init(void);
void GPIO_init(void);

#endif
