/******************************************************************************
 *@brief  bsp������
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

#define ADC_STATE_RESET()  \
    {                      \
        ADC0_CFG |= BIT11; \
        ADC1_CFG |= BIT11; \
    } /* ADC0 ״̬����λ,���Լ��������ȷ��ADC����״̬ */

#define ADC0_STATE_RESET() \
    {                      \
        ADC0_CFG |= BIT11; \
    } /* ADC0 ״̬����λ,���Լ��������ȷ��ADC����״̬ */

#define ADC1_STATE_RESET() \
    {                      \
        ADC1_CFG |= BIT11; \
    }

/* ---------------------------- ADC����ͨ������ ----------------------- */
#define ADC0_CHANNEL_OPA0 ADC_CHANNEL_0
#define ADC0_CHANNEL_OPA1 ADC_CHANNEL_1
#define ADC0_CHANNEL_OPA2 ADC_CHANNEL_2
#define ADC0_CHANNEL_OPA3 ADC_CHANNEL_3

#define ADC1_CHANNEL_OPA0 ADC_CHANNEL_0
#define ADC1_CHANNEL_OPA1 ADC_CHANNEL_1
#define ADC1_CHANNEL_OPA2 ADC_CHANNEL_2
#define ADC1_CHANNEL_OPA3 ADC_CHANNEL_3

#define ADC0_CURRETN_A_CHANNEL (ADC0_CHANNEL_OPA2)
#define ADC0_CURRETN_B_CHANNEL (ADC0_CHANNEL_OPA1)
#define ADC0_CURRETN_C_CHANNEL (ADC0_CHANNEL_OPA0)

#define ADC_BUS_VOL_CHANNEL (ADC_CHANNEL_8)     /* ĸ�ߵ�ѹADC����ͨ�� */
#define M0_ADC_BUS_CURR_CH  (ADC0_CHANNEL_OPA3) /* ĸ�ߵ���ADC����ͨ�� */

#define ADC0_TEMP_CHANNEL (ADC_CHANNEL_7) /* �¶ȼ�� */
#define ADC0_VSP_CHANNEL  (ADC_CHANNEL_9) /*��λ��VSP���ͨ��*/

#define BEMF_CH_A ADC_CHANNEL_4  /* A�෴���Ƽ��ADCͨ���� */
#define BEMF_CH_B ADC_CHANNEL_11 /* B�෴���Ƽ��ADCͨ���� */
#define BEMF_CH_C ADC_CHANNEL_5  /* C�෴���Ƽ��ADCͨ���� */

void SystemInit(void);
void SoftDelay(u32 cnt);
void Clock_Init(void);

void Hardware_init(void);
void GPIO_init(void);
void UART_init(void);
void ADC_3Shunt_NormalModeCFG(void);
void ADC0_init(void);
void ADC1_init(void);

#endif
