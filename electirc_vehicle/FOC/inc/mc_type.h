/******************************************************************************
 *@brief  ������Ʊ������Ͷ���
 *@author By Spotted Owl
 *@date     2025.10.07
 ******************************************************************************/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

#include "global_variable.h"

/* ---------------------------- hardware parameter ----------------------- */
#define CURRENT_SAMPLE_1SHUNT 1 /* ������ */
#define CURRENT_SAMPLE_2SHUNT 2 /* ˫���� */
#define CURRENT_SAMPLE_3SHUNT 3 /* ������ */

#define EPWM0_CURRENT_SAMPLE_TYPE (CURRENT_SAMPLE_3SHUNT)

#define LKS32MC070 0x01 /*оƬ�ͺ�*/
#define LKS32MC071 0x02 /*оƬ�ͺ�*/
#define LKS32MC072 0x03 /*оƬ�ͺ�*/

#define P_HIGH__N_HIGH 0 /* Ԥ��Ԥ���������� �Ϲܸߵ�ƽ��Ч���¹ܸߵ�ƽ��Ч */
#define P_HIGH__N_LOW  1 /* Ԥ��Ԥ���������� �Ϲܸߵ�ƽ��Ч���¹ܵ͵�ƽ��Ч */

#define CHIP_PART_NUMBER LKS32MC071 /* оƬ�ͺ�ѡ��ѡ����ȷ��Ӱ��оƬģ��ĳ�ʼ�� */
#if ((CHIP_PART_NUMBER == LKS32MC074D) || (CHIP_PART_NUMBER == LKS32MC076D) || (CHIP_PART_NUMBER == LKS32MC077D))
#define MCPWM_SWAP_FUNCTION 1
#define PRE_DRIVER_POLARITY P_HIGH__N_HIGH /* Ԥ��Ԥ���������� �Ϲܸߵ�ƽ��Ч���¹ܸߵ�ƽ��Ч */
#else
#define PRE_DRIVER_POLARITY P_HIGH__N_LOW /* Ԥ��Ԥ���������� �Ϲܸߵ�ƽ��Ч���¹ܵ͵�ƽ��Ч */
#endif

/* ****************************************** �ϵ紦��  ****************************************** */
typedef struct
{
    u16 power_up_time; /* �ϵ�ʱ����� */
} PowerUp_t;

/* ****************************************** ״̬��  ****************************************** */
typedef enum {
    IDLE = 0, /* ����״̬ */
    INIT = 1, /* ��ʼ��״̬ */
    RUN = 2,  /* ��������״̬ */
    STOP = 3, /* ֹͣ״̬ */
    WAIT = 4, /* �ȴ�״̬ */
} SystStatus_m;

/* PID�㷨�����ṹ�� */
typedef struct
{
    s16 hKp_Gain;
    u16 hKp_Divisor;
    s16 hKi_Gain;
    u16 hKi_Divisor;
    s16 hLower_Limit_Output;   // Lower Limit for Output limitation
    s16 hUpper_Limit_Output;   // Lower Limit for Output limitation
    s32 wLower_Limit_Integral; // Lower Limit for Integral term limitation
    s32 wUpper_Limit_Integral; // Lower Limit for Integral term limitation
    s32 wIntegral;
} PI_Ctrl_t;

/* FOC���Ʋ����ṹ�� */
typedef struct
{
    volatile u16 nSys_TimerPWM; /* PWM���ڼ���Cnt */

    s16 nBusVoltage;       /* ֱ��ĸ�ߵ�ѹ */
    s16 nBusVoltagefir;    /* ֱ��ĸ�ߵ�ѹ�˲�ֵ */
    s16 nBusCurrentadc;    /* ֱ��ĸ�ߵ�������ֵ */
    s16 nBusCurrentadcfir; /* ֱ��ĸ�ߵ����˲�ֵ */

    int16_t nSampCurDat0; // ��������ֵ0
    int16_t nSampCurDat1; // ��������ֵ1

    s16 nPhaseAOffset; /* A��ADC���� Offsetֵ */
    s16 nPhaseBOffset; /* B��ADC���� Offsetֵ */
    s16 nPhaseCOffset; /* C��ADC���� Offsetֵ */

    volatile s16 nDCur_Reference; /* D��������� */
    volatile s16 nRequestPower;   /* Q���ѹ���� */
    PI_Ctrl_t struPI_Torque;      /* Q�������PI���� */
    PI_Ctrl_t struPI_Flux;        /* D�������PI���� */
} FOC_Ctrl_t;

#endif
