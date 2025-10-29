/******************************************************************************
 *@brief  电机控制变量类型定义
 *@author By Spotted Owl
 *@date     2025.10.07
 ******************************************************************************/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

#include "global_variable.h"

/* ---------------------------- hardware parameter ----------------------- */
#define CURRENT_SAMPLE_1SHUNT 1 /* 单电阻 */
#define CURRENT_SAMPLE_2SHUNT 2 /* 双电阻 */
#define CURRENT_SAMPLE_3SHUNT 3 /* 三电阻 */

#define EPWM0_CURRENT_SAMPLE_TYPE (CURRENT_SAMPLE_3SHUNT)

#define LKS32MC070 0x01 /*芯片型号*/
#define LKS32MC071 0x02 /*芯片型号*/
#define LKS32MC072 0x03 /*芯片型号*/

#define P_HIGH__N_HIGH 0 /* 预驱预动极性设置 上管高电平有效，下管高电平有效 */
#define P_HIGH__N_LOW  1 /* 预驱预动极性设置 上管高电平有效，下管低电平有效 */

#define CHIP_PART_NUMBER LKS32MC071 /* 芯片型号选择，选择不正确将影响芯片模块的初始化 */
#if ((CHIP_PART_NUMBER == LKS32MC074D) || (CHIP_PART_NUMBER == LKS32MC076D) || (CHIP_PART_NUMBER == LKS32MC077D))
#define MCPWM_SWAP_FUNCTION 1
#define PRE_DRIVER_POLARITY P_HIGH__N_HIGH /* 预驱预动极性设置 上管高电平有效，下管高电平有效 */
#else
#define PRE_DRIVER_POLARITY P_HIGH__N_LOW /* 预驱预动极性设置 上管高电平有效，下管低电平有效 */
#endif

/* ****************************************** 上电处理  ****************************************** */
typedef struct
{
    u16 power_up_time; /* 上电时间计数 */
} PowerUp_t;

/* ****************************************** 状态机  ****************************************** */
typedef enum {
    IDLE = 0, /* 空闲状态 */
    INIT = 1, /* 初始化状态 */
    RUN = 2,  /* 正常运行状态 */
    STOP = 3, /* 停止状态 */
    WAIT = 4, /* 等待状态 */
} SystStatus_m;

/* PID算法参数结构体 */
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

/* FOC控制参数结构体 */
typedef struct
{
    volatile u16 nSys_TimerPWM; /* PWM周期计数Cnt */

    s16 nBusVoltage;       /* 直流母线电压 */
    s16 nBusVoltagefir;    /* 直流母线电压滤波值 */
    s16 nBusCurrentadc;    /* 直流母线电流采样值 */
    s16 nBusCurrentadcfir; /* 直流母线电流滤波值 */

    int16_t nSampCurDat0; // 电流采样值0
    int16_t nSampCurDat1; // 电流采样值1

    s16 nPhaseAOffset; /* A相ADC采样 Offset值 */
    s16 nPhaseBOffset; /* B相ADC采样 Offset值 */
    s16 nPhaseCOffset; /* C相ADC采样 Offset值 */

    volatile s16 nDCur_Reference; /* D轴电流给定 */
    volatile s16 nRequestPower;   /* Q轴电压给定 */
    PI_Ctrl_t struPI_Torque;      /* Q轴电流环PI参数 */
    PI_Ctrl_t struPI_Flux;        /* D轴电流环PI参数 */
} FOC_Ctrl_t;

#endif
