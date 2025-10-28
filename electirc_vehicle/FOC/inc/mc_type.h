/******************************************************************************
 *@brief  电机控制变量类型定义
 *@author By Spotted Owl
 *@date     2025.10.07
 ******************************************************************************/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

#include "global_variable.h"

#define FUNCTION_ON  1
#define FUNCTION_OFF 0

#define S16FRACT_MAX (0.999969482421875)
#define S16FRACT_MIN (-1.0)

#define S32FRACT_MAX (0.9999999995343387126922607421875)
#define S32FRACT_MIN (-1.0)

#define FRAC16(x) ((s16)((x) < (S16FRACT_MAX) ? ((x) >= S16FRACT_MIN ? (x) * 0x8000 : 0x8000) : 0x7fff))

/*! Macro converting a signed fractional [-1,1) number into a fixed point 32-bit
 * number in the format Q1.31. */
#define FRAC32(x)      ((s32)((x) < (S32FRACT_MAX) ? ((x) >= S32FRACT_MIN ? (x) * 0x80000000 : 0x80000000) : 0x7fffffff))
#define ABS(X)         (((X) >= 0) ? (X) : -(X))
#define sat(x, ll, ul) ((x) > (ul)) ? (ul) : (((x) < (ll)) ? (ll) : (x))
#define _IQ15mpy(A, B) (((s32)A * B) >> 15) /* Q15 */
#define _IQ12mpy(A, B) (((s32)A * B) >> 12) /* Q12 */
#define _IQ15(A)       (s32)(32767.0 * A)
#define SQRT_2         (1.414213562373095)

/* ----------------------------direction const define----------------------- */
#define CW  (0) /* motor dir:cw */
#define CCW (1) /* motor dir:ccw*/

#define CURRENT_LOOP (0) /* current run */
#define SPEED_LOOP   (1) /* speed run */
#define POWER_LOOP   (2) /* power run */

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
