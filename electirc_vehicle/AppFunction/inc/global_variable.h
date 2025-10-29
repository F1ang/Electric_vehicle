/******************************************************************************
 *@brief  全局变量头文件
 *@author By Spotted Owl
 *@date     2025.10.07
 ******************************************************************************/
#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "main.h"
#include "mc_type.h"

#define TEST_XH_CTRL_OUT_ENABLE 1 /* 波形验证配置 */

#define FUNCTION_ON  1
#define FUNCTION_OFF 0

#define RTT_FUNCTION   (FUNCTION_ON) /* RTT 调试功能 */
#define MCU_DSP_SINCOS (FUNCTION_ON) /* MCU DSP 正弦波生成 */
#define MCU_DSP_DIV    (FUNCTION_ON) /* MCU DSP 除法运算 */

#define PLANTFORM_DRV_MODULE_NUM (1) /* 驱动单元数目，单电机固定为1，不需要修改 */
#define MAX_DRV_MODULE_USED      (PLANTFORM_DRV_MODULE_NUM)

#define S16FRACT_MAX (0.999969482421875)
#define S16FRACT_MIN (-1.0)

#define S32FRACT_MAX (0.9999999995343387126922607421875)
#define S32FRACT_MIN (-1.0)

/*! Macro converting a signed fractional [-1,1) number into a fixed point 32-bit
 * number in the format Q1.31. */
#define FRAC32(x)      ((s32)((x) < (S32FRACT_MAX) ? ((x) >= S32FRACT_MIN ? (x) * 0x80000000 : 0x80000000) : 0x7fffffff))
#define FRAC16(x)      ((s16)((x) < (S16FRACT_MAX) ? ((x) >= S16FRACT_MIN ? (x) * 0x8000 : 0x8000) : 0x7fff))
#define ABS(X)         (((X) >= 0) ? (X) : -(X))
#define sat(x, ll, ul) ((x) > (ul)) ? (ul) : (((x) < (ll)) ? (ll) : (x))
#define _IQ15mpy(A, B) (((s32)A * B) >> 15) /* Q15 */
#define _IQ12mpy(A, B) (((s32)A * B) >> 12) /* Q12 */
#define _IQ15(A)       (s32)(32767.0 * A)
#define SQRT_2         (1.414213562373095)
#define SQRT_3         1.732051
#define DIVSQRT3       (s16)0x49E6 /* 1/sqrt(3) in q1.15 format=0.5773315*/
/* Offset for cos(Theta)=sin_cos_Table[index_sin+offset]*/
#define OFFSET 64
#define PI     3.1416

// to be deleted if not used by anyone
#define q0dot5          ((s16)0x4000) // 0.5 in 1.5
#define qSqrtd2         ((s16)0x6ED9) // sqrt3/2 in 1.15
#define qSqrt_inverse   0x5A11        // sqrt(3)* Vdc_invT
#define qSqrtd2_inverse 0x2D0f        //(sqrt(3))/2* Vdc_invT
#define q3d2_inverse    0x4E0C        // 3/2* Vdc_invT

#define SPEED_CONTROL    (u32)0x0001
#define FIRST_START      (u32)0x0002
#define START_UP_FAILURE (u32)0x0004
#define SPEED_FEEDBACK   (u32)0x0008
#define BRAKE_ON         (u32)0x0010
#define OVERHEAT         (u32)0x0100
#define OVER_CURRENT     (u32)0x0200
#define OVER_VOLTAGE     (u32)0x0400
#define UNDER_VOLTAGE    (u32)0x0800

/* ----------------------------direction const define----------------------- */
#define CW  (0) /* motor dir:cw */
#define CCW (1) /* motor dir:ccw*/

#define CURRENT_LOOP (0) /* current run */
#define SPEED_LOOP   (1) /* speed run */
#define POWER_LOOP   (2) /* power run */

extern u16 g_hDriverPolarity[MAX_DRV_MODULE_USED];
extern FOC_Ctrl_t foc_ctrl;

#endif
