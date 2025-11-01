/******************************************************************************
 *@brief  电机建系解耦相关
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef __MC_MATCH_H
#define __MC_MATCH_H

#include "global_variable.h"
#include "mc_type.h"

/* ---------------------------- Const define ----------------------- */
#define U8_MAX  ((u8)255)
#define S8_MAX  ((s8)127)
#define S8_MIN  ((s8) - 128)
#define U16_MAX ((u16)65535u)
#define S16_MAX ((s16)32767)
#define S16_MIN ((s16) - 32768)
#define U32_MAX ((u32)4294967295uL)
#define S32_MAX ((s32)2147483647)
#define S32_MIN ((s32) - 2147483648)

#define S16FRACT_MAX (0.999969482421875)
#define S16FRACT_MIN (-1.0)

#define S32FRACT_MAX (0.9999999995343387126922607421875)
#define S32FRACT_MIN (-1.0)

#define FRAC32(x)      ((s32)((x) < (S32FRACT_MAX) ? ((x) >= S32FRACT_MIN ? (x) * 0x80000000 : 0x80000000) : 0x7fffffff))
#define FRAC16(x)      ((s16)((x) < (S16FRACT_MAX) ? ((x) >= S16FRACT_MIN ? (x) * 0x8000 : 0x8000) : 0x7fff))
#define _IQ15mpy(A, B) (((s32)A * B) >> 15) /* Q15 */
#define _IQ12mpy(A, B) (((s32)A * B) >> 12) /* Q12 */
#define _IQ15(A)       (s32)(32767.0 * A)

#define divSQRT_3 (int32_t)0x49E6 /* 1/sqrt(3) in q1.15 format=0.5773315*/
#define OFFSET    64              /* Offset for cos(Theta)=sin_cos_Table[index_sin+offset]*/
#define PI        3.1416
#define SQRT_2    1.4142
#define SQRT_3    1.732051

// to be deleted if not used by anyone
#define q0dot5          ((s16)0x4000) // 0.5 in 1.5
#define qSqrtd2         ((s16)0x6ED9) // sqrt3/2 in 1.15
#define qSqrt_inverse   0x5A11        // sqrt(3)* Vdc_invT
#define qSqrtd2_inverse 0x2D0f        //(sqrt(3))/2* Vdc_invT
#define q3d2_inverse    0x4E0C        // 3/2* Vdc_invT

#define ABS(X)         (((X) >= 0) ? (X) : -(X))
#define sat(x, ll, ul) ((x) > (ul)) ? (ul) : (((x) < (ll)) ? (ll) : (x))

/* 正弦函数 */
typedef struct
{
    s16 hCos;
    s16 hSin;
} Trig_Components;

/* 斜坡运算 */
typedef struct
{
    s32 wRampIn;
    s32 wRampTmp;
    s32 wRampIncrease;
    s32 wRampDecrease;
    s32 wRampOut;
} STR_RampGen32, *PSTR_RampGen32;

s16 Sqrt_Functions(s16 x, s16 y);
s16 Div_Functions(s32 y, s16 x);
void CopyFromBuffer(u8 *nDestAddr, u8 *pSrcBuff, u16 nSize);
void ramp32GenInit(PSTR_RampGen32 this);
void ramp32GenCalc(PSTR_RampGen32 this);
Trig_Components Trig_Functions(s16 hAngle);
Curr_Components Clarke(Curr_Components Curr_Input);
Curr_Components Park(Curr_Components Curr_Input, s16 Theta);
Volt_Components Rev_Park(Volt_Components Volt_Input);

#endif
