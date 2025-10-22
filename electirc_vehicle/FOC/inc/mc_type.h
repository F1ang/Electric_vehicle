/******************************************************************************
 *@brief  ������Ʊ������Ͷ���
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

#endif
