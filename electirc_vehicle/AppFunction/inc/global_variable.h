/******************************************************************************
 *@brief  ȫ�ֱ��������붨��
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "main.h"

/* ---------------------------- ����ʹ��/ʧ������ ----------------------- */
#define TEST_XH_CTRL_OUT_ENABLE 0 /* ������֤���� */

#define FUNCTION_ON  1
#define FUNCTION_OFF 0

#define RTT_FUNCTION   (FUNCTION_ON) /* RTT ���Թ��� */
#define MCU_DSP_SINCOS (FUNCTION_ON) /* MCU DSP ���Ҳ����� */
#define MCU_DSP_DIV    (FUNCTION_ON) /* MCU DSP �������� */

#define PLANTFORM_DRV_MODULE_NUM (1) /* ������Ԫ��Ŀ��������̶�Ϊ1������Ҫ�޸� */
#define MAX_DRV_MODULE_USED      (PLANTFORM_DRV_MODULE_NUM)

/* ---------------------------- direction const define ----------------------- */
#define CW  0 /* motor dir:cw */
#define CCW 1 /* motor dir:ccw*/

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

extern u16 g_hDriverPolarity[MAX_DRV_MODULE_USED];

#endif
