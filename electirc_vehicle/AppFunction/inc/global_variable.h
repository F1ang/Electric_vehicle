/******************************************************************************
 *@brief  全局变量声明与定义
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "main.h"

/* ---------------------------- 功能使能/失能配置 ----------------------- */
#define TEST_XH_CTRL_OUT_ENABLE 0 /* 波形验证配置 */

#define FUNCTION_ON  1
#define FUNCTION_OFF 0

#define RTT_FUNCTION   (FUNCTION_ON) /* RTT 调试功能 */
#define MCU_DSP_SINCOS (FUNCTION_ON) /* MCU DSP 正弦波生成 */
#define MCU_DSP_DIV    (FUNCTION_ON) /* MCU DSP 除法运算 */

#define PLANTFORM_DRV_MODULE_NUM (1) /* 驱动单元数目，单电机固定为1，不需要修改 */
#define MAX_DRV_MODULE_USED      (PLANTFORM_DRV_MODULE_NUM)

/* ---------------------------- direction const define ----------------------- */
#define CW  0 /* motor dir:cw */
#define CCW 1 /* motor dir:ccw*/

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

extern u16 g_hDriverPolarity[MAX_DRV_MODULE_USED];

#endif
