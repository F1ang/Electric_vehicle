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

#define RTT_FUNCTION (FUNCTION_ON) /* RTT 调试功能 */

#define PLANTFORM_DRV_MODULE_NUM (1) /* 驱动单元数目，单电机固定为1，不需要修改 */
#define MAX_DRV_MODULE_USED      (PLANTFORM_DRV_MODULE_NUM)

extern u16 g_hDriverPolarity[MAX_DRV_MODULE_USED];

#endif
