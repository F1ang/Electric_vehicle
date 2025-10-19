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

#endif

