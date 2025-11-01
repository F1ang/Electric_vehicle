/******************************************************************************
 *@brief  电机控制数据结构相关
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef MC_TYPE_H
#define MC_TYPE_H

#include "global_variable.h"

typedef struct
{
    int16_t qI_Component1; /* q alpha a */
    int16_t qI_Component2; /* d beta b */
} Curr_Components;

typedef struct
{
    int16_t qV_Component1; /* q alpha a */
    int16_t qV_Component2; /* d beta b */
} Volt_Components;

#endif
