/******************************************************************************
 *@brief  电动自行车应用层开发
 *@author By Spotted Owl
 *@date     2025.06.22
 ******************************************************************************/
#ifndef BSP_ELECMOBILE_H
#define BSP_ELECMOBILE_H

#include "main.h"
#include "bsp_global.h"

#define DMA_BUFF_LEN 3
#define MAX_HALL_UQ  15000

typedef struct {
    float alpha;    /* 滤波系数 α = dt/(dt + RC) 或 α = 1/(1 + 2πf_c*T) */
    float prev_out; /* 上一次的输出值 */
} FirstOrderLowPassFilter;

/* 转把与刹车 */
typedef struct
{
    uint16_t dma_adc_buff[DMA_BUFF_LEN];
    uint16_t handle_power;        /* 油门转把 */
    uint16_t handle_power_filter; /* 油门转把滤波 */
    uint16_t break_value;         /* 刹车 */
    uint16_t break_value_filter;  /* 刹车滤波 */
    uint16_t vbus_value;          /* 电池电压 */
    uint16_t vbus_value_filter;   /* 电池电压滤波 */
} Handle_Break_t;

extern FirstOrderLowPassFilter handle_power_filter;
extern Handle_Break_t handle_break;

void LowPassFilter_Init(FirstOrderLowPassFilter *filter, float alpha, float init_value);
float LowPassFilter_Update(FirstOrderLowPassFilter *filter, float input);
void Ele_Get_Handle_Break(Handle_Break_t *pHandle);
void Ele_Speed_PowerOut(Handle_Break_t *pHandle, FOCVars_t *pFOCVars);

#endif /* BSP_ELECMOBILE_H */
