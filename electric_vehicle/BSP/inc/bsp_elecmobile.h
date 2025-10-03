/******************************************************************************
 *@brief  �綯���г�Ӧ�ò㿪��
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
    float alpha;    /* �˲�ϵ�� �� = dt/(dt + RC) �� �� = 1/(1 + 2��f_c*T) */
    float prev_out; /* ��һ�ε����ֵ */
} FirstOrderLowPassFilter;

/* ת����ɲ�� */
typedef struct
{
    uint16_t dma_adc_buff[DMA_BUFF_LEN];
    uint16_t handle_power;        /* ����ת�� */
    uint16_t handle_power_filter; /* ����ת���˲� */
    uint16_t break_value;         /* ɲ�� */
    uint16_t break_value_filter;  /* ɲ���˲� */
    uint16_t vbus_value;          /* ��ص�ѹ */
    uint16_t vbus_value_filter;   /* ��ص�ѹ�˲� */
} Handle_Break_t;

extern FirstOrderLowPassFilter handle_power_filter;
extern Handle_Break_t handle_break;

void LowPassFilter_Init(FirstOrderLowPassFilter *filter, float alpha, float init_value);
float LowPassFilter_Update(FirstOrderLowPassFilter *filter, float input);
void Ele_Get_Handle_Break(Handle_Break_t *pHandle);
void Ele_Speed_PowerOut(Handle_Break_t *pHandle, FOCVars_t *pFOCVars);

#endif /* BSP_ELECMOBILE_H */
