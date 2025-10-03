#include "bsp_elecmobile.h"
#include "adc.h"
#include "bsp_log.h"
#include "bsp_motor.h"
#include "bsp_mci.h"

FirstOrderLowPassFilter handle_power_filter;

Handle_Break_t handle_break = {
    .dma_adc_buff = { 0 },
    .handle_power = 0,
    .break_value = 0,
    .vbus_value = 0,
};

/**
 * @brief ��ʼ��һ�׵�ͨ�˲���
 * @param filter �˲����ṹ��ָ��
 * @param alpha �˲�ϵ��(0 < �� < 1)
 * @param init_value ��ʼֵ
 * @retval None
 */
void LowPassFilter_Init(FirstOrderLowPassFilter *filter, float alpha, float init_value)
{
    filter->alpha = alpha;
    filter->prev_out = init_value;
}

/**
 * @brief һ�׵�ͨ�˲�����
 * @param filter �˲����ṹ��ָ��
 * @param input ����ֵ
 * @return �˲�������ֵ
 */
float LowPassFilter_Update(FirstOrderLowPassFilter *filter, float input)
{
    /* ���㹫ʽ: y[n] = �� * x[n] + (1 - ��) * y[n-1] */
    filter->prev_out = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_out;
    return filter->prev_out;
}

/**
 * @brief ��ȡ����ת�Ѻ�ɲ��ֵ
 * @param *pHandle ת����ɲ��
 * @retval None
 */
void Ele_Get_Handle_Break(Handle_Break_t *pHandle)
{
    pHandle->handle_power = pHandle->dma_adc_buff[0] >> 1; /* Q15���ۻ� */
    pHandle->break_value = pHandle->dma_adc_buff[1] >> 1;
    pHandle->vbus_value = pHandle->dma_adc_buff[2] >> 1;

    pHandle->handle_power_filter = LowPassFilter_Update(&handle_power_filter, pHandle->handle_power);
    pHandle->break_value_filter = LowPassFilter_Update(&handle_power_filter, pHandle->break_value);
    pHandle->vbus_value_filter = LowPassFilter_Update(&handle_power_filter, pHandle->vbus_value);

#if HANDLE_BREAK
    Ele_Speed_PowerOut(pHandle, &FOCVars);
#endif
}

/**
 * @brief ת�����żӼ���
 * @param *pHandle ת����ɲ��
 * @param *pFOCVars ���FOC����
 * @retval None
 */
void Ele_Speed_PowerOut(Handle_Break_t *pHandle, FOCVars_t *pFOCVars)
{
#if ENCODER_ENABLE
    if (STM.bState == RUN) {
        if (pHandle->handle_power_filter > (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 += 50;
        } else if (pHandle->handle_power_filter < (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 -= 50;
        }
        FOCVars.Iqdref.qI_Component2 = 0;
    }
#elif HALL_ENABLE
    if (STM.bState == RUN) {
        if (pHandle->handle_power_filter > (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 += 50;
        } else if (pHandle->handle_power_filter < (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 -= 50;
        }
        pFOCVars->Vqd.qV_Component1 = (pFOCVars->Vqd.qV_Component1 > MAX_HALL_UQ) ? MAX_HALL_UQ : (pFOCVars->Vqd.qV_Component1 < -MAX_HALL_UQ) ? -MAX_HALL_UQ
                                                                                                                                               : pFOCVars->Vqd.qV_Component1;
        FOCVars.Iqdref.qI_Component2 = 0;
    }
#elif SENSORLESS_ENABLE
    if (STM.bState == RUN && STM.bSenLess_Handle == 1) {
        /* TODO:SMO�۲��BEMF������ */
        if (pHandle->handle_power_filter > (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 += 1;
        } else if (pHandle->handle_power_filter < (pFOCVars->Vqd.qV_Component1)) {
            pFOCVars->Vqd.qV_Component1 -= 1;
        }
        pFOCVars->Vqd.qV_Component1 = (pFOCVars->Vqd.qV_Component1 > 15000) ? 15000 : (pFOCVars->Vqd.qV_Component1 < -15000) ? -15000
                                                                                                                             : pFOCVars->Vqd.qV_Component1;
        FOCVars.Iqdref.qI_Component2 = 0;
    }
#endif

    /* ת��������״̬�� */
    if ((pHandle->handle_power_filter < 2000) && (STM.bState != ANY_STOP)) {
        STM.bState = ANY_STOP;
        STM.bStop_Motor = 0;
    } else {
        STM.bStop_Motor = 1;
    }
}

/**
 * @brief ��������
 * @param *pHandle ת����ɲ��
 * @param *pFOCVars ���FOC����
 * @retval None
 */
void Ele_BackCar_Ctrl(Handle_Break_t *pHandle, FOCVars_t *pFOCVars)
{
}

/**
 * @brief ��������
 * @param *pHandle ת����ɲ��
 * @param *pFOCVars ���FOC����
 * @retval None
 */
void Ele_AntiTheft_Ctrl(Handle_Break_t *pHandle, FOCVars_t *pFOCVars)
{
}
