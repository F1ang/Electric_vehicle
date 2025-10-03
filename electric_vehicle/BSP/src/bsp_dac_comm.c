#include "bsp_dac_comm.h"
#include "dac.h"
#include "bsp_motor.h"

/* ����ص���������(����) */
UI_Handle_t UI_Params = {
    .bDriveNum = 0,
    .pFct_DACInit = &DAC_Init,
    .pFct_DACExec = &DAC_Exec,
    .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
    .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,

};

/* �����ز���(������) */
DAC_UI_Handle_t DAC_UI_Params = {
    .hUserValue = 0,
    .hDAC_CH1_ENABLED = ENABLE,
};

/**
 * @brief DAC��ʼ��
 * @param *pHandle ����ص���������
 * @retval None
 */
void UI_DACInit(UI_Handle_t *pHandle)
{
    if (pHandle->pFct_DACInit) {
        pHandle->pFct_DACInit(pHandle);
    }
}

/**
 * @brief DACִ��
 * @param *pHandle ����ص���������
 * @retval None
 */
void UI_DACExec(UI_Handle_t *pHandle)
{
    if (pHandle->pFct_DACExec) {
        pHandle->pFct_DACExec(pHandle);
    }
}

/**
 * @brief DAC��ʼ���Ļص�
 * @param *pHandle ����ص���������
 * @retval None
 */
void DAC_Init(UI_Handle_t *pHandle)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;

    if (pDacHandle->hDAC_CH1_ENABLED == ENABLE) {
        __HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_1);
    }
}

/**
 * @brief DACִ�еĻص�
 * @param *pHandle ����ص���������
 * @retval None
 */
void DAC_Exec(UI_Handle_t *pHandle)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    if (pDacHandle->hDAC_CH1_ENABLED == ENABLE) {
        /* Q16 */
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_L, 32768 + pDacHandle->hUserValue);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    }
}

/**
 * @brief ����DAC���ֵ
 * @param *pHandle ����ص���������
 * @param bUserChNumber DACͨ����
 * @param hValue DAC���ֵ3V3:�����[0~65520]  �Ҷ���[0~4095]
 * @retval None
 */
void DAC_SetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber, int16_t hValue)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    pDacHandle->hUserValue = hValue;
}

/**
 * @brief ��ȡDAC���ֵ
 * @param *pHandle ����ص���������
 * @param bUserChNumber DACͨ����
 * @retval DAC���ֵ
 */
int16_t DAC_GetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    return (pDacHandle->hUserValue);
}

/**
 * @brief DAC���ε���
 * @retval None
 */
void DAC_Output_Printf(void)
{
#if DAC_TEST
    static int16_t cnt2 = 0;
    cnt2 += 100;        /* cnt -= 100;��ת */
    if (cnt2 > S16_MAX) /* -32768~32767 */
        cnt2 = S16_MIN;
    DAC_SetUserChannelValue(&pDAC->_Super, 0, cnt2);
    UI_DACExec(&pDAC->_Super);
#else
    DAC_SetUserChannelValue(&pDAC->_Super, 0, pwmcHandle->hIa); // ADC1->JDR1��pwmcHandle->hIa��pSTC->SPD->hElAngle
    UI_DACExec(&pDAC->_Super);
#endif
}
