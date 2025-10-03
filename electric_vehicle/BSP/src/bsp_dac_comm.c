#include "bsp_dac_comm.h"
#include "dac.h"
#include "bsp_motor.h"

/* 输出回调函数对象(基类) */
UI_Handle_t UI_Params = {
    .bDriveNum = 0,
    .pFct_DACInit = &DAC_Init,
    .pFct_DACExec = &DAC_Exec,
    .pFctDACSetUserChannelValue = &DAC_SetUserChannelValue,
    .pFctDACGetUserChannelValue = &DAC_GetUserChannelValue,

};

/* 输出相关参数(派生类) */
DAC_UI_Handle_t DAC_UI_Params = {
    .hUserValue = 0,
    .hDAC_CH1_ENABLED = ENABLE,
};

/**
 * @brief DAC初始化
 * @param *pHandle 输出回调函数对象
 * @retval None
 */
void UI_DACInit(UI_Handle_t *pHandle)
{
    if (pHandle->pFct_DACInit) {
        pHandle->pFct_DACInit(pHandle);
    }
}

/**
 * @brief DAC执行
 * @param *pHandle 输出回调函数对象
 * @retval None
 */
void UI_DACExec(UI_Handle_t *pHandle)
{
    if (pHandle->pFct_DACExec) {
        pHandle->pFct_DACExec(pHandle);
    }
}

/**
 * @brief DAC初始化的回调
 * @param *pHandle 输出回调函数对象
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
 * @brief DAC执行的回调
 * @param *pHandle 输出回调函数对象
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
 * @brief 设置DAC输出值
 * @param *pHandle 输出回调函数对象
 * @param bUserChNumber DAC通道号
 * @param hValue DAC输出值3V3:左对齐[0~65520]  右对齐[0~4095]
 * @retval None
 */
void DAC_SetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber, int16_t hValue)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    pDacHandle->hUserValue = hValue;
}

/**
 * @brief 获取DAC输出值
 * @param *pHandle 输出回调函数对象
 * @param bUserChNumber DAC通道号
 * @retval DAC输出值
 */
int16_t DAC_GetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    return (pDacHandle->hUserValue);
}

/**
 * @brief DAC波形调试
 * @retval None
 */
void DAC_Output_Printf(void)
{
#if DAC_TEST
    static int16_t cnt2 = 0;
    cnt2 += 100;        /* cnt -= 100;反转 */
    if (cnt2 > S16_MAX) /* -32768~32767 */
        cnt2 = S16_MIN;
    DAC_SetUserChannelValue(&pDAC->_Super, 0, cnt2);
    UI_DACExec(&pDAC->_Super);
#else
    DAC_SetUserChannelValue(&pDAC->_Super, 0, pwmcHandle->hIa); // ADC1->JDR1、pwmcHandle->hIa、pSTC->SPD->hElAngle
    UI_DACExec(&pDAC->_Super);
#endif
}
