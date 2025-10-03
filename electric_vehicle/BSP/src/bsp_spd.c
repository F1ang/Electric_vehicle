#include "bsp_spd.h"

/**
 * @brief  获取电角度
 * @param *pHandle 速度位置反馈对象
 * @retval 电角度
 */
int16_t SPD_GetElAngle(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hElAngle);
}

/**
 * @brief  获取机械角度
 * @param *pHandle
 * @retval 机械角度
 */
int16_t SPD_GetMecAngle(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hMecAngle);
}

/**
 * @brief 获取平均机械速度0.1Hz
 * @param *pHandle 速度位置反馈对象
 * @retval 平均机械速度0.1Hz
 */
int16_t SPD_GetAvrgMecSpeed01Hz(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hAvrMecSpeed01Hz);
}

/**
 * @brief 获取dpp
 * @param *pHandle 速度位置反馈对象
 * @retval dpp
 */
int16_t SPD_GetElSpeedDpp(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hElSpeedDpp);
}

/**
 * @brief 检查速度是否有效
 * @param *pHandle 速度位置反馈对象
 * @retval 速度是否有效
 */
bool SPD_Check(SpeednPosFdbk_Handle_t *pHandle)
{
    bool SpeedSensorReliability = true;
    if (pHandle->bSpeedErrorNumber ==
        pHandle->bMaximumSpeedErrorsNumber) {
        SpeedSensorReliability = false;
    }
    return (SpeedSensorReliability);
}

/**
 * @brief 机械速度在限制范围内
 *        1.检查速度上下限 2.检查加速度上限 3.更新速度异常次数 4.返回机械速度是否有效
 * @param *pHandle 速度位置反馈对象
 * @param *pMecSpeed01Hz 机械速度
 * @retval 机械速度是否有效 1:有效 0:无效
 */
bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
{
    bool SpeedSensorReliability = true;
    uint8_t bSpeedErrorNumber;
    uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;

    bool SpeedError = false;
    uint16_t hAbsMecSpeed01Hz, hAbsMecAccel01HzP;
    int16_t hAux;

    bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

    /* 1.计算机械速度绝对值0.1Hz */
    if (*pMecSpeed01Hz < 0) {
        hAux = -(*pMecSpeed01Hz);
        hAbsMecSpeed01Hz = (uint16_t)(hAux);
    } else {
        hAbsMecSpeed01Hz = (uint16_t)(*pMecSpeed01Hz);
    }

    /* 2.检查速度上下限 */
    if (hAbsMecSpeed01Hz > pHandle->hMaxReliableMecSpeed01Hz) {
        SpeedError = true;
    }

    if (hAbsMecSpeed01Hz < pHandle->hMinReliableMecSpeed01Hz) {
        SpeedError = true;
    }

    /* 3.计算机械加速度绝对值0.1HzP */
    if (pHandle->hMecAccel01HzP < 0) {
        hAux = -(pHandle->hMecAccel01HzP);
        hAbsMecAccel01HzP = (uint16_t)(hAux);
    } else {
        hAbsMecAccel01HzP = (uint16_t)(pHandle->hMecAccel01HzP);
    }

    /* 4.检查加速度上限 */
    if (hAbsMecAccel01HzP > pHandle->hMaxReliableMecAccel01HzP) {
        SpeedError = true;
    }

    /* 5.更新速度异常次数 */
    if (SpeedError == true) {
        if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber) {
            bSpeedErrorNumber++;
        }
    } else {
        if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber) {
            bSpeedErrorNumber = 0u;
        }
    }

    if (bSpeedErrorNumber == bMaximumSpeedErrorsNumber) {
        SpeedSensorReliability = false;
    }

    pHandle->bSpeedErrorNumber = bSpeedErrorNumber;

    return (SpeedSensorReliability);
}

/**
 * @brief 获取S16速度
 * @param *pHandle 速度位置反馈对象
 * @retval S16速度
 */
int16_t SPD_GetS16Speed(SpeednPosFdbk_Handle_t *pHandle)
{
    int32_t wAux = (int32_t)pHandle->hAvrMecSpeed01Hz;
    wAux *= INT16_MAX;
    wAux /= (int16_t)pHandle->hMaxReliableMecSpeed01Hz;
    return (int16_t)wAux;
}

/**
 * @brief 获取极对数
 * @param *pHandle 速度位置反馈对象
 * @retval 极对数
 */
uint8_t SPD_GetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->bElToMecRatio);
}

/**
 * @brief  设置极对数
 * @param *pHandle 速度位置反馈对象
 * @param bPP 极对数
 * @retval None
 */
void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP)
{
    pHandle->bElToMecRatio = bPP;
}
