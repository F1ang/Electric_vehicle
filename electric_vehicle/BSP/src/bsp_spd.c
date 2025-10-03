#include "bsp_spd.h"

/**
 * @brief  ��ȡ��Ƕ�
 * @param *pHandle �ٶ�λ�÷�������
 * @retval ��Ƕ�
 */
int16_t SPD_GetElAngle(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hElAngle);
}

/**
 * @brief  ��ȡ��е�Ƕ�
 * @param *pHandle
 * @retval ��е�Ƕ�
 */
int16_t SPD_GetMecAngle(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hMecAngle);
}

/**
 * @brief ��ȡƽ����е�ٶ�0.1Hz
 * @param *pHandle �ٶ�λ�÷�������
 * @retval ƽ����е�ٶ�0.1Hz
 */
int16_t SPD_GetAvrgMecSpeed01Hz(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hAvrMecSpeed01Hz);
}

/**
 * @brief ��ȡdpp
 * @param *pHandle �ٶ�λ�÷�������
 * @retval dpp
 */
int16_t SPD_GetElSpeedDpp(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->hElSpeedDpp);
}

/**
 * @brief ����ٶ��Ƿ���Ч
 * @param *pHandle �ٶ�λ�÷�������
 * @retval �ٶ��Ƿ���Ч
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
 * @brief ��е�ٶ������Ʒ�Χ��
 *        1.����ٶ������� 2.�����ٶ����� 3.�����ٶ��쳣���� 4.���ػ�е�ٶ��Ƿ���Ч
 * @param *pHandle �ٶ�λ�÷�������
 * @param *pMecSpeed01Hz ��е�ٶ�
 * @retval ��е�ٶ��Ƿ���Ч 1:��Ч 0:��Ч
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

    /* 1.�����е�ٶȾ���ֵ0.1Hz */
    if (*pMecSpeed01Hz < 0) {
        hAux = -(*pMecSpeed01Hz);
        hAbsMecSpeed01Hz = (uint16_t)(hAux);
    } else {
        hAbsMecSpeed01Hz = (uint16_t)(*pMecSpeed01Hz);
    }

    /* 2.����ٶ������� */
    if (hAbsMecSpeed01Hz > pHandle->hMaxReliableMecSpeed01Hz) {
        SpeedError = true;
    }

    if (hAbsMecSpeed01Hz < pHandle->hMinReliableMecSpeed01Hz) {
        SpeedError = true;
    }

    /* 3.�����е���ٶȾ���ֵ0.1HzP */
    if (pHandle->hMecAccel01HzP < 0) {
        hAux = -(pHandle->hMecAccel01HzP);
        hAbsMecAccel01HzP = (uint16_t)(hAux);
    } else {
        hAbsMecAccel01HzP = (uint16_t)(pHandle->hMecAccel01HzP);
    }

    /* 4.�����ٶ����� */
    if (hAbsMecAccel01HzP > pHandle->hMaxReliableMecAccel01HzP) {
        SpeedError = true;
    }

    /* 5.�����ٶ��쳣���� */
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
 * @brief ��ȡS16�ٶ�
 * @param *pHandle �ٶ�λ�÷�������
 * @retval S16�ٶ�
 */
int16_t SPD_GetS16Speed(SpeednPosFdbk_Handle_t *pHandle)
{
    int32_t wAux = (int32_t)pHandle->hAvrMecSpeed01Hz;
    wAux *= INT16_MAX;
    wAux /= (int16_t)pHandle->hMaxReliableMecSpeed01Hz;
    return (int16_t)wAux;
}

/**
 * @brief ��ȡ������
 * @param *pHandle �ٶ�λ�÷�������
 * @retval ������
 */
uint8_t SPD_GetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle)
{
    return (pHandle->bElToMecRatio);
}

/**
 * @brief  ���ü�����
 * @param *pHandle �ٶ�λ�÷�������
 * @param bPP ������
 * @retval None
 */
void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP)
{
    pHandle->bElToMecRatio = bPP;
}
