#include "bsp_vss.h"
#include "bsp_spd.h"

/**
 * @brief �����ٶȴ�������λ
 * @param *pHandle
 * @retval None
 */
void VSS_Init(VirtualSpeedSensor_Handle_t *pHandle)
{
    pHandle->_Super.bSpeedErrorNumber = 0u;
    pHandle->_Super.hElAngle = 0;
    pHandle->_Super.hMecAngle = 0;
    pHandle->_Super.hAvrMecSpeed01Hz = 0;
    pHandle->_Super.hElSpeedDpp = 0;
    pHandle->_Super.hMecAccel01HzP = 0;
    pHandle->_Super.bSpeedErrorNumber = 0u;

    pHandle->wElAccDppP32 = 0;
    pHandle->wElSpeedDpp32 = 0;
    pHandle->hRemainingStep = 0u;
    pHandle->hElAngleAccu = 0;

    pHandle->bTransitionStarted = false;
    pHandle->bTransitionEnded = false;
    pHandle->hTransitionRemainingSteps = pHandle->hTransitionSteps;
    pHandle->bTransitionLocked = false;

    pHandle->bCopyObserver = false;
}

/**
 * @brief ���������Ƕ�,�����Ƕ����ۻ���ǶȵĲ�ֵ,������,�Ż�б�µĵ�Ƕ�
 *        1.I/F�׶�:dpp���ֳ���Ƕ� 2.б�������׶�:SMO�۲��Ƕ�-dpp���ֵ�ǶȽ��е�Ƕ�����,�ù��Ƶ�Ƕ�
 * @param *pHandle �����ٶȴ���������
 * @param *pInputVars_str �������
 * @retval ���������Ƕ�
 */
int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, void *pInputVars_str)
{
    int16_t hRetAngle;
    int16_t hAngleDiff;
    int16_t hAngleCorr;
    int32_t wAux;
    int16_t hSignCorr = 1;

    if (pHandle->bCopyObserver == true) {
        /* ֱ��ʹ�ô���ĵ�Ƕȵ������Ƕ� */
        hRetAngle = *(int16_t *)pInputVars_str;
    } else {
        /* ��Ƕȱ仯�����ۼ�,���ֳ���Ƕ� */
        pHandle->hElAngleAccu += pHandle->_Super.hElSpeedDpp;

        /* �ۼӳ���е�Ƕ� */
        pHandle->_Super.hMecAngle += pHandle->_Super.hElSpeedDpp /
                                     (int16_t)pHandle->_Super.bElToMecRatio;

        /* ״̬ת��(б��)��ʼ */
        if (pHandle->bTransitionStarted == true) {
            if (pHandle->hTransitionRemainingSteps == 0) {
                /* ״̬ת�����,�����Ƕ�ֱ�Ӹ�ֵ */
                hRetAngle = *(int16_t *)pInputVars_str;
                pHandle->bTransitionEnded = true;
                pHandle->_Super.bSpeedErrorNumber = 0u;
            } else {
                /* ״̬ת���� */
                pHandle->hTransitionRemainingSteps--;

                /* ���������Ƕ����ۻ���ǶȵĲ�ֵ */
                if (pHandle->_Super.hElSpeedDpp >= 0) {
                    hAngleDiff = *(int16_t *)pInputVars_str - pHandle->hElAngleAccu;
                } else {
                    hAngleDiff = pHandle->hElAngleAccu - *(int16_t *)pInputVars_str;
                    hSignCorr = -1;
                }

                /* ����Ƕ�����ֵ=cnt/cnt_sum * angle_diff(��ֱ�������εı�����ϵ),�ǶȲ�ֵ������״̬ת������ */
                wAux = (int32_t)hAngleDiff * pHandle->hTransitionRemainingSteps;
                hAngleCorr = (int16_t)(wAux / pHandle->hTransitionSteps);
                hAngleCorr *= hSignCorr;

                if (hAngleDiff >= 0) {
                    /* �ǶȲ�Ϊ��,����״̬,����Ƕ�-�����Ƕ�(��״̬����) */
                    pHandle->bTransitionLocked = true;
                    hRetAngle = *(int16_t *)pInputVars_str - hAngleCorr;
                } else {
                    if (pHandle->bTransitionLocked == false) {
                        hRetAngle = pHandle->hElAngleAccu;
                    } else {
                        hRetAngle = *(int16_t *)pInputVars_str + hAngleCorr;
                    }
                }
            }
        } else {
            /* ����״̬ת��,ֱ�Ӹ�ֵ */
            hRetAngle = pHandle->hElAngleAccu;
        }
    }

    /* ���������Ƕ� */
    pHandle->_Super.hElAngle = hRetAngle;

    return hRetAngle;
}

/**
 * @brief ��������е�ٶ�(���ٶȼ����ٶ�)
 *      1.bTransitionEnded=false,Vss_CalcAvrqMecSpeed01Hzֱ�ӷ����ٶȲ��ɿ�,StartUpTransitionEnded=false => �����л��ջ��۲�������
 *      2.bTransitionEnded=true,�����ٶ�SPD_IsMecSpeedReliable => StartUpTransitionEnded=1 => �л��ջ��۲�������
 * @param *pHandle �����ٶȴ���������
 * @param *hMecSpeed01Hz ��е�ٶ�0.1Hz
 * @retval �ٶ��Ƿ�ɿ� 1�ɿ� 0���ɿ�
 */
bool VSS_CalcAvrgMecSpeed01Hz(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeed01Hz)
{
    bool SpeedSensorReliability = false;

    if (pHandle->hRemainingStep > 1u) {
        /* 1.���ٶȼ�����ٶ�/��е�ٶ� */
        pHandle->wElSpeedDpp32 += pHandle->wElAccDppP32;
        pHandle->_Super.hElSpeedDpp = (int16_t)(pHandle->wElSpeedDpp32 / 65536);

        /* 2.Convert dpp into Mec01Hz */
        *hMecSpeed01Hz = (int16_t)((pHandle->_Super.hElSpeedDpp *
                                    (int32_t)pHandle->_Super.hMeasurementFrequency * 10) /
                                   (65536 * (int32_t)pHandle->_Super.bElToMecRatio));

        pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

        pHandle->hRemainingStep--;
    } else if (pHandle->hRemainingStep == 1u) {
        *hMecSpeed01Hz = pHandle->hFinalMecSpeed01Hz; /* �л���һ�׶� */
        pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;
        pHandle->_Super.hElSpeedDpp = (int16_t)(((int32_t)(*hMecSpeed01Hz) *
                                                 (int32_t)65536) /
                                                ((int32_t)10 * (int32_t)pHandle->_Super.hMeasurementFrequency)); /* rpm->dpp */

        pHandle->_Super.hElSpeedDpp *= (int16_t)(pHandle->_Super.bElToMecRatio);

        pHandle->hRemainingStep = 0u;
    } else {
        *hMecSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;
    }

    if (pHandle->bTransitionEnded == false) {
        /* 3.bTransitionEnded=false,Vss_CalcAvrqMecSpeed01Hzֱ�ӷ����ٶȲ��ɿ�,StartUpTransitionEnded=false => �����л��ջ��۲������� */
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        SpeedSensorReliability = false;
    } else {
        /* 4.hTransitionRemainingsteps=0=>bTransitionEnded=true,�����ٶ�SPD_IsMecSpeedReliable => StartUpTransitionEnded=1 => �л��ջ��۲������� */
        SpeedSensorReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeed01Hz);
    }

    return (SpeedSensorReliability);
}

/**
 * @brief ����ĵ�Ƕ�
 * @param *pHandle �����ٶȴ���������
 * @param hMecAngle ��е�Ƕ�
 * @retval None
 */
void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle)
{
    pHandle->hElAngleAccu = hMecAngle;
    pHandle->_Super.hMecAngle = pHandle->hElAngleAccu / (int16_t)pHandle->_Super.bElToMecRatio;
    pHandle->_Super.hElAngle = hMecAngle;
}

/**
 * @brief ���û�е���ٶ�����
 * @param *pHandle �����ٶȴ���������
 * @param hFinalMecSpeed01Hz Ŀ���е�ٶ�0.1Hz
 * @param hDurationms ����ʱ��ms
 * @retval None
 */
void VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t hFinalMecSpeed01Hz, uint16_t hDurationms)
{
    uint16_t hNbrStep;
    int16_t hCurrentMecSpeedDpp;
    int32_t wMecAccDppP32;
    int16_t hFinalMecSpeedDpp;

    if (pHandle->bTransitionStarted == false) {
        /* ��ʼ�� */
        if (hDurationms == 0u) {
            pHandle->_Super.hAvrMecSpeed01Hz = hFinalMecSpeed01Hz;

            /* dpp(�Ƕȱ仯��)=W_e * T */
            pHandle->_Super.hElSpeedDpp = (int16_t)(((int32_t)(hFinalMecSpeed01Hz) *
                                                     (int32_t)65536) /
                                                    ((int32_t)10 * (int32_t)pHandle->_Super.hMeasurementFrequency));

            /* ���뼫���� */
            pHandle->_Super.hElSpeedDpp *= (int16_t)(pHandle->_Super.bElToMecRatio);

            pHandle->hRemainingStep = 0u;

            pHandle->hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
        } else {
            /* cnt=t/f */
            hNbrStep = (uint16_t)(((uint32_t)hDurationms *
                                   (uint32_t)pHandle->hSpeedSamplingFreqHz) /
                                  1000u);
            hNbrStep++;
            pHandle->hRemainingStep = hNbrStep;

            /* ��ǰ��е�ٶ�dpp */
            hCurrentMecSpeedDpp = pHandle->_Super.hElSpeedDpp /
                                  (int16_t)pHandle->_Super.bElToMecRatio;

            /* Ŀ���е�ٶ�dpp */
            hFinalMecSpeedDpp = (int16_t)(((int32_t)hFinalMecSpeed01Hz * (int32_t)65536) /
                                          ((int32_t)10 * (int32_t)pHandle->_Super.hMeasurementFrequency));

            /* ��еdpp����ֵ */
            wMecAccDppP32 = (((int32_t)hFinalMecSpeedDpp - (int32_t)hCurrentMecSpeedDpp) *
                             (int32_t)65536) /
                            (int32_t)hNbrStep;

            /* ��dpp����ֵ */
            pHandle->wElAccDppP32 = wMecAccDppP32 * (int16_t)pHandle->_Super.bElToMecRatio;

            pHandle->hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
            pHandle->wElSpeedDpp32 = (int32_t)pHandle->_Super.hElSpeedDpp * (int32_t)65536;
        }
    }
}

/**
 * @brief ��ȡб�¼�����ɱ�־
 * @param *pHandle �����ٶȴ���������
 * @retval б�����
 */
bool VSS_RampCompleted(VirtualSpeedSensor_Handle_t *pHandle)
{
    bool retVal = false;
    if (pHandle->hRemainingStep == 0u) {
        retVal = true;
    }
    return retVal;
}

/**
 * @brief ��ȡб�µ������ٶ�
 * @param *pHandle �����ٶȴ���������
 * @retval б�µ������ٶ�
 */
int16_t VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle)
{
    return pHandle->hFinalMecSpeed01Hz;
}

/**
 * @brief ��λ����ִ��б�±�־
 * @param *pHandle �����ٶȴ���������
 * @param bCommand ִ��/�ر�
 * @retval 0-��ִ����� 1-ִ��
 */
bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand)
{
    bool bAux = true;
    if (bCommand == true) {
        pHandle->bTransitionStarted = true; /* ����б�� */

        /*  hTransitionSteps!=0 => StartUpTransitionEnded!=1 => �����бջ��۲������� */
        if (pHandle->hTransitionSteps == 0) {
            pHandle->bTransitionEnded = true;
            pHandle->_Super.bSpeedErrorNumber = 0u;
            bAux = false;
        }
    }

    return bAux;
}

/**
 * @brief �Ƿ�����ִ��б��
 * @param *pHandle �����ٶȴ���������
 * @retval 1-б�¿�ʼ
 */
bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle)
{
    uint16_t hTS = 0u, hTE = 0u, hAux;
    bool retVal = false;
    if (pHandle->bTransitionStarted == true) { /* б�¿�ʼ */
        hTS = 1u;
    }
    if (pHandle->bTransitionEnded == true) {
        hTE = 1u;
    }
    hAux = hTS ^ hTE;
    if (hAux != 0u) {
        retVal = true;
    }
    return (retVal);
}

/**
 * @brief ���ù۲���
 * @param *pHandle
 * @retval None
 */
void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle)
{
    pHandle->bCopyObserver = true;
}

/**
 * @brief ���õ�Ƕ�
 * @param *pHandle �����ٶȴ���������
 * @param hElAngle ��Ƕ�
 * @retval None
 */
void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle)
{
    pHandle->hElAngleAccu = hElAngle;
    pHandle->_Super.hElAngle = hElAngle;
}
