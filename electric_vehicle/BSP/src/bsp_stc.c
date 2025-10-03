#include "bsp_stc.h"
#include "bsp_spd.h"
#include "bsp_pid.h"

/**
 * @brief �ٶ�ת�س�ʼ��
 * @param *pHandle �ٶ�ת�ض���
 * @param *pPI PID������
 * @param *SPD_Handle �ٶȻ�����
 * @retval None
 */
void STC_Init(SpeednTorqCtrl_Handle_t *pHandle, PID_Handle_t *pPI, SpeednPosFdbk_Handle_t *SPD_Handle)
{
    pHandle->PISpeed = pPI;
    pHandle->SPD = SPD_Handle;
    pHandle->Mode = pHandle->ModeDefault;
    pHandle->SpeedRef01HzExt = (int32_t)pHandle->MecSpeedRef01HzDefault * 65536; /* 100Hz*65536 */
    pHandle->TorqueRef = (int32_t)pHandle->TorqueRefDefault * 65536;             /* 316*65536 */
    pHandle->TargetFinal = 0;
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
}

/**
 * @brief �ٶȶ���ʵ����
 * @param *pHandle �ٶ�ת�ض���
 * @param *SPD_Handle �ٶȻ�����
 * @retval None
 */
void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *SPD_Handle)
{
    pHandle->SPD = SPD_Handle;
}

/**
 * @brief ��ȡ�ٶȻ�����
 * @param *pHandle �ٶ�ת�ض���
 * @retval SpeednPosFdbk_Handle_t * �ٶȻ�����
 */
SpeednPosFdbk_Handle_t *STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle)
{
    return (pHandle->SPD);
}

/**
 * @brief ��λ������
 * @param *pHandle
 * @retval None
 */
void STC_Clear(SpeednTorqCtrl_Handle_t *pHandle)
{
    if (pHandle->Mode == STC_SPEED_MODE) {
        PID_SetIntegralTerm(pHandle->PISpeed, 0);
    }
}

/**
 * @brief ��ȡ��е�ٶ�
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval int16_t ��е�ٶ�
 */
int16_t STC_GetMecSpeedRef01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return ((int16_t)(pHandle->SpeedRef01HzExt / 65536));
}

/**
 * @brief ��ȡ�ο�ת��
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval int16_t �ο�ת��
 */
int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle)
{
    return ((int16_t)(pHandle->TorqueRef / 65536));
}

/**
 * @brief ���õ������ģʽ
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @param bMode ת��/�ٶ�
 * @retval None
 */
void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, STC_Modality_t bMode)
{
    pHandle->Mode = bMode;
    pHandle->RampRemainingStep = 0u; /* Interrupts previous ramp. */
}

/**
 * @brief ��ȡ����ģʽ
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval STC_Modality_t ����ģʽ
 */
STC_Modality_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->Mode;
}

#define CHECK_BOUNDARY
/**
 * @brief ִ���ٶ�/ת��б��
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @param hTargetFinal Ŀ��ֵ
 * @param hDurationms ����ʱ��
 * @retval bool ������Χ
 */
bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms)
{
    bool AllowedRange = true;
    uint32_t wAux;
    int32_t wAux1;
    int16_t hCurrentReference;

    /* Check if the hTargetFinal is out of the bound of application. */
    if (pHandle->Mode == STC_TORQUE_MODE) {
        hCurrentReference = STC_GetTorqueRef(pHandle);
#ifdef CHECK_BOUNDARY
        if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxPositiveTorque) {
            AllowedRange = false;
        }
        if ((int32_t)hTargetFinal < (int32_t)pHandle->MinNegativeTorque) {
            AllowedRange = false;
        }
#endif
    } else {
        hCurrentReference = (int16_t)(pHandle->SpeedRef01HzExt >> 16);
#ifdef CHECK_BOUNDARY
        if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxAppPositiveMecSpeed01Hz) {
            AllowedRange = false;
        } else if (hTargetFinal < pHandle->MinAppNegativeMecSpeed01Hz) {
            AllowedRange = false;
        } else if ((int32_t)hTargetFinal < (int32_t)pHandle->MinAppPositiveMecSpeed01Hz) {
            if (hTargetFinal > pHandle->MaxAppNegativeMecSpeed01Hz) {
                AllowedRange = false;
            }
        } else {
        }
#endif
    }

    if (AllowedRange == true) {
        /* б�´���ʼ��0��ʼ */
        if (hDurationms == 0u) {
            if (pHandle->Mode == STC_SPEED_MODE) {
                pHandle->SpeedRef01HzExt = (int32_t)hTargetFinal * 65536;
            } else {
                pHandle->TorqueRef = (int32_t)hTargetFinal * 65536;
            }
            pHandle->RampRemainingStep = 0u;
            pHandle->IncDecAmount = 0;
        } else {
            /* Store the hTargetFinal to be applied in the last step */
            pHandle->TargetFinal = hTargetFinal;

            /* cnt*1/f=t */
            wAux = (uint32_t)hDurationms * (uint32_t)pHandle->STCFrequencyHz;
            wAux /= 1000u;
            pHandle->RampRemainingStep = wAux;
            pHandle->RampRemainingStep++;

            /* I_lamp */
            wAux1 = ((int32_t)hTargetFinal - (int32_t)hCurrentReference) * 65536;
            wAux1 /= (int32_t)pHandle->RampRemainingStep;
            pHandle->IncDecAmount = wAux1;
        }
    }

    return AllowedRange;
}

/**
 * @brief ֹͣ�ٶ�/ת��б��
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval None
 */
void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle)
{
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
}

/**
 * @brief ����ο�ת�ط���(��Ҫб�´���)
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval int16_t �ο�ת�ط���
 */
int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle)
{
    /* λ��->ʵʱ����Ŀ���ٶ�->���->�ٶȲο�ֵ */
    /* λ��->����I��Ť��->�������ٶ�->ת�� */
    int32_t wCurrentReference;
    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (pHandle->Mode == STC_TORQUE_MODE) {
        wCurrentReference = pHandle->TorqueRef; /* ��ʼת��316*65536,ת������ */
    } else {
        wCurrentReference = pHandle->SpeedRef01HzExt; /* ��ʼ�ٶ�166*65536 */
    }

    /* Update the speed reference or the torque reference according to the mode
       and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1u) {
        /* Increment/decrement the reference value. */
        wCurrentReference += pHandle->IncDecAmount;

        /* Decrement the number of remaining steps */
        pHandle->RampRemainingStep--;
    } else if (pHandle->RampRemainingStep == 1u) {
        /* Set the backup value of hTargetFinal. */
        wCurrentReference = (int32_t)pHandle->TargetFinal * 65536;
        pHandle->RampRemainingStep = 0u;
    } else {
        /* Do nothing. */
    }

    /* λ��->ʵʱ����Ŀ���ٶ�->���->�ٶȲο�ֵ */
    if (pHandle->Mode == STC_SPEED_MODE) {
        /* Run the speed control loop */

        /* �ٶ�������Iq */
        hTargetSpeed = (int16_t)(wCurrentReference / 65536);
        hMeasuredSpeed = SPD_GetAvrgMecSpeed01Hz(pHandle->SPD);
        hError = hTargetSpeed - hMeasuredSpeed;
        hTorqueReference = PI_Controller(pHandle->PISpeed, (int32_t)hError);

        pHandle->SpeedRef01HzExt = wCurrentReference; /* ������Դ�Ϊ����ʼ�� */
        pHandle->TorqueRef = (int32_t)hTorqueReference * 65536;
    } else {
        /* λ��->����I��Ť��->�������ٶ�->ת��, */
        pHandle->TorqueRef = wCurrentReference; /* ������Դ�Ϊ����ʼ�� */
        hTorqueReference = (int16_t)(wCurrentReference / 65536);
    }

    return hTorqueReference;
}

/**
 * @brief ��ȡĿ���е�ٶ�,��λ0.1Hz
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval int16_t Ĭ�ϲο���е�ٶ�
 */
int16_t STC_GetMecSpeedRef01HzDefault(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MecSpeedRef01HzDefault;
}

/**
 * @brief ��ȡ�������е�ٶ�0.1Hz
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval uint16_t �������е�ٶ�0.1Hz
 */
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MaxAppPositiveMecSpeed01Hz;
}

/**
 * @brief ��ȡ��С����е�ٶ�0.1Hz
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval int16_t ��С����е�ٶ�0.1Hz
 */
int16_t STC_GetMinAppNegativeMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MinAppNegativeMecSpeed01Hz;
}

/**
 * @brief ���б�¹���
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval bool ���״̬
 */
bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle)
{
    bool retVal = false;
    if (pHandle->RampRemainingStep == 0u) {
        retVal = true;
    }
    return retVal;
}

/**
 * @brief ֹͣ�ٶ�б��
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval bool ֹͣб��״̬
 */
bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle)
{
    bool retVal = false;
    if (pHandle->Mode == STC_SPEED_MODE) {
        pHandle->RampRemainingStep = 0u;
        retVal = true;
    }
    return retVal;
}

/**
 * @brief ��ȡĬ��Iqd�ο�ֵ
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval Curr_Components Ĭ��Iqd�ο�ֵ
 */
Curr_Components STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle)
{
    Curr_Components IqdRefDefault;
    IqdRefDefault.qI_Component1 = pHandle->TorqueRefDefault; /* 316 */
    IqdRefDefault.qI_Component2 = pHandle->IdrefDefault;
    return IqdRefDefault;
}

/**
 * @brief ���ö����
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @param hNominalCurrent �����
 * @retval None
 */
void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent)
{
    pHandle->MaxPositiveTorque = hNominalCurrent;
    pHandle->MinNegativeTorque = -hNominalCurrent;
}

/**
 * @brief �ٶ�ת��0.1Hz
 * @param *pHandle �ٶ�ת�ؿ��ƶ���
 * @retval None
 */
void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle)
{
    pHandle->SpeedRef01HzExt = (int32_t)SPD_GetAvrgMecSpeed01Hz(pHandle->SPD) * (int32_t)65536;
}
