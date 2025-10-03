#include "bsp_ramp.h"
#include "bsp_pwmc.h"

uint32_t getScalingFactor(int32_t Target);

/* б�´���ṹ�� */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 = {
    .FrequencyHz = TF_REGULATION_RATE /* 16kHz */
};

/**
 * @brief  ���б�º�����״̬����
 * @param  pHandle б�¹�����
 * @retval None
 */
void REMNG_Init(RampExtMngr_Handle_t *pHandle)
{
    pHandle->Ext = 0;
    pHandle->TargetFinal = 0;
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
    pHandle->ScalingFactor = 1u;
}

/**
 * @brief б�¼���,������������
 * @note ƽ����Iq,��ֹת������
 * @param *pHandle б�¹�����
 * @retval int32_t б�¼�����
 */
int32_t REMNG_Calc(RampExtMngr_Handle_t *pHandle)
{
    int32_t ret_val;
    int32_t current_ref;

    current_ref = pHandle->Ext;

    /* Update the variable and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1u) {
        /* Increment/decrement the reference value. */
        current_ref += pHandle->IncDecAmount;

        /* Decrement the number of remaining steps */
        pHandle->RampRemainingStep--;
    } else if (pHandle->RampRemainingStep == 1u) {
        /* Set the backup value of TargetFinal. */
        current_ref = pHandle->TargetFinal * (int32_t)(pHandle->ScalingFactor);
        pHandle->RampRemainingStep = 0u;
    } else {
        /* Do nothing. */
    }

    pHandle->Ext = current_ref;
    ret_val = pHandle->Ext / (int32_t)(pHandle->ScalingFactor);

    return ret_val;
}

/**
 * @brief  Setup the ramp to be executed
 * @param  pHandle related Handle of struct RampMngr_Handle_t
 * @param  hTargetFinal (signed 32bit) final value of state variable at the end
 *         of the ramp.
 * @param  hDurationms (unsigned 32bit) the duration of the ramp expressed in
 *         milliseconds. It is possible to set 0 to perform an instantaneous
 *         change in the value.
 * @retval bool It returns true is command is valid, false otherwise
 */
/**
 * @brief ִ��б�²�������
 * @param *pHandle б�¹�����
 * @param TargetFinal Ŀ��ֵ
 * @param Durationms ����ʱ��
 * @retval 1
 */
bool REMNG_ExecRamp(RampExtMngr_Handle_t *pHandle, int32_t TargetFinal, uint32_t Durationms)
{
    uint32_t aux;
    int32_t aux1;
    int32_t current_ref;
    bool retVal = true;

    /* ��ǰ״̬ */
    current_ref = pHandle->Ext / (int32_t)(pHandle->ScalingFactor);

    if (Durationms == 0u) {
        /* ʱ��Ϊ0,ֱ������ */
        pHandle->ScalingFactor = getScalingFactor(TargetFinal);
        pHandle->Ext = TargetFinal * (int32_t)(pHandle->ScalingFactor);
        pHandle->RampRemainingStep = 0u;
        pHandle->IncDecAmount = 0;
    } else {
        /* ��ȡĿ��͵�ǰֵ���������� */
        uint32_t wScalingFactor = getScalingFactor(TargetFinal - current_ref);
        uint32_t wScalingFactor2 = getScalingFactor(current_ref);
        uint32_t wScalingFactor3 = getScalingFactor(TargetFinal);
        uint32_t wScalingFactorMin;

        /* ��Iq��ǰֵ��Ŀ��ֵ�Ĳ�������������,ѡ����С���������� */
        if (wScalingFactor < wScalingFactor2) {
            if (wScalingFactor < wScalingFactor3) {
                wScalingFactorMin = wScalingFactor;
            } else {
                wScalingFactorMin = wScalingFactor3;
            }
        } else {
            if (wScalingFactor2 < wScalingFactor3) {
                wScalingFactorMin = wScalingFactor2;
            } else {
                wScalingFactorMin = wScalingFactor3;
            }
        }

        /* ��С��������->��ǰ�źŵķŴ�ֵ */
        pHandle->ScalingFactor = wScalingFactorMin;
        pHandle->Ext = current_ref * (int32_t)(pHandle->ScalingFactor);

        /* �洢Ŀ��ֵ */
        pHandle->TargetFinal = TargetFinal;

        /* б�²���-���� */
        aux = Durationms * (uint32_t)pHandle->FrequencyHz; /* Check for overflow and use prescaler */
        aux /= 1000u;
        pHandle->RampRemainingStep = aux;
        pHandle->RampRemainingStep++;

        /* б�²���-б�� */
        aux1 = (TargetFinal - current_ref) * (int32_t)(pHandle->ScalingFactor);
        aux1 /= (int32_t)(pHandle->RampRemainingStep);
        pHandle->IncDecAmount = aux1;
    }

    return retVal;
}

/**
 * @brief  Returns the current value of the state variable.
 * @param  pHandle related Handle of struct RampMngr_Handle_t
 * @retval int32_t value of the state variable
 */
/**
 * @brief ��ȡ��ǰֵ(������������)
 * @param *pHandle б�¹�����
 * @retval int32_t ��ǰֵ
 */
int32_t REMNG_GetValue(RampExtMngr_Handle_t *pHandle)
{
    int32_t ret_val;
    ret_val = pHandle->Ext / (int32_t)(pHandle->ScalingFactor);

    return ret_val;
}

/**
 * @brief ���б���Ƿ����
 * @param *pHandle б�¹�����
 * @retval 1-���,0-δ���
 */
bool REMNG_RampCompleted(RampExtMngr_Handle_t *pHandle)
{
    bool retVal = false;
    if (pHandle->RampRemainingStep == 0u) {
        retVal = true;
    }

    return retVal;
}

/**
 * @brief ֹͣб��
 * @param *pHandle б�¹�����
 * @retval None
 */
void REMNG_StopRamp(RampExtMngr_Handle_t *pHandle)
{
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
}

/**
 * @brief ������������ 2^int(31-log2(Target))
 * @param Target Ŀ��ֵ
 * @retval uint32_t ��������
 */
uint32_t getScalingFactor(int32_t Target)
{
    uint8_t i;
    uint32_t TargetAbs;
    int32_t aux;

    if (Target < 0) {
        aux = -Target;
        TargetAbs = (uint32_t)(aux);
    } else {
        TargetAbs = (uint32_t)(Target);
    }
    for (i = 1u; i < 32u; i++) {
        uint32_t limit = ((uint32_t)(1) << (31u - i));
        if (TargetAbs >= limit) {
            break;
        }
    }

    return ((uint32_t)(1u) << (i - 1u));
}
