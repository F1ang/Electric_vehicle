#include "bsp_enc_align.h"
#include "mc_type.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_vss.h"
#include "bsp_stc.h"

/**
 * @brief �������������
 * @param *pHandle �������
 * @param *pSTC �ٶ�ת�ض���
 * @param *pVSS �����ٶȴ���������
 * @param *pENC ����������
 * @retval None
 */
void EAC_Init(EncAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, ENCODER_Handle_t *pENC)
{
    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->pENC = pENC;
    pHandle->EncAligned = false;
    pHandle->EncRestart = true; // true�Ż����ý���У׼
}

/**
 * @brief ����У׼����IF
 * @param *pHandle �������
 * @retval None
 */
void EAC_StartAlignment(EncAlign_Handle_t *pHandle)
{
    /* ����У׼����IF */
    uint32_t wAux;

    /* Set pVSS mechanical speed to zero.*/
    VSS_SetMecAcceleration(pHandle->pVSS, 0, 0u); // dpp

    /* Set pVSS mechanical angle. ����Ƕ� */
    VSS_SetMecAngle(pHandle->pVSS, pHandle->hElAngle);

    /* Set pSTC in STC_TORQUE_MODE.*/
    STC_SetControlMode(pHandle->pSTC, STC_TORQUE_MODE);

    /* pHandle->TorqueRef=316 -> 0 */
    STC_ExecRamp(pHandle->pSTC, 0, 0u);

    /* RampRemainingStep=f_speed*t��IncDecAmount */
    /* pHandle->hFinalTorque=317  pHandle->TorqueRef=0 700ms */
    STC_ExecRamp(pHandle->pSTC, pHandle->hFinalTorque, (uint32_t)(pHandle->hDurationms));

    /* hRemainingTicks=f_speed*t*/
    wAux = (uint32_t)pHandle->hDurationms * (uint32_t)pHandle->hEACFrequencyHz; /* 500hz */
    wAux /= 1000u;
    pHandle->hRemainingTicks = (uint16_t)(wAux);
    pHandle->hRemainingTicks++;
}

/**
 * @brief ִ�ж���
 * @param *pHandle
 * @retval None
 */
bool EAC_Exec(EncAlign_Handle_t *pHandle)
{
    bool retVal = true;

    /* ת��/�ٶ�ִ�д��� */
    if (pHandle->hRemainingTicks > 0u) {
        pHandle->hRemainingTicks--;

        if (pHandle->hRemainingTicks == 0u) {
            /* Set pVSS mechanical angle.*/
            /* ���õ�ǶȽ���ENC,90���Ƕȶ�Ӧ�Ļ�е�Ƕ�90��/pole */
            ENC_SetMecAngle(pHandle->pENC, pHandle->hElAngle / (int16_t)(pHandle->bElToMecRatio));
            pHandle->EncAligned = true;
            retVal = true;
        } else {
            retVal = false;
        }
    }

    return retVal;
}

/**
 * @brief �Ƿ��Ѷ���
 * @param *pHandle �������
 * @retval None
 */
bool EAC_IsAligned(EncAlign_Handle_t *pHandle)
{
    return pHandle->EncAligned;
}

/**
 * @brief  ��������������״̬
 * @param *pHandle �������
 * @param restart ����
 * @retval None
 */
void EAC_SetRestartState(EncAlign_Handle_t *pHandle, bool restart)
{
    /* ����У׼��ִֻ��һ�� */
    pHandle->EncRestart = restart;
}

/**
 * @brief  ��������������״̬
 * @param *pHandle �������
 * @retval None
 */
bool EAC_GetRestartState(EncAlign_Handle_t *pHandle)
{
    return pHandle->EncRestart;
}
