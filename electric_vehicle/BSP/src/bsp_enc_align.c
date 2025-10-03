#include "bsp_enc_align.h"
#include "mc_type.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_vss.h"
#include "bsp_stc.h"

/**
 * @brief 编码器对齐对象
 * @param *pHandle 对齐对象
 * @param *pSTC 速度转矩对象
 * @param *pVSS 虚拟速度传感器对象
 * @param *pENC 编码器对象
 * @retval None
 */
void EAC_Init(EncAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, ENCODER_Handle_t *pENC)
{
    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->pENC = pENC;
    pHandle->EncAligned = false;
    pHandle->EncRestart = true; // true才会配置进行校准
}

/**
 * @brief 启动校准对齐IF
 * @param *pHandle 对齐对象
 * @retval None
 */
void EAC_StartAlignment(EncAlign_Handle_t *pHandle)
{
    /* 启动校准对齐IF */
    uint32_t wAux;

    /* Set pVSS mechanical speed to zero.*/
    VSS_SetMecAcceleration(pHandle->pVSS, 0, 0u); // dpp

    /* Set pVSS mechanical angle. 对齐角度 */
    VSS_SetMecAngle(pHandle->pVSS, pHandle->hElAngle);

    /* Set pSTC in STC_TORQUE_MODE.*/
    STC_SetControlMode(pHandle->pSTC, STC_TORQUE_MODE);

    /* pHandle->TorqueRef=316 -> 0 */
    STC_ExecRamp(pHandle->pSTC, 0, 0u);

    /* RampRemainingStep=f_speed*t、IncDecAmount */
    /* pHandle->hFinalTorque=317  pHandle->TorqueRef=0 700ms */
    STC_ExecRamp(pHandle->pSTC, pHandle->hFinalTorque, (uint32_t)(pHandle->hDurationms));

    /* hRemainingTicks=f_speed*t*/
    wAux = (uint32_t)pHandle->hDurationms * (uint32_t)pHandle->hEACFrequencyHz; /* 500hz */
    wAux /= 1000u;
    pHandle->hRemainingTicks = (uint16_t)(wAux);
    pHandle->hRemainingTicks++;
}

/**
 * @brief 执行对齐
 * @param *pHandle
 * @retval None
 */
bool EAC_Exec(EncAlign_Handle_t *pHandle)
{
    bool retVal = true;

    /* 转矩/速度执行次数 */
    if (pHandle->hRemainingTicks > 0u) {
        pHandle->hRemainingTicks--;

        if (pHandle->hRemainingTicks == 0u) {
            /* Set pVSS mechanical angle.*/
            /* 设置电角度进入ENC,90°电角度对应的机械角度90°/pole */
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
 * @brief 是否已对齐
 * @param *pHandle 对齐对象
 * @retval None
 */
bool EAC_IsAligned(EncAlign_Handle_t *pHandle)
{
    return pHandle->EncAligned;
}

/**
 * @brief  编码器重启对齐状态
 * @param *pHandle 对齐对象
 * @param restart 重启
 * @retval None
 */
void EAC_SetRestartState(EncAlign_Handle_t *pHandle, bool restart)
{
    /* 对齐校准可只执行一次 */
    pHandle->EncRestart = restart;
}

/**
 * @brief  编码器重启对齐状态
 * @param *pHandle 对齐对象
 * @retval None
 */
bool EAC_GetRestartState(EncAlign_Handle_t *pHandle)
{
    return pHandle->EncRestart;
}
