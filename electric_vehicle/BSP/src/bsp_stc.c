#include "bsp_stc.h"
#include "bsp_spd.h"
#include "bsp_pid.h"

/**
 * @brief 速度转矩初始化
 * @param *pHandle 速度转矩对象
 * @param *pPI PID控制器
 * @param *SPD_Handle 速度环对象
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
 * @brief 速度对象实例化
 * @param *pHandle 速度转矩对象
 * @param *SPD_Handle 速度环对象
 * @retval None
 */
void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *SPD_Handle)
{
    pHandle->SPD = SPD_Handle;
}

/**
 * @brief 获取速度环对象
 * @param *pHandle 速度转矩对象
 * @retval SpeednPosFdbk_Handle_t * 速度环对象
 */
SpeednPosFdbk_Handle_t *STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle)
{
    return (pHandle->SPD);
}

/**
 * @brief 复位积分项
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
 * @brief 获取机械速度
 * @param *pHandle 速度转矩控制对象
 * @retval int16_t 机械速度
 */
int16_t STC_GetMecSpeedRef01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return ((int16_t)(pHandle->SpeedRef01HzExt / 65536));
}

/**
 * @brief 获取参考转矩
 * @param *pHandle 速度转矩控制对象
 * @retval int16_t 参考转矩
 */
int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle)
{
    return ((int16_t)(pHandle->TorqueRef / 65536));
}

/**
 * @brief 设置电机控制模式
 * @param *pHandle 速度转矩控制对象
 * @param bMode 转矩/速度
 * @retval None
 */
void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, STC_Modality_t bMode)
{
    pHandle->Mode = bMode;
    pHandle->RampRemainingStep = 0u; /* Interrupts previous ramp. */
}

/**
 * @brief 获取控制模式
 * @param *pHandle 速度转矩控制对象
 * @retval STC_Modality_t 控制模式
 */
STC_Modality_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->Mode;
}

#define CHECK_BOUNDARY
/**
 * @brief 执行速度/转矩斜坡
 * @param *pHandle 速度转矩控制对象
 * @param hTargetFinal 目标值
 * @param hDurationms 持续时间
 * @retval bool 正常范围
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
        /* 斜坡从起始点0开始 */
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
 * @brief 停止速度/转矩斜坡
 * @param *pHandle 速度转矩控制对象
 * @retval None
 */
void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle)
{
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
}

/**
 * @brief 计算参考转矩分量(需要斜坡处理)
 * @param *pHandle 速度转矩控制对象
 * @retval int16_t 参考转矩分量
 */
int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle)
{
    /* 位置->实时更新目标速度->误差->速度参考值 */
    /* 位置->调整I的扭矩->调整加速度->转速 */
    int32_t wCurrentReference;
    int16_t hTorqueReference = 0;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (pHandle->Mode == STC_TORQUE_MODE) {
        wCurrentReference = pHandle->TorqueRef; /* 起始转矩316*65536,转矩曲线 */
    } else {
        wCurrentReference = pHandle->SpeedRef01HzExt; /* 起始速度166*65536 */
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

    /* 位置->实时更新目标速度->误差->速度参考值 */
    if (pHandle->Mode == STC_SPEED_MODE) {
        /* Run the speed control loop */

        /* 速度误差计算Iq */
        hTargetSpeed = (int16_t)(wCurrentReference / 65536);
        hMeasuredSpeed = SPD_GetAvrgMecSpeed01Hz(pHandle->SPD);
        hError = hTargetSpeed - hMeasuredSpeed;
        hTorqueReference = PI_Controller(pHandle->PISpeed, (int32_t)hError);

        pHandle->SpeedRef01HzExt = wCurrentReference; /* 对齐后以此为新起始点 */
        pHandle->TorqueRef = (int32_t)hTorqueReference * 65536;
    } else {
        /* 位置->调整I的扭矩->调整加速度->转速, */
        pHandle->TorqueRef = wCurrentReference; /* 对齐后以此为新起始点 */
        hTorqueReference = (int16_t)(wCurrentReference / 65536);
    }

    return hTorqueReference;
}

/**
 * @brief 获取目标机械速度,单位0.1Hz
 * @param *pHandle 速度转矩控制对象
 * @retval int16_t 默认参考机械速度
 */
int16_t STC_GetMecSpeedRef01HzDefault(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MecSpeedRef01HzDefault;
}

/**
 * @brief 获取最大正机械速度0.1Hz
 * @param *pHandle 速度转矩控制对象
 * @retval uint16_t 最大正机械速度0.1Hz
 */
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MaxAppPositiveMecSpeed01Hz;
}

/**
 * @brief 获取最小负机械速度0.1Hz
 * @param *pHandle 速度转矩控制对象
 * @retval int16_t 最小负机械速度0.1Hz
 */
int16_t STC_GetMinAppNegativeMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle)
{
    return pHandle->MinAppNegativeMecSpeed01Hz;
}

/**
 * @brief 完成斜坡过程
 * @param *pHandle 速度转矩控制对象
 * @retval bool 完成状态
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
 * @brief 停止速度斜坡
 * @param *pHandle 速度转矩控制对象
 * @retval bool 停止斜坡状态
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
 * @brief 获取默认Iqd参考值
 * @param *pHandle 速度转矩控制对象
 * @retval Curr_Components 默认Iqd参考值
 */
Curr_Components STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle)
{
    Curr_Components IqdRefDefault;
    IqdRefDefault.qI_Component1 = pHandle->TorqueRefDefault; /* 316 */
    IqdRefDefault.qI_Component2 = pHandle->IdrefDefault;
    return IqdRefDefault;
}

/**
 * @brief 设置额定电流
 * @param *pHandle 速度转矩控制对象
 * @param hNominalCurrent 额定电流
 * @retval None
 */
void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent)
{
    pHandle->MaxPositiveTorque = hNominalCurrent;
    pHandle->MinNegativeTorque = -hNominalCurrent;
}

/**
 * @brief 速度转化0.1Hz
 * @param *pHandle 速度转矩控制对象
 * @retval None
 */
void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle)
{
    pHandle->SpeedRef01HzExt = (int32_t)SPD_GetAvrgMecSpeed01Hz(pHandle->SPD) * (int32_t)65536;
}
