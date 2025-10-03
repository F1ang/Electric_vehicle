#include "bsp_vss.h"
#include "bsp_spd.h"

/**
 * @brief 虚拟速度传感器复位
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
 * @brief 计算虚拟电角度,输入电角度与累积电角度的差值,并量化,优化斜坡的电角度
 *        1.I/F阶段:dpp积分出电角度 2.斜坡启动阶段:SMO观测电角度-dpp积分电角度进行电角度修正,得估计电角度
 * @param *pHandle 虚拟速度传感器对象
 * @param *pInputVars_str 输入变量
 * @retval 更新虚拟电角度
 */
int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, void *pInputVars_str)
{
    int16_t hRetAngle;
    int16_t hAngleDiff;
    int16_t hAngleCorr;
    int32_t wAux;
    int16_t hSignCorr = 1;

    if (pHandle->bCopyObserver == true) {
        /* 直接使用传入的电角度当虚拟电角度 */
        hRetAngle = *(int16_t *)pInputVars_str;
    } else {
        /* 电角度变化量的累加,积分出电角度 */
        pHandle->hElAngleAccu += pHandle->_Super.hElSpeedDpp;

        /* 累加出机械角度 */
        pHandle->_Super.hMecAngle += pHandle->_Super.hElSpeedDpp /
                                     (int16_t)pHandle->_Super.bElToMecRatio;

        /* 状态转换(斜坡)开始 */
        if (pHandle->bTransitionStarted == true) {
            if (pHandle->hTransitionRemainingSteps == 0) {
                /* 状态转换完成,虚拟电角度直接赋值 */
                hRetAngle = *(int16_t *)pInputVars_str;
                pHandle->bTransitionEnded = true;
                pHandle->_Super.bSpeedErrorNumber = 0u;
            } else {
                /* 状态转换中 */
                pHandle->hTransitionRemainingSteps--;

                /* 计算输入电角度与累积电角度的差值 */
                if (pHandle->_Super.hElSpeedDpp >= 0) {
                    hAngleDiff = *(int16_t *)pInputVars_str - pHandle->hElAngleAccu;
                } else {
                    hAngleDiff = pHandle->hElAngleAccu - *(int16_t *)pInputVars_str;
                    hSignCorr = -1;
                }

                /* 计算角度量化值=cnt/cnt_sum * angle_diff(列直角三角形的比例关系),角度差值量化到状态转换步数 */
                wAux = (int32_t)hAngleDiff * pHandle->hTransitionRemainingSteps;
                hAngleCorr = (int16_t)(wAux / pHandle->hTransitionSteps);
                hAngleCorr *= hSignCorr;

                if (hAngleDiff >= 0) {
                    /* 角度差为正,锁定状态,输入角度-量化角度(按状态步数) */
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
            /* 结束状态转换,直接赋值 */
            hRetAngle = pHandle->hElAngleAccu;
        }
    }

    /* 更新虚拟电角度 */
    pHandle->_Super.hElAngle = hRetAngle;

    return hRetAngle;
}

/**
 * @brief 虚拟计算机械速度(加速度计算速度)
 *      1.bTransitionEnded=false,Vss_CalcAvrqMecSpeed01Hz直接返回速度不可靠,StartUpTransitionEnded=false => 不会切换闭环观测器运行
 *      2.bTransitionEnded=true,检验速度SPD_IsMecSpeedReliable => StartUpTransitionEnded=1 => 切换闭环观测器运行
 * @param *pHandle 虚拟速度传感器对象
 * @param *hMecSpeed01Hz 机械速度0.1Hz
 * @retval 速度是否可靠 1可靠 0不可靠
 */
bool VSS_CalcAvrgMecSpeed01Hz(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeed01Hz)
{
    bool SpeedSensorReliability = false;

    if (pHandle->hRemainingStep > 1u) {
        /* 1.加速度计算电速度/机械速度 */
        pHandle->wElSpeedDpp32 += pHandle->wElAccDppP32;
        pHandle->_Super.hElSpeedDpp = (int16_t)(pHandle->wElSpeedDpp32 / 65536);

        /* 2.Convert dpp into Mec01Hz */
        *hMecSpeed01Hz = (int16_t)((pHandle->_Super.hElSpeedDpp *
                                    (int32_t)pHandle->_Super.hMeasurementFrequency * 10) /
                                   (65536 * (int32_t)pHandle->_Super.bElToMecRatio));

        pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

        pHandle->hRemainingStep--;
    } else if (pHandle->hRemainingStep == 1u) {
        *hMecSpeed01Hz = pHandle->hFinalMecSpeed01Hz; /* 切换下一阶段 */
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
        /* 3.bTransitionEnded=false,Vss_CalcAvrqMecSpeed01Hz直接返回速度不可靠,StartUpTransitionEnded=false => 不会切换闭环观测器运行 */
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        SpeedSensorReliability = false;
    } else {
        /* 4.hTransitionRemainingsteps=0=>bTransitionEnded=true,检验速度SPD_IsMecSpeedReliable => StartUpTransitionEnded=1 => 切换闭环观测器运行 */
        SpeedSensorReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeed01Hz);
    }

    return (SpeedSensorReliability);
}

/**
 * @brief 对齐的电角度
 * @param *pHandle 虚拟速度传感器对象
 * @param hMecAngle 机械角度
 * @retval None
 */
void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle)
{
    pHandle->hElAngleAccu = hMecAngle;
    pHandle->_Super.hMecAngle = pHandle->hElAngleAccu / (int16_t)pHandle->_Super.bElToMecRatio;
    pHandle->_Super.hElAngle = hMecAngle;
}

/**
 * @brief 设置机械加速度曲线
 * @param *pHandle 虚拟速度传感器对象
 * @param hFinalMecSpeed01Hz 目标机械速度0.1Hz
 * @param hDurationms 持续时间ms
 * @retval None
 */
void VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t hFinalMecSpeed01Hz, uint16_t hDurationms)
{
    uint16_t hNbrStep;
    int16_t hCurrentMecSpeedDpp;
    int32_t wMecAccDppP32;
    int16_t hFinalMecSpeedDpp;

    if (pHandle->bTransitionStarted == false) {
        /* 初始化 */
        if (hDurationms == 0u) {
            pHandle->_Super.hAvrMecSpeed01Hz = hFinalMecSpeed01Hz;

            /* dpp(角度变化量)=W_e * T */
            pHandle->_Super.hElSpeedDpp = (int16_t)(((int32_t)(hFinalMecSpeed01Hz) *
                                                     (int32_t)65536) /
                                                    ((int32_t)10 * (int32_t)pHandle->_Super.hMeasurementFrequency));

            /* 引入极对数 */
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

            /* 当前机械速度dpp */
            hCurrentMecSpeedDpp = pHandle->_Super.hElSpeedDpp /
                                  (int16_t)pHandle->_Super.bElToMecRatio;

            /* 目标机械速度dpp */
            hFinalMecSpeedDpp = (int16_t)(((int32_t)hFinalMecSpeed01Hz * (int32_t)65536) /
                                          ((int32_t)10 * (int32_t)pHandle->_Super.hMeasurementFrequency));

            /* 机械dpp步进值 */
            wMecAccDppP32 = (((int32_t)hFinalMecSpeedDpp - (int32_t)hCurrentMecSpeedDpp) *
                             (int32_t)65536) /
                            (int32_t)hNbrStep;

            /* 电dpp步进值 */
            pHandle->wElAccDppP32 = wMecAccDppP32 * (int16_t)pHandle->_Super.bElToMecRatio;

            pHandle->hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
            pHandle->wElSpeedDpp32 = (int32_t)pHandle->_Super.hElSpeedDpp * (int32_t)65536;
        }
    }
}

/**
 * @brief 获取斜坡加速完成标志
 * @param *pHandle 虚拟速度传感器对象
 * @retval 斜坡完成
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
 * @brief 获取斜坡的最终速度
 * @param *pHandle 虚拟速度传感器对象
 * @retval 斜坡的最终速度
 */
int16_t VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle)
{
    return pHandle->hFinalMecSpeed01Hz;
}

/**
 * @brief 置位开启执行斜坡标志
 * @param *pHandle 虚拟速度传感器对象
 * @param bCommand 执行/关闭
 * @retval 0-仅执行完毕 1-执行
 */
bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand)
{
    bool bAux = true;
    if (bCommand == true) {
        pHandle->bTransitionStarted = true; /* 开启斜坡 */

        /*  hTransitionSteps!=0 => StartUpTransitionEnded!=1 => 不会切闭环观测器运行 */
        if (pHandle->hTransitionSteps == 0) {
            pHandle->bTransitionEnded = true;
            pHandle->_Super.bSpeedErrorNumber = 0u;
            bAux = false;
        }
    }

    return bAux;
}

/**
 * @brief 是否正在执行斜坡
 * @param *pHandle 虚拟速度传感器对象
 * @retval 1-斜坡开始
 */
bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle)
{
    uint16_t hTS = 0u, hTE = 0u, hAux;
    bool retVal = false;
    if (pHandle->bTransitionStarted == true) { /* 斜坡开始 */
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
 * @brief 设置观测器
 * @param *pHandle
 * @retval None
 */
void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle)
{
    pHandle->bCopyObserver = true;
}

/**
 * @brief 设置电角度
 * @param *pHandle 虚拟速度传感器对象
 * @param hElAngle 电角度
 * @retval None
 */
void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle)
{
    pHandle->hElAngleAccu = hElAngle;
    pHandle->_Super.hElAngle = hElAngle;
}
