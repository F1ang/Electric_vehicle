#include "bsp_revup.h"
#include "bsp_sto_pll.h"
#include "bsp_vss.h"
#include "bsp_stc.h"
#include "bsp_spd.h"
#include "bsp_pwmc.h"

/* 电机启动对象 */
RevUpCtrl_Handle_t RevUpControlM1 = {
    .hRUCFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE,                               /* 执行频率=速度环频率 */
    .hStartingMecAngle = (int16_t)((int32_t)(STARTING_ANGLE_DEG) * 65536 / 360), /* 启动的起始机械角度 */
    .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,                           /* 开始加速阶段 */
    .hMinStartUpValidSpeed = OBS_MINIMUM_SPEED,                                  /* 启动所需的最低转速 */
    .hMinStartUpFlySpeed = (int16_t)(OBS_MINIMUM_SPEED / 2),                     /* 运行观测器收敛判断,所需的最低转速01Hz */
    .OTFStartupEnabled = false,                                                  /* 观测器启动使能标志 */

    /* 实时启动阶段参数 */
    .OTFPhaseParams = {
        (uint16_t)500,                 /* 持续时间 */
        0,                             /* 目标速度 */
        (int16_t)PHASE5_FINAL_CURRENT, /* 目标转矩 */
        (void *)0,                     /* 链表指针 */
    },

    /* 启动阶段参数序列数组(5状态转换) */
    .ParamsData = {
        { (uint16_t)PHASE1_DURATION, (int16_t)(PHASE1_FINAL_SPEED_RPM / 6), (int16_t)PHASE1_FINAL_CURRENT, &RevUpControlM1.ParamsData[1] },
        { (uint16_t)PHASE2_DURATION, (int16_t)(PHASE2_FINAL_SPEED_RPM / 6), (int16_t)PHASE2_FINAL_CURRENT, &RevUpControlM1.ParamsData[2] },
        { (uint16_t)PHASE3_DURATION, (int16_t)(PHASE3_FINAL_SPEED_RPM / 6), (int16_t)PHASE3_FINAL_CURRENT, &RevUpControlM1.ParamsData[3] },
        { (uint16_t)PHASE4_DURATION, (int16_t)(PHASE4_FINAL_SPEED_RPM / 6), (int16_t)PHASE4_FINAL_CURRENT, &RevUpControlM1.ParamsData[4] },
        { (uint16_t)PHASE5_DURATION, (int16_t)(PHASE5_FINAL_SPEED_RPM / 6), (int16_t)PHASE5_FINAL_CURRENT, (void *)0 },
    },
};

/**
 * @brief 初始化启动阶段参数
 * @retval None
 */
void RUC_Init(RevUpCtrl_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS,
              STO_Handle_t *pSNSL, PWMC_Handle_t *pPWM)
{
    RevUpCtrl_PhaseParams_t *pRUCPhaseParams = &pHandle->ParamsData[0];
    uint8_t bPhase = 0u;

    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->pSNSL = pSNSL;
    pHandle->pPWM = pPWM;
    pHandle->OTFSCLowside = false;
    pHandle->EnteredZone1 = false;

    /* 启动阶段序列号链表初始化 */
    while ((pRUCPhaseParams != 0) && (bPhase < RUC_MAX_PHASE_NUMBER)) {
        pRUCPhaseParams = pRUCPhaseParams->pNext;
        bPhase++;
    }
    pHandle->ParamsData[bPhase - 1u].pNext = 0;

    pHandle->bPhaseNbr = bPhase;

    /* 1/f*cnt=100ms,清PLL */
    pHandle->bResetPLLTh = (uint8_t)((RUC_OTF_PLL_RESET_TIMEOUT * pHandle->hRUCFrequencyHz) / 1000u);
}

/**
 * @brief  Initialize internal RevUp controller state.
 * @param  pHandle: Pointer on Handle structure of RevUp controller.
 * @param  hMotorDirection: rotor rotation direction.
 *         This parameter must be -1 or +1.
 *  @retval none
 */
/**
 * @brief 初始化启动控制器状态
 * @param *pHandle 电机启动对象
 * @param hMotorDirection 电机方向
 * @retval None
 */
void RUC_Clear(RevUpCtrl_Handle_t *pHandle, int16_t hMotorDirection)
{
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    SpeednTorqCtrl_Handle_t *pSTC = pHandle->pSTC;
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->ParamsData;

    pHandle->hDirection = hMotorDirection;
    pHandle->EnteredZone1 = false;

    pHandle->bStageCnt = 0u;
    pHandle->bOTFRelCounter = 0u;
    pHandle->OTFSCLowside = false;

    /* 虚拟速度传感器对象 */
    VSS_Init(pVSS);

    /* 设置电机控制模式 */
    STC_SetControlMode(pSTC, STC_TORQUE_MODE);

    /* 对齐的电角度 */
    VSS_SetMecAngle(pVSS, pHandle->hStartingMecAngle * hMotorDirection);

    /* 清速度/转矩斜坡参数 */
    STC_ExecRamp(pSTC, 0, 0u);

    /* 执行速度/转矩斜坡 */
    STC_ExecRamp(pSTC, pPhaseParams->hFinalTorque * hMotorDirection,
                 (uint32_t)(pPhaseParams->hDurationms));

    /* 设置机械加速度曲线:速度步进 */
    VSS_SetMecAcceleration(pVSS, pPhaseParams->hFinalMecSpeed01Hz * hMotorDirection,
                           pPhaseParams->hDurationms);

    /* 完成当前阶段的ticks 1/f*cnt=t,ms */
    pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                (uint32_t)pHandle->hRUCFrequencyHz) /
                                               1000u);
    pHandle->hPhaseRemainingTicks++;

    /* 跳转到启动的下一阶段 */
    pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

    /* 在线启动超时,清PLL计数 */
    pHandle->bResetPLLCnt = 0u;
}

/**
 * @brief 启动电机、速度相关性、外同步加速及转矩动态调整、启动阶段的配置
 * @param *pHandle 电机启动对象
 * @retval Bool 0-启动完成
 */
bool RUC_OTF_Exec(RevUpCtrl_Handle_t *pHandle)
{
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->pCurrentPhaseParams;
    bool IsSpeedReliable;
    bool retVal = true;
    bool condition = false;

    if (pHandle->hPhaseRemainingTicks > 0u) {
        pHandle->hPhaseRemainingTicks--;

        /* 1.电机启动 */
        if (pHandle->bStageCnt == 0u) {
            /* 未进入启动区域1,计数清除PLL参数 */
            if (pHandle->EnteredZone1 == false) {
                if (pHandle->pSNSL->pFctStoOtfResetPLL != 0) {
                    pHandle->bResetPLLCnt++;
                    if (pHandle->bResetPLLCnt > pHandle->bResetPLLTh) {
                        pHandle->pSNSL->pFctStoOtfResetPLL(pHandle->pSNSL);
                        pHandle->bOTFRelCounter = 0u;
                        pHandle->bResetPLLCnt = 0u;
                    }
                }

                /* 2.速度可靠性计数,进入下一步 */
                IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck(pHandle->pSNSL);
                if (IsSpeedReliable) {
                    if (pHandle->bOTFRelCounter < 127u) {
                        pHandle->bOTFRelCounter++;
                    }
                } else {
                    pHandle->bOTFRelCounter = 0u;
                }

                if (pHandle->pSNSL->pFctStoOtfResetPLL != 0) {
                    if (pHandle->bOTFRelCounter == (pHandle->bResetPLLTh >> 1)) {
                        condition = true;
                    }
                } else {
                    if (pHandle->bOTFRelCounter == 127) {
                        condition = true;
                    }
                }

                /* 3.速度相关性、外同步加速及转矩动态调整、启动阶段的配置 */
                if (condition == true) {
                    bool bCollinearSpeed = false;
                    int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz(pHandle->pSNSL->_Super);
                    int16_t hObsSpeed01HzAbsValue = (hObsSpeed01Hz < 0 ? (-hObsSpeed01Hz) : (hObsSpeed01Hz)); /* hObsSpeed01Hz absolute value */

                    /* 4.目标速度和实际速度是否线性相关 */
                    if (pHandle->hDirection > 0) {
                        if (hObsSpeed01Hz > 0) {
                            bCollinearSpeed = true;
                        }
                    } else {
                        if (hObsSpeed01Hz < 0) {
                            bCollinearSpeed = true;
                        }
                    }

                    if (bCollinearSpeed == false) { /* 非相关 */
                        pHandle->bOTFRelCounter = 0u;
                    } else { /* 相关 */
                        if ((uint16_t)(hObsSpeed01HzAbsValue) > pHandle->hMinStartUpValidSpeed) {
                            /* 5.观测速度>最小启动速度,进入启动区域1 */
                            pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
                            pHandle->EnteredZone1 = true;
                        } else if ((uint16_t)(hObsSpeed01HzAbsValue) > pHandle->hMinStartUpFlySpeed) {
                            /* 6.可执行外同步加速 */
                            int16_t hOldFinalMecSpeed01Hz = 0;
                            int16_t hOldFinalTorque = 0;
                            int32_t wDeltaSpeedRevUp;
                            int32_t wDeltaTorqueRevUp;
                            bool bError = false;
                            VSS_SetCopyObserver(pHandle->pVSS);
                            pHandle->pSNSL->pFctForceConvergency2(pHandle->pSNSL); /* 强制收敛标志2标志 */

                            if (pPhaseParams == 0) { /* 无启动阶段,报错 */
                                bError = true;
                                pHandle->hPhaseRemainingTicks = 0u;
                            } else {
                                /* 7.目标速度>当前阶段观测速度,跳转下一阶段(查询链表) */
                                while (pPhaseParams->hFinalMecSpeed01Hz < hObsSpeed01HzAbsValue) {
                                    if (pPhaseParams->pNext == 0) {
                                        /* 无下一阶段,报错 */
                                        bError = true;
                                        pHandle->hPhaseRemainingTicks = 0u;
                                        break;
                                    } else {
                                        /* 链表跳转下一阶段,且配置参数 */
                                        hOldFinalMecSpeed01Hz = pPhaseParams->hFinalMecSpeed01Hz;
                                        hOldFinalTorque = pPhaseParams->hFinalTorque;
                                        pHandle->pCurrentPhaseParams = pPhaseParams->pNext;
                                        pPhaseParams = pHandle->pCurrentPhaseParams;
                                        pHandle->bStageCnt++;
                                    }
                                }
                            }

                            if (bError == false) {
                                /* 8.计算外同步加速的转矩Iq Te=1.5Pn*磁链*Iq */
                                int16_t hTorqueReference;

                                /* y(n+1)=y(n)+k*Δx,根据观测速度,动态调整转矩 */
                                wDeltaSpeedRevUp = (int32_t)(pPhaseParams->hFinalMecSpeed01Hz) - (int32_t)(hOldFinalMecSpeed01Hz);
                                wDeltaTorqueRevUp = (int32_t)(pPhaseParams->hFinalTorque) - (int32_t)(hOldFinalTorque);
                                hTorqueReference = (int16_t)((((int32_t)hObsSpeed01Hz) * wDeltaTorqueRevUp) / wDeltaSpeedRevUp) + hOldFinalTorque;

                                /* 9.执行外同步加速 */
                                STC_ExecRamp(pHandle->pSTC, hTorqueReference, 0u);

                                pHandle->hPhaseRemainingTicks = 1u;

                                pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;

                                pHandle->bStageCnt = 6u;
                            } /* no 0 error */
                        } /* speed > MinStartupFly */
                        else {
                        }
                    } /* speeds are collinear */
                } /* speed is reliable */
            } /*EnteredZone1 1 is false */
            else {
                pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
            }
        } /*stage 0*/
    } /* hPhaseRemainingTicks > 0 */

    if (pHandle->hPhaseRemainingTicks == 0u) {
        if (pHandle->pCurrentPhaseParams != 0) {
            if (pHandle->bStageCnt == 0u) {
                /* 11.完成启动 */
                PWMC_SwitchOffPWM(pHandle->pPWM);
                pHandle->OTFSCLowside = true;
                PWMC_TurnOnLowSides(pHandle->pPWM);
                pHandle->bOTFRelCounter = 0u;
            } else if ((pHandle->bStageCnt == 1u)) {
                PWMC_SwitchOnPWM(pHandle->pPWM);
                pHandle->OTFSCLowside = false;
            } else {
            }

            /* 10.配置第x次阶段启动 */
            STC_ExecRamp(pHandle->pSTC, pPhaseParams->hFinalTorque * pHandle->hDirection,
                         (uint32_t)(pPhaseParams->hDurationms));

            VSS_SetMecAcceleration(pHandle->pVSS,
                                   pPhaseParams->hFinalMecSpeed01Hz * pHandle->hDirection,
                                   pPhaseParams->hDurationms);

            /* 启动的ticks次数 */
            pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                        (uint32_t)pHandle->hRUCFrequencyHz) /
                                                       1000u);
            pHandle->hPhaseRemainingTicks++;

            /* 链表指针跳转,启动下一阶段 */
            pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

            /*Increases the rev up stages counter.*/
            pHandle->bStageCnt++;
        } else {
            if (pHandle->bStageCnt == pHandle->bPhaseNbr - 1) /* End of user programmed revup */
            {
                retVal = false;                  /* 启动成功 */
            } else if (pHandle->bStageCnt == 7u) /* End of first OTF runs */
            {
                pHandle->bStageCnt = 0u; /* Breaking state */
                pHandle->hPhaseRemainingTicks = 0u;
            } else {
            }
        }
    }

    return retVal;
}

/**
 * @brief 启动阶段配置,此处具体执行启动过程(I/F分5阶段)
 * @param *pHandle 电机启动对象
 * @retval Bool 1-继续执行启动流程
 */
bool RUC_Exec(RevUpCtrl_Handle_t *pHandle)
{
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->pCurrentPhaseParams;
    bool retVal = true;

    if (pHandle->hPhaseRemainingTicks > 0u) {
        pHandle->hPhaseRemainingTicks--;
    }

    if (pHandle->hPhaseRemainingTicks == 0u) {
        if (pHandle->pCurrentPhaseParams != 0) {

            STC_ExecRamp(pHandle->pSTC, pPhaseParams->hFinalTorque * pHandle->hDirection,
                         (uint32_t)(pPhaseParams->hDurationms));

            VSS_SetMecAcceleration(pHandle->pVSS,
                                   pPhaseParams->hFinalMecSpeed01Hz * pHandle->hDirection,
                                   pPhaseParams->hDurationms);

            /* Compute hPhaseRemainingTicks. 执行次数 */
            pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                        (uint32_t)pHandle->hRUCFrequencyHz) /
                                                       1000u);
            pHandle->hPhaseRemainingTicks++;

            /* 链表指向下一启动阶段 */
            pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

            /* 增加启动阶段次数*/
            pHandle->bStageCnt++;
        } else {
            /* 5个启动阶段完毕,观测器的速度仍未收敛,启动失败 */
            retVal = false;
        }
    }

    return retVal;
}

/**
 * @brief  Provide current state of revup controller procedure.
 * @param  pHandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to true when entire revup phases have been completed.
 */
/**
 * @brief  启动是否完成
 * @param *pHandle 电机启动对象
 * @retval Bool 1-启动完成
 */
bool RUC_Completed(RevUpCtrl_Handle_t *pHandle)
{
    bool retVal = false;
    if (pHandle->pCurrentPhaseParams == 0) {
        retVal = true;
    }

    return retVal;
}

/**
 * @brief 推出启动
 * @param *pHandle 电机启动对象
 * @retval None
 */
void RUC_Stop(RevUpCtrl_Handle_t *pHandle)
{
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    pHandle->pCurrentPhaseParams = 0;
    pHandle->hPhaseRemainingTicks = 0u;
    VSS_SetMecAcceleration(pVSS, SPD_GetAvrgMecSpeed01Hz(&pVSS->_Super), 0u);
}

/**
 * @brief 检查第一级加速阶段是否完成
 * @param *pHandle 电机启动对象
 * @retval Bool 1-第一级加速阶段完成
 */
bool RUC_FirstAccelerationStageReached(RevUpCtrl_Handle_t *pHandle)
{
    bool retVal = false;

    if (pHandle->bStageCnt >= pHandle->bFirstAccelerationStage) {
        retVal = true;
    }

    return retVal;
}

/**
 * @brief 修改每个阶段的持续时间
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @param hDurationms 持续时间
 * @retval None
 */
void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms)
{
    pHandle->ParamsData[bPhase].hDurationms = hDurationms;
}

/**
 * @brief 修改每个阶段的最终机械转速
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @param hFinalMecSpeed01Hz 最终机械转速
 * @retval None
 */
void RUC_SetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalMecSpeed01Hz)
{
    pHandle->ParamsData[bPhase].hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
}

/**
 * @brief 修改每个阶段的最终转矩
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @param hFinalTorque 最终转矩
 * @retval None
 */
void RUC_SetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalTorque)
{
    pHandle->ParamsData[bPhase].hFinalTorque = hFinalTorque;
}

/**
 * @brief 获取每个阶段的持续时间
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @retval uint16_t 持续时间
 */
uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((uint16_t)pHandle->ParamsData[bPhase].hDurationms);
}

/**
 * @brief 获取每个阶段的最终机械转速
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @retval int16_t 最终机械转速
 */
int16_t RUC_GetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((int16_t)pHandle->ParamsData[bPhase].hFinalMecSpeed01Hz);
}

/**
 * @brief 获取每个阶段的最终转矩
 * @param *pHandle 电机启动对象
 * @param bPhase 阶段号
 * @retval int16_t 最终转矩
 */
int16_t RUC_GetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((int16_t)pHandle->ParamsData[bPhase].hFinalTorque);
}

/**
 * @brief 获取阶段号
 * @param *pHandle 电机启动对象
 * @retval uint8_t 阶段号
 */
uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle)
{
    return ((uint8_t)pHandle->bPhaseNbr);
}

/**
 * @brief 获取低侧开关状态
 * @param *pHandle 电机启动对象
 * @retval Bool 1-低侧开关打开
 */
bool RUC_Get_SCLowsideOTF_Status(RevUpCtrl_Handle_t *pHandle)
{
    return (pHandle->OTFSCLowside);
}
