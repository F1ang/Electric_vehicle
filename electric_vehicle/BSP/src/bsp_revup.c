#include "bsp_revup.h"
#include "bsp_sto_pll.h"
#include "bsp_vss.h"
#include "bsp_stc.h"
#include "bsp_spd.h"
#include "bsp_pwmc.h"

/* ����������� */
RevUpCtrl_Handle_t RevUpControlM1 = {
    .hRUCFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE,                               /* ִ��Ƶ��=�ٶȻ�Ƶ�� */
    .hStartingMecAngle = (int16_t)((int32_t)(STARTING_ANGLE_DEG) * 65536 / 360), /* ��������ʼ��е�Ƕ� */
    .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,                           /* ��ʼ���ٽ׶� */
    .hMinStartUpValidSpeed = OBS_MINIMUM_SPEED,                                  /* ������������ת�� */
    .hMinStartUpFlySpeed = (int16_t)(OBS_MINIMUM_SPEED / 2),                     /* ���й۲��������ж�,��������ת��01Hz */
    .OTFStartupEnabled = false,                                                  /* �۲�������ʹ�ܱ�־ */

    /* ʵʱ�����׶β��� */
    .OTFPhaseParams = {
        (uint16_t)500,                 /* ����ʱ�� */
        0,                             /* Ŀ���ٶ� */
        (int16_t)PHASE5_FINAL_CURRENT, /* Ŀ��ת�� */
        (void *)0,                     /* ����ָ�� */
    },

    /* �����׶β�����������(5״̬ת��) */
    .ParamsData = {
        { (uint16_t)PHASE1_DURATION, (int16_t)(PHASE1_FINAL_SPEED_RPM / 6), (int16_t)PHASE1_FINAL_CURRENT, &RevUpControlM1.ParamsData[1] },
        { (uint16_t)PHASE2_DURATION, (int16_t)(PHASE2_FINAL_SPEED_RPM / 6), (int16_t)PHASE2_FINAL_CURRENT, &RevUpControlM1.ParamsData[2] },
        { (uint16_t)PHASE3_DURATION, (int16_t)(PHASE3_FINAL_SPEED_RPM / 6), (int16_t)PHASE3_FINAL_CURRENT, &RevUpControlM1.ParamsData[3] },
        { (uint16_t)PHASE4_DURATION, (int16_t)(PHASE4_FINAL_SPEED_RPM / 6), (int16_t)PHASE4_FINAL_CURRENT, &RevUpControlM1.ParamsData[4] },
        { (uint16_t)PHASE5_DURATION, (int16_t)(PHASE5_FINAL_SPEED_RPM / 6), (int16_t)PHASE5_FINAL_CURRENT, (void *)0 },
    },
};

/**
 * @brief ��ʼ�������׶β���
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

    /* �����׶����к������ʼ�� */
    while ((pRUCPhaseParams != 0) && (bPhase < RUC_MAX_PHASE_NUMBER)) {
        pRUCPhaseParams = pRUCPhaseParams->pNext;
        bPhase++;
    }
    pHandle->ParamsData[bPhase - 1u].pNext = 0;

    pHandle->bPhaseNbr = bPhase;

    /* 1/f*cnt=100ms,��PLL */
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
 * @brief ��ʼ������������״̬
 * @param *pHandle �����������
 * @param hMotorDirection �������
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

    /* �����ٶȴ��������� */
    VSS_Init(pVSS);

    /* ���õ������ģʽ */
    STC_SetControlMode(pSTC, STC_TORQUE_MODE);

    /* ����ĵ�Ƕ� */
    VSS_SetMecAngle(pVSS, pHandle->hStartingMecAngle * hMotorDirection);

    /* ���ٶ�/ת��б�²��� */
    STC_ExecRamp(pSTC, 0, 0u);

    /* ִ���ٶ�/ת��б�� */
    STC_ExecRamp(pSTC, pPhaseParams->hFinalTorque * hMotorDirection,
                 (uint32_t)(pPhaseParams->hDurationms));

    /* ���û�е���ٶ�����:�ٶȲ��� */
    VSS_SetMecAcceleration(pVSS, pPhaseParams->hFinalMecSpeed01Hz * hMotorDirection,
                           pPhaseParams->hDurationms);

    /* ��ɵ�ǰ�׶ε�ticks 1/f*cnt=t,ms */
    pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                (uint32_t)pHandle->hRUCFrequencyHz) /
                                               1000u);
    pHandle->hPhaseRemainingTicks++;

    /* ��ת����������һ�׶� */
    pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

    /* ����������ʱ,��PLL���� */
    pHandle->bResetPLLCnt = 0u;
}

/**
 * @brief ����������ٶ�����ԡ���ͬ�����ټ�ת�ض�̬�����������׶ε�����
 * @param *pHandle �����������
 * @retval Bool 0-�������
 */
bool RUC_OTF_Exec(RevUpCtrl_Handle_t *pHandle)
{
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->pCurrentPhaseParams;
    bool IsSpeedReliable;
    bool retVal = true;
    bool condition = false;

    if (pHandle->hPhaseRemainingTicks > 0u) {
        pHandle->hPhaseRemainingTicks--;

        /* 1.������� */
        if (pHandle->bStageCnt == 0u) {
            /* δ������������1,�������PLL���� */
            if (pHandle->EnteredZone1 == false) {
                if (pHandle->pSNSL->pFctStoOtfResetPLL != 0) {
                    pHandle->bResetPLLCnt++;
                    if (pHandle->bResetPLLCnt > pHandle->bResetPLLTh) {
                        pHandle->pSNSL->pFctStoOtfResetPLL(pHandle->pSNSL);
                        pHandle->bOTFRelCounter = 0u;
                        pHandle->bResetPLLCnt = 0u;
                    }
                }

                /* 2.�ٶȿɿ��Լ���,������һ�� */
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

                /* 3.�ٶ�����ԡ���ͬ�����ټ�ת�ض�̬�����������׶ε����� */
                if (condition == true) {
                    bool bCollinearSpeed = false;
                    int16_t hObsSpeed01Hz = SPD_GetAvrgMecSpeed01Hz(pHandle->pSNSL->_Super);
                    int16_t hObsSpeed01HzAbsValue = (hObsSpeed01Hz < 0 ? (-hObsSpeed01Hz) : (hObsSpeed01Hz)); /* hObsSpeed01Hz absolute value */

                    /* 4.Ŀ���ٶȺ�ʵ���ٶ��Ƿ�������� */
                    if (pHandle->hDirection > 0) {
                        if (hObsSpeed01Hz > 0) {
                            bCollinearSpeed = true;
                        }
                    } else {
                        if (hObsSpeed01Hz < 0) {
                            bCollinearSpeed = true;
                        }
                    }

                    if (bCollinearSpeed == false) { /* ����� */
                        pHandle->bOTFRelCounter = 0u;
                    } else { /* ��� */
                        if ((uint16_t)(hObsSpeed01HzAbsValue) > pHandle->hMinStartUpValidSpeed) {
                            /* 5.�۲��ٶ�>��С�����ٶ�,������������1 */
                            pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
                            pHandle->EnteredZone1 = true;
                        } else if ((uint16_t)(hObsSpeed01HzAbsValue) > pHandle->hMinStartUpFlySpeed) {
                            /* 6.��ִ����ͬ������ */
                            int16_t hOldFinalMecSpeed01Hz = 0;
                            int16_t hOldFinalTorque = 0;
                            int32_t wDeltaSpeedRevUp;
                            int32_t wDeltaTorqueRevUp;
                            bool bError = false;
                            VSS_SetCopyObserver(pHandle->pVSS);
                            pHandle->pSNSL->pFctForceConvergency2(pHandle->pSNSL); /* ǿ��������־2��־ */

                            if (pPhaseParams == 0) { /* �������׶�,���� */
                                bError = true;
                                pHandle->hPhaseRemainingTicks = 0u;
                            } else {
                                /* 7.Ŀ���ٶ�>��ǰ�׶ι۲��ٶ�,��ת��һ�׶�(��ѯ����) */
                                while (pPhaseParams->hFinalMecSpeed01Hz < hObsSpeed01HzAbsValue) {
                                    if (pPhaseParams->pNext == 0) {
                                        /* ����һ�׶�,���� */
                                        bError = true;
                                        pHandle->hPhaseRemainingTicks = 0u;
                                        break;
                                    } else {
                                        /* ������ת��һ�׶�,�����ò��� */
                                        hOldFinalMecSpeed01Hz = pPhaseParams->hFinalMecSpeed01Hz;
                                        hOldFinalTorque = pPhaseParams->hFinalTorque;
                                        pHandle->pCurrentPhaseParams = pPhaseParams->pNext;
                                        pPhaseParams = pHandle->pCurrentPhaseParams;
                                        pHandle->bStageCnt++;
                                    }
                                }
                            }

                            if (bError == false) {
                                /* 8.������ͬ�����ٵ�ת��Iq Te=1.5Pn*����*Iq */
                                int16_t hTorqueReference;

                                /* y(n+1)=y(n)+k*��x,���ݹ۲��ٶ�,��̬����ת�� */
                                wDeltaSpeedRevUp = (int32_t)(pPhaseParams->hFinalMecSpeed01Hz) - (int32_t)(hOldFinalMecSpeed01Hz);
                                wDeltaTorqueRevUp = (int32_t)(pPhaseParams->hFinalTorque) - (int32_t)(hOldFinalTorque);
                                hTorqueReference = (int16_t)((((int32_t)hObsSpeed01Hz) * wDeltaTorqueRevUp) / wDeltaSpeedRevUp) + hOldFinalTorque;

                                /* 9.ִ����ͬ������ */
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
                /* 11.������� */
                PWMC_SwitchOffPWM(pHandle->pPWM);
                pHandle->OTFSCLowside = true;
                PWMC_TurnOnLowSides(pHandle->pPWM);
                pHandle->bOTFRelCounter = 0u;
            } else if ((pHandle->bStageCnt == 1u)) {
                PWMC_SwitchOnPWM(pHandle->pPWM);
                pHandle->OTFSCLowside = false;
            } else {
            }

            /* 10.���õ�x�ν׶����� */
            STC_ExecRamp(pHandle->pSTC, pPhaseParams->hFinalTorque * pHandle->hDirection,
                         (uint32_t)(pPhaseParams->hDurationms));

            VSS_SetMecAcceleration(pHandle->pVSS,
                                   pPhaseParams->hFinalMecSpeed01Hz * pHandle->hDirection,
                                   pPhaseParams->hDurationms);

            /* ������ticks���� */
            pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                        (uint32_t)pHandle->hRUCFrequencyHz) /
                                                       1000u);
            pHandle->hPhaseRemainingTicks++;

            /* ����ָ����ת,������һ�׶� */
            pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

            /*Increases the rev up stages counter.*/
            pHandle->bStageCnt++;
        } else {
            if (pHandle->bStageCnt == pHandle->bPhaseNbr - 1) /* End of user programmed revup */
            {
                retVal = false;                  /* �����ɹ� */
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
 * @brief �����׶�����,�˴�����ִ����������(I/F��5�׶�)
 * @param *pHandle �����������
 * @retval Bool 1-����ִ����������
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

            /* Compute hPhaseRemainingTicks. ִ�д��� */
            pHandle->hPhaseRemainingTicks = (uint16_t)(((uint32_t)pPhaseParams->hDurationms *
                                                        (uint32_t)pHandle->hRUCFrequencyHz) /
                                                       1000u);
            pHandle->hPhaseRemainingTicks++;

            /* ����ָ����һ�����׶� */
            pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

            /* ���������׶δ���*/
            pHandle->bStageCnt++;
        } else {
            /* 5�������׶����,�۲������ٶ���δ����,����ʧ�� */
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
 * @brief  �����Ƿ����
 * @param *pHandle �����������
 * @retval Bool 1-�������
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
 * @brief �Ƴ�����
 * @param *pHandle �����������
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
 * @brief ����һ�����ٽ׶��Ƿ����
 * @param *pHandle �����������
 * @retval Bool 1-��һ�����ٽ׶����
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
 * @brief �޸�ÿ���׶εĳ���ʱ��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @param hDurationms ����ʱ��
 * @retval None
 */
void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms)
{
    pHandle->ParamsData[bPhase].hDurationms = hDurationms;
}

/**
 * @brief �޸�ÿ���׶ε����ջ�еת��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @param hFinalMecSpeed01Hz ���ջ�еת��
 * @retval None
 */
void RUC_SetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalMecSpeed01Hz)
{
    pHandle->ParamsData[bPhase].hFinalMecSpeed01Hz = hFinalMecSpeed01Hz;
}

/**
 * @brief �޸�ÿ���׶ε�����ת��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @param hFinalTorque ����ת��
 * @retval None
 */
void RUC_SetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalTorque)
{
    pHandle->ParamsData[bPhase].hFinalTorque = hFinalTorque;
}

/**
 * @brief ��ȡÿ���׶εĳ���ʱ��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @retval uint16_t ����ʱ��
 */
uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((uint16_t)pHandle->ParamsData[bPhase].hDurationms);
}

/**
 * @brief ��ȡÿ���׶ε����ջ�еת��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @retval int16_t ���ջ�еת��
 */
int16_t RUC_GetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((int16_t)pHandle->ParamsData[bPhase].hFinalMecSpeed01Hz);
}

/**
 * @brief ��ȡÿ���׶ε�����ת��
 * @param *pHandle �����������
 * @param bPhase �׶κ�
 * @retval int16_t ����ת��
 */
int16_t RUC_GetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
    return ((int16_t)pHandle->ParamsData[bPhase].hFinalTorque);
}

/**
 * @brief ��ȡ�׶κ�
 * @param *pHandle �����������
 * @retval uint8_t �׶κ�
 */
uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle)
{
    return ((uint8_t)pHandle->bPhaseNbr);
}

/**
 * @brief ��ȡ�Ͳ࿪��״̬
 * @param *pHandle �����������
 * @retval Bool 1-�Ͳ࿪�ش�
 */
bool RUC_Get_SCLowsideOTF_Status(RevUpCtrl_Handle_t *pHandle)
{
    return (pHandle->OTFSCLowside);
}
