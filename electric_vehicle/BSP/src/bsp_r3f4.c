#include "bsp_r3f4.h"
#include "tim.h"

/**
 * @brief 扇区采样相电流. 建系上,beta轴为右手坐标系的反方向为正.
 * @param *pHdl 电机PWM句柄
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    uint8_t bSector;
    int32_t wAux;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl; /* 派生类的基类地址,可访问派生类成员 */

    // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_RESET);

    /* 禁用CCR4触发ADC */
    TIM8->CCER &= ~TIM_CCER_CC4E;

    bSector = pHandle->_Super.hSector;
    switch (bSector) {
    case SECTOR_4: /* 采集A B相电流常规只需才A B相电流即可) */
    case SECTOR_5:
        /* Current on Phase C is not accessible     */
        /* Ia = PhaseAOffset - ADC converted value) */
        wAux = (int32_t)(ADC1->JDR1);
        wAux *= 2;
        wAux = (int32_t)(pHandle->wPhaseAOffset) - wAux;

        /* Saturation of Ia */
        if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component1 = -INT16_MAX;
        } else if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component1 = INT16_MAX;
        } else {
            pStator_Currents->qI_Component1 = (int16_t)wAux;
        }

        /* Ib = PhaseBOffset - ADC converted value) */
        wAux = (int32_t)(ADC2->JDR1);
        wAux *= 2;
        wAux = (int32_t)(pHandle->wPhaseBOffset) - wAux;

        /* Saturation of Ib */
        if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component2 = -INT16_MAX;
        } else if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component2 = INT16_MAX;
        } else {
            pStator_Currents->qI_Component2 = (int16_t)wAux;
        }
        break;

    case SECTOR_6: /* 采集B C相电流 */
    case SECTOR_1:
        /* Current on Phase A is not accessible     */
        /* Ib = PhaseBOffset - ADC converted value) */
        wAux = (int32_t)(ADC1->JDR1);
        wAux *= 2;
        wAux = (int32_t)(pHandle->wPhaseBOffset) - wAux;

        /* Saturation of Ib */
        if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component2 = -INT16_MAX;
        } else if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component2 = INT16_MAX;
        } else {
            pStator_Currents->qI_Component2 = (int16_t)wAux;
        }

        /* Ia = -Ic -Ib */
        wAux = (int32_t)(ADC2->JDR1);
        wAux *= 2;
        wAux -= (int32_t)pHandle->wPhaseCOffset;
        wAux -= (int32_t)pStator_Currents->qI_Component2;

        /* Saturation of Ia */
        if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component1 = INT16_MAX;
        } else if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component1 = -INT16_MAX;
        } else {
            pStator_Currents->qI_Component1 = (int16_t)wAux;
        }
        break;

    case SECTOR_2: /* 采集A C相电流 */
    case SECTOR_3:
        /* Current on Phase B is not accessible     */
        /* Ia = PhaseAOffset - ADC converted value) */
        wAux = (int32_t)(ADC1->JDR1);
        wAux *= 2;
        wAux = (int32_t)(pHandle->wPhaseAOffset) - wAux;

        /* Saturation of Ia */
        if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component1 = -INT16_MAX;
        } else if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component1 = INT16_MAX;
        } else {
            pStator_Currents->qI_Component1 = (int16_t)wAux;
        }

        /* Ib = -Ic -Ia */
        wAux = (int32_t)(ADC2->JDR1);
        wAux *= 2;
        wAux -= (int32_t)pHandle->wPhaseCOffset;
        wAux -= (int32_t)pStator_Currents->qI_Component1;

        /* Saturation of Ib */
        if (wAux > INT16_MAX) {
            pStator_Currents->qI_Component2 = INT16_MAX;
        } else if (wAux < -INT16_MAX) {
            pStator_Currents->qI_Component2 = -INT16_MAX;
        } else {
            pStator_Currents->qI_Component2 = (int16_t)wAux;
        }
        break;

    default:
        break;
    }

    pHandle->_Super.hIa = pStator_Currents->qI_Component1;
    pHandle->_Super.hIb = pStator_Currents->qI_Component2;
    pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
 * @brief 等待UPDATE时间,关闭TIM8主输出,开启CH1~CH3及互补输出
 * @retval None
 */
void R3F4XX_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
    /* Disable UPDATE ISR */
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);

    /* 停止通道1/2/3及互补输出 */
    TIM8->CCER &= ~((uint16_t)0x555u);

    /* 触发1次更新事件/中断 */
    while (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) == RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            break;
        }
    }

    /* 关闭主输出使能 */
    __HAL_TIM_MOE_DISABLE(&htim8);

    /* 启动通道1/2/3及互补输出 */
    TIM8->CCER |= ((uint16_t)0x555u);

    return;
}

/**
 * @brief 清更新事件,开启更新中断,主输出使能
 * @retval None
 */
void R3F4XX_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;
    pHandle->_Super.bTurnOnLowSidesAction = false;

    /* It clears ADCs JSTRT and JEOC bits */
    ADC1->SR &= ~((uint32_t)(0xC));
    ADC2->SR &= ~((uint32_t)(0xC));

    /* 清更新事件*/
    TIM8->SR = ~TIM_FLAG_UPDATE;

    /* 开启更新中断 */
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    /* 开启主输出使能MOE */
    __HAL_TIM_MOE_ENABLE(&htim8);

    return;
}

/**
 * @brief 读取offset校准值,关闭CHx输出,配置ADC采样通道;清更新事件,开启更新中断,主输出使能;开启CHx输出
 * @param *pHdl 电机PWM句柄
 * @retval None
 */
void R3F4XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    pHandle->wPhaseAOffset = 0u;
    pHandle->wPhaseBOffset = 0u;
    pHandle->wPhaseCOffset = 0u;
    pHandle->bIndex = 0u;

    /* 禁用TIM8的CHy and CHyN */
    TIM8->CCER &= ((uint16_t)~0x1555u);

    /* 改变在ADCx_ISR执行的A B相电流获取回调 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationAB;

    /* 配置ADCx的采样 */
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;

    /* 清更新事件,开启更新中断,主输出使能->进入TIM8更新中断配置CC4触发ADC采样 */
    R3F4XX_SwitchOnPWM(&pHandle->_Super);

    /* 需要和PWMC_GetPhaseCurrents(pwmcHandle, &Iab)配合使用,pHandle->bIndex++ */
    while (pHandle->bIndex < (NB_CONVERSIONS)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            /* 更新中断已使能 */
        } else {
            pHandle->bIndex = NB_CONVERSIONS;
        }
    }

    /* Offset calibration for C phase */
    pHandle->bIndex = 0u;

    /* 改变在ADCx_ISR执行的C相电流获取回调 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationC;

    /* 配置ADCx的采样 */
    pHandle->wADC1Channel = PHASE_C_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;

    /* 开启TIM8主输出和更新中断,清ADC的中断标志位 */
    R3F4XX_SwitchOnPWM(&pHandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    while (pHandle->bIndex < (NB_CONVERSIONS / 2u)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            /* 更新中断已使能 */
        } else {
            pHandle->bIndex = NB_CONVERSIONS;
        }
    }

    /* <<1 >>4获取偏置值 */
    pHandle->wPhaseAOffset >>= 3;
    pHandle->wPhaseBOffset >>= 3;
    pHandle->wPhaseCOffset >>= 3;

    /* 改变在ADCx_ISR执行的相电流获取回调 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_GetPhaseCurrents;

    /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
       force 50% duty cycle on the three inverer legs */
    /* Disable TIMx preload */
    htim8.Instance->CCMR1 &= 0xF7F7u;
    htim8.Instance->CCMR2 &= 0xF7F7u;
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pHandle->Half_PWMPeriod);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pHandle->Half_PWMPeriod);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pHandle->Half_PWMPeriod);

    /* Enable TIMx preload */
    htim8.Instance->CCMR1 |= 0x0808u;
    htim8.Instance->CCMR2 |= 0x0808u;

    /* 启动TIM8的CHy and CHyN */
    htim8.Instance->CCER |= 0x555u;
}

/**
 * @brief 下管开
 * @param *pHdl 电机句柄
 * @retval None
 */
void R3F4XX_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
    /* Clear Update Flag */
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);

    /* 开通下管 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

    /* 下一次触发更新中断:TIM8计数溢出会产生Update事件 */
    while (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) == RESET) {
    }

    /* Main PWM Output Enable */
    __HAL_TIM_MOE_ENABLE(&htim8);

    return;
}

/**
 * @brief 扇区1采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0; /* 上升沿采样 */
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector1最小的A相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->_Super.hTafter) {
        /* 采集A B相电流,可有扇区4/扇区5 */
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u;

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_ab */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);

        /* T_ab/2 > T_a/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhA - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhA + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1; /* 下降沿采样 */
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_B_MSK;
        pHandle->wADC2Channel = PHASE_C_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;
    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 扇区2采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector2最小的B相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u;

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_ba */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);

        /* T_ab/2 > T_b/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhB - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhB + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1; /* 下降沿采样 */
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_C_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;
    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 扇区3采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector3最小的B相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u;

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_bc */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);

        /* T_bc/2 > T_b/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhB - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhB + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1;
                /* 下降沿采样 */
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_C_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;
    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 扇区4采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector4最小的C相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u;

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_ca */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);

        /* T_ca/2 > T_c/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhC - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhC + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1; /* 下降沿采样 */
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;
    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 扇区5采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector5最小的C相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u; /* 5250-2 */

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_ac */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);

        /* T_ac/2 > T_c/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhC - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhC + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1;
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;
    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 扇区6采样点
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 复位采样边沿位 */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector6最小的A相,仍有足够时间采样-情况1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* 设置ADC采样点于中心点 */
        hCntSmp = pHandle->Half_PWMPeriod - 1u;

        pHandle->wADC1Channel = PHASE_A_MSK;
        pHandle->wADC2Channel = PHASE_B_MSK;
    } else {
        /* T_ac */
        hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);

        /* T_ac/2 > T_c/2 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) * 2u) {
            hCntSmp = pHandle->_Super.hCntPhA - pHandle->_Super.hTbefore;
        } else {
            hCntSmp = pHandle->_Super.hCntPhA + pHandle->_Super.hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod) {
                adcTrig = ADC_CR2_JEXTEN_1;
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }

        pHandle->wADC1Channel = PHASE_B_MSK;
        pHandle->wADC2Channel = PHASE_C_MSK;
    }

    /* set ADC trigger edge */
    pHandle->wADCTriggerSet |= adcTrig;

    /* Set TIMx_CH4 value */
    TIM8->CCR4 = hCntSmp;

    return R3F4XX_WriteTIMRegisters(&pHandle->_Super);
}

/**
 * @brief 设置PWM占空比
 * @param *pHdl 电机句柄
 * @retval 0,正常
 */
uint16_t R3F4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 5250 */
    TIM8->CCR1 = pHandle->_Super.hCntPhA;
    TIM8->CCR2 = pHandle->_Super.hCntPhB;
    TIM8->CCR3 = pHandle->_Super.hCntPhC;

    return 0;
}

/**
 * @brief 禁用CCR4输出使能位,即暂时关ADC触发,进行AB采样;TIM8更新中断会重启CC4触发
 * @param *pHdl 电机PWM句柄
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 禁用CCR4输出使能位,即暂时关ADC触发 */
    // CLEAR_BIT(TIM8->CCER, TIM_CCER_CC4E);
    TIM8->CCER &= ~TIM_CCER_CC4E;

    pHandle->bSoFOC = 0u;

    if (pHandle->bIndex < NB_CONVERSIONS) {
        pHandle->wPhaseAOffset += ADC1->JDR1;
        pHandle->wPhaseBOffset += ADC2->JDR1;
        pHandle->bIndex++;
    }
}

/**
 * @brief 禁用CCR4输出使能位,即暂时关ADC触发,进行C采样;TIM8更新中断会重启CC4触发
 * @param *pHdl 电机PWM句柄
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* 停用CCR4触发ADC */
    TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_4, TIM_CCx_DISABLE);

    if (pHandle->bIndex < NB_CONVERSIONS / 2u) {
        pHandle->wPhaseCOffset += ADC1->JDR1;
        pHandle->wPhaseCOffset += ADC2->JDR1;
        pHandle->bIndex++;
    }
}
