#include "bsp_r3f4.h"
#include "tim.h"

/**
 * @brief �������������. ��ϵ��,beta��Ϊ��������ϵ�ķ�����Ϊ��.
 * @param *pHdl ���PWM���
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    uint8_t bSector;
    int32_t wAux;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl; /* ������Ļ����ַ,�ɷ����������Ա */

    // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_RESET);

    /* ����CCR4����ADC */
    TIM8->CCER &= ~TIM_CCER_CC4E;

    bSector = pHandle->_Super.hSector;
    switch (bSector) {
    case SECTOR_4: /* �ɼ�A B���������ֻ���A B���������) */
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

    case SECTOR_6: /* �ɼ�B C����� */
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

    case SECTOR_2: /* �ɼ�A C����� */
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
 * @brief �ȴ�UPDATEʱ��,�ر�TIM8�����,����CH1~CH3���������
 * @retval None
 */
void R3F4XX_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
    /* Disable UPDATE ISR */
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);

    /* ֹͣͨ��1/2/3��������� */
    TIM8->CCER &= ~((uint16_t)0x555u);

    /* ����1�θ����¼�/�ж� */
    while (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) == RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            break;
        }
    }

    /* �ر������ʹ�� */
    __HAL_TIM_MOE_DISABLE(&htim8);

    /* ����ͨ��1/2/3��������� */
    TIM8->CCER |= ((uint16_t)0x555u);

    return;
}

/**
 * @brief ������¼�,���������ж�,�����ʹ��
 * @retval None
 */
void R3F4XX_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;
    pHandle->_Super.bTurnOnLowSidesAction = false;

    /* It clears ADCs JSTRT and JEOC bits */
    ADC1->SR &= ~((uint32_t)(0xC));
    ADC2->SR &= ~((uint32_t)(0xC));

    /* ������¼�*/
    TIM8->SR = ~TIM_FLAG_UPDATE;

    /* ���������ж� */
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    /* ���������ʹ��MOE */
    __HAL_TIM_MOE_ENABLE(&htim8);

    return;
}

/**
 * @brief ��ȡoffsetУ׼ֵ,�ر�CHx���,����ADC����ͨ��;������¼�,���������ж�,�����ʹ��;����CHx���
 * @param *pHdl ���PWM���
 * @retval None
 */
void R3F4XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    pHandle->wPhaseAOffset = 0u;
    pHandle->wPhaseBOffset = 0u;
    pHandle->wPhaseCOffset = 0u;
    pHandle->bIndex = 0u;

    /* ����TIM8��CHy and CHyN */
    TIM8->CCER &= ((uint16_t)~0x1555u);

    /* �ı���ADCx_ISRִ�е�A B�������ȡ�ص� */
    pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationAB;

    /* ����ADCx�Ĳ��� */
    pHandle->wADC1Channel = PHASE_A_MSK;
    pHandle->wADC2Channel = PHASE_B_MSK;

    /* ������¼�,���������ж�,�����ʹ��->����TIM8�����ж�����CC4����ADC���� */
    R3F4XX_SwitchOnPWM(&pHandle->_Super);

    /* ��Ҫ��PWMC_GetPhaseCurrents(pwmcHandle, &Iab)���ʹ��,pHandle->bIndex++ */
    while (pHandle->bIndex < (NB_CONVERSIONS)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            /* �����ж���ʹ�� */
        } else {
            pHandle->bIndex = NB_CONVERSIONS;
        }
    }

    /* Offset calibration for C phase */
    pHandle->bIndex = 0u;

    /* �ı���ADCx_ISRִ�е�C�������ȡ�ص� */
    pHandle->_Super.pFctGetPhaseCurrents = &R3F4XX_HFCurrentsCalibrationC;

    /* ����ADCx�Ĳ��� */
    pHandle->wADC1Channel = PHASE_C_MSK;
    pHandle->wADC2Channel = PHASE_C_MSK;

    /* ����TIM8������͸����ж�,��ADC���жϱ�־λ */
    R3F4XX_SwitchOnPWM(&pHandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    while (pHandle->bIndex < (NB_CONVERSIONS / 2u)) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE)) {
            /* �����ж���ʹ�� */
        } else {
            pHandle->bIndex = NB_CONVERSIONS;
        }
    }

    /* <<1 >>4��ȡƫ��ֵ */
    pHandle->wPhaseAOffset >>= 3;
    pHandle->wPhaseBOffset >>= 3;
    pHandle->wPhaseCOffset >>= 3;

    /* �ı���ADCx_ISRִ�е��������ȡ�ص� */
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

    /* ����TIM8��CHy and CHyN */
    htim8.Instance->CCER |= 0x555u;
}

/**
 * @brief �¹ܿ�
 * @param *pHdl ������
 * @retval None
 */
void R3F4XX_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
    /* Clear Update Flag */
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);

    /* ��ͨ�¹� */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

    /* ��һ�δ��������ж�:TIM8������������Update�¼� */
    while (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) == RESET) {
    }

    /* Main PWM Output Enable */
    __HAL_TIM_MOE_ENABLE(&htim8);

    return;
}

/**
 * @brief ����1������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0; /* �����ز��� */
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector1��С��A��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->_Super.hTafter) {
        /* �ɼ�A B�����,��������4/����5 */
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
                adcTrig = ADC_CR2_JEXTEN_1; /* �½��ز��� */
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
 * @brief ����2������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector2��С��B��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
                adcTrig = ADC_CR2_JEXTEN_1; /* �½��ز��� */
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
 * @brief ����3������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector3��С��B��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
                /* �½��ز��� */
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
 * @brief ����4������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector4��С��C��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
                adcTrig = ADC_CR2_JEXTEN_1; /* �½��ز��� */
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
 * @brief ����5������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector5��С��C��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
 * @brief ����6������
 * @param *pHdl ������
 * @retval 0,����
 */
uint16_t R3F4XX_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    uint32_t adcTrig = ADC_CR2_JEXTEN_0;
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ��λ��������λ */
    pHandle->wADCTriggerSet &= ~(ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTEN_0);

    /* sector6��С��A��,�����㹻ʱ�����-���1 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->_Super.hTafter) {
        pHandle->_Super.hSector = SECTOR_4;

        /* ����ADC�����������ĵ� */
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
 * @brief ����PWMռ�ձ�
 * @param *pHdl ������
 * @retval 0,����
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
 * @brief ����CCR4���ʹ��λ,����ʱ��ADC����,����AB����;TIM8�����жϻ�����CC4����
 * @param *pHdl ���PWM���
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ����CCR4���ʹ��λ,����ʱ��ADC���� */
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
 * @brief ����CCR4���ʹ��λ,����ʱ��ADC����,����C����;TIM8�����жϻ�����CC4����
 * @param *pHdl ���PWM���
 * @param *pStator_Currents Iab
 * @retval None
 */
void R3F4XX_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
    PWMC_R3_F4_Handle_t *pHandle = (PWMC_R3_F4_Handle_t *)pHdl;

    /* ͣ��CCR4����ADC */
    TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_4, TIM_CCx_DISABLE);

    if (pHandle->bIndex < NB_CONVERSIONS / 2u) {
        pHandle->wPhaseCOffset += ADC1->JDR1;
        pHandle->wPhaseCOffset += ADC2->JDR1;
        pHandle->bIndex++;
    }
}
