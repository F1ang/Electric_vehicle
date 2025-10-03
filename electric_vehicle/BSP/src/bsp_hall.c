#include "bsp_hall.h"
#include "tim.h"
#include "bsp_spd.h"

/* HALL����������ʵ���� */
HALL_Handle_t HALL_M1 = {
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* ������ */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* ���ɿ���е�ٶ�0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* ��С�ɿ���е�ٶ�0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* ������������� */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* ���ɿ���е���ٶ�0.1Hz  */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* ����Ƶ��16kHz */
    },
    .SensorPlacement = DEGREES_120,                          /* 120degrees */
    .PhaseShift = (int16_t)(HALL_PHASE_SHIFT * 65536 / 360), /* H1��������A�����BEMF��ƫ�� */
    .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,       /* �ٶ�Ƶ��500hz */
    .SpeedBufferSize = HALL_AVERAGING_FIFO_DEPTH,            /* ����ƽ���ٶ� */
    .TIMClockFreq = HALL_TIM_CLK,                            /* ʱ��Ƶ��84MHz */
};

/**
 * @brief HALL��ʼ��
 * @param *pHandle Hall����������
 *  Ӳ����λģʽ�����ô���ԴΪTI1F_ED�������ź�˫���أ�htim->Instance->SMCR |= TIM_TS_TI1F_ED
    ����Ϊ��ģʽ��λ��ÿ���������ظ�λCNT��htim->Instance->SMCR |= TIM_SLAVEMODE_RESET;
    HAL_TIMEx_HallSensor_Start_IT(&htim5);
 * @retval None
 */
void HALL_Init(HALL_Handle_t *pHandle)
{
    /* ��Ƶ�� */
    uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz *
                                       pHandle->_Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz *
                                       pHandle->_Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;

    hMinReliableElSpeed01Hz /= 4u;
    hMaxReliableElSpeed01Hz *= 2u;

    /* ���Ƶ��(���),clk/(arr+1) */
    pHandle->OvfFreq = (uint16_t)(pHandle->TIMClockFreq / 65536u);

    if (hMinReliableElSpeed01Hz == 0u) {
        /* Set fixed to 150 ms */
        pHandle->HallTimeout = 150u;
    } else {
        /* t=��/w=10*10^3*60/360/x=1/f_cnt  �����ٶ�w=x(Ȧ/s):0.1hz->����10*10^3ms Ȧ=360��!!���HALL��60��һ�β����ж�!! */
        pHandle->HallTimeout = 10000u / (6u * hMinReliableElSpeed01Hz);
    }

    /* ms->s,clk/(arr+1)/f_cnt=psc */
    pHandle->HALLMaxRatio = (pHandle->HallTimeout * pHandle->OvfFreq) / 1000;

    /* psc*arr */
    pHandle->MaxPeriod = (pHandle->HALLMaxRatio) * 65536uL;

    /* �������ٶ�(Ȧ/s)=f_wm(��ֵ) */
    pHandle->SatSpeed = hMaxReliableElSpeed01Hz;

    /* �����ٶ�:W_s16degree/s=F_hall*60��=W_e, ����ΪFOC�����µ�dpp=W_e*T_foc=����_foc */
    /* dpp=����_e/n,n=F_mea/F_hall :  W_e=����_e/T_hall=����_e*f_hall   We*T_foc=����_foc */
    /* F_mea=F_foc */
    pHandle->PseudoFreqConv = ((pHandle->TIMClockFreq / 6u) / (pHandle->_Super.hMeasurementFrequency)) * 65536u;

    /* ��С����=Fclk/Fw_m,����ٶ��Ƿ���� 10����Ϊ��λ0.1Hz,6�����HALL�Ħ�=60��һ�β����ж� w=360��*f_wm(Ȧ/s����ֵ����) = ��*Fw_m(���迼��60��) */
    pHandle->MinPeriod = ((10u * pHandle->TIMClockFreq) / 6u) / hMaxReliableElSpeed01Hz;

    /* n=����Ƶ��(f_foc)/�ٶ�Ƶ��,����dpp2=����/n */
    pHandle->PWMNbrPSamplingFreq = (pHandle->_Super.hMeasurementFrequency /
                                    pHandle->SpeedSamplingFreqHz) -
                                   1u;

    /* ���ü����� */
    pHandle->SensorIsReliable = true;

    /* �޸�psc,�����ɸ����¼� */
    TIM5->PSC = pHandle->HALLMaxRatio;
    TIM5->EGR = TIM_EGR_UG;

    /* ����жϱ�־ */
    TIM5->SR = 0;

    /* ���ø����¼�Դ�����Լ��������/����(�쳣) */
    __HAL_TIM_URS_ENABLE(&htim5);

    /* ʹ�ܲ��������ж� */
    // HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); /* HALL�����жϺ������� */
    HAL_TIM_Base_Start_IT(&htim5);
    // HAL_TIM_Base_Start(&htim5);
    // HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_1);

    /* ���ü����� */
    __HAL_TIM_SET_COUNTER(&htim5, HALL_COUNTER_RESET);

    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->SpeedBufferSize;

    for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++) {
        pHandle->SensorSpeed[bIndex] = 0;
    }
}

/**
 * @brief HALL���
 * @param *pHandle Hall����������
 * @retval None
 */
void HALL_Clear(HALL_Handle_t *pHandle)
{
    /* disable cc1 interrupt */
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC1);

    pHandle->RatioDec = false;
    pHandle->RatioInc = false;

    /* �������Ƿ�ɿ� */
    pHandle->SensorIsReliable = true;

    /* ��е���ٶ�0.1Hz */
    pHandle->_Super.hMecAccel01HzP = 0;

    /* �ٶȼ�������� */
    pHandle->FirstCapt = 0u;
    pHandle->BufferFilled = 0u;
    pHandle->OVFCounter = 0u;

    pHandle->CompSpeed = 0;
    pHandle->ElSpeedSum = 0;

    pHandle->Direction = POSITIVE;

    /* ����index */
    pHandle->SpeedFIFOIdx = 0u;

    /* �����ٶȲɼ� */
    pHandle->NewSpeedAcquisition = 0;

    TIM5->PSC = pHandle->HALLMaxRatio;
    __HAL_TIM_SET_COUNTER(&htim5, HALL_COUNTER_RESET);
    __HAL_TIM_ENABLE(&htim5);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

    HALL_Init_Electrical_Angle(pHandle);
}

/**
 * @brief Ԥ��λ
 * @param *pHandle Hall����������
 * @retval None
 */
static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle)
{
    if (pHandle->SensorPlacement == DEGREES_120) {
        pHandle->HallState = ((HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) == GPIO_PIN_SET) << 2) |
                             ((HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) == GPIO_PIN_SET) << 1) |
                             (HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) == GPIO_PIN_SET);
    } else {
        pHandle->HallState = ((HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) == GPIO_PIN_SET) << 2) |
                             ((HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) == GPIO_PIN_SET) << 1) |
                             (HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) == GPIO_PIN_SET);
    }

    switch (pHandle->HallState) {
    case STATE_5:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2);
        break;
    case STATE_1:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                                             S16_60_PHASE_SHIFT / 2);
        break;
    case STATE_3:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                             S16_60_PHASE_SHIFT / 2);
        break;
    case STATE_2:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                                             S16_60_PHASE_SHIFT / 2);
        break;
    case STATE_6:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                                             S16_60_PHASE_SHIFT / 2);
        break;
    case STATE_4:
        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT / 2);
        break;
    default:
        /* Bad hall sensor configutarion so update the speed reliability */
        pHandle->SensorIsReliable = false;
        break;
    }

    /* Initialize the measured angle */
    pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;
}

/**
 * @brief FOC�岹�������Ƕ�
 * @param *pHandle Hall����������
 * @retval None
 */
int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle)
{
    /* ��Ƶ����f_foc */
    if (pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED) {
        pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp; /* ������Ƕ�dpp1 */
        pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;   /* Ŀ���Ƕ� */
        //  pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed; /* hall_foc����dpp1+speed_foc����dpp2,dpp2���ֲ��������pwm������*/
        pHandle->_Super.hElAngle += pHandle->AvrElSpeedDpp + pHandle->CompSpeed; /* hall_foc����dpp1+speed_foc����dpp2,dpp2���ֲ��������pwm������*/
        pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
    } else {
        pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
    }

    return pHandle->_Super.hElAngle;
}

/**
 * @brief ��Ƶ����,�ٶȻ�500hz
 * @param *pHandle Hall����������
 * @param *hMecSpeed01Hz ��е�ٶ�0.1Hz
 * @retval None
 */
bool HALL_CalcAvrgMecSpeed01Hz(HALL_Handle_t *pHandle, int16_t *hMecSpeed01Hz)
{
    /* ��Ƶ����,�ٶȻ�500hz */
    int16_t SpeedMeasAux;
    bool bReliability;

    /* ����ٶȲ��� dpp1:hall_foc �����Ƕ� dpp1=��_e/n,n = f_foc/f_hall */
    SpeedMeasAux = pHandle->CurrentSpeed;

    if (pHandle->SensorIsReliable) {
        if (TIM5->PSC >= pHandle->HALLMaxRatio) {
            /* At start-up or very low freq */
            pHandle->_Super.hElSpeedDpp = 0;
            *hMecSpeed01Hz = 0;
        } else {
            pHandle->_Super.hElSpeedDpp = SpeedMeasAux;
            if (SpeedMeasAux == 0) {
                /* Speed is too low */
                *hMecSpeed01Hz = 0;
            } else {
                /* Check if speed is not to fast */
                if (SpeedMeasAux != HALL_MAX_PSEUDO_SPEED) {
                    /* f_�ٶȻ� �� f_foc�Ľ��ٶȲ���dpp2 */
                    pHandle->TargetElAngle = pHandle->MeasuredElAngle;
                    pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
                    /* dpp2;f_hall��dpp1ֻ������Сfoc���ڵĵ�Ƕ�,f_speed��f_focƵ�ʲ���,����speed��foc������Ҫ������Ƕ�dpp2 */
                    /* ��dpp2���ֲ��������foc(pwm)������ */
                    pHandle->CompSpeed = (int16_t)((int32_t)(pHandle->DeltaAngle) /
                                                   (int32_t)(pHandle->PWMNbrPSamplingFreq));

                    /* ˲ʱ�ٶȻ��߻���ƽ������ٶ� */
                    *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp(pHandle);

                    /* W*360/f_����=dpp,360~65536,�����ٶ�=dpp*f_����/pole , f_����=f_foc */
                    *hMecSpeed01Hz = (int16_t)((*hMecSpeed01Hz * (int32_t)pHandle->_Super.hMeasurementFrequency * 10) /
                                               (65536 * (int32_t)pHandle->_Super.bElToMecRatio));

                } else {
                    *hMecSpeed01Hz = (int16_t)pHandle->SatSpeed;
                }
            }
        }
        bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeed01Hz);
    } else {
        bReliability = false;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        /* If speed is not reliable the El and Mec speed is set to 0 */
        pHandle->_Super.hElSpeedDpp = 0;
        *hMecSpeed01Hz = 0;
    }

    pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;

    return (bReliability);
}

/**
 * @brief ����ƽ������ٶ�
 * @param *pHandle Hall����������
 * @retval ƽ������ٶ�
 */
static int16_t HALL_CalcAvrgElSpeedDpp(HALL_Handle_t *pHandle)
{
    /* HALL�������ٶ�,���л���ƽ���ٶ� */
    if (pHandle->NewSpeedAcquisition == 1) {

        if (pHandle->BufferFilled < pHandle->SpeedBufferSize) {
            /* ȡ˲ʱ�ٶ� */
            pHandle->AvrElSpeedDpp = (int16_t)pHandle->CurrentSpeed;
        } else {
            /* ȡƽ���ٶ� */
            pHandle->AvrElSpeedDpp = (int16_t)(pHandle->ElSpeedSum / (int32_t)(pHandle->SpeedBufferSize)); /* Average value */
        }

        /* Clear new speed acquisitions flag */
        pHandle->NewSpeedAcquisition = 0;
    }

    return pHandle->AvrElSpeedDpp;
}

/**
 * @brief HALL�����ж�
 * @param *pHandleVoid Hall����������
 * @retval None
 */
void *HALL_TIMx_CC_IRQHandler(void *pHandleVoid)
{
    HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid;
    uint8_t bPrevHallState;
    uint32_t wCaptBuf;
    uint16_t hPrscBuf;
    uint16_t hHighSpeedCapture;
    uint32_t new_psc;

    if (pHandle->SensorIsReliable) {
        /* A capture event generated this interrupt */
        bPrevHallState = pHandle->HallState;

        if (pHandle->SensorPlacement == DEGREES_120) {
            pHandle->HallState = ((HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) == GPIO_PIN_SET) << 2) |
                                 ((HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) == GPIO_PIN_SET) << 1) |
                                 (HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) == GPIO_PIN_SET);
        } else {
            pHandle->HallState = ((HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) == GPIO_PIN_SET) << 2) |
                                 ((HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) == GPIO_PIN_SET) << 1) |
                                 (HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) == GPIO_PIN_SET);
        }

        /* ��Ƕ�У׼ */
        switch (pHandle->HallState) {
        case STATE_5:
            if (bPrevHallState == STATE_4) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = pHandle->PhaseShift;
            } else if (bPrevHallState == STATE_1) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT);
            } else {
            }
            break;

        case STATE_1:
            if (bPrevHallState == STATE_5) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
            } else if (bPrevHallState == STATE_3) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT);
            } else {
            }
            break;

        case STATE_3:
            if (bPrevHallState == STATE_1) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT);
            } else if (bPrevHallState == STATE_2) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                                     S16_60_PHASE_SHIFT);
            } else {
            }

            break;

        case STATE_2:
            if (bPrevHallState == STATE_3) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
            } else if (bPrevHallState == STATE_6) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT);
            } else {
            }
            break;

        case STATE_6:
            if (bPrevHallState == STATE_2) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT);
            } else if (bPrevHallState == STATE_4) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT);
            } else {
            }
            break;

        case STATE_4:
            if (bPrevHallState == STATE_6) {
                pHandle->Direction = POSITIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT);
            } else if (bPrevHallState == STATE_5) {
                pHandle->Direction = NEGATIVE;
                pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift);
            } else {
            }
            break;

        default:
            /* Bad hall sensor configutarion so update the speed reliability */
            pHandle->SensorIsReliable = false;

            break;
        }

        /* ������һ�β��� */
        if (pHandle->FirstCapt == 0u) {
            pHandle->FirstCapt++;
            __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
        } else {
            /* �ٶȻ���ƽ��index */
            if (pHandle->BufferFilled < pHandle->SpeedBufferSize) {
                pHandle->BufferFilled++;
            }

            /* �洢���µ��ٶȻ�ȡֵ*/
            hHighSpeedCapture = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
            wCaptBuf = (uint32_t)hHighSpeedCapture;
            hPrscBuf = TIM5->PSC;

            /* ������������ӵ��������� */
            wCaptBuf += (uint32_t)pHandle->OVFCounter * 0x10000uL;

            if (pHandle->OVFCounter != 0u) {
                /* ʹ��Ԥ��Ƶ���������� */
                uint16_t hAux;
                hAux = hPrscBuf + 1u;
                wCaptBuf *= hAux; // TIMƵ���µļ���

                /* ��һ���Ѿ��޸�psc,�������,��һ���ݲ��޸�psc */
                if (pHandle->RatioInc) {
                    pHandle->RatioInc = false;
                } else {
                    if (TIM5->PSC < pHandle->HALLMaxRatio) /* �����Ƶ�� */
                    {
                        new_psc = TIM5->PSC + 1;
                        __HAL_TIM_SET_PRESCALER(&htim5, new_psc);
                        pHandle->RatioInc = true;
                    }
                }
            } else {
                /* ��һ���޸�psc,��δ���,����ݲ��޸�psc */
                if (pHandle->RatioDec) {
                    /* �������� */
                    uint16_t hAux;
                    hAux = hPrscBuf + 2u; /* Ӱ�ӼĴ���,��ȴ������¼�--(PSC+1)+1 */
                    wCaptBuf *= hAux;

                    pHandle->RatioDec = false;
                } else /* ������ϴβ��������δ�Է�Ƶ�������޸� */
                {
                    uint16_t hAux = hPrscBuf + 1u;
                    wCaptBuf *= hAux;

                    /* ����ֵС����ֵ,˵������Ƶ�ʹ���,ת�ٽϿ�,�޸�psc,��߼���Ƶ�� */
                    if (hHighSpeedCapture < LOW_RES_THRESHOLD) /* If capture range correct */
                    {
                        if (TIM5->PSC > 0u) /* or prescaler cannot be further reduced */
                        {
                            /* ����psc=0,�ұ������޸���psc */
                            new_psc = TIM5->PSC - 1;
                            __HAL_TIM_SET_PRESCALER(&htim5, new_psc);
                            pHandle->RatioDec = true;
                        }
                    }
                }
            }

            /* С����С��������,˵������ */
            if (wCaptBuf < pHandle->MinPeriod) {
                pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
                pHandle->NewSpeedAcquisition = 0;
            } else {
                /* ������ֵ�˲� */
                pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
                /* �ٶȹ��� */
                if (wCaptBuf >= pHandle->MaxPeriod) {
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = 0;
                } else {
                    /* �����ٶ�:W_s16degree/s=F_hall*60��=W_e, ����ΪFOC�����µ�dpp=W_e*T_foc=����_foc */
                    /* dpp=����_e/n,n=F_mea/F_hall :  W_e=����_e/T_hall=����_e*f_hall   We*T_foc=����_foc */
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = (int16_t)(pHandle->PseudoFreqConv / wCaptBuf);
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
                    pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* ����ٶ�:hall_foc �����Ƕ� */
                }

                /* Update pointers to speed buffer */
                // pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* ˲ʱ���ٶ� */
                pHandle->CurrentSpeed = (HALL_LPF_FACTOR * pHandle->SensorSpeed[0] + (1 - HALL_LPF_FACTOR) * pHandle->CurrentSpeed_Last); /* ȡ��ͨ�˲� */
                pHandle->CurrentSpeed_Last = pHandle->CurrentSpeed;

                pHandle->SpeedFIFOIdx++;
                if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize) {
                    pHandle->SpeedFIFOIdx = 0u;
                }
                /* Indicate new speed acquisitions */
                pHandle->NewSpeedAcquisition = 1;
            }
            /* Reset the number of overflow occurred */
            pHandle->OVFCounter = 0u;
        }
    }

    return 0;
}

/**
 * @brief HALL���
 * @param *pHandleVoid
 * @retval None
 */
void *HALL_TIMx_UP_IRQHandler(void *pHandleVoid)
{
    HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid;

    if (pHandle->SensorIsReliable) {
        uint16_t hMaxTimerOverflow;
        pHandle->OVFCounter++;

        /* pscԽ��,���������������Խ�� */
        /* HallTimeout(ms):���ת����HALL�����źŵļ��  OvfFreq/psc:��ʱ���ļ������Ƶ�� */
        /* ���Ƶ�ʼ�����,���ת�ٵ�HALL�źż��֮����������,�����������,˵��ת��Ϊ0 */
        /* t(s) * f(cnt/s)=cnt */
        hMaxTimerOverflow = (uint16_t)(((uint32_t)pHandle->HallTimeout * pHandle->OvfFreq) / ((TIM5->PSC + 1) * 1000u));
        if (pHandle->OVFCounter >= hMaxTimerOverflow) {
            pHandle->_Super.hElSpeedDpp = 0;
            /* ����У׼��Ƕ� */
            HALL_Init_Electrical_Angle(pHandle);

            pHandle->OVFCounter = 0u;

            uint8_t bIndex;
            for (bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++) {
                pHandle->SensorSpeed[bIndex] = 0;
            }
            pHandle->BufferFilled = 0;
            pHandle->CurrentSpeed = 0;
            pHandle->SpeedFIFOIdx = 1;
            pHandle->ElSpeedSum = 0;
        }
    }

    return 0;
}

/**
 * @brief HALL�����жϻص�
 * @param *htim TIM5
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        HALL_TIMx_CC_IRQHandler(&HALL_M1);
    }
}
