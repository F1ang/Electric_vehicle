#include "bsp_hall.h"
#include "tim.h"
#include "bsp_spd.h"

/* HALL传感器对象实例化 */
HALL_Handle_t HALL_M1 = {
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* 极对数 */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* 最大可靠机械速度0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* 最小可靠机械速度0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* 最大测量错误次数 */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* 最大可靠机械加速度0.1Hz  */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* 测量频率16kHz */
    },
    .SensorPlacement = DEGREES_120,                          /* 120degrees */
    .PhaseShift = (int16_t)(HALL_PHASE_SHIFT * 65536 / 360), /* H1上升沿与A相最大BEMF的偏移 */
    .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,       /* 速度频率500hz */
    .SpeedBufferSize = HALL_AVERAGING_FIFO_DEPTH,            /* 滑动平均速度 */
    .TIMClockFreq = HALL_TIM_CLK,                            /* 时钟频率84MHz */
};

/**
 * @brief HALL初始化
 * @param *pHandle Hall传感器对象
 *  硬件复位模式：配置触发源为TI1F_ED（霍尔信号双边沿）htim->Instance->SMCR |= TIM_TS_TI1F_ED
    设置为从模式复位（每个霍尔边沿复位CNT）htim->Instance->SMCR |= TIM_SLAVEMODE_RESET;
    HAL_TIMEx_HallSensor_Start_IT(&htim5);
 * @retval None
 */
void HALL_Init(HALL_Handle_t *pHandle)
{
    /* 电频率 */
    uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz *
                                       pHandle->_Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz *
                                       pHandle->_Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;

    hMinReliableElSpeed01Hz /= 4u;
    hMaxReliableElSpeed01Hz *= 2u;

    /* 最大频率(溢出),clk/(arr+1) */
    pHandle->OvfFreq = (uint16_t)(pHandle->TIMClockFreq / 65536u);

    if (hMinReliableElSpeed01Hz == 0u) {
        /* Set fixed to 150 ms */
        pHandle->HallTimeout = 150u;
    } else {
        /* t=θ/w=10*10^3*60/360/x=1/f_cnt  电气速度w=x(圈/s):0.1hz->补偿10*10^3ms 圈=360°!!针对HALL的60°一次捕获中断!! */
        pHandle->HallTimeout = 10000u / (6u * hMinReliableElSpeed01Hz);
    }

    /* ms->s,clk/(arr+1)/f_cnt=psc */
    pHandle->HALLMaxRatio = (pHandle->HallTimeout * pHandle->OvfFreq) / 1000;

    /* psc*arr */
    pHandle->MaxPeriod = (pHandle->HALLMaxRatio) * 65536uL;

    /* 最大电气速度(圈/s)=f_wm(数值) */
    pHandle->SatSpeed = hMaxReliableElSpeed01Hz;

    /* 计算速度:W_s16degree/s=F_hall*60°=W_e, 量化为FOC周期下的dpp=W_e*T_foc=Δθ_foc */
    /* dpp=Δθ_e/n,n=F_mea/F_hall :  W_e=Δθ_e/T_hall=Δθ_e*f_hall   We*T_foc=Δθ_foc */
    /* F_mea=F_foc */
    pHandle->PseudoFreqConv = ((pHandle->TIMClockFreq / 6u) / (pHandle->_Super.hMeasurementFrequency)) * 65536u;

    /* 最小周期=Fclk/Fw_m,检测速度是否过快 10是因为单位0.1Hz,6是针对HALL的θ=60°一次捕获中断 w=360°*f_wm(圈/s的数值量化) = θ*Fw_m(仅需考虑60°) */
    pHandle->MinPeriod = ((10u * pHandle->TIMClockFreq) / 6u) / hMaxReliableElSpeed01Hz;

    /* n=测量频率(f_foc)/速度频率,用于dpp2=Δθ/n */
    pHandle->PWMNbrPSamplingFreq = (pHandle->_Super.hMeasurementFrequency /
                                    pHandle->SpeedSamplingFreqHz) -
                                   1u;

    /* 重置计数器 */
    pHandle->SensorIsReliable = true;

    /* 修改psc,并生成更新事件 */
    TIM5->PSC = pHandle->HALLMaxRatio;
    TIM5->EGR = TIM_EGR_UG;

    /* 清除中断标志 */
    TIM5->SR = 0;

    /* 设置更新事件源仅来自计数器溢出/下溢(异常) */
    __HAL_TIM_URS_ENABLE(&htim5);

    /* 使能捕获和溢出中断 */
    // HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); /* HALL捕获中断后面已有 */
    HAL_TIM_Base_Start_IT(&htim5);
    // HAL_TIM_Base_Start(&htim5);
    // HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_1);

    /* 重置计数器 */
    __HAL_TIM_SET_COUNTER(&htim5, HALL_COUNTER_RESET);

    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->SpeedBufferSize;

    for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++) {
        pHandle->SensorSpeed[bIndex] = 0;
    }
}

/**
 * @brief HALL清除
 * @param *pHandle Hall传感器对象
 * @retval None
 */
void HALL_Clear(HALL_Handle_t *pHandle)
{
    /* disable cc1 interrupt */
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC1);

    pHandle->RatioDec = false;
    pHandle->RatioInc = false;

    /* 传感器是否可靠 */
    pHandle->SensorIsReliable = true;

    /* 机械加速度0.1Hz */
    pHandle->_Super.hMecAccel01HzP = 0;

    /* 速度及捕获参数 */
    pHandle->FirstCapt = 0u;
    pHandle->BufferFilled = 0u;
    pHandle->OVFCounter = 0u;

    pHandle->CompSpeed = 0;
    pHandle->ElSpeedSum = 0;

    pHandle->Direction = POSITIVE;

    /* 滑动index */
    pHandle->SpeedFIFOIdx = 0u;

    /* 最新速度采集 */
    pHandle->NewSpeedAcquisition = 0;

    TIM5->PSC = pHandle->HALLMaxRatio;
    __HAL_TIM_SET_COUNTER(&htim5, HALL_COUNTER_RESET);
    __HAL_TIM_ENABLE(&htim5);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

    HALL_Init_Electrical_Angle(pHandle);
}

/**
 * @brief 预定位
 * @param *pHandle Hall传感器对象
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
 * @brief FOC插补法计算电角度
 * @param *pHandle Hall传感器对象
 * @retval None
 */
int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle)
{
    /* 高频任务f_foc */
    if (pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED) {
        pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp; /* 测量电角度dpp1 */
        pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;   /* 目标电角度 */
        //  pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed; /* hall_foc补偿dpp1+speed_foc补偿dpp2,dpp2均分补偿到多个pwm周期中*/
        pHandle->_Super.hElAngle += pHandle->AvrElSpeedDpp + pHandle->CompSpeed; /* hall_foc补偿dpp1+speed_foc补偿dpp2,dpp2均分补偿到多个pwm周期中*/
        pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
    } else {
        pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
    }

    return pHandle->_Super.hElAngle;
}

/**
 * @brief 中频任务,速度环500hz
 * @param *pHandle Hall传感器对象
 * @param *hMecSpeed01Hz 机械速度0.1Hz
 * @retval None
 */
bool HALL_CalcAvrgMecSpeed01Hz(HALL_Handle_t *pHandle, int16_t *hMecSpeed01Hz)
{
    /* 中频任务,速度环500hz */
    int16_t SpeedMeasAux;
    bool bReliability;

    /* 电角速度补偿 dpp1:hall_foc 补偿角度 dpp1=θ_e/n,n = f_foc/f_hall */
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
                    /* f_速度环 与 f_foc的角速度补偿dpp2 */
                    pHandle->TargetElAngle = pHandle->MeasuredElAngle;
                    pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
                    /* dpp2;f_hall的dpp1只补偿最小foc周期的电角度,f_speed和f_foc频率差异,导致speed到foc周期需要补偿电角度dpp2 */
                    /* 且dpp2均分补偿到多个foc(pwm)周期中 */
                    pHandle->CompSpeed = (int16_t)((int32_t)(pHandle->DeltaAngle) /
                                                   (int32_t)(pHandle->PWMNbrPSamplingFreq));

                    /* 瞬时速度或者滑动平均电角速度 */
                    *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp(pHandle);

                    /* W*360/f_测量=dpp,360~65536,电气速度=dpp*f_测量/pole , f_测量=f_foc */
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
 * @brief 计算平均电角速度
 * @param *pHandle Hall传感器对象
 * @retval 平均电角速度
 */
static int16_t HALL_CalcAvrgElSpeedDpp(HALL_Handle_t *pHandle)
{
    /* HALL捕获处理速度,进行滑动平均速度 */
    if (pHandle->NewSpeedAcquisition == 1) {

        if (pHandle->BufferFilled < pHandle->SpeedBufferSize) {
            /* 取瞬时速度 */
            pHandle->AvrElSpeedDpp = (int16_t)pHandle->CurrentSpeed;
        } else {
            /* 取平均速度 */
            pHandle->AvrElSpeedDpp = (int16_t)(pHandle->ElSpeedSum / (int32_t)(pHandle->SpeedBufferSize)); /* Average value */
        }

        /* Clear new speed acquisitions flag */
        pHandle->NewSpeedAcquisition = 0;
    }

    return pHandle->AvrElSpeedDpp;
}

/**
 * @brief HALL捕获中断
 * @param *pHandleVoid Hall传感器对象
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

        /* 电角度校准 */
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

        /* 跳过第一次捕获 */
        if (pHandle->FirstCapt == 0u) {
            pHandle->FirstCapt++;
            __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
        } else {
            /* 速度滑动平均index */
            if (pHandle->BufferFilled < pHandle->SpeedBufferSize) {
                pHandle->BufferFilled++;
            }

            /* 存储最新的速度获取值*/
            hHighSpeedCapture = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
            wCaptBuf = (uint32_t)hHighSpeedCapture;
            hPrscBuf = TIM5->PSC;

            /* 将溢出的数量加到计数器上 */
            wCaptBuf += (uint32_t)pHandle->OVFCounter * 0x10000uL;

            if (pHandle->OVFCounter != 0u) {
                /* 使用预分频器调整捕获 */
                uint16_t hAux;
                hAux = hPrscBuf + 1u;
                wCaptBuf *= hAux; // TIM频率下的计数

                /* 上一次已经修改psc,还是溢出,下一次暂不修改psc */
                if (pHandle->RatioInc) {
                    pHandle->RatioInc = false;
                } else {
                    if (TIM5->PSC < pHandle->HALLMaxRatio) /* 溢出降频率 */
                    {
                        new_psc = TIM5->PSC + 1;
                        __HAL_TIM_SET_PRESCALER(&htim5, new_psc);
                        pHandle->RatioInc = true;
                    }
                }
            } else {
                /* 上一次修改psc,且未溢出,这次暂不修改psc */
                if (pHandle->RatioDec) {
                    /* 计算周期 */
                    uint16_t hAux;
                    hAux = hPrscBuf + 2u; /* 影子寄存器,需等待更新事件--(PSC+1)+1 */
                    wCaptBuf *= hAux;

                    pHandle->RatioDec = false;
                } else /* 如果在上次捕获操作中未对分频器进行修改 */
                {
                    uint16_t hAux = hPrscBuf + 1u;
                    wCaptBuf *= hAux;

                    /* 捕获值小于阈值,说明计数频率过慢,转速较快,修改psc,提高计数频率 */
                    if (hHighSpeedCapture < LOW_RES_THRESHOLD) /* If capture range correct */
                    {
                        if (TIM5->PSC > 0u) /* or prescaler cannot be further reduced */
                        {
                            /* 避免psc=0,且标记这次修改了psc */
                            new_psc = TIM5->PSC - 1;
                            __HAL_TIM_SET_PRESCALER(&htim5, new_psc);
                            pHandle->RatioDec = true;
                        }
                    }
                }
            }

            /* 小于最小计数周期,说明超速 */
            if (wCaptBuf < pHandle->MinPeriod) {
                pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
                pHandle->NewSpeedAcquisition = 0;
            } else {
                /* 滑动均值滤波 */
                pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
                /* 速度过慢 */
                if (wCaptBuf >= pHandle->MaxPeriod) {
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = 0;
                } else {
                    /* 计算速度:W_s16degree/s=F_hall*60°=W_e, 量化为FOC周期下的dpp=W_e*T_foc=Δθ_foc */
                    /* dpp=Δθ_e/n,n=F_mea/F_hall :  W_e=Δθ_e/T_hall=Δθ_e*f_hall   We*T_foc=Δθ_foc */
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = (int16_t)(pHandle->PseudoFreqConv / wCaptBuf);
                    pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
                    pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* 电角速度:hall_foc 补偿角度 */
                }

                /* Update pointers to speed buffer */
                // pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* 瞬时电速度 */
                pHandle->CurrentSpeed = (HALL_LPF_FACTOR * pHandle->SensorSpeed[0] + (1 - HALL_LPF_FACTOR) * pHandle->CurrentSpeed_Last); /* 取低通滤波 */
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
 * @brief HALL溢出
 * @param *pHandleVoid
 * @retval None
 */
void *HALL_TIMx_UP_IRQHandler(void *pHandleVoid)
{
    HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid;

    if (pHandle->SensorIsReliable) {
        uint16_t hMaxTimerOverflow;
        pHandle->OVFCounter++;

        /* psc越大,允许的最大溢出次数越少 */
        /* HallTimeout(ms):最低转速下HALL两次信号的间隔  OvfFreq/psc:定时器的计数溢出频率 */
        /* 最低频率计数下,最低转速的HALL信号间隔之间的溢出次数,超出这个次数,说明转速为0 */
        /* t(s) * f(cnt/s)=cnt */
        hMaxTimerOverflow = (uint16_t)(((uint32_t)pHandle->HallTimeout * pHandle->OvfFreq) / ((TIM5->PSC + 1) * 1000u));
        if (pHandle->OVFCounter >= hMaxTimerOverflow) {
            pHandle->_Super.hElSpeedDpp = 0;
            /* 重新校准电角度 */
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
 * @brief HALL捕获中断回调
 * @param *htim TIM5
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        HALL_TIMx_CC_IRQHandler(&HALL_M1);
    }
}
