/******************************************************************************
 *@brief  Encoder function
 *@author By Spotted Owl
 *@date     2025.05.07
 ******************************************************************************/
#include "bsp_encoder.h"
#include "tim.h"
#include "bsp_pid.h"
#include "bsp_spd.h"

/* 虚拟编码器 */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 = {
    /* 位置传感器的基类实例化 */
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* 极对数 */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* 最大可靠机械速度0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* 最小可靠机械速度0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* 最大速度异常次数 */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* 最大可靠机械加速度0.1Hz */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* 测量频率16kHz */
    },
    .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE, /* 速度采样频率(Hz) 500hz */
    .hTransitionSteps = (int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION / 1000.0),
};

/* 物理编码器 */
ENCODER_Handle_t ENCODER_M1 = {
    /* 位置传感器的基类实例化 */
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* 极对数 */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* 最大可靠机械速度0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* 最小可靠机械速度0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* 最大速度异常次数 */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* 最大可靠机械加速度0.1Hz  */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* 测量频率16kHz */
    },
    .PulseNumber = M1_ENCODER_PPR * 4,                        /* 编码器1000线数 */
    .RevertSignal = (FunctionalState)ENC_INVERT_SPEED,        /* 编码器信号是否翻转 */
    .SpeedSamplingFreq01Hz = 10 * MEDIUM_FREQUENCY_TASK_RATE, /* 速度采样频率(0.1Hz) 500hz */
    .SpeedBufferSize = ENC_AVERAGING_FIFO_DEPTH,              /* Speed buffer size */
};

/* 编码器对齐 */
EncAlign_Handle_t EncAlignCtrlM1 = {
    .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE, /* 500hz */
    .hFinalTorque = FINAL_I_ALIGNMENT,             /* 最终转矩319 */
    .hElAngle = ALIGNMENT_ANGLE_S16,               /* 90度 */
    .hDurationms = ALIGNMENT_DURATION,             /* 700ms */
    .bElToMecRatio = POLE_PAIR_NUM,                /* 极对数4 */
};

/**
 * @brief 初始化编码器
 * @param *pHandle 编码器对象
 * @retval None
 */
void ENC_Init(ENCODER_Handle_t *pHandle)
{
    uint8_t BufferSize;
    uint8_t Index;

    /* Reset counter */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /*Calculations of convenience*/
    pHandle->U32MAXdivPulseNumber = UINT32_MAX / (uint32_t)(pHandle->PulseNumber);
    pHandle->SpeedSamplingFreqHz = pHandle->SpeedSamplingFreq01Hz / 10u; // 采样频率,与REP_RATE--Fpwm有关

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* Erase speed buffer */
    BufferSize = pHandle->SpeedBufferSize;

    for (Index = 0u; Index < BufferSize; Index++) {
        pHandle->DeltaCapturesBuffer[Index] = 0;
    }

    pHandle->SensorIsReliable = true;
}

/**
 * @brief 复位编码器buff
 * @param *pHandle 编码器对象
 * @retval None
 */
void ENC_Clear(ENCODER_Handle_t *pHandle)
{
    uint8_t Index;
    for (Index = 0u; Index < pHandle->SpeedBufferSize; Index++) {
        pHandle->DeltaCapturesBuffer[Index] = 0;
    }
    pHandle->SensorIsReliable = true;
}

/**
 * @brief 计算电角度
 * @param *pHandle 编码器对象
 * @retval 电角度
 */
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle)
{
    int32_t wtemp1;
    int32_t wtemp2;
    int16_t htemp1;
    int16_t htemp2;

    /* U32MAXdivPulseNumber=UINT32_MAX/PulseNumber(编码器一圈定时器脉冲数)
     极对数bElToMecRatio  */
    /* 65536/pulse*cap */
    wtemp1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) * (int32_t)(pHandle->U32MAXdivPulseNumber);

    /* 电角度 */
    wtemp2 = wtemp1 * (int32_t)pHandle->_Super.bElToMecRatio;
    htemp1 = (int16_t)(wtemp2 / 65536);
    pHandle->_Super.hElAngle = htemp1;

    /* 机械角度 */
    htemp2 = (int16_t)(wtemp1 / 65536);
    pHandle->_Super.hMecAngle = htemp2;

    /*Returns rotor electrical angle*/
    return (htemp1);
}

/**
 * @brief 计算机械速度0.1Hz
 * @param *pHandle 编码器对象
 * @param *pMecSpeed01Hz 机械速度0.1Hz
 * @retval None
 */
bool ENC_CalcAvrgMecSpeed01Hz(ENCODER_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
{
    /* 平均机械转速0.1Hz */
    int32_t wOverallAngleVariation = 0;
    int32_t wtemp1;
    int32_t wtemp2;
    uint8_t bBufferIndex = 0u;
    bool bReliability = true;
    uint8_t bBufferSize = pHandle->SpeedBufferSize;
    uint32_t OverflowCntSample;
    uint32_t CntCapture;
    uint32_t directionSample;
    uint8_t OFbit = 0;

    CntCapture = __HAL_TIM_GET_COUNTER(&htim2);
    OverflowCntSample = pHandle->TimerOverflowNb;
    pHandle->TimerOverflowNb = 0;
    directionSample = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

    /* 编码器溢出 */
    if ((OverflowCntSample + OFbit) > 1) {
        pHandle->TimerOverflowError = true;
    }

    /* Δθ计算 */
    if (directionSample == true) {
        OverflowCntSample = (CntCapture > pHandle->PreviousCapture) ? 1 : 0; // 向下溢出
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
            (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) -
            ((int32_t)(OverflowCntSample) + OFbit) * (int32_t)(pHandle->PulseNumber);
    } else {
        OverflowCntSample = (CntCapture < pHandle->PreviousCapture) ? 1 : 0; // 向上溢出
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
            (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) +
            ((int32_t)(OverflowCntSample) + OFbit) * (int32_t)(pHandle->PulseNumber);
    }

    /* 平均机械转速0.1Hz,wtemp1 */
    for (bBufferIndex = 0u; bBufferIndex < bBufferSize; bBufferIndex++) {
        wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
    }
    /* w_m=θ*F_采样=θ*F_pwm */
    wtemp1 = wOverallAngleVariation * (int32_t)(pHandle->SpeedSamplingFreq01Hz);
    /* θ_cnt/PulseNumber=>抵掉计数,得到真正物理意义下的θ(本质:θ_cnt仅是计数值) */
    wtemp2 = (int32_t)(pHandle->PulseNumber) *
             (int32_t)(pHandle->SpeedBufferSize);
    wtemp1 /= wtemp2; /* 取速度平均 */
    *pMecSpeed01Hz = (int16_t)(wtemp1);

    /* 加速度计算 */
    pHandle->_Super.hMecAccel01HzP = (int16_t)(wtemp1 -
                                               pHandle->_Super.hAvrMecSpeed01Hz);

    pHandle->_Super.hAvrMecSpeed01Hz = (int16_t)wtemp1;

    /* 电角速度dpp=w_e/F_foc  hMeasurementFrequency=F_foc  bElToMecRatio=pole */
    /* dpp=θ_e*F_采样/F_foc */
    wtemp1 = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] *
             (int32_t)(pHandle->SpeedSamplingFreqHz) *
             (int32_t)pHandle->_Super.bElToMecRatio;
    wtemp1 /= (int32_t)(pHandle->PulseNumber);
    wtemp1 *= (int32_t)UINT16_MAX;
    wtemp1 /= (int32_t)(pHandle->_Super.hMeasurementFrequency);

    pHandle->_Super.hElSpeedDpp = (int16_t)wtemp1;

    /*last captured value update*/
    pHandle->PreviousCapture = CntCapture;
    /*Buffer index update*/
    pHandle->DeltaCapturesIndex++;

    if (pHandle->DeltaCapturesIndex == pHandle->SpeedBufferSize) {
        pHandle->DeltaCapturesIndex = 0u;
    }

    /* 检查速度是否可靠 */
    if (pHandle->TimerOverflowError) {
        bReliability = false;
        pHandle->SensorIsReliable = false;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;

    } else {
        bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeed01Hz);
    }

    return (bReliability);
}

/**
 * @brief 编码器对齐,复位计数
 * @param *pHandle 编码器对齐对象
 * @param hMecAngle 机械角度
 * @retval None
 */
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle)
{
    uint16_t hAngleCounts;
    uint16_t hMecAngleuint;

    /* 过零点处理 */
    if (hMecAngle < 0) {
        hMecAngle *= -1;
        hMecAngleuint = 65535u - (uint16_t)hMecAngle;
    } else {
        hMecAngleuint = (uint16_t)hMecAngle;
    }

    /* 机械角度对应的编码器计数 */
    hAngleCounts = (uint16_t)(((uint32_t)hMecAngleuint *
                               (uint32_t)pHandle->PulseNumber) /
                              65535u);

    /* 复位编码器 */
    TIM2->CNT = (uint16_t)(hAngleCounts);
}

/**
 * @brief 编码器中断溢出处理
 * @param *pHandleVoid
 * @retval None
 */
void *ENC_IRQHandler(void *pHandleVoid)
{
    ENCODER_Handle_t *pHandle = (ENCODER_Handle_t *)pHandleVoid;

    pHandle->TimerOverflowNb += 1u;

    return 0;
}
