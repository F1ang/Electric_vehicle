/******************************************************************************
 *@brief  Encoder function
 *@author By Spotted Owl
 *@date     2025.05.07
 ******************************************************************************/
#include "bsp_encoder.h"
#include "tim.h"
#include "bsp_pid.h"
#include "bsp_spd.h"

/* ��������� */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 = {
    /* λ�ô������Ļ���ʵ���� */
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* ������ */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* ���ɿ���е�ٶ�0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* ��С�ɿ���е�ٶ�0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* ����ٶ��쳣���� */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* ���ɿ���е���ٶ�0.1Hz */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* ����Ƶ��16kHz */
    },
    .hSpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE, /* �ٶȲ���Ƶ��(Hz) 500hz */
    .hTransitionSteps = (int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION / 1000.0),
};

/* ��������� */
ENCODER_Handle_t ENCODER_M1 = {
    /* λ�ô������Ļ���ʵ���� */
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* ������ */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* ���ɿ���е�ٶ�0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* ��С�ɿ���е�ٶ�0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* ����ٶ��쳣���� */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* ���ɿ���е���ٶ�0.1Hz  */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* ����Ƶ��16kHz */
    },
    .PulseNumber = M1_ENCODER_PPR * 4,                        /* ������1000���� */
    .RevertSignal = (FunctionalState)ENC_INVERT_SPEED,        /* �������ź��Ƿ�ת */
    .SpeedSamplingFreq01Hz = 10 * MEDIUM_FREQUENCY_TASK_RATE, /* �ٶȲ���Ƶ��(0.1Hz) 500hz */
    .SpeedBufferSize = ENC_AVERAGING_FIFO_DEPTH,              /* Speed buffer size */
};

/* ���������� */
EncAlign_Handle_t EncAlignCtrlM1 = {
    .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE, /* 500hz */
    .hFinalTorque = FINAL_I_ALIGNMENT,             /* ����ת��319 */
    .hElAngle = ALIGNMENT_ANGLE_S16,               /* 90�� */
    .hDurationms = ALIGNMENT_DURATION,             /* 700ms */
    .bElToMecRatio = POLE_PAIR_NUM,                /* ������4 */
};

/**
 * @brief ��ʼ��������
 * @param *pHandle ����������
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
    pHandle->SpeedSamplingFreqHz = pHandle->SpeedSamplingFreq01Hz / 10u; // ����Ƶ��,��REP_RATE--Fpwm�й�

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
 * @brief ��λ������buff
 * @param *pHandle ����������
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
 * @brief �����Ƕ�
 * @param *pHandle ����������
 * @retval ��Ƕ�
 */
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle)
{
    int32_t wtemp1;
    int32_t wtemp2;
    int16_t htemp1;
    int16_t htemp2;

    /* U32MAXdivPulseNumber=UINT32_MAX/PulseNumber(������һȦ��ʱ��������)
     ������bElToMecRatio  */
    /* 65536/pulse*cap */
    wtemp1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) * (int32_t)(pHandle->U32MAXdivPulseNumber);

    /* ��Ƕ� */
    wtemp2 = wtemp1 * (int32_t)pHandle->_Super.bElToMecRatio;
    htemp1 = (int16_t)(wtemp2 / 65536);
    pHandle->_Super.hElAngle = htemp1;

    /* ��е�Ƕ� */
    htemp2 = (int16_t)(wtemp1 / 65536);
    pHandle->_Super.hMecAngle = htemp2;

    /*Returns rotor electrical angle*/
    return (htemp1);
}

/**
 * @brief �����е�ٶ�0.1Hz
 * @param *pHandle ����������
 * @param *pMecSpeed01Hz ��е�ٶ�0.1Hz
 * @retval None
 */
bool ENC_CalcAvrgMecSpeed01Hz(ENCODER_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
{
    /* ƽ����еת��0.1Hz */
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

    /* ��������� */
    if ((OverflowCntSample + OFbit) > 1) {
        pHandle->TimerOverflowError = true;
    }

    /* ���ȼ��� */
    if (directionSample == true) {
        OverflowCntSample = (CntCapture > pHandle->PreviousCapture) ? 1 : 0; // �������
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
            (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) -
            ((int32_t)(OverflowCntSample) + OFbit) * (int32_t)(pHandle->PulseNumber);
    } else {
        OverflowCntSample = (CntCapture < pHandle->PreviousCapture) ? 1 : 0; // �������
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
            (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) +
            ((int32_t)(OverflowCntSample) + OFbit) * (int32_t)(pHandle->PulseNumber);
    }

    /* ƽ����еת��0.1Hz,wtemp1 */
    for (bBufferIndex = 0u; bBufferIndex < bBufferSize; bBufferIndex++) {
        wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
    }
    /* w_m=��*F_����=��*F_pwm */
    wtemp1 = wOverallAngleVariation * (int32_t)(pHandle->SpeedSamplingFreq01Hz);
    /* ��_cnt/PulseNumber=>�ֵ�����,�õ��������������µĦ�(����:��_cnt���Ǽ���ֵ) */
    wtemp2 = (int32_t)(pHandle->PulseNumber) *
             (int32_t)(pHandle->SpeedBufferSize);
    wtemp1 /= wtemp2; /* ȡ�ٶ�ƽ�� */
    *pMecSpeed01Hz = (int16_t)(wtemp1);

    /* ���ٶȼ��� */
    pHandle->_Super.hMecAccel01HzP = (int16_t)(wtemp1 -
                                               pHandle->_Super.hAvrMecSpeed01Hz);

    pHandle->_Super.hAvrMecSpeed01Hz = (int16_t)wtemp1;

    /* ����ٶ�dpp=w_e/F_foc  hMeasurementFrequency=F_foc  bElToMecRatio=pole */
    /* dpp=��_e*F_����/F_foc */
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

    /* ����ٶ��Ƿ�ɿ� */
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
 * @brief ����������,��λ����
 * @param *pHandle �������������
 * @param hMecAngle ��е�Ƕ�
 * @retval None
 */
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle)
{
    uint16_t hAngleCounts;
    uint16_t hMecAngleuint;

    /* ����㴦�� */
    if (hMecAngle < 0) {
        hMecAngle *= -1;
        hMecAngleuint = 65535u - (uint16_t)hMecAngle;
    } else {
        hMecAngleuint = (uint16_t)hMecAngle;
    }

    /* ��е�Ƕȶ�Ӧ�ı��������� */
    hAngleCounts = (uint16_t)(((uint32_t)hMecAngleuint *
                               (uint32_t)pHandle->PulseNumber) /
                              65535u);

    /* ��λ������ */
    TIM2->CNT = (uint16_t)(hAngleCounts);
}

/**
 * @brief �������ж��������
 * @param *pHandleVoid
 * @retval None
 */
void *ENC_IRQHandler(void *pHandleVoid)
{
    ENCODER_Handle_t *pHandle = (ENCODER_Handle_t *)pHandleVoid;

    pHandle->TimerOverflowNb += 1u;

    return 0;
}
