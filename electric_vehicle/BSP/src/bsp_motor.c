/******************************************************************************
 *@brief  motor start function
 *@author By Spotted Owl
 *@date     2025.05.08
 ******************************************************************************/
#include "bsp_motor.h"
#include "tim.h"
#include "adc.h"
#include "bsp_encoder.h"
#include "bsp_foc.h"
#include "bsp_state_machine.h"
#include "bsp_enc_align.h"
#include "bsp_pid.h"
#include "bsp_mci.h"
#include "bsp_stc.h"
#include "bsp_task.h"
#include "bsp_pwmc.h"
#include "bsp_vss.h"
#include "bsp_r3f4.h"
#include "bsp_spd.h"
#include "bsp_log.h"
#include "bsp_elecmobile.h"
#include "bsp_hall.h"
#include "bsp_sto_pll.h"
#include "bsp_sto_cordic.h"
#include "bsp_revup.h"
#include "bsp_ramp.h"
#include "foc_sensorless.h"

/* FOC��صĻص������Ͳ��� */
PWMC_Handle_t *pwmcHandle;

SpeednTorqCtrl_Handle_t *pSTC;
PID_Handle_t *pPIDSpeed;
PID_Handle_t *pPIDIq;
PID_Handle_t *pPIDId;
FOCVars_t FOCVars;
MCI_Handle_t *oMCInterface;
MCI_Handle_t Mci;
STM_Handle_t STM;
CircleLimitation_Handle_t *pCLM;
DAC_UI_Handle_t *pDAC = (uint16_t)(0x0000u);
RampExtMngr_Handle_t *pREMNG;

static volatile uint16_t hMFTaskCounterM1 = 0;

#if PRINT_TEST
volatile uint16_t cnt_500ms = 0;
#endif

/* FOC��صĻص������Ͳ���+ADCУ׼��������ز��� */
PWMC_R3_F4_Handle_t PWM_Handle_M1 = {
    /* ����ʵ���� */
    {
        .pFctGetPhaseCurrents = &R3F4XX_GetPhaseCurrents,
        .pFctSwitchOffPwm = &R3F4XX_SwitchOffPWM,
        .pFctSwitchOnPwm = &R3F4XX_SwitchOnPWM,
        .pFctCurrReadingCalib = &R3F4XX_CurrentReadingCalibration,
        .pFctTurnOnLowSides = &R3F4XX_TurnOnLowSides,
        .pFctSetADCSampPointSect1 = &R3F4XX_SetADCSampPointSect1,
        .pFctSetADCSampPointSect2 = &R3F4XX_SetADCSampPointSect2,
        .pFctSetADCSampPointSect3 = &R3F4XX_SetADCSampPointSect3,
        .pFctSetADCSampPointSect4 = &R3F4XX_SetADCSampPointSect4,
        .pFctSetADCSampPointSect5 = &R3F4XX_SetADCSampPointSect5,
        .pFctSetADCSampPointSect6 = &R3F4XX_SetADCSampPointSect6,
        .hT_Sqrt3 = (PWM_PERIOD_CYCLES * SQRT3FACTOR) / 16384u,
        .hOffCalibrWaitTimeCounter = 0,
        .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS) / 1000),
        .bTurnOnLowSidesAction = false,
        .hPWMperiod = PWM_PERIOD_CYCLES,
        .hDeadTime = DEADTIME_NS,
        .hTafter = TW_AFTER,
        .hTbefore = TW_BEFORE,
        .Tw = MAX_TWAIT,
    },

    /* ADCУ׼��������ز��� */
    .wPhaseAOffset = 0,
    .wPhaseBOffset = 0,
    .wPhaseCOffset = 0,
    .wADC1Channel = 0,
    .wADC2Channel = 0,
    .wADCTriggerUnSet = 0, // ADC1->CR2 & 0xFFC0FFFFu
    .wADCTriggerSet = 0,   // pHandle->wADCTriggerUnSet | 0x001E0000u
    .Half_PWMPeriod = PWM_PERIOD_CYCLES / 2u,
    .bIndex = 0,
    .bSoFOC = 0,
};

CircleLimitation_Handle_t CircleLimitationM1 = {
    .MaxModule = MAX_MODULE,
    .Circle_limit_table = MMITABLE,
    .Start_index = START_INDEX,
};

/**
 * @brief ��ʼ���������
 * @retval None
 */
void MX_MotorControl_Init(void)
{
    /* ��������ϵͳ�δ�ʱ����ʱ��Ϊ 500 us */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 2000);

    /* ����ʵ������ADC���� */
    MCboot();

    /* DAC��� */
    pDAC = &DAC_UI_Params;
    pDAC->_Super = UI_Params;
    UI_DACInit(&pDAC->_Super);
}

/**
 * @brief ADC������CCR4����
 * @retval None
 */
void MCboot(void)
{
    /* ʵ�������������� */
    pCLM = &CircleLimitationM1;

    /* ADC��ʼ������,��ַָ��:������Ļ����ַ,�ɷ�������������Ա */
    pwmcHandle = &PWM_Handle_M1._Super;
    R3F4XX_Init(&PWM_Handle_M1);

    /* ״̬����ʼ�� */
    STM_Init(&STM);

    /* PID��ʼ�� */
    PID_HandleInit(&PIDSpeedHandle_M1);
    pPIDSpeed = &PIDSpeedHandle_M1;
    pSTC = &SpeednTorqCtrlM1;

#if ENCODER_ENABLE
    /* ��������ʼ�� */
    ENC_Init(&ENCODER_M1);

    /* ��������������ʼ�� */
    EAC_Init(&EncAlignCtrlM1, pSTC, &VirtualSpeedSensorM1, &ENCODER_M1);

    /* �ٶȺ�ת�ض����ʼ�� */
    STC_Init(pSTC, pPIDSpeed, &ENCODER_M1._Super);

    /* ����dc��ѹ�����ʼ�� */
    VSS_Init(&VirtualSpeedSensorM1);
#elif HALL_ENABLE
    HALL_Init(&HALL_M1);

    /* �ٶȺ�ת�ض����ʼ�� */
    STC_Init(pSTC, pPIDSpeed, &HALL_M1._Super);

    /* ����dc��ѹ�����ʼ�� */
    VSS_Init(&VirtualSpeedSensorM1);
#elif SENSORLESS_ENABLE
    /* d-qϵ,���綯���ع�,�и��� */
    STO_PLL_Init(&STO_PLL_M1);

    /* �ٶȺ�ת�ض����ʼ�� */
    STC_Init(pSTC, pPIDSpeed, &STO_PLL_M1._Super);

    /* ��ת����ϵ,�����ع�,�����͵��ٽ׶� */
    STO_CR_Init(&STO_CR_M1);

    /* ����dc��ѹ�����ʼ�� */
    VSS_Init(&VirtualSpeedSensorM1);

    /* ������������ʼ�� */
    RUC_Init(&RevUpControlM1, pSTC, &VirtualSpeedSensorM1, &STO_M1, pwmcHandle);

    /* б�¶����ʼ�� */
    pREMNG = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG);
#elif HFI_SMO_ENABLE || OPEN_SENSORLESS_MODE || FLUX_SENSOR_ENABLE
    FOC_Sensorless_Init();
#endif

    /* ������PID������ʼ�� */
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
    pPIDIq = &PIDIqHandle_M1;
    pPIDId = &PIDIdHandle_M1;

    /* ��λFOC���� */
    FOC_Clear(0);
    FOCVars.bDriveInput = EXTERNAL;
    FOCVars.Iqdref = STC_GetDefaultIqdref(pSTC);
    FOCVars.UserIdref = STC_GetDefaultIqdref(pSTC).qI_Component2;

    /* ������ƶ����ʼ�� */
    oMCInterface = &Mci; /* ��ַָ��,��ֹ��� */
    MCI_Init(oMCInterface, &STM, pSTC, &FOCVars);

    /* �ٶ�б�²��� */
    // MCI_ExecSpeedRamp(oMCInterface, STC_GetMecSpeedRef01HzDefault(pSTC), 10000);

    /* ת��б�²��� */
    // MCI_ExecTorqueRamp(oMCInterface, -4997, 2000); /* ��ֵNOMINAL_CURRENT */
}

/**
 * @brief ADC��ʼ������
 * @param *pHandle
 * @retval None
 */
void R3F4XX_Init(PWMC_R3_F4_Handle_t *pHandle)
{
    uint32_t trigOut;

    /* ֹͣ��ʱ������ */
    HAL_TIM_Base_Stop(&htim8);

    /* Ԥ���ع��� */
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM8->CCMR2 |= TIM_CCMR2_OC4PE;

    /* �ֶ����ɸ����¼� */
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);

    /* ���Ķ���ģʽ���� */
    __HAL_TIM_SET_COUNTER(&htim8, pHandle->Half_PWMPeriod - 1);

    /* ʹ��ADC1 ADC2 */
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    /* ʹ��ADC1ע���ж� */
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    /* ����ת�����г���Ϊ3 */
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 |= (2 << ADC_SQR1_L_Pos);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&handle_break.dma_adc_buff[0], 3);
    LowPassFilter_Init(&handle_power_filter, 0.8f, 0.0f);

    /* ע�봥��ʧ��  JEXTEN = 00b (Disable), JEXTSEL = 0000b (TIM8_CC4 "dummy") */
    pHandle->wADCTriggerUnSet = ADC1->CR2 & 0xFFC0FFFFu;

    /* ע�봥��ʹ�� JEXTEN = 01b (Enable), JEXTSEL = 1110b (TIM8_CC4) */
    pHandle->wADCTriggerSet = pHandle->wADCTriggerUnSet | 0x001E0000u;

    /* TIM2ͬ��TIM8��ʼ��ʱ */
    trigOut = TIM2->CR2 & TIM_CR2_MMS;
    htim2.Instance->CR2 |= TIM_TRGO_UPDATE; /* TRGO����Դ����ΪUPDATE�¼� */
    TIM2->EGR = TIM_EGR_UG;                 /* UPDATE�¼� */
    htim2.Instance->CR2 = (htim2.Instance->CR2 & ~TIM_CR2_MMS_Msk) | trigOut;
}

/**
 * @brief 2khz=500us��Ƶ����
 * @retval None
 */
void MC_RunMotorControlTasks(void)
{
    MC_Scheduler();
}

/**
 * @brief ��Ƶ����
 * @retval None
 */
void MC_Scheduler(void)
{
    hMFTaskCounterM1++;
    if (hMFTaskCounterM1 >= 4u) {
        hMFTaskCounterM1 = 0u;
        // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_RESET);
        TSK_MediumFrequencyTaskM1(); // ִ���е�Ƶ�ʵ����� 2ms=500hz
    }

    if (hBootCapDelayCounterM1 > 0u) {
        hBootCapDelayCounterM1--;
    }

    if (hStopPermanencyCounterM1 > 0u) {
        hStopPermanencyCounterM1--;
    }

#if PRINT_TEST
    if (cnt_500ms < 1000u)
        cnt_500ms++;
    // printf("%d,%d\n", pSTC->SpeedRef01HzExt, pSTC->SPD->hAvrMecSpeed01Hz);
    // printf("%d\n", PWM_Handle_M1._Super.hSector);
    // printf("%d\n", pwmcHandle->hIa); /* ���ڸ�Ƶ������,���λ��ƽ�� */
    // printf("%d,%d,%d\n", pwmcHandle->hIa, pwmcHandle->hIb, pwmcHandle->hIc);
#endif
}
