
#include "bsp_task.h"
#include "bsp_motor.h"
#include "tim.h"
#include "adc.h"
#include "bsp_log.h"
#include "bsp_encoder.h"
#include "bsp_foc.h"
#include "bsp_state_machine.h"
#include "bsp_enc_align.h"
#include "bsp_pid.h"
#include "bsp_mci.h"
#include "bsp_stc.h"
#include "bsp_pwmc.h"
#include "bsp_vss.h"
#include "bsp_r3f4.h"
#include "bsp_spd.h"
#include "bsp_elecmobile.h"
#include "bsp_hall.h"
#include "bsp_sto_pll.h"
#include "bsp_sto_cordic.h"
#include "bsp_revup.h"
#include "bsp_ramp.h"
#include "foc_sensorless.h"

uint8_t bMCBootCompleted = 0;
volatile uint16_t hBootCapDelayCounterM1 = 0;
volatile uint16_t hStopPermanencyCounterM1 = 0;
static bool SWO_transitionStartM1 = false; /* ִ��б�±�־ */

/**
 * @brief ״̬��
 * @retval None
 */
void TSK_MediumFrequencyTaskM1(void)
{
#if HFI_SMO_ENABLE == 0 && FLUX_SENSOR_ENABLE == 0
    State_t StateM1;
    int16_t wAux = 0;
#endif

    /* dpp2���� */
    Ele_Get_Handle_Break(&handle_break);
#if ENCODER_ENABLE
    (void)ENC_CalcAvrgMecSpeed01Hz(&ENCODER_M1, &wAux);
    StateM1 = STM_GetState(&STM);
    switch (StateM1) {
    case IDLE:
        if (EAC_GetRestartState(&EncAlignCtrlM1)) {
            EAC_SetRestartState(&EncAlignCtrlM1, false); /* Reset restart flag*/
        }

        if (STM.bStop_Motor == 1) {
            STM_NextState(&STM, IDLE_START);
        }

        break;

    case IDLE_START:
        if (EAC_IsAligned(&EncAlignCtrlM1) == false) {
            EAC_SetRestartState(&EncAlignCtrlM1, true); /* Set restart flag. Run after align*/
            STM_NextState(&STM, IDLE_ALIGNMENT);
            break;
        }
        /* ���ݳ�� */
        R3F4XX_TurnOnLowSides(pwmcHandle);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        STM_NextState(&STM, CHARGE_BOOT_CAP);
        break;

    case CHARGE_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1()) {
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START);
            STM_NextState(&STM, OFFSET_CALIB);
        }
        break;

    case OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle, CRC_EXEC)) {
            STM_NextState(&STM, CLEAR);
        }
        break;

    case CLEAR:
        ENC_Clear(&ENCODER_M1);
        if (STM_NextState(&STM, START) == true) {
            FOC_Clear(0);
            R3F4XX_SwitchOnPWM(pwmcHandle);
        }
        break;

    case IDLE_ALIGNMENT: /*  only for encoder*/
        R3F4XX_TurnOnLowSides(pwmcHandle);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        STM_NextState(&STM, ALIGN_CHARGE_BOOT_CAP);
        break;

    case ALIGN_CHARGE_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1()) {
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START); /* ׼����ȡoffset ���ߵ��� */
            STM_NextState(&STM, ALIGN_OFFSET_CALIB);
        }
        break;

    case ALIGN_OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle, CRC_EXEC)) {
            STM_NextState(&STM, ALIGN_CLEAR);
        }
        break;

    case ALIGN_CLEAR:
        FOCVars.bDriveInput = EXTERNAL;
        /* ����ʹ��->�����ٶȴ�����  */
        STC_SetSpeedSensor(pSTC, &VirtualSpeedSensorM1._Super);

        /* ����,ת��ģʽIq++ */
        EAC_StartAlignment(&EncAlignCtrlM1);
        if (STM_NextState(&STM, ALIGNMENT) == true) {
            FOC_Clear(0);
            R3F4XX_SwitchOnPWM(pwmcHandle);
        }
        break;

    case START:
        STM_NextState(&STM, START_RUN); /* only for sensored*/
        break;

    case ALIGNMENT:
        /* ִ��IF����ǿ��,Iq=0,Id=x,��=90��,ת�Ӵų����붨�Ӵų� */
        if (!EAC_Exec(&EncAlignCtrlM1)) {
            Curr_Components IqdRef;
            IqdRef.qI_Component1 = 0;
            IqdRef.qI_Component2 = STC_CalcTorqueReference(pSTC);
            FOCVars.Iqdref = IqdRef;
        } else {
            R3F4XX_SwitchOffPWM(pwmcHandle);
            STC_SetControlMode(pSTC, STC_SPEED_MODE);     /* ���������л��ٶ�ģʽ */
            STC_SetSpeedSensor(pSTC, &ENCODER_M1._Super); /* ��������->������ SpeednTorqCtrl_Handle_t */
            STM_NextState(&STM, ANY_STOP);
        }
        break;

    case START_RUN:
        /* �ٶ�/ת��б�²��� */
        // MCI_ExecTorqueRamp(oMCInterface, -3000, 1000); /* MaxPositiveTorque */
        MCI_ExecSpeedRamp(oMCInterface, 60, 100); /* MaxAppPositiveMecSpeed01Hz */

        FOC_CalcCurrRef(0);
        STM_NextState(&STM, RUN);

        /* ����ʼ�ٶ�/ת�ص�,������б�� */
        STC_ForceSpeedReferenceToCurrentSpeed(pSTC);
        MCI_ExecBufferedCommands(oMCInterface);
        break;

    case RUN:
        MCI_ExecBufferedCommands(oMCInterface);
        FOC_CalcCurrRef(0);
        break;

    case ANY_STOP:
        R3F4XX_SwitchOffPWM(pwmcHandle);
        FOC_Clear(0);
        // MPM_Clear((MotorPowMeas_Handle_t *)pMPM);
        TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
        STM_NextState(&STM, STOP);
        break;

    case STOP:
        if (TSK_StopPermanencyTimeHasElapsedM1()) {
            STM_NextState(&STM, STOP_IDLE);
        }
        break;

    case STOP_IDLE:
        STM_NextState(&STM, IDLE);
        break;

    default:
        break;
    }
#elif HALL_ENABLE
    (void)HALL_CalcAvrgMecSpeed01Hz(&HALL_M1, &wAux);
    StateM1 = STM_GetState(&STM);
    switch (StateM1) {
    case IDLE:
        if (STM.bStop_Motor == 1) {
            STM_NextState(&STM, IDLE_START);
        }
        break;

    case IDLE_START:
        HALL_Init(&HALL_M1); /* ��ʼ��HALL������ */
        R3F4XX_TurnOnLowSides(pwmcHandle);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        STM_NextState(&STM, CHARGE_BOOT_CAP);
        break;

    case CHARGE_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1()) {
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START);
            STM_NextState(&STM, OFFSET_CALIB);
        }
        break;

    case OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle, CRC_EXEC)) {
            STM_NextState(&STM, CLEAR);
        }
        break;

    case CLEAR:
        HALL_Clear(&HALL_M1); /* HALL�������������� */
        if (STM_NextState(&STM, START) == true) {
            FOC_Clear(0);
            R3F4XX_SwitchOnPWM(pwmcHandle);
        }
        break;

    case START:
        MCI_ExecSpeedRamp(oMCInterface, 100, 1000);
        STM_NextState(&STM, START_RUN);
        break;

    case START_RUN:
        FOC_CalcCurrRef(0);
        STM_NextState(&STM, RUN);

        STC_ForceSpeedReferenceToCurrentSpeed(pSTC);
        MCI_ExecBufferedCommands(oMCInterface);

        break;

    case RUN:
        MCI_ExecBufferedCommands(oMCInterface);
        FOC_CalcCurrRef(0);

        break;

    case ANY_STOP:
        HAL_TIM_Base_Stop_IT(&htim5);
        HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
        R3F4XX_SwitchOffPWM(pwmcHandle);
        FOC_Clear(0);
        TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
        STM_NextState(&STM, STOP);
        break;

    case STOP:
        if (TSK_StopPermanencyTimeHasElapsedM1()) {
            STM_NextState(&STM, STOP_IDLE);
        }
        break;

    case STOP_IDLE:
        STM_NextState(&STM, IDLE);
        break;

    default:
        break;
    }
#elif SENSORLESS_ENABLE
    int16_t hForcedMecSpeed01Hz;
    Curr_Components IqdRef;
    bool StartUpTransitionEnded;
    bool StartUpDoTransition;

    (void)STO_CR_CalcAvrgMecSpeed01Hz(&STO_CR_M1, &wAux);
    bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeed01Hz(&STO_PLL_M1, &wAux); /* STO_PLL�ٶȿɿ��� */
    StateM1 = STM_GetState(&STM);
    switch (StateM1) {
    case IDLE:
        if (STM.bStop_Motor == 1) {
            STM_NextState(&STM, IDLE_START);
        }
        break;

    case IDLE_START:
        R3F4XX_TurnOnLowSides(pwmcHandle);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        STM_NextState(&STM, CHARGE_BOOT_CAP);
        break;

    case CHARGE_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1()) {
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START);
            STM_NextState(&STM, OFFSET_CALIB);
        }
        break;

    case OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle, CRC_EXEC)) {
            STM_NextState(&STM, CLEAR);
        }
        break;

    case CLEAR:
        FOCVars.bDriveInput = EXTERNAL;
        STC_SetSpeedSensor(pSTC, &VirtualSpeedSensorM1._Super);                 /* ����ѡ�������ٶȴ����� */
        RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(oMCInterface)); /* ��ʼ������������״̬ */
        SWO_transitionStartM1 = false;                                          /* ������ʼ */
        STM.bSenLess_Handle = 0;

        STO_PLL_Clear(&STO_PLL_M1);
        STO_CR_Clear(&STO_CR_M1);
        if (STM_NextState(&STM, START) == true) {
            FOC_Clear(0);
            R3F4XX_SwitchOnPWM(pwmcHandle);
        }
        break;

    case START:
        /* 1.I/Fǿ�Ͻ׶�bStageCnt,������Զ���ת��һĿ������ */
        /* I/F�׶�:�����ȫ��ֹ->������ת(��δ�л����ջ��۲���) */
        if (!RUC_Exec(&RevUpControlM1)) {
            STM_FaultProcessing(&STM, MC_START_UP, 0); /* ����ʧ�� */
        } else {
            if (SWO_transitionStartM1 == false) {
                /* ת��Iq,I/Fǿ������(�л�����̬ʱ,ע��˫dqϵ����Iq����������) */
                IqdRef.qI_Component1 = STC_CalcTorqueReference(pSTC);
                IqdRef.qI_Component2 = FOCVars.UserIdref;
                FOCVars.Iqdref = IqdRef;
            }
        }

        /* 2.�����е�ٶ�,�����ٶȿɿ���(б��bTransitionEnded=0,���ɿ�;
        б��ʣ�����hTransitionRemainingsteps=0=>���bTransitionEnded=1,�ټ����ٶȿɿ���) */
        StartUpTransitionEnded = VSS_CalcAvrgMecSpeed01Hz(&VirtualSpeedSensorM1, &hForcedMecSpeed01Hz); /* ����dpp,����FOC������Ƕ� */

        /* 3.I/Fǿ��->�����ٶ��ڵ�ǰ�ٶ�������Χ��->�㷨����IsAlgorithmConverged=1->����ִ��б��bTransitionStarted=1 */
        /* W_est��W_sto���㷨�����ж� */
        StartUpDoTransition = VSS_SetStartTransition(&VirtualSpeedSensorM1, STO_PLL_IsObserverConverged(&STO_PLL_M1, hForcedMecSpeed01Hz));

        /* 4.б��ƽ���ջ��׶�(Ŀ��:��ֹIqͻ��,SMO��������BEMF�ź�,�п�����ɹ۲�BEMF��λ����,SMO���ܱջ�) */
        /* ����ǿ�ϵ�SMO����->б��ƽ��Iq->�ջ��۲��� */
        if (VSS_IsTransitionOngoing(&VirtualSpeedSensorM1)) {
            if (SWO_transitionStartM1 == false) {
                int16_t Iq = 0;
                Curr_Components StatorCurrent = MCM_Park(FOCVars.Ialphabeta, SPD_GetElAngle(&STO_PLL_M1._Super));
                Iq = StatorCurrent.qI_Component1;

                /* 5.б�°�FOCVars.Iqdref.qI_Component1 ������ Iq => �л��ջ��۲������� */
                /* б����ǰ���RampRemainingstep=0 => ��ִ��REMNG_Calc => FoCVars.Iqdref.qI_Component1�̶� */
                REMNG_Init(pREMNG);
                REMNG_ExecRamp(pREMNG, Iq, 0);
                REMNG_ExecRamp(pREMNG, FOCVars.Iqdref.qI_Component1, TRANSITION_DURATION);
                SWO_transitionStartM1 = true;
            } else {
                /* 6.FOCƵ�����������REMNG_Calc */
            }
        } else {
            if (SWO_transitionStartM1 == true) {
                SWO_transitionStartM1 = false;
            }
        }

        if (StartUpDoTransition == false) {
            StartUpTransitionEnded = true;
        }

        /* 7.�ٶȿɿ�,�л����������� */
        if (StartUpTransitionEnded == true) {
            PID_SetIntegralTerm(pPIDSpeed, (int32_t)(FOCVars.Iqdref.qI_Component1 * PID_GetKIDivisor(pPIDSpeed) / PID_SPEED_INTEGRAL_INIT_DIV));
            STM_NextState(&STM, START_RUN);
        }
        break;

    case START_RUN:
        FOCVars.hTeref = FOCVars.Iqdref.qI_Component1;
        MCI_ExecTorqueRamp(oMCInterface, 220, 1000);
        // MCI_ExecSpeedRamp(oMCInterface, 300, 1000);
        STC_SetSpeedSensor(pSTC, &STO_PLL_M1._Super); /* ����ѡ��STO_PLL�ٶȴ����� */
        FOC_CalcCurrRef(0);
        STM_NextState(&STM, RUN);
        STC_ForceSpeedReferenceToCurrentSpeed(pSTC);
        MCI_ExecBufferedCommands(oMCInterface);
        break;

    case RUN:
        MCI_ExecBufferedCommands(oMCInterface);
        FOC_CalcCurrRef(0);

#if HANDLE_BREAK
        if (STM.bSenLess_Handle == 0) {
            if (STO_PLL_M1._Super.hAvrMecSpeed01Hz > 290 && STO_PLL_M1._Super.hAvrMecSpeed01Hz < 310) {
                STM.bSenLess_Handle = 1;
            }
        }
#endif

        if (!IsSpeedReliable) {
            STM_FaultProcessing(&STM, MC_SPEED_FDBK, 0);
        }
        break;

    case ANY_STOP:
        R3F4XX_SwitchOffPWM(pwmcHandle);
        FOC_Clear(0);
        TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
        STM_NextState(&STM, STOP);
        break;

    case STOP:
        if (TSK_StopPermanencyTimeHasElapsedM1()) {
            STM_NextState(&STM, STOP_IDLE);
        }
        break;

    case STOP_IDLE:
        STC_SetSpeedSensor(pSTC, &VirtualSpeedSensorM1._Super);
        VSS_Init(&VirtualSpeedSensorM1);
        STM_NextState(&STM, IDLE);
        break;

    default:
        break;
    }

#elif HFI_SMO_ENABLE || SENSORLESS_ENABLE || FLUX_SENSOR_ENABLE
    MOTOR_State_t state;

    FOC_Sensorless_Speed_Cale();
    state = FOC_Sensorless_GetState();
    switch (state) {
    case MOTOR_IDLE:
        if (FOC_Sensorless_GetStopFlag() == 1) {
            FOC_Sensorless_NextState(MOTOR_IDLE_START);
        }
        break;

    case MOTOR_IDLE_START:
        R3F4XX_TurnOnLowSides(pwmcHandle);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        FOC_Sensorless_NextState(MOTOR_BOOT_CAP);
        break;

    case MOTOR_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1()) {
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START);
            FOC_Sensorless_NextState(MOTOR_OFFSET_CALIB);
        }
        break;

    case MOTOR_OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle, CRC_EXEC)) {
            FOC_Sensorless_NextState(MOTOR_CLEAR);
        }
        break;

    case MOTOR_CLEAR:
        FOC_Sensorless_Clear();
        R3F4XX_SwitchOnPWM(pwmcHandle);
        FOC_Sensorless_NextState(MOTOR_START);
        break;

    case MOTOR_START:
        FOC_Sensorless_NextState(MOTOR_SL_RUN);
        break;

    case MOTOR_SL_RUN:
        /* ��Ƶ����ִ��FOC�㷨 */
        break;

    default:
        break;
    }

#endif
}

/**
 * @brief �Ծٵ��ݳ��
 * @param hTickCount ̧������
 * @retval None
 */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
    hBootCapDelayCounterM1 = hTickCount;
}

/**
 * @brief �����ʱ�����Ƿ����
 * @retval bool true:���� false:δ����
 */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
    bool retVal = false;
    if (hBootCapDelayCounterM1 == 0) {
        retVal = true;
    }
    return (retVal);
}

/**
 * @brief STOP״̬��ά��ʱ��
 * @param hTickCount ֹͣ����ʱ��
 * @retval None
 */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
    hStopPermanencyCounterM1 = hTickCount;
}

/**
 * @brief ����STOP״̬��
 * @retval bool true:���� false:δ����
 */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
    bool retVal = false;
    if (hStopPermanencyCounterM1 == 0) {
        retVal = true;
    }
    return (retVal);
}

/**
 * @brief ��ƵFOC�㷨����
 * @retval uint8_t 0
 */
uint8_t TSK_HighFrequencyTask(void)
{
#if HFI_SMO_ENABLE == 0 && FLUX_SENSOR_ENABLE == 0
    uint8_t bMotorNbr = 0;
    uint16_t hFOCreturn;

#if ENCODER_ENABLE
    /* ����ʹ�������ٶȴ����� VirtualSpeedSensorM1._Super,���Դ˴���Ч */
    ENC_CalcAngle(&ENCODER_M1); /* if not sensorless then 2nd parameter is MC_NULL*/
#elif HALL_ENABLE
    HALL_CalcElAngle(&HALL_M1); // �����Ƕ�
#elif SENSORLESS_ENABLE
    uint16_t hState;
    Observer_Inputs_t STO_Inputs;
    Observer_Inputs_t STO_aux_Inputs;

    STO_aux_Inputs.Valfa_beta = FOCVars.Valphabeta;
    STO_Inputs.Valfa_beta = FOCVars.Valphabeta;

    /* 1.ִ��б�� */
    if (SWO_transitionStartM1 == true) {
        if (!REMNG_RampCompleted(pREMNG)) {
            FOCVars.Iqdref.qI_Component1 = REMNG_Calc(pREMNG);
        }
    }
#endif

    hFOCreturn = FOC_CurrController(0);

#if SENSORLESS_ENABLE
    if (hFOCreturn == MC_FOC_DURATION) {
        STM_FaultProcessing(&STM, MC_FOC_DURATION, 0);
    } else {
        /* 2.���I/F�ĵ�һ�׶� */
        bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);

        /* 3.STO�����ǶȺ͵���ٶ�,���⴫��������Ч */
        STO_Inputs.Ialfa_beta = FOCVars.Ialphabeta;
        STO_Inputs.Vbus = handle_break.vbus_value >> 1;
        STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
        STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1);

        /* 4.I/F�ĵ�1�������,����STO+PLL */
        if (IsAccelerationStageReached == false) {
            STO_ResetPLL(&STO_PLL_M1);
        }

        /* 5.����+STO_PLL���������Ƕ� */
        hState = STM_GetState(&STM);
        if ((hState == START) || (hState == START_RUN)) {
            int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
            /* 6.���������Ƕ�;�Ż���Ƕ�У׼ */
            /* б��ʣ�����hTransitionRemainingsteps=0=>���bTransitionEnded=1=>VSS_IsTransitionOngoing����0
            =>SWO_transitionStartM1=0=>��ִ��REMNG_Calc */
            VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
        }

        /* 7.���ģ��,�����ع���Ƕȡ�����ٶȡ����� */
        STO_aux_Inputs.Ialfa_beta = FOCVars.Ialphabeta;
        STO_aux_Inputs.Vbus = handle_break.vbus_value >> 1;
        STO_CR_CalcElAngle(&STO_CR_M1, &STO_aux_Inputs);
        STO_CR_CalcAvrgElSpeedDpp(&STO_CR_M1);
    }
#endif

    return bMotorNbr;
#else
    FOC_CurrController(0);
    return 0;
#endif
}

/**
 * @brief ��ȫ��������
 * @retval None
 */
void TSK_SafetyTask(void)
{
    if (bMCBootCompleted == 1) {
        TSK_SafetyTask_PWMOFF(0);
        // RCM_ExecUserConv();
    }
}

/**
 * @brief ��ȫ����-�����
 * @param bMotor �����
 * @retval None
 */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
    uint16_t CodeReturn;

    // CodeReturn = NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns
    //                                                             MC_OVER_TEMP or MC_NO_ERROR */
    // CodeReturn |= PWMC_CheckOverCurrent(
    //     pwmcHandle[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for
    //                             STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */

    // if (bMotor == 0) {
    //     CodeReturn |= RVBS_CalcAvVbus(pBusSensorM1);
    // }

    STM_FaultProcessing(&STM, CodeReturn, ~CodeReturn); /* Update the STM according error code */
    switch (STM_GetState(&STM))                         /* Acts on PWM outputs in case of faults */
    {
    case FAULT_NOW:
        PWMC_SwitchOffPWM(pwmcHandle);
        FOC_Clear(bMotor);
        // MPM_Clear((MotorPowMeas_Handle_t *)pMPM);
        break;
    case FAULT_OVER:
        PWMC_SwitchOffPWM(pwmcHandle);
        break;
    default:
        break;
    }
}

/**
 * @brief Hardware Fault����
 * @retval None
 */
void TSK_HardwareFaultTask(void)
{
    /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

    /* USER CODE END TSK_HardwareFaultTask 0 */

    R3F4XX_SwitchOffPWM(pwmcHandle);
    STM_FaultProcessing(&STM, MC_SW_ERROR, 0);
    /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

    /* USER CODE END TSK_HardwareFaultTask 1 */
}
