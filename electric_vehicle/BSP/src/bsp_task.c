
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
static bool SWO_transitionStartM1 = false; /* 执行斜坡标志 */

/**
 * @brief 状态机
 * @retval None
 */
void TSK_MediumFrequencyTaskM1(void)
{
#if HFI_SMO_ENABLE == 0 && FLUX_SENSOR_ENABLE == 0
    State_t StateM1;
    int16_t wAux = 0;
#endif

    /* dpp2计算 */
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
        /* 电容充电 */
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
            PWMC_CurrentReadingCalibr(pwmcHandle, CRC_START); /* 准备读取offset 相线电流 */
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
        /* 对齐使用->虚拟速度传感器  */
        STC_SetSpeedSensor(pSTC, &VirtualSpeedSensorM1._Super);

        /* 对齐,转矩模式Iq++ */
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
        /* 执行IF对齐强拖,Iq=0,Id=x,θ=90°,转子磁场对齐定子磁场 */
        if (!EAC_Exec(&EncAlignCtrlM1)) {
            Curr_Components IqdRef;
            IqdRef.qI_Component1 = 0;
            IqdRef.qI_Component2 = STC_CalcTorqueReference(pSTC);
            FOCVars.Iqdref = IqdRef;
        } else {
            R3F4XX_SwitchOffPWM(pwmcHandle);
            STC_SetControlMode(pSTC, STC_SPEED_MODE);     /* 正常运行切换速度模式 */
            STC_SetSpeedSensor(pSTC, &ENCODER_M1._Super); /* 正常运行->编码器 SpeednTorqCtrl_Handle_t */
            STM_NextState(&STM, ANY_STOP);
        }
        break;

    case START_RUN:
        /* 速度/转矩斜坡参数 */
        // MCI_ExecTorqueRamp(oMCInterface, -3000, 1000); /* MaxPositiveTorque */
        MCI_ExecSpeedRamp(oMCInterface, 60, 100); /* MaxAppPositiveMecSpeed01Hz */

        FOC_CalcCurrRef(0);
        STM_NextState(&STM, RUN);

        /* 新起始速度/转矩点,计算新斜坡 */
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
        HALL_Init(&HALL_M1); /* 初始化HALL传感器 */
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
        HALL_Clear(&HALL_M1); /* HALL传感器数据清零 */
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
    bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeed01Hz(&STO_PLL_M1, &wAux); /* STO_PLL速度可靠性 */
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
        STC_SetSpeedSensor(pSTC, &VirtualSpeedSensorM1._Super);                 /* 启动选用虚拟速度传感器 */
        RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(oMCInterface)); /* 初始化启动控制器状态 */
        SWO_transitionStartM1 = false;                                          /* 启动开始 */
        STM.bSenLess_Handle = 0;

        STO_PLL_Clear(&STO_PLL_M1);
        STO_CR_Clear(&STO_CR_M1);
        if (STM_NextState(&STM, START) == true) {
            FOC_Clear(0);
            R3F4XX_SwitchOnPWM(pwmcHandle);
        }
        break;

    case START:
        /* 1.I/F强拖阶段bStageCnt,链表会自动跳转下一目标曲线 */
        /* I/F阶段:电机完全静止->低速旋转(尚未切换到闭环观测器) */
        if (!RUC_Exec(&RevUpControlM1)) {
            STM_FaultProcessing(&STM, MC_START_UP, 0); /* 启动失败 */
        } else {
            if (SWO_transitionStartM1 == false) {
                /* 转矩Iq,I/F强拖启动(切换运行态时,注意双dq系处理、Iq不连续处理) */
                IqdRef.qI_Component1 = STC_CalcTorqueReference(pSTC);
                IqdRef.qI_Component2 = FOCVars.UserIdref;
                FOCVars.Iqdref = IqdRef;
            }
        }

        /* 2.虚拟机械速度,返回速度可靠性(斜坡bTransitionEnded=0,不可靠;
        斜坡剩余次数hTransitionRemainingsteps=0=>完成bTransitionEnded=1,再检验速度可靠性) */
        StartUpTransitionEnded = VSS_CalcAvrgMecSpeed01Hz(&VirtualSpeedSensorM1, &hForcedMecSpeed01Hz); /* 计算dpp,用于FOC补偿电角度 */

        /* 3.I/F强拖->估计速度在当前速度收敛范围内->算法收敛IsAlgorithmConverged=1->开启执行斜坡bTransitionStarted=1 */
        /* W_est与W_sto做算法收敛判断 */
        StartUpDoTransition = VSS_SetStartTransition(&VirtualSpeedSensorM1, STO_PLL_IsObserverConverged(&STO_PLL_M1, hForcedMecSpeed01Hz));

        /* 4.斜坡平滑闭环阶段(目地:防止Iq突变,SMO无连续的BEMF信号,有可能造成观测BEMF相位错误,SMO不能闭环) */
        /* 开环强拖到SMO收敛->斜坡平滑Iq->闭环观测器 */
        if (VSS_IsTransitionOngoing(&VirtualSpeedSensorM1)) {
            if (SWO_transitionStartM1 == false) {
                int16_t Iq = 0;
                Curr_Components StatorCurrent = MCM_Park(FOCVars.Ialphabeta, SPD_GetElAngle(&STO_PLL_M1._Super));
                Iq = StatorCurrent.qI_Component1;

                /* 5.斜坡把FOCVars.Iqdref.qI_Component1 调整到 Iq => 切换闭环观测器运行 */
                /* 斜坡提前完成RampRemainingstep=0 => 不执行REMNG_Calc => FoCVars.Iqdref.qI_Component1固定 */
                REMNG_Init(pREMNG);
                REMNG_ExecRamp(pREMNG, Iq, 0);
                REMNG_ExecRamp(pREMNG, FOCVars.Iqdref.qI_Component1, TRANSITION_DURATION);
                SWO_transitionStartM1 = true;
            } else {
                /* 6.FOC频率任务里调用REMNG_Calc */
            }
        } else {
            if (SWO_transitionStartM1 == true) {
                SWO_transitionStartM1 = false;
            }
        }

        if (StartUpDoTransition == false) {
            StartUpTransitionEnded = true;
        }

        /* 7.速度可靠,切换到正常运行 */
        if (StartUpTransitionEnded == true) {
            PID_SetIntegralTerm(pPIDSpeed, (int32_t)(FOCVars.Iqdref.qI_Component1 * PID_GetKIDivisor(pPIDSpeed) / PID_SPEED_INTEGRAL_INIT_DIV));
            STM_NextState(&STM, START_RUN);
        }
        break;

    case START_RUN:
        FOCVars.hTeref = FOCVars.Iqdref.qI_Component1;
        MCI_ExecTorqueRamp(oMCInterface, 220, 1000);
        // MCI_ExecSpeedRamp(oMCInterface, 300, 1000);
        STC_SetSpeedSensor(pSTC, &STO_PLL_M1._Super); /* 启动选用STO_PLL速度传感器 */
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
        /* 高频任务执行FOC算法 */
        break;

    default:
        break;
    }

#endif
}

/**
 * @brief 自举电容充电
 * @param hTickCount 抬升计数
 * @retval None
 */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
    hBootCapDelayCounterM1 = hTickCount;
}

/**
 * @brief 充电延时计数是否结束
 * @retval bool true:结束 false:未结束
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
 * @brief STOP状态机维持时间
 * @param hTickCount 停止持续时间
 * @retval None
 */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
    hStopPermanencyCounterM1 = hTickCount;
}

/**
 * @brief 跳出STOP状态机
 * @retval bool true:跳出 false:未跳出
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
 * @brief 高频FOC算法任务
 * @retval uint8_t 0
 */
uint8_t TSK_HighFrequencyTask(void)
{
#if HFI_SMO_ENABLE == 0 && FLUX_SENSOR_ENABLE == 0
    uint8_t bMotorNbr = 0;
    uint16_t hFOCreturn;

#if ENCODER_ENABLE
    /* 对齐使用虚拟速度传感器 VirtualSpeedSensorM1._Super,所以此处无效 */
    ENC_CalcAngle(&ENCODER_M1); /* if not sensorless then 2nd parameter is MC_NULL*/
#elif HALL_ENABLE
    HALL_CalcElAngle(&HALL_M1); // 计算电角度
#elif SENSORLESS_ENABLE
    uint16_t hState;
    Observer_Inputs_t STO_Inputs;
    Observer_Inputs_t STO_aux_Inputs;

    STO_aux_Inputs.Valfa_beta = FOCVars.Valphabeta;
    STO_Inputs.Valfa_beta = FOCVars.Valphabeta;

    /* 1.执行斜坡 */
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
        /* 2.检查I/F的第一阶段 */
        bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);

        /* 3.STO计算电角度和电角速度,虚拟传感器下无效 */
        STO_Inputs.Ialfa_beta = FOCVars.Ialphabeta;
        STO_Inputs.Vbus = handle_break.vbus_value >> 1;
        STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
        STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1);

        /* 4.I/F的第1曲线完成,加入STO+PLL */
        if (IsAccelerationStageReached == false) {
            STO_ResetPLL(&STO_PLL_M1);
        }

        /* 5.虚拟+STO_PLL补偿计算电角度 */
        hState = STM_GetState(&STM);
        if ((hState == START) || (hState == START_RUN)) {
            int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
            /* 6.计算虚拟电角度;优化电角度校准 */
            /* 斜坡剩余次数hTransitionRemainingsteps=0=>完成bTransitionEnded=1=>VSS_IsTransitionOngoing返回0
            =>SWO_transitionStartM1=0=>不执行REMNG_Calc */
            VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
        }

        /* 7.电机模型,电流重构电角度、电角速度、低速 */
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
 * @brief 安全保护任务
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
 * @brief 安全任务-关输出
 * @param bMotor 电机号
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
 * @brief Hardware Fault任务
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
