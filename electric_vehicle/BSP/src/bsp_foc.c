/******************************************************************************
 *@brief  FOC
 *@author By Spotted Owl
 *@date     2025.05.08
 ******************************************************************************/
#include "bsp_foc.h"
#include "bsp_motor.h"
#include "tim.h"
#include "adc.h"
#include "circle_limitation.h"
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
#include "bsp_dac_comm.h"
#include "bsp_hall.h"
#include "foc_sensorless.h"

/**
 * @brief TIMx Overflow Callback function
 * @param *htim Timer handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t wADCInjFlags;

    /* GPIO测试单路PWM频率=16k*2 */
    /* 使能CC4触发ADC,改变ADC边沿配置 */
    if (htim->Instance == TIM8) {
        // HAL_GPIO_WritePin(FREQ_TEST1_GPIO_Port, FREQ_TEST1_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(FREQ_TEST1_GPIO_Port, FREQ_TEST1_Pin, GPIO_PIN_RESET);

        PWM_Handle_M1.bSoFOC = 1;

        /* 获取上一次ADC采样标志 */
        wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;

        /* 等待ADC采集完成,已兼容上下边沿采样 */
        if (wADCInjFlags == CONV_STARTED) {
            do {
                wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;
            } while (wADCInjFlags != CONV_FINISHED);
        } else if (wADCInjFlags == FLAGS_CLEARED) {
            while ((TIM8->CNT) < (PWM_Handle_M1._Super.Tw)) { /* 注入ADC转换标志被清除,等待Tw后进行再次检测 */
            }
            wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;

            if (wADCInjFlags == CONV_STARTED) {
                do {
                    wADCInjFlags = (ADC1->SR) & ADC_SR_MASK;
                } while (wADCInjFlags != CONV_FINISHED);
            }
        }

        /* 关ADC采样触发 */
        ADC1->CR2 = PWM_Handle_M1.wADCTriggerUnSet;
        ADC2->CR2 = PWM_Handle_M1.wADCTriggerUnSet;

        /* 使能CC4触发ADC采样 */
        TIM8->CCER |= 0x1000u;

        /* 开ADC采样触发及配置边沿触发极性 */
        ADC1->CR2 = PWM_Handle_M1.wADCTriggerSet;
        ADC2->CR2 = PWM_Handle_M1.wADCTriggerSet;

        /* 配置相电流采样通道 */
        ADC1->JSQR = PWM_Handle_M1.wADC1Channel;
        ADC2->JSQR = PWM_Handle_M1.wADC2Channel;
    }

    // Encoder
    if (htim->Instance == TIM2) {
        // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(FREQ_TEST0_GPIO_Port, FREQ_TEST0_Pin, GPIO_PIN_RESET);
    }

    if (htim->Instance == TIM5) {
        HALL_TIMx_UP_IRQHandler(&HALL_M1);
    }
}
/**
 * @brief ADC JEOC Callback function
 * @param *hadc ADC handle
 * @retval None
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#if DAC_TEST_ENABLE
    DAC_Output_Printf();
    // printf("%d\n", ADC1->JDR1);
    // printf("%d\n", pwmcHandle->hIa); /* 放在高频任务中,波形会更平滑 */
#endif
    // HAL_GPIO_WritePin(FREQ_TEST1_GPIO_Port, FREQ_TEST1_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(FREQ_TEST1_GPIO_Port, FREQ_TEST1_Pin, GPIO_PIN_RESET);
    TSK_HighFrequencyTask();
}

/**
 * @brief 复位FOC变量
 * @param bMotor 电机编号
 * @retval None
 */
void FOC_Clear(uint8_t bMotor)
{
    Curr_Components Inull = { (int16_t)0, (int16_t)0 };
    Volt_Components Vnull = { (int16_t)0, (int16_t)0 };

    FOCVars.Iab = Inull;
    FOCVars.Ialphabeta = Inull;
    FOCVars.Iqd = Inull;
    FOCVars.Iqdref = Inull;
    FOCVars.hTeref = (int16_t)0;
    FOCVars.Vqd = Vnull;
    FOCVars.Valphabeta = Vnull;
    FOCVars.hElAngle = (int16_t)0;

    PID_SetIntegralTerm(pPIDIq, (int32_t)0);
    PID_SetIntegralTerm(pPIDId, (int32_t)0);

    STC_Clear(pSTC);
    PWMC_SwitchOffPWM(pwmcHandle);
}

/**
 * @brief Iq计算
 * @param bMotor 电机编号
 * @retval None
 */
void FOC_CalcCurrRef(uint8_t bMotor)
{
    if (FOCVars.bDriveInput == INTERNAL) {
        FOCVars.Iqdref.qI_Component1 = FOCVars.hTeref; /* Iq */
        FOCVars.hTeref = STC_CalcTorqueReference(pSTC);
    }
}

/**
 * @brief FOC算法
 * @param bMotor 电机号
 * @retval SVPWM错误码
 */
uint16_t FOC_CurrController(uint8_t bMotor)
{
#if HFI_SMO_ENABLE == 0 && FLUX_SENSOR_ENABLE == 0
    Curr_Components Iab, Ialphabeta, Iqd;
    Volt_Components Valphabeta, Vqd;
    int16_t hElAngle;
    uint16_t hCodeError;

#if OPEN_DEBUG_MODE /* 开环调试 */
    /* 开环调试 Uq、Ud恒定,θ_e按矢量圆变化=>Valpha、Vbeta=>Sector and Time=>SVPWM */
    static s16 cnt = S16_MIN;
    hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC));
    PWMC_GetPhaseCurrents(pwmcHandle, &Iab);
    Vqd.qV_Component1 = 0;
    Vqd.qV_Component2 = 5000; /* 10000相电流波形基本正常(offset可以再三个取平均,进行平滑) */

    cnt += 100;        /* cnt -= 100;反转 */
    if (cnt > S16_MAX) /* -32768~32767 */
        cnt = S16_MIN;
    Valphabeta = MCM_Rev_Park(Vqd, cnt);
    PWMC_SetPhaseVoltage(pwmcHandle, Valphabeta);

    // printf("%d\n", pwmcHandle->hIa); /* 放在高频任务中,波形会更平滑 */

    /* 强制定位 */
    // Vqd.qV_Component1 = 0;
    // Vqd.qV_Component2 = 5000;
    // if (PWM_Handle_M1.bIndex<20)
    //     PWM_Handle_M1.bIndex++;
    // hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC));
    // Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    // hCodeError = PWMC_SetPhaseVoltage(pwmcHandle, Valphabeta);
#else /* 闭环控制 */
    /* 获取虚拟还是编码器传感器角度,对齐使用 */
    hElAngle = SPD_GetElAngle(STC_GetSpeedSensor(pSTC));
    PWMC_GetPhaseCurrents(pwmcHandle, &Iab);
    Ialphabeta = MCM_Clarke(Iab);
    Iqd = MCM_Park(Ialphabeta, hElAngle);

#if ENCODER_ENABLE
    if (EAC_IsAligned(&EncAlignCtrlM1) == true) {
#if HANDLE_BREAK
        Vqd.qV_Component1 = FOCVars.Vqd.qV_Component1;
        Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
#else
        Vqd.qV_Component1 = PI_Controller(pPIDIq, (int32_t)(FOCVars.Iqdref.qI_Component1) - Iqd.qI_Component1);
        Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
#endif
    } else {
        Vqd.qV_Component1 = PI_Controller(pPIDIq, (int32_t)(FOCVars.Iqdref.qI_Component1) - Iqd.qI_Component1);
        Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
    }

#elif HALL_ENABLE
#if HANDLE_BREAK
    Vqd.qV_Component1 = FOCVars.Vqd.qV_Component1;
    Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
#else
    Vqd.qV_Component1 = PI_Controller(pPIDIq, (int32_t)(FOCVars.Iqdref.qI_Component1) - Iqd.qI_Component1);
    Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
#endif

#elif SENSORLESS_ENABLE
#if HANDLE_BREAK
    if (STM.bState == RUN && STM.bSenLess_Handle == 1) {
        Vqd.qV_Component1 = FOCVars.Vqd.qV_Component1;
        Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
    } else {
        Vqd.qV_Component1 = PI_Controller(pPIDIq, (int32_t)(FOCVars.Iqdref.qI_Component1) - Iqd.qI_Component1);
        Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
    }
#else
    Vqd.qV_Component1 = PI_Controller(pPIDIq, (int32_t)(FOCVars.Iqdref.qI_Component1) - Iqd.qI_Component1);
    Vqd.qV_Component2 = PI_Controller(pPIDId, (int32_t)(FOCVars.Iqdref.qI_Component2) - Iqd.qI_Component2);
#endif

#endif

    FOCVars.Vqd = Vqd;
    Vqd = Circle_Limitation(pCLM, Vqd);
    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle, Valphabeta);
    FOCVars.Iab = Iab;
    FOCVars.Ialphabeta = Ialphabeta;
    FOCVars.Iqd = Iqd;
    FOCVars.Valphabeta = Valphabeta;
    FOCVars.hElAngle = hElAngle;

#endif

    return (hCodeError);

#else

#if OPEN_SENSORLESS_MODE
    Curr_Components Iab;
    abc_t Iabc = { 0 };
    float ElAngle;
    qd_t In = { 0 };
    qd_t Out = { 0 };
    FOC_SENSORLESS_t *foc_t = FOC_Sensorless_pThis();
    alphabeta_t I_alphabeta = { 0 };

    /* 开环调试 Uq、Ud恒定,θ_e按矢量圆变化=>Valpha、Vbeta=>Sector and Time=>SVPWM */
    static s16 cnt = S16_MIN;
    PWMC_GetPhaseCurrents(pwmcHandle, &Iab);
    Iabc = FOC_Sensorless_GetIabc(Iab);
    I_alphabeta = FOC_Clarke(Iabc);

    cnt += 100;        /* cnt -= 100;反转 */
    if (cnt > S16_MAX) /* -32768~32767 */
        cnt = S16_MIN;
    ElAngle = (float)(cnt / 32767.0f * _PI);
    In = Foc_Park(I_alphabeta, ElAngle);

    Out.q = 5; /* 0-24V */
    Out.d = 0;

    FOC_PWM_Run(&foc_t->pwm, Out, ElAngle);
    FOC_Sensorless_SetADCSampPointSect(pwmcHandle);
    return 0;
#else
    Curr_Components Iab;
    alphabeta_t I_alphabeta = { 0 };
    float ElAngle;
    abc_t Iabc = { 0 };
    qd_t In = { 0 };
    qd_t Out = { 0 };
    FOC_SENSORLESS_t *foc_t = FOC_Sensorless_pThis();
    alphabeta_t Last_Ualphabeta = foc_t->pwm.alpha_beta;

    MOTOR_State_t state = FOC_Sensorless_GetState();
    PWMC_GetPhaseCurrents(pwmcHandle, &Iab);
    Iabc = FOC_Sensorless_GetIabc(Iab);
    I_alphabeta = FOC_Clarke(Iabc);

    if (state == MOTOR_SL_RUN) {
        I_alphabeta = FOC_SL_RUN(foc_t, I_alphabeta, Last_Ualphabeta, foc_t->target_speed);
        ElAngle = foc_t->speed.ElAngle;
        In = Foc_Park(I_alphabeta, ElAngle);

        /* 速度环 */
        foc_t->pid_cur_iq.SetPoint = foc_t->foc_var.I_qd_ref.q;
        foc_t->pid_cur_id.SetPoint = foc_t->foc_var.I_qd_ref.d;

        /* 电流环 */
        Out.q = PID_Position_ctrl(&foc_t->pid_cur_iq, In.q);
        Out.d = PID_Position_ctrl(&foc_t->pid_cur_id, In.d) + foc_t->d_bias;

        switch (foc_t->sta) {
        case HFI_POL_STA:
            PID_Clear(&foc_t->pid_cur_iq);
            PID_Clear(&foc_t->pid_cur_id);
            PID_Clear(&foc_t->pid_speed);
            Out.q = 0;
            Out.d = foc_t->d_bias; /* 极性辨识注入=偏置注入 */
            break;

        case HFI_STA:
            /*高频注入的时候不要对d轴电流进行PID控制，否则堵转下容易反转 并且要及时清除d轴PID参数，不然跳转到SMO会出问题*/
            PID_Clear(&foc_t->pid_cur_id);
            Out.d = foc_t->d_bias;

            if (foc_t->pid_cur_iq.ActualValue_limit != foc_t->pid_qd_limit_hif) {
                /*清除积分，设置高频注入q轴电流限制*/
                PID_Clear(&foc_t->pid_cur_iq);
                foc_t->pid_cur_iq.ActualValue_limit = foc_t->pid_qd_limit_hif; /*高频注入状态 需要限制电流环*/
            }
            break;

        case SMO_STA:
            foc_t->pid_cur_iq.ActualValue_limit = foc_t->pid_qd_limit;
            break;

        case FLUX_STA:
            /* 电流环限幅Uq */
            foc_t->pid_cur_iq.ActualValue_limit = foc_t->pid_qd_limit;
            break;

        default:
            break;
        }

        FOC_PWM_Run(&foc_t->pwm, Out, ElAngle);
        FOC_Sensorless_SetADCSampPointSect(pwmcHandle);

        /*保存参数*/
        foc_t->foc_var.speed = foc_t->speed;
        foc_t->foc_var.I_abc = Iabc;
        foc_t->foc_var.U_qd = foc_t->pwm.Uqd;
        foc_t->foc_var.I_qd = In;
        foc_t->foc_var.I_alphabeta = I_alphabeta;
    }
    return 0;
#endif

#endif
}
