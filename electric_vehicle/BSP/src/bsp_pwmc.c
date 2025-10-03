#include "bsp_pwmc.h"
#include "bsp_r3f4.h"

/**
 * @brief 获取电流值
 * @param *pHandle 电机句柄
 * @param *pStator_Currents Iab
 * @retval None
 */
void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle, Curr_Components *pStator_Currents)
{
    pHandle->pFctGetPhaseCurrents(pHandle, pStator_Currents);
}

/**
 * @brief SVPWM
 * @param *pHandle 电机句柄
 * @param Valfa_beta SVPWM value
 * @retval 采样点设置状态
 */
uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, Volt_Components Valfa_beta)
{ /* SVPWM:画ap和xyz图->确定扇区  扇区矢量分解+平均值等效原理->作用时间 */
    int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
    PWMC_SetSampPointSectX_Cb_t pSetADCSamplingPoint;

    /* Valpha、Vbeta矢量 ap */
    wUAlpha = Valfa_beta.qV_Component1 * (int32_t)pHandle->hT_Sqrt3;           /* sqrt(3)*Ts*V_alpha */
    wUBeta = -(Valfa_beta.qV_Component2 * (int32_t)(pHandle->hPWMperiod)) * 2; /* -Ts*V_beta */

    /* 矢量划分扇区 xyz,参数表达式 */
    wX = wUBeta;                 /* -Ts*V_beta */
    wY = (wUBeta + wUAlpha) / 2; /* -Ts*V_beta + sqrt(3)*Ts*V_alpha/2 */
    wZ = (wUBeta - wUAlpha) / 2; /* -Ts*V_beta - sqrt(3)*Ts*V_alpha/2 */

    /* Sector calculation from wX, wY, wZ */
    if (wY < 0) {
        if (wZ < 0) {
            /* 建系:q与alpha轴重合为0电角度;beta向下为正.  求扇区:列曲线参数方程(隐函数) 求矢量作用时间:PWM1的7段式,按伏秒平衡+平均值等效 */
            pHandle->hSector = SECTOR_5;
            wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wY - wZ) / (int32_t)262144); /* pu=2*2 *2^16=8*32768 */
            wTimePhB = wTimePhA + wZ / 131072;
            wTimePhC = wTimePhA - wY / 131072;
            pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect5;
        } else /* wZ >= 0 */
            if (wX <= 0) {
                pHandle->hSector = SECTOR_4;
                wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wX - wZ) / (int32_t)262144);
                wTimePhB = wTimePhA + wZ / 131072;
                wTimePhC = wTimePhB - wX / 131072;
                pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect4;
            } else /* wX > 0 */
            {
                pHandle->hSector = SECTOR_3;
                wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wY - wX) / (int32_t)262144);
                wTimePhC = wTimePhA - wY / 131072;
                wTimePhB = wTimePhC + wX / 131072;
                pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect3;
            }
    } else /* wY > 0 */
    {
        if (wZ >= 0) {
            pHandle->hSector = SECTOR_2;
            wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wY - wZ) / (int32_t)262144);
            wTimePhB = wTimePhA + wZ / 131072;
            wTimePhC = wTimePhA - wY / 131072;
            pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect2;
        } else /* wZ < 0 */
            if (wX <= 0) {
                pHandle->hSector = SECTOR_6;
                wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wY - wX) / (int32_t)262144);
                wTimePhC = wTimePhA - wY / 131072;
                wTimePhB = wTimePhC + wX / 131072;
                pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect6;
            } else /* wX > 0 */
            {
                pHandle->hSector = SECTOR_1;
                /* 注:hPWMPeriod = Timer_Fclk/Fpwm=>中心对齐PWM的计数周期Ts=2T(单PWM)  Timer_Fclk/(2*Fpwm)=>单路PWM的计数周期 */
                /* T_周期计数/2 抬升->T取了中心对齐PWM周期Ts=2T->T/4  原公式(((T + wX) - wZ) / 2)   注:131072=2*2^15*2*2,Q15与公式T取了Ts,且1/2^15≈0 */
                wTimePhA = (int32_t)(pHandle->hPWMperiod) / 4 + ((wX - wZ) / (int32_t)262144);
                wTimePhB = wTimePhA + wZ / 131072;
                wTimePhC = wTimePhB - wX / 131072;
                pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect1;
            }
    }

    pHandle->hCntPhA = (uint16_t)wTimePhA;
    pHandle->hCntPhB = (uint16_t)wTimePhB;
    pHandle->hCntPhC = (uint16_t)wTimePhC;

    /* 采样点设置回调 */
    return (pSetADCSamplingPoint(pHandle));
}

/**
 * @brief  等待UPDATE事件,关闭TIM8主输出,开启CH1~CH3及互补输出
 * @param  pHandle 电机句柄
 * @retval None
 */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
    pHandle->pFctSwitchOffPwm(pHandle);
}

/**
 * @brief  开启PWM输出
 * @param  pHandle  电机句柄
 * @retval None
 */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
    pHandle->pFctSwitchOnPwm(pHandle);
}

/**
 * @brief 电流校准
 * @param *pHandle 电机句柄
 * @param action 校准动作
 * @retval None
 */
bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle, CRCAction_t action)
{
    bool retVal = false;
    if (action == CRC_START) {
        PWMC_SwitchOffPWM(pHandle);
        pHandle->hOffCalibrWaitTimeCounter = pHandle->hOffCalibrWaitTicks; /* 4ms */
        if (pHandle->hOffCalibrWaitTicks == 0u) {
            pHandle->pFctCurrReadingCalib(pHandle);
            retVal = true;
        }
    } else if (action == CRC_EXEC) {
        if (pHandle->hOffCalibrWaitTimeCounter > 0u) {
            pHandle->hOffCalibrWaitTimeCounter--;
            if (pHandle->hOffCalibrWaitTimeCounter == 0u) {
                pHandle->pFctCurrReadingCalib(pHandle);
                retVal = true;
            }
        } else {
            retVal = true;
        }
    } else {
    }
    return retVal;
}

/**
 * @brief 开启下管驱动
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle)
{
    pHandle->pFctTurnOnLowSides(pHandle);
}

/**
 * @brief 是否过流
 * @param *pHandle 电机句柄
 * @retval 过流回调
 */
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle)
{
    // return pHandle->pFctIsOverCurrentOccurred(pHandle);
    return 0;
}

/**
 * @brief 过压回调
 * @param *pHandle 电机句柄
 * @param hDACVref DAC值
 * @retval None
 */
void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle, uint16_t hDACVref)
{
    // if (pHandle->pFctOCPSetReferenceVoltage) {
    //     pHandle->pFctOCPSetReferenceVoltage(pHandle, hDACVref);
    // }
}

/**
 * @brief 获取电流值回调
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterGetPhaseCurrentsCallBack(PWMC_GetPhaseCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctGetPhaseCurrents = pCallBack;
}

/**
 * @brief 关闭PWM输出回调
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSwitchOffPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief 开启PWM输出回调
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval  None
 */
void PWMC_RegisterSwitchonPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
/**
 * @brief 电流测量校准回调
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterReadingCalibrationCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
 * @brief 开启下管驱动回调
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterTurnOnLowSidesCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sector 1 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect1CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect1 = pCallBack;
}

/**
 * @brief Sector 2 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect2CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect2 = pCallBack;
}

/**
 * @brief Sector 3 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect3CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect3 = pCallBack;
}

/**
 * @brief Sector 4 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect4CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect4 = pCallBack;
}

/**
 * @brief Sector 5 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect5CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect5 = pCallBack;
}

/**
 * @brief Sector 6 ADC sampling point callback
 * @param pCallBack pointer on the callback
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterSampPointSect6CallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    pHandle->pFctSetADCSampPointSect6 = pCallBack;
}

/**
 * @brief 过流检测回调函数
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterIsOverCurrentOccurredCallBack(PWMC_OverCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    // pHandle->pFctIsOverCurrentOccurred = pCallBack;
}

/**
 * @brief 过流保护回调函数
 * @param pCallBack 回调函数
 * @param *pHandle 电机句柄
 * @retval None
 */
void PWMC_RegisterOCPSetRefVoltageCallBack(PWMC_SetOcpRefVolt_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
    // pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}
