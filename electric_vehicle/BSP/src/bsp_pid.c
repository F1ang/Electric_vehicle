/******************************************************************************
 *@brief  PID实现
 *@author By Spotted Owl
 *@date     2025.06.04
 ******************************************************************************/
#include "bsp_pid.h"

/* 速度PID实例化 */
PID_Handle_t PIDSpeedHandle_M1 = {
    .hDefKpGain = (int16_t)PID_SPEED_KP_DEFAULT,
    .hDefKiGain = (int16_t)PID_SPEED_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
    .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
    .hUpperOutputLimit = (int16_t)IQMAX,
    .hLowerOutputLimit = -(int16_t)IQMAX,
    .hKpDivisor = (uint16_t)SP_KPDIV,
    .hKiDivisor = (uint16_t)SP_KIDIV,
    .hKpDivisorPOW2 = (uint16_t)SP_KPDIV_LOG,
    .hKiDivisorPOW2 = (uint16_t)SP_KIDIV_LOG,
    .hDefKdGain = 0x0000U,
    .hKdDivisor = 0x0000U,
    .hKdDivisorPOW2 = 0x0000U,
};

/* 速度转矩实例化 */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 = {
    .STCFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE,
    .MaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED / 6),
    .MinAppPositiveMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),
    .MaxAppNegativeMecSpeed01Hz = (int16_t)(-MIN_APPLICATION_SPEED / 6),
    .MinAppNegativeMecSpeed01Hz = (int16_t)(-MAX_APPLICATION_SPEED / 6),
    .MaxPositiveTorque = (int16_t)NOMINAL_CURRENT,
    .MinNegativeTorque = -(int16_t)NOMINAL_CURRENT,
    .ModeDefault = DEFAULT_CONTROL_MODE,
    .MecSpeedRef01HzDefault = (int16_t)(DEFAULT_TARGET_SPEED_RPM / 6),
    .TorqueRefDefault = (int16_t)DEFAULT_TORQUE_COMPONENT,
    .IdrefDefault = (int16_t)DEFAULT_FLUX_COMPONENT,
};

/* Iq实例化 */
PID_Handle_t PIDIqHandle_M1 = {
    .hDefKpGain = (int16_t)PID_TORQUE_KP_DEFAULT,
    .hDefKiGain = (int16_t)PID_TORQUE_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
    .hUpperOutputLimit = INT16_MAX,
    .hLowerOutputLimit = -INT16_MAX,
    .hKpDivisor = (uint16_t)TF_KPDIV,
    .hKiDivisor = (uint16_t)TF_KIDIV,
    .hKpDivisorPOW2 = (uint16_t)TF_KPDIV_LOG,
    .hKiDivisorPOW2 = (uint16_t)TF_KIDIV_LOG,
    .hDefKdGain = 0x0000U,
    .hKdDivisor = 0x0000U,
    .hKdDivisorPOW2 = 0x0000U,
};

/* Id实例化 */
PID_Handle_t PIDIdHandle_M1 = {
    .hDefKpGain = (int16_t)PID_FLUX_KP_DEFAULT,
    .hDefKiGain = (int16_t)PID_FLUX_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
    .hUpperOutputLimit = INT16_MAX,
    .hLowerOutputLimit = -INT16_MAX,
    .hKpDivisor = (uint16_t)TF_KPDIV,
    .hKiDivisor = (uint16_t)TF_KIDIV,
    .hKpDivisorPOW2 = (uint16_t)TF_KPDIV_LOG,
    .hKiDivisorPOW2 = (uint16_t)TF_KIDIV_LOG,
    .hDefKdGain = 0x0000U,
    .hKdDivisor = 0x0000U,
    .hKdDivisorPOW2 = 0x0000U,
};

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */
void PID_HandleInit(PID_Handle_t *pHandle)
{
    pHandle->hKpGain = pHandle->hDefKpGain;
    pHandle->hKiGain = pHandle->hDefKiGain;
    pHandle->hKdGain = pHandle->hDefKdGain;
    pHandle->wIntegralTerm = 0x00000000UL;
    pHandle->wPrevProcessVarError = 0x00000000UL;
}

/**
 * @brief  It updates the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpGain: new Kp gain
 * @retval None
 */
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain)
{
    pHandle->hKpGain = hKpGain;
}

/**
 * @brief  It updates the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiGain: new Ki gain
 * @retval None
 */
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain)
{
    pHandle->hKiGain = hKiGain;
}

/**
 * @brief  It returns the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain
 */
int16_t PID_GetKP(PID_Handle_t *pHandle)
{
    return (pHandle->hKpGain);
}

/**
 * @brief  It returns the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain
 */
int16_t PID_GetKI(PID_Handle_t *pHandle)
{
    return (pHandle->hKiGain);
}

/**
 * @brief  It returns the Default Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Kp gain
 */
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle)
{
    return (pHandle->hDefKpGain);
}

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Ki gain
 */
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle)
{
    return (pHandle->hDefKiGain);
}

/**
 * @brief  It set a new value into the PI integral term
 * pHandle: handler of the current instance of the PID component
 * @param  wIntegralTermValue: new integral term value
 * @retval None
 */
void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue)
{
    pHandle->wIntegralTerm = wIntegralTermValue;

    return;
}

/**
 * @brief  It returns the Kp gain divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain divisor
 */
uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle)
{
    return (pHandle->hKpDivisor);
}

/**
 * @brief  It updates the Kp divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpDivisorPOW2: new Kp divisor expressed as power of 2
 * @retval None
 */
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2)
{
    pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
    pHandle->hKpDivisor = ((uint16_t)(1u) << hKpDivisorPOW2);
}

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain divisor
 */
uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle)
{
    return (pHandle->hKiDivisor);
}

/**
 * @brief  It updates the Ki divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiDivisorPOW2: new Ki divisor expressed as power of 2
 * @retval None
 */
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2)
{
    int32_t wKiDiv = ((int32_t)(1u) << hKiDivisorPOW2);
    pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
    pHandle->hKiDivisor = (uint16_t)(wKiDiv);
    PID_SetUpperIntegralTermLimit(pHandle, (int32_t)INT16_MAX * wKiDiv);
    PID_SetLowerIntegralTermLimit(pHandle, (int32_t)-INT16_MAX * wKiDiv);
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wLowerLimit: new lower integral term limit value
 * @retval None
 */
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit)
{
    pHandle->wLowerIntegralLimit = wLowerLimit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wUpperLimit: new upper integral term limit value
 * @retval None
 */
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit)
{
    pHandle->wUpperIntegralLimit = wUpperLimit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hLowerLimit: new lower output limit value
 * @retval None
 */
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit)
{
    pHandle->hLowerOutputLimit = hLowerLimit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hUpperLimit: new upper output limit value
 * @retval None
 */
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit)
{
    pHandle->hUpperOutputLimit = hUpperLimit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wPrevProcessVarError: New previous error variable
 * @retval None
 */
void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError)
{
    pHandle->wPrevProcessVarError = wPrevProcessVarError;
    return;
}

/**
 * @brief  It updates the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKdGain: new Kd gain
 * @retval None
 */
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain)
{
    pHandle->hKdGain = hKdGain;
}

/**
 * @brief  It returns the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain
 */
int16_t PID_GetKD(PID_Handle_t *pHandle)
{
    return pHandle->hKdGain;
}

/**
 * @brief  It returns the Kd gain divisor of the PID object passed
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain divisor
 */
uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle)
{
    return (pHandle->hKdDivisor);
}

/**
 * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
 *
 * @param pHandle handle on the instance of the PID component to update
 * @param hKdDivisorPOW2
 */
void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2) {}

/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
    int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
    int32_t wDischarge = 0;
    int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
    int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

    /* Proportional term computation*/
    wProportional_Term = pHandle->hKpGain * wProcessVarError;

    /* Integral term computation */
    if (pHandle->hKiGain == 0) {
        pHandle->wIntegralTerm = 0;
    } else {
        wIntegral_Term = pHandle->hKiGain * wProcessVarError;
        wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

        if (wIntegral_sum_temp < 0) {
            if (pHandle->wIntegralTerm > 0) {
                if (wIntegral_Term > 0) {
                    wIntegral_sum_temp = INT32_MAX;
                }
            }
        } else {
            if (pHandle->wIntegralTerm < 0) {
                if (wIntegral_Term < 0) {
                    wIntegral_sum_temp = -INT32_MAX;
                }
            }
        }

        if (wIntegral_sum_temp > pHandle->wUpperIntegralLimit) {
            pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
        } else if (wIntegral_sum_temp < pHandle->wLowerIntegralLimit) {
            pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
        } else {
            pHandle->wIntegralTerm = wIntegral_sum_temp;
        }
    }

    wOutput_32 = (wProportional_Term >> pHandle->hKpDivisorPOW2) + (pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2);

    if (wOutput_32 > hUpperOutputLimit) {

        wDischarge = hUpperOutputLimit - wOutput_32;
        wOutput_32 = hUpperOutputLimit;
    } else if (wOutput_32 < hLowerOutputLimit) {

        wDischarge = hLowerOutputLimit - wOutput_32;
        wOutput_32 = hLowerOutputLimit;
    } else { /* Nothing to do here */
    }

    pHandle->wIntegralTerm += wDischarge;

    return ((int16_t)(wOutput_32));
}
