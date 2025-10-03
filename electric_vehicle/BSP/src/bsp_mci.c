#include "bsp_mci.h"
#include "bsp_stc.h"
#include "bsp_state_machine.h"
#include "bsp_spd.h"

/**
 * @brief 电机控制层对象:状态机对象、速度转矩对象、FOC对象初始化
 * @param *pHandle 电机控制层对象
 * @param *pSTM 状态机对象
 * @param *pSTC 速度转矩对象
 * @param pFOCVars FOC对象
 * @retval None
 */
void MCI_Init(MCI_Handle_t *pHandle, STM_Handle_t *pSTM, SpeednTorqCtrl_Handle_t *pSTC, pFOCVars_t pFOCVars)
{
    pHandle->pSTM = pSTM;
    pHandle->pSTC = pSTC;
    pHandle->pFOCVars = pFOCVars;

    /* Buffer related initialization */
    pHandle->lastCommand = MCI_NOCOMMANDSYET;
    pHandle->hFinalSpeed = 0;
    pHandle->hFinalTorque = 0;
    pHandle->hDurationms = 0;
    pHandle->CommandState = MCI_BUFFER_EMPTY;
}

/**
 * @brief 速度斜坡控制
 * @param *pHandle 电机控制层对象
 * @param hFinalSpeed 目标速度
 * @param hDurationms 持续时间
 * @retval None
 */
void MCI_ExecSpeedRamp(MCI_Handle_t *pHandle, int16_t hFinalSpeed, uint16_t hDurationms)
{
    pHandle->lastCommand = MCI_EXECSPEEDRAMP;
    pHandle->hFinalSpeed = hFinalSpeed;
    pHandle->hDurationms = hDurationms;
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    pHandle->LastModalitySetByUser = STC_SPEED_MODE;
}

/**
 * @brief 转矩斜坡控制
 * @param *pHandle 电机控制层对象
 * @param hFinalTorque 目标转矩
 * @param hDurationms 持续时间
 * @retval None
 */
void MCI_ExecTorqueRamp(MCI_Handle_t *pHandle, int16_t hFinalTorque, uint16_t hDurationms)
{
    pHandle->lastCommand = MCI_EXECTORQUERAMP;
    pHandle->hFinalTorque = hFinalTorque;
    pHandle->hDurationms = hDurationms;
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
 * @brief 设置Iqdref
 * @param *pHandle 电机控制层对象
 * @param Iqdref 电流参考值
 * @retval None
 */
void MCI_SetCurrentReferences(MCI_Handle_t *pHandle, Curr_Components Iqdref)
{
    pHandle->lastCommand = MCI_SETCURRENTREFERENCES;
    pHandle->Iqdref.qI_Component1 = Iqdref.qI_Component1;
    pHandle->Iqdref.qI_Component2 = Iqdref.qI_Component2;
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
 * @brief 启动电机
 * @param *pHandle 电机控制层对象
 * @retval true-状态转换成功
 */
bool MCI_StartMotor(MCI_Handle_t *pHandle)
{
    bool RetVal = STM_NextState(pHandle->pSTM, IDLE_START);

    if (RetVal == true) {
        pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    }

    return RetVal;
}

/**
 * @brief 停止电机
 * @param *pHandle 电机控制层对象
 * @retval true-状态转换成功
 */
bool MCI_StopMotor(MCI_Handle_t *pHandle)
{
    return STM_NextState(pHandle->pSTM, ANY_STOP);
}

/**
 * @brief 故障是否跳出
 * @param *pHandle
 * @retval 跳出
 */
bool MCI_FaultAcknowledged(MCI_Handle_t *pHandle)
{
    return STM_FaultAcknowledged(pHandle->pSTM);
}

/**
 * @brief 编码器对齐
 * @param *pHandle
 * @retval true-对齐成功
 */
bool MCI_EncoderAlign(MCI_Handle_t *pHandle)
{
    return STM_NextState(pHandle->pSTM, IDLE_ALIGNMENT);
}

/**
 * @brief 速度/转矩控制选择,只需要执行一次速度/转矩指令
 * @param *pHandle 电机控制层对象
 * @retval None
 */
void MCI_ExecBufferedCommands(MCI_Handle_t *pHandle)
{
    if (pHandle != 0) {
        if (pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED) {
            bool commandHasBeenExecuted = false;
            switch (pHandle->lastCommand) {
            case MCI_EXECSPEEDRAMP: {
                pHandle->pFOCVars->bDriveInput = INTERNAL;
                STC_SetControlMode(pHandle->pSTC, STC_SPEED_MODE);
                commandHasBeenExecuted = STC_ExecRamp(pHandle->pSTC, pHandle->hFinalSpeed, pHandle->hDurationms);
            } break;
            case MCI_EXECTORQUERAMP: {
                pHandle->pFOCVars->bDriveInput = INTERNAL;
                STC_SetControlMode(pHandle->pSTC, STC_TORQUE_MODE);
                commandHasBeenExecuted = STC_ExecRamp(pHandle->pSTC, pHandle->hFinalTorque, pHandle->hDurationms);
            } break;
            case MCI_SETCURRENTREFERENCES: {
                pHandle->pFOCVars->bDriveInput = EXTERNAL;
                pHandle->pFOCVars->Iqdref = pHandle->Iqdref;
                commandHasBeenExecuted = true;
            } break;
            default:
                break;
            }

            /* 只需要执行一次速度/转矩指令 */
            if (commandHasBeenExecuted) {
                pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
            } else {
                pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
            }
        }
    }
}

/**
 * @brief 控制层执行状态
 * @param *pHandle 电机控制层对象
 * @retval 电机控制层状态
 */
MCI_CommandState_t MCI_IsCommandAcknowledged(MCI_Handle_t *pHandle)
{
    MCI_CommandState_t retVal = pHandle->CommandState;

    if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) |
        (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY)) {
        pHandle->CommandState = MCI_BUFFER_EMPTY;
    }
    return retVal;
}

/**
 * @brief 获取当前状态机
 * @param *pHandle 电机控制层对象
 * @retval 电机控制层状态机
 */
State_t MCI_GetSTMState(MCI_Handle_t *pHandle)
{
    return STM_GetState(pHandle->pSTM);
}

/**
 * @brief 获取故障状态
 * @param *pHandle 电机控制层对象
 * @retval 故障状态
 */
uint16_t MCI_GetOccurredFaults(MCI_Handle_t *pHandle)
{
    return (uint16_t)(STM_GetFaultState(pHandle->pSTM));
}

/**
 * @brief 获取电流故障状态
 * @param *pHandle 电机控制层对象
 * @retval 电流故障状态
 */
uint16_t MCI_GetCurrentFaults(MCI_Handle_t *pHandle)
{
    return (uint16_t)(STM_GetFaultState(pHandle->pSTM) >> 16);
}

/**
 * @brief 获取控制模式
 * @param *pHandle 电机控制层对象
 * @retval STC_Modality_t 控制模式
 */
STC_Modality_t MCI_GetControlMode(MCI_Handle_t *pHandle)
{
    return pHandle->LastModalitySetByUser;
}

/**
 * @brief 获取电机方向
 * @param *pHandle 电机控制层对象
 * @retval 1,正 -1,反
 */
int16_t MCI_GetImposedMotorDirection(MCI_Handle_t *pHandle)
{
    int16_t retVal = 1;

    switch (pHandle->lastCommand) {
    case MCI_EXECSPEEDRAMP:
        if (pHandle->hFinalSpeed < 0) {
            retVal = -1;
        }
        break;
    case MCI_EXECTORQUERAMP:
        if (pHandle->hFinalTorque < 0) {
            retVal = -1;
        }
        break;
    case MCI_SETCURRENTREFERENCES:
        if (pHandle->Iqdref.qI_Component1 < 0) {
            retVal = -1;
        }
        break;
    default:
        break;
    }
    return retVal;
}

/**
 * @brief 获取上一次的斜坡目标速度
 * @param *pHandle 电机控制层对象
 * @retval hRetVal
 */
int16_t MCI_GetLastRampFinalSpeed(MCI_Handle_t *pHandle)
{
    int16_t hRetVal = 0;

    /* Examine the last buffered commands */
    if (pHandle->lastCommand == MCI_EXECSPEEDRAMP) {
        hRetVal = pHandle->hFinalSpeed;
    }
    return hRetVal;
}

/**
 * @brief 是否完成斜坡过程
 * @param *pHandle 电机控制层对象
 * @retval true,完成 false,未完成
 */
bool MCI_RampCompleted(MCI_Handle_t *pHandle)
{
    bool retVal = false;

    if ((STM_GetState(pHandle->pSTM)) == RUN) {
        retVal = STC_RampCompleted(pHandle->pSTC);
    }

    return retVal;
}

/**
 * @brief 停止速度斜坡
 * @param *pHandle 电机控制层对象
 * @retval true,停止成功 false,停止失败
 */
bool MCI_StopSpeedRamp(MCI_Handle_t *pHandle)
{
    return STC_StopSpeedRamp(pHandle->pSTC);
}

/**
 * @brief 速度是否可靠
 * @param *pHandle 电机控制层对象
 * @retval true,可靠 false,不可靠
 */
bool MCI_GetSpdSensorReliability(MCI_Handle_t *pHandle)
{
    SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);

    return (SPD_Check(SpeedSensor));
}

/**
 * @brief 获取平均机械速度0.1Hz
 * @param *pHandle 电机控制层对象
 * @retval int16_t 平均机械速度0.1Hz
 */
int16_t MCI_GetAvrgMecSpeed01Hz(MCI_Handle_t *pHandle)
{
    SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);

    return (SPD_GetAvrgMecSpeed01Hz(SpeedSensor));
}

/**
 * @brief 获取机械速度参考值0.1Hz
 * @param *pHandle 电机控制层对象
 * @retval int16_t 机械速度参考值0.1Hz
 */
int16_t MCI_GetMecSpeedRef01Hz(MCI_Handle_t *pHandle)
{
    return (STC_GetMecSpeedRef01Hz(pHandle->pSTC));
}

/**
 * @brief 获取Iab
 * @param *pHandle 电机控制层对象
 * @retval Curr_Components Iab
 */
Curr_Components MCI_GetIab(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iab);
}

/**
 * @brief 获取Ialphabeta
 * @param *pHandle 电机控制层对象
 * @retval Curr_Components Ialphabeta
 */
Curr_Components MCI_GetIalphabeta(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Ialphabeta);
}

/**
 * @brief 获取Iqd
 * @param *pHandle 电机控制层对象
 * @retval Curr_Components Iqd
 */
Curr_Components MCI_GetIqd(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iqd);
}

/**
 * @brief 获取高频注入的Iqd
 * @param *pHandle 电机控制层对象
 * @retval Curr_Components IqdHF
 */
Curr_Components MCI_GetIqdHF(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->IqdHF);
}

/**
 * @brief 获取参考Iqd
 * @param *pHandle 电机控制层对象
 * @retval Curr_Components Iqdref
 */
Curr_Components MCI_GetIqdref(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iqdref);
}

/**
 * @brief 获取电压Vqd
 * @param *pHandle 电机控制层对象
 * @retval Volt_Components Vqd
 */
Volt_Components MCI_GetVqd(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Vqd);
}

/**
 * @brief 获取电压Valphabeta
 * @param *pHandle 电机控制层对象
 * @retval Volt_Components Valphabeta
 */
Volt_Components MCI_GetValphabeta(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Valphabeta);
}

/**
 * @brief 获取电角速度dpp
 * @param *pHandle 电机控制层对象
 * @retval int16_t dpp
 */
int16_t MCI_GetElAngledpp(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->hElAngle);
}

/**
 * @brief 获取参考转矩
 * @param *pHandle 电机控制层对象
 * @retval int16_t Tref
 */
int16_t MCI_GetTeref(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->hTeref);
}

/**
 * @brief 获取相电流幅值
 * @param *pHandle 电机控制层对象
 * @retval int16_t 相电流幅值
 */
int16_t MCI_GetPhaseCurrentAmplitude(MCI_Handle_t *pHandle)
{
    Curr_Components Local_Curr;
    int32_t wAux1, wAux2;

    Local_Curr = pHandle->pFOCVars->Ialphabeta;
    wAux1 = (int32_t)(Local_Curr.qI_Component1) * Local_Curr.qI_Component1;
    wAux2 = (int32_t)(Local_Curr.qI_Component2) * Local_Curr.qI_Component2;

    wAux1 += wAux2;
    wAux1 = MCM_Sqrt(wAux1);

    if (wAux1 > INT16_MAX) {
        wAux1 = (int32_t)INT16_MAX;
    }

    return ((int16_t)wAux1);
}

/**
 * @brief 获取相电压幅值
 * @param *pHandle 电机控制层对象
 * @retval int16_t 相电压幅值
 */
int16_t MCI_GetPhaseVoltageAmplitude(MCI_Handle_t *pHandle)
{
    Volt_Components Local_Voltage;
    int32_t wAux1, wAux2;

    Local_Voltage = pHandle->pFOCVars->Valphabeta;
    wAux1 = (int32_t)(Local_Voltage.qV_Component1) * Local_Voltage.qV_Component1;
    wAux2 = (int32_t)(Local_Voltage.qV_Component2) * Local_Voltage.qV_Component2;

    wAux1 += wAux2;
    wAux1 = MCM_Sqrt(wAux1);

    if (wAux1 > INT16_MAX) {
        wAux1 = (int32_t)INT16_MAX;
    }

    return ((int16_t)wAux1);
}

/**
 * @brief 设置Idref
 * @param *pHandle 电机控制层对象
 * @param hNewIdref 新的Idref
 * @retval None
 */
void MCI_SetIdref(MCI_Handle_t *pHandle, int16_t hNewIdref)
{
    pHandle->pFOCVars->Iqdref.qI_Component2 = hNewIdref;
    pHandle->pFOCVars->UserIdref = hNewIdref;
}

/**
 * @brief 清除Idref
 * @param *pHandle 电机控制层对象
 * @retval None
 */
void MCI_Clear_Iqdref(MCI_Handle_t *pHandle)
{
    pHandle->pFOCVars->Iqdref = STC_GetDefaultIqdref(pHandle->pSTC);
}
