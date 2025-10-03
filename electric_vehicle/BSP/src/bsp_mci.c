#include "bsp_mci.h"
#include "bsp_stc.h"
#include "bsp_state_machine.h"
#include "bsp_spd.h"

/**
 * @brief ������Ʋ����:״̬�������ٶ�ת�ض���FOC�����ʼ��
 * @param *pHandle ������Ʋ����
 * @param *pSTM ״̬������
 * @param *pSTC �ٶ�ת�ض���
 * @param pFOCVars FOC����
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
 * @brief �ٶ�б�¿���
 * @param *pHandle ������Ʋ����
 * @param hFinalSpeed Ŀ���ٶ�
 * @param hDurationms ����ʱ��
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
 * @brief ת��б�¿���
 * @param *pHandle ������Ʋ����
 * @param hFinalTorque Ŀ��ת��
 * @param hDurationms ����ʱ��
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
 * @brief ����Iqdref
 * @param *pHandle ������Ʋ����
 * @param Iqdref �����ο�ֵ
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
 * @brief �������
 * @param *pHandle ������Ʋ����
 * @retval true-״̬ת���ɹ�
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
 * @brief ֹͣ���
 * @param *pHandle ������Ʋ����
 * @retval true-״̬ת���ɹ�
 */
bool MCI_StopMotor(MCI_Handle_t *pHandle)
{
    return STM_NextState(pHandle->pSTM, ANY_STOP);
}

/**
 * @brief �����Ƿ�����
 * @param *pHandle
 * @retval ����
 */
bool MCI_FaultAcknowledged(MCI_Handle_t *pHandle)
{
    return STM_FaultAcknowledged(pHandle->pSTM);
}

/**
 * @brief ����������
 * @param *pHandle
 * @retval true-����ɹ�
 */
bool MCI_EncoderAlign(MCI_Handle_t *pHandle)
{
    return STM_NextState(pHandle->pSTM, IDLE_ALIGNMENT);
}

/**
 * @brief �ٶ�/ת�ؿ���ѡ��,ֻ��Ҫִ��һ���ٶ�/ת��ָ��
 * @param *pHandle ������Ʋ����
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

            /* ֻ��Ҫִ��һ���ٶ�/ת��ָ�� */
            if (commandHasBeenExecuted) {
                pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
            } else {
                pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
            }
        }
    }
}

/**
 * @brief ���Ʋ�ִ��״̬
 * @param *pHandle ������Ʋ����
 * @retval ������Ʋ�״̬
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
 * @brief ��ȡ��ǰ״̬��
 * @param *pHandle ������Ʋ����
 * @retval ������Ʋ�״̬��
 */
State_t MCI_GetSTMState(MCI_Handle_t *pHandle)
{
    return STM_GetState(pHandle->pSTM);
}

/**
 * @brief ��ȡ����״̬
 * @param *pHandle ������Ʋ����
 * @retval ����״̬
 */
uint16_t MCI_GetOccurredFaults(MCI_Handle_t *pHandle)
{
    return (uint16_t)(STM_GetFaultState(pHandle->pSTM));
}

/**
 * @brief ��ȡ��������״̬
 * @param *pHandle ������Ʋ����
 * @retval ��������״̬
 */
uint16_t MCI_GetCurrentFaults(MCI_Handle_t *pHandle)
{
    return (uint16_t)(STM_GetFaultState(pHandle->pSTM) >> 16);
}

/**
 * @brief ��ȡ����ģʽ
 * @param *pHandle ������Ʋ����
 * @retval STC_Modality_t ����ģʽ
 */
STC_Modality_t MCI_GetControlMode(MCI_Handle_t *pHandle)
{
    return pHandle->LastModalitySetByUser;
}

/**
 * @brief ��ȡ�������
 * @param *pHandle ������Ʋ����
 * @retval 1,�� -1,��
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
 * @brief ��ȡ��һ�ε�б��Ŀ���ٶ�
 * @param *pHandle ������Ʋ����
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
 * @brief �Ƿ����б�¹���
 * @param *pHandle ������Ʋ����
 * @retval true,��� false,δ���
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
 * @brief ֹͣ�ٶ�б��
 * @param *pHandle ������Ʋ����
 * @retval true,ֹͣ�ɹ� false,ֹͣʧ��
 */
bool MCI_StopSpeedRamp(MCI_Handle_t *pHandle)
{
    return STC_StopSpeedRamp(pHandle->pSTC);
}

/**
 * @brief �ٶ��Ƿ�ɿ�
 * @param *pHandle ������Ʋ����
 * @retval true,�ɿ� false,���ɿ�
 */
bool MCI_GetSpdSensorReliability(MCI_Handle_t *pHandle)
{
    SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);

    return (SPD_Check(SpeedSensor));
}

/**
 * @brief ��ȡƽ����е�ٶ�0.1Hz
 * @param *pHandle ������Ʋ����
 * @retval int16_t ƽ����е�ٶ�0.1Hz
 */
int16_t MCI_GetAvrgMecSpeed01Hz(MCI_Handle_t *pHandle)
{
    SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);

    return (SPD_GetAvrgMecSpeed01Hz(SpeedSensor));
}

/**
 * @brief ��ȡ��е�ٶȲο�ֵ0.1Hz
 * @param *pHandle ������Ʋ����
 * @retval int16_t ��е�ٶȲο�ֵ0.1Hz
 */
int16_t MCI_GetMecSpeedRef01Hz(MCI_Handle_t *pHandle)
{
    return (STC_GetMecSpeedRef01Hz(pHandle->pSTC));
}

/**
 * @brief ��ȡIab
 * @param *pHandle ������Ʋ����
 * @retval Curr_Components Iab
 */
Curr_Components MCI_GetIab(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iab);
}

/**
 * @brief ��ȡIalphabeta
 * @param *pHandle ������Ʋ����
 * @retval Curr_Components Ialphabeta
 */
Curr_Components MCI_GetIalphabeta(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Ialphabeta);
}

/**
 * @brief ��ȡIqd
 * @param *pHandle ������Ʋ����
 * @retval Curr_Components Iqd
 */
Curr_Components MCI_GetIqd(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iqd);
}

/**
 * @brief ��ȡ��Ƶע���Iqd
 * @param *pHandle ������Ʋ����
 * @retval Curr_Components IqdHF
 */
Curr_Components MCI_GetIqdHF(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->IqdHF);
}

/**
 * @brief ��ȡ�ο�Iqd
 * @param *pHandle ������Ʋ����
 * @retval Curr_Components Iqdref
 */
Curr_Components MCI_GetIqdref(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Iqdref);
}

/**
 * @brief ��ȡ��ѹVqd
 * @param *pHandle ������Ʋ����
 * @retval Volt_Components Vqd
 */
Volt_Components MCI_GetVqd(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Vqd);
}

/**
 * @brief ��ȡ��ѹValphabeta
 * @param *pHandle ������Ʋ����
 * @retval Volt_Components Valphabeta
 */
Volt_Components MCI_GetValphabeta(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->Valphabeta);
}

/**
 * @brief ��ȡ����ٶ�dpp
 * @param *pHandle ������Ʋ����
 * @retval int16_t dpp
 */
int16_t MCI_GetElAngledpp(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->hElAngle);
}

/**
 * @brief ��ȡ�ο�ת��
 * @param *pHandle ������Ʋ����
 * @retval int16_t Tref
 */
int16_t MCI_GetTeref(MCI_Handle_t *pHandle)
{
    return (pHandle->pFOCVars->hTeref);
}

/**
 * @brief ��ȡ�������ֵ
 * @param *pHandle ������Ʋ����
 * @retval int16_t �������ֵ
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
 * @brief ��ȡ���ѹ��ֵ
 * @param *pHandle ������Ʋ����
 * @retval int16_t ���ѹ��ֵ
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
 * @brief ����Idref
 * @param *pHandle ������Ʋ����
 * @param hNewIdref �µ�Idref
 * @retval None
 */
void MCI_SetIdref(MCI_Handle_t *pHandle, int16_t hNewIdref)
{
    pHandle->pFOCVars->Iqdref.qI_Component2 = hNewIdref;
    pHandle->pFOCVars->UserIdref = hNewIdref;
}

/**
 * @brief ���Idref
 * @param *pHandle ������Ʋ����
 * @retval None
 */
void MCI_Clear_Iqdref(MCI_Handle_t *pHandle)
{
    pHandle->pFOCVars->Iqdref = STC_GetDefaultIqdref(pHandle->pSTC);
}
