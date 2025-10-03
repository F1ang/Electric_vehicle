#ifndef BSP_MCI_H
#define BSP_MCI_H

#include "main.h"
#include "bsp_global.h"

/* ½Ó¿Ú²ãº¯Êý */
void MCI_Init(MCI_Handle_t *pHandle, STM_Handle_t *pSTM, SpeednTorqCtrl_Handle_t *pSTC, pFOCVars_t pFOCVars);
void MCI_ExecSpeedRamp(MCI_Handle_t *pHandle, int16_t hFinalSpeed, uint16_t hDurationms);
void MCI_ExecTorqueRamp(MCI_Handle_t *pHandle, int16_t hFinalTorque, uint16_t hDurationms);
void MCI_SetCurrentReferences(MCI_Handle_t *pHandle, Curr_Components Iqdref);
bool MCI_StartMotor(MCI_Handle_t *pHandle);
bool MCI_StopMotor(MCI_Handle_t *pHandle);
bool MCI_FaultAcknowledged(MCI_Handle_t *pHandle);
bool MCI_EncoderAlign(MCI_Handle_t *pHandle);
void MCI_ExecBufferedCommands(MCI_Handle_t *pHandle);
MCI_CommandState_t MCI_IsCommandAcknowledged(MCI_Handle_t *pHandle);
State_t MCI_GetSTMState(MCI_Handle_t *pHandle);
uint16_t MCI_GetOccurredFaults(MCI_Handle_t *pHandle);
uint16_t MCI_GetCurrentFaults(MCI_Handle_t *pHandle);
STC_Modality_t MCI_GetControlMode(MCI_Handle_t *pHandle);
int16_t MCI_GetImposedMotorDirection(MCI_Handle_t *pHandle);
int16_t MCI_GetLastRampFinalSpeed(MCI_Handle_t *pHandle);
bool MCI_RampCompleted(MCI_Handle_t *pHandle);
bool MCI_StopSpeedRamp(MCI_Handle_t *pHandle);
bool MCI_GetSpdSensorReliability(MCI_Handle_t *pHandle);
int16_t MCI_GetAvrgMecSpeed01Hz(MCI_Handle_t *pHandle);
int16_t MCI_GetMecSpeedRef01Hz(MCI_Handle_t *pHandle);
Curr_Components MCI_GetIab(MCI_Handle_t *pHandle);
Curr_Components MCI_GetIalphabeta(MCI_Handle_t *pHandle);
Curr_Components MCI_GetIqd(MCI_Handle_t *pHandle);
Curr_Components MCI_GetIqdHF(MCI_Handle_t *pHandle);
Curr_Components MCI_GetIqdref(MCI_Handle_t *pHandle);
Volt_Components MCI_GetVqd(MCI_Handle_t *pHandle);
Volt_Components MCI_GetValphabeta(MCI_Handle_t *pHandle);
int16_t MCI_GetElAngledpp(MCI_Handle_t *pHandle);
int16_t MCI_GetTeref(MCI_Handle_t *pHandle);
int16_t MCI_GetPhaseCurrentAmplitude(MCI_Handle_t *pHandle);
int16_t MCI_GetPhaseVoltageAmplitude(MCI_Handle_t *pHandle);
void MCI_SetIdref(MCI_Handle_t *pHandle, int16_t hNewIdref);
void MCI_Clear_Iqdref(MCI_Handle_t *pHandle);

#endif /* BSP_MCI_H */
