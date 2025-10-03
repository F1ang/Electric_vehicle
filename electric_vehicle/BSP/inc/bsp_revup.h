#ifndef BSP_REVUP_H
#define BSP_REVUP_H

#include "main.h"
#include "bsp_global.h"

#define RUC_MAX_PHASE_NUMBER 5u

/* �޴����������Ľ׶����� */
#define STARTING_ANGLE_DEG 90 /*!< degrees [0...359] */
/* Phase 1 */
#define PHASE1_DURATION        300 /*milliseconds */
#define PHASE1_FINAL_SPEED_RPM 50  /* rpm */
#define PHASE1_FINAL_CURRENT   798
/* Phase 2 */
#define PHASE2_DURATION        300 /*milliseconds */
#define PHASE2_FINAL_SPEED_RPM 100 /* rpm */
#define PHASE2_FINAL_CURRENT   1117
/* Phase 3 */
#define PHASE3_DURATION        500 /*milliseconds */
#define PHASE3_FINAL_SPEED_RPM 300 /* rpm */
#define PHASE3_FINAL_CURRENT   1436
/* Phase 4 */
#define PHASE4_DURATION        600 /*milliseconds */
#define PHASE4_FINAL_SPEED_RPM 500 /* rpm */
#define PHASE4_FINAL_CURRENT   1596
/* Phase 5 */
#define PHASE5_DURATION        800  /* milliseconds */
#define PHASE5_FINAL_SPEED_RPM 1000 /* rpm */
#define PHASE5_FINAL_CURRENT   1596

/* �޴������������� */
#define ENABLE_SL_ALGO_FROM_PHASE 1
#define FIRST_SLESS_ALGO_PHASE    (ENABLE_SL_ALGO_FROM_PHASE - 1u)
#define RUC_OTF_PLL_RESET_TIMEOUT 100u /* 100ms */

/* ��ѹ�������� */
typedef struct
{
    uint16_t hDurationms;       /* ��ѹʱ�� */
    int16_t hFinalMecSpeed01Hz; /* Ŀ���еת�٣���λ0.1Hz */
    int16_t hFinalTorque;       /* Ŀ��Ť�� Iq */
    void *pNext;                /* ����ָ�� */
} RevUpCtrl_PhaseParams_t;

/* ����������� */
typedef struct
{
    uint16_t hRUCFrequencyHz;                                 /* ִ��Ƶ��=�ٶȻ�Ƶ�� */
    int16_t hStartingMecAngle;                                /* ��������ʼ��е�Ƕ� */
    uint16_t hPhaseRemainingTicks;                            /* ��ɵ�ǰ�׶ε�ticks */
    int16_t hDirection;                                       /* -1 or +1 */
    RevUpCtrl_PhaseParams_t *pCurrentPhaseParams;             /* �����Ľ��� */
    RevUpCtrl_PhaseParams_t ParamsData[RUC_MAX_PHASE_NUMBER]; /* �����׶����к� */
    uint8_t bPhaseNbr;                                        /* �������к��� */

    uint8_t bFirstAccelerationStage; /* б�������ĵ�һ�μ��ٽ׶� */
    uint16_t hMinStartUpValidSpeed;  /* ������������ת�� */
    uint16_t hMinStartUpFlySpeed;    /* ������������ת��01Hz */
    int16_t hOTFFinalRevUpCurrent;   /* ����Ŀ��ת��(˳�������׶�) */
    uint16_t hOTFSection1Duration;   /* ʵʱ����ʱ��ms */
    bool OTFStartupEnabled;          /* �۲�������ʹ�ܱ�־ */
    uint8_t bOTFRelCounter;          /* ״̬�۲����ɿ���ֵ */
    bool OTFSCLowside;               /* �Ͳ࿪��״̬ */
    bool EnteredZone1;               /* ������������1 */

    uint8_t bResetPLLTh;                    /* ����PLL�ļ�����ֵ */
    uint8_t bResetPLLCnt;                   /* OTF,��PLL�ļ�����ֵ */
    uint8_t bStageCnt;                      /* �����׶μ��� */
    RevUpCtrl_PhaseParams_t OTFPhaseParams; /* ʵʱ�����׶β��� */

    SpeednTorqCtrl_Handle_t *pSTC;     /* �ٶ�ת�ض��� */
    VirtualSpeedSensor_Handle_t *pVSS; /* �����ٶȴ��������� */
    STO_Handle_t *pSNSL;               /* ״̬�۲������� */
    PWMC_Handle_t *pPWM;               /* FOC��صĻص������Ͳ��� */
} RevUpCtrl_Handle_t;

extern RevUpCtrl_Handle_t RevUpControlM1;

void RUC_Init(RevUpCtrl_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS,
              STO_Handle_t *pSNSL, PWMC_Handle_t *pPWM);
void RUC_Clear(RevUpCtrl_Handle_t *pHandle, int16_t hMotorDirection);
bool RUC_Exec(RevUpCtrl_Handle_t *pHandle);
bool RUC_OTF_Exec(RevUpCtrl_Handle_t *pHandle);
bool RUC_Completed(RevUpCtrl_Handle_t *pHandle);
void RUC_Stop(RevUpCtrl_Handle_t *pHandle);
bool RUC_FirstAccelerationStageReached(RevUpCtrl_Handle_t *pHandle);
void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms);
void RUC_SetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalMecSpeed01Hz);
void RUC_SetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalTorque);
uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);
int16_t RUC_GetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);
int16_t RUC_GetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);
uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle);
bool RUC_Get_SCLowsideOTF_Status(RevUpCtrl_Handle_t *pHandle);

#endif /* BSP_REVUP_H */
