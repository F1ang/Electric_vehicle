#ifndef BSP_REVUP_H
#define BSP_REVUP_H

#include "main.h"
#include "bsp_global.h"

#define RUC_MAX_PHASE_NUMBER 5u

/* 无传感器启动的阶段序列 */
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

/* 无传感器启动参数 */
#define ENABLE_SL_ALGO_FROM_PHASE 1
#define FIRST_SLESS_ALGO_PHASE    (ENABLE_SL_ALGO_FROM_PHASE - 1u)
#define RUC_OTF_PLL_RESET_TIMEOUT 100u /* 100ms */

/* 升压启动对象 */
typedef struct
{
    uint16_t hDurationms;       /* 升压时间 */
    int16_t hFinalMecSpeed01Hz; /* 目标机械转速，单位0.1Hz */
    int16_t hFinalTorque;       /* 目标扭矩 Iq */
    void *pNext;                /* 链表指针 */
} RevUpCtrl_PhaseParams_t;

/* 电机启动对象 */
typedef struct
{
    uint16_t hRUCFrequencyHz;                                 /* 执行频率=速度环频率 */
    int16_t hStartingMecAngle;                                /* 启动的起始机械角度 */
    uint16_t hPhaseRemainingTicks;                            /* 完成当前阶段的ticks */
    int16_t hDirection;                                       /* -1 or +1 */
    RevUpCtrl_PhaseParams_t *pCurrentPhaseParams;             /* 启动的进度 */
    RevUpCtrl_PhaseParams_t ParamsData[RUC_MAX_PHASE_NUMBER]; /* 启动阶段序列号 */
    uint8_t bPhaseNbr;                                        /* 启动序列号数 */

    uint8_t bFirstAccelerationStage; /* 斜坡启动的第一次加速阶段 */
    uint16_t hMinStartUpValidSpeed;  /* 启动所需的最低转速 */
    uint16_t hMinStartUpFlySpeed;    /* 运行所需的最低转速01Hz */
    int16_t hOTFFinalRevUpCurrent;   /* 最终目标转矩(顺序启动阶段) */
    uint16_t hOTFSection1Duration;   /* 实时启动时间ms */
    bool OTFStartupEnabled;          /* 观测器启动使能标志 */
    uint8_t bOTFRelCounter;          /* 状态观测器可靠性值 */
    bool OTFSCLowside;               /* 低侧开关状态 */
    bool EnteredZone1;               /* 进入启动区域1 */

    uint8_t bResetPLLTh;                    /* 重置PLL的计数阈值 */
    uint8_t bResetPLLCnt;                   /* OTF,清PLL的计数阈值 */
    uint8_t bStageCnt;                      /* 启动阶段计数 */
    RevUpCtrl_PhaseParams_t OTFPhaseParams; /* 实时启动阶段参数 */

    SpeednTorqCtrl_Handle_t *pSTC;     /* 速度转矩对象 */
    VirtualSpeedSensor_Handle_t *pVSS; /* 虚拟速度传感器对象 */
    STO_Handle_t *pSNSL;               /* 状态观测器对象 */
    PWMC_Handle_t *pPWM;               /* FOC相关的回调函数和参数 */
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
