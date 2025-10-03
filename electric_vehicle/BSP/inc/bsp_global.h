#ifndef BSP_GLOBAL_H
#define BSP_GLOBAL_H

#include "main.h"
#include "mc_math.h"
#include "mc_type.h"
#include <arm_math.h>

#define OPEN_DEBUG_MODE   0 /* 是否使能FOC开环调试模式(至少选下面一种传感器) */
#define DAC_TEST_ENABLE   1 /* 是否使能DAC模式 */
#define DAC_TEST          0 /* 是否使能DAC测试模式 */
#define PRINT_TEST        0 /* 是否使能打印测试 */
#define ENCODER_ENABLE    0 /* 是否使能编码器 */
#define HANDLE_BREAK      0 /* 是否使能Uq与转把的标幺化 */
#define HALL_ENABLE       1 /* 是否使能HALL */
#define SENSORLESS_ENABLE 0 /* 是否使能无感IF+SMO+PLL */

#define HFI_SMO_ENABLE       0 /* 是否使能HFI */
#define OPEN_SENSORLESS_MODE 0 /* 是否使能无感开环调试模式 */
#define FLUX_SENSOR_ENABLE   0 /* 是否使能磁链观测器 */

/* 选择最大采样窗口 */
#define MAX_MODULATION_99_PER_CENT 1
#define MAX_MODULATION_96_PER_CENT 0
#define MAX_MODULATION_95_PER_CENT 0
#define MAX_MODULATION_94_PER_CENT 0

/* 扇区 */
#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u
#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* 采样通道 */
#define NB_CONVERSIONS 16u /* 16次采样 */
#define PHASE_A_MSK    (uint32_t)((uint32_t)(6) << 15)
#define PHASE_B_MSK    (uint32_t)((uint32_t)(8) << 15)
#define PHASE_C_MSK    (uint32_t)((uint32_t)(9) << 15)
#define ADC_SR_MASK    ((uint32_t)(0xC))

#define CONV_STARTED  ((uint32_t)(0x8))
#define CONV_FINISHED ((uint32_t)(0xC))
#define FLAGS_CLEARED ((uint32_t)(0x0))
#define MAX_TWAIT     0 /* 注入ADC转换标志被清除,等待Tw后进行再次检测 */

/* 采样点 */
// #define DEADTIME_NS 200
#define ADC_CLK_MHz          21
#define SAMPLING_TIME_NS     (3 * 1000uL / ADC_CLK_MHz)
#define TRIG_CONV_LATENCY_NS 150
#define MAX_TNTR_NS          150 /* rise time */
#define TW_AFTER             ((uint16_t)(((DEADTIME_NS + MAX_TNTR_NS) * ADV_TIM_CLK_MHz) / 1000ul))
#define TW_BEFORE            (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS + TRIG_CONV_LATENCY_NS))) * ADV_TIM_CLK_MHz) / 1000ul)) + 1u)

/* 时间滴答 */
#define CHARGE_BOOT_CAP_MS     10
#define CHARGE_BOOT_CAP_MS2    10
#define OFFCALIBRWAIT_MS       0
#define OFFCALIBRWAIT_MS2      0
#define STOPPERMANENCY_MS      400
#define STOPPERMANENCY_MS2     400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS) / 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2) / 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS) / 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2) / 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS) / 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / 1000)

/* 电源 */
#define MCU_SUPPLY_VOLTAGE 3.30

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE (uint16_t)((uint16_t)(PWM_FREQUENCY) / 1)
#define REP_COUNTER        (uint16_t)((1 * 2u) - 1u)

#define SPEED_LOOP_FREQUENCY_HZ    500 /* 速度环频率500hz */
#define MEDIUM_FREQUENCY_TASK_RATE (uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define SYS_TICK_FREQUENCY         2000
#define UI_TASK_FREQUENCY_HZ       10
#define SERIAL_COM_TIMEOUT_INVERSE 25
#define SERIAL_COM_ATR_TIME_MS     20

#define MF_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY / SPEED_LOOP_FREQUENCY_HZ) - 1u
#define UI_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY / UI_TASK_FREQUENCY_HZ) - 1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY / SERIAL_COM_TIMEOUT_INVERSE) - 1u
#define SERIALCOM_ATR_TIME_TICKS          (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED     10000 /* rpm, mechanical */
#define MIN_APPLICATION_SPEED     0
#define MEAS_ERRORS_BEFORE_FAULTS 200

/*** Encoder **********************/
#define ENC_MEAS_ERRORS_BEFORE_FAULTS 200
#define ENC_INVERT_SPEED              DISABLE
#define ENC_AVERAGING_FIFO_DEPTH      16
#define ENC_SPEED_ARRAY_SIZE          ((uint8_t)16) /* 2^4 */

/* Encoder alignment */
#define ALIGNMENT_DURATION  700 /* milliseconds */
#define ALIGNMENT_ANGLE_DEG 90  /* degrees [0...359] */
#define FINAL_I_ALIGNMENT   317 /* s16A */
#define ALIGNMENT_ANGLE_S16 (int16_t)(ALIGNMENT_ANGLE_DEG * 65536u / 360u)
/*
    Av=4.02 Rs=0.02 I=0.2A
    FINAL_I_ALIGNMENT/65535=I*Rs*Av/3.3=>FINAL_I_ALIGNMENT=317.5
*/

/****** Hall sensors ************/
#define HALL_MEAS_ERRORS_BEFORE_FAULTS 200
#define HALL_AVERAGING_FIFO_DEPTH      6

#define DEGREES_120          0u
#define DEGREES_60           1u
#define HALL_SPEED_FIFO_SIZE ((uint8_t)18)
#define HALL_PHASE_SHIFT     270
#define HALL_TIM_CLK         84000000uL

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          4        /* Number of motor pole pairs */
#define RS                     0.65     /* Stator resistance , ohm*/
#define LS                     0.000620 /* Stator inductance, H For I-PMSM it is equal to Lq */
#define NOMINAL_CURRENT        4997
#define MOTOR_MAX_SPEED_RPM    3000  /* Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT 4.1   /* Volts RMS ph-ph /kRPM */
#define ID_DEMAG               -4997 /* Demagnetization current */

/************************* 状态观测器+PLL参数 **************************/
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM  580 /* Minimum speed for observer */
#define NB_CONSECUTIVE_TESTS   2
#define SPEED_BAND_UPPER_LIMIT 17
#define SPEED_BAND_LOWER_LIMIT 15
#define TRANSITION_DURATION    500 /* Switch over duration, ms */

#define RSHUNT             0.020 /* 电阻0.020 */
#define AMPLIFICATION_GAIN 4.02  /*  ISC增益 */

#define MAX_BEMF_VOLTAGE (uint16_t)((MAX_APPLICATION_SPEED * 1.2 *      \
                                     MOTOR_VOLTAGE_CONSTANT * SQRT_2) / \
                                    (1000u * SQRT_3))

#define VBUS_PARTITIONING_FACTOR 0.0463 /* VBUS衰减程度 */
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR
#define MAX_VOLTAGE              (int16_t)((MCU_SUPPLY_VOLTAGE / 2) / BUS_ADC_CONV_RATIO) /* 最大相电压 */
#define MAX_CURRENT              (MCU_SUPPLY_VOLTAGE / (2 * RSHUNT * AMPLIFICATION_GAIN)) /* 最大电流 */

/******************** 静止坐标系 ********************/
#define F1    16384 /* Q14 */
#define F2    2048  /* Q12 */
#define GAIN1 -23932
#define GAIN2 20828

#define VARIANCE_THRESHOLD    0.4 /* 速度估计值的最大可接受方差(百分比) */
#define BEMF_CONSISTENCY_TOL  64  /* BEMF的准确度 */
#define BEMF_CONSISTENCY_GAIN 64  /* BEMF的增益准确度 */

/******************** 旋转坐标系 ********************/
#define C6_COMP_CONST1 (int32_t)1043038 /* 2*pi*f0信号的带宽 */
#define C6_COMP_CONST2 (int32_t)10430   /* 2*pi*f0信号的相角裕度 */
#define CORD_F1        16384            /* 系数F1 */
#define CORD_F2        2048             /* 系数F2 */
#define CORD_GAIN1     -20932           /* 增益K1 */
#define CORD_GAIN2     20828            /* 增益K2 */

#define CORD_VARIANCE_THRESHOLD 4 /* 估计速度最大可接受的偏差 */

#define CORD_MEAS_ERRORS_BEFORE_FAULTS 16 /* 旋转坐标系,报告速度反馈错误之前进行方差测试时出现的连续错误次数 */
#define CORD_FIFO_DEPTH_DPP            64 /* 速度平滑缓存大小 */
#define CORD_FIFO_DEPTH_01HZ           64 /* 速度平滑缓存大小 */
#define CORD_MAX_ACCEL_DPPP            39 /* 最大瞬时加速度 单位：dpp*/

#define CORD_BEMF_CONSISTENCY_TOL  64 /* BEMF的准确度 */
#define CORD_BEMF_CONSISTENCY_GAIN 64 /* BEMF的增益准确度  */

/* Default settings */
#define DEFAULT_CONTROL_MODE STC_SPEED_MODE /*!< STC_TORQUE_MODE or \
                                              STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM 0
#define DEFAULT_TORQUE_COMPONENT 0
#define DEFAULT_FLUX_COMPONENT   0

/* SensorLess parameters */
#define MOVE_FILTER_BUF_MAXSIZE 500 /*滑动滤波最大区间*/

/* FOC相关的回调函数和参数 */
typedef struct PWMC_Handle PWMC_Handle_t;

/* 地址指针函数参数类型返回值 */
typedef void (*PWMC_GetPhaseCurr_Cb_t)(PWMC_Handle_t *pHandle, Curr_Components *pStator_Currents);
typedef void (*PWMC_Generic_Cb_t)(PWMC_Handle_t *pHandle);
typedef uint16_t (*PWMC_OverCurr_Cb_t)(PWMC_Handle_t *pHandle);
typedef uint16_t (*PWMC_SetSampPointSectX_Cb_t)(PWMC_Handle_t *pHandle);
typedef void (*PWMC_SetOcpRefVolt_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDACVref);

/* STO指针函数 */
typedef struct STO_Handle STO_Handle_t;
typedef void (*STO_ForceConvergency1_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_ForceConvergency2_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_OtfResetPLL_Cb_t)(STO_Handle_t *pHandle);
typedef bool (*STO_SpeedReliabilityCheck_Cb_t)(STO_Handle_t *pHandle);

/* FOC相关的回调函数和参数 */
struct PWMC_Handle {
    PWMC_GetPhaseCurr_Cb_t pFctGetPhaseCurrents; /* 获取相电流 */
    PWMC_Generic_Cb_t pFctSwitchOffPwm;          /* 关闭PWM输出 */
    PWMC_Generic_Cb_t pFctSwitchOnPwm;           /* 开启PWM输出 */
    PWMC_Generic_Cb_t pFctCurrReadingCalib;      /* 电流测量校准 */
    PWMC_Generic_Cb_t pFctTurnOnLowSides;        /* 开启下管 */

    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect1; /* 设置扇区1的ADC采样点 */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect2; /* 设置扇区2的ADC采样点 */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect3; /* 设置扇区3的ADC采样点 */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect4; /* 设置扇区4的ADC采样点 */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect5; /* 设置扇区5的ADC采样点 */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect6; /* 设置扇区6的ADC采样点 */

    uint16_t hT_Sqrt3; /* sqrt(3)*T */
    uint16_t hSector;  /* 扇区 */
    uint16_t hCntPhA;  /* CCR1 */
    uint16_t hCntPhB;  /* CCR2 */
    uint16_t hCntPhC;  /* CCR3 */
    uint16_t SWerror;  /* 开关错误 */

    int16_t hIa; /* 电流Ia */
    int16_t hIb; /* 电流Ib */
    int16_t hIc; /* 电流Ic */

    uint16_t hDeadTime; /* 死区 */
    uint16_t hTafter;   /* 中间点后 */
    uint16_t hTbefore;  /* 中间点前 */
    uint16_t Tw;        /* 注入ADC转换标志被清除,等待Tw后进行再次检测 */

    uint16_t hPWMperiod;                /* PWM周期 */
    uint16_t hOffCalibrWaitTimeCounter; /* 校准等待时间计数 */
    uint16_t hOffCalibrWaitTicks;       /* 校准等待时间 */
    bool bTurnOnLowSidesAction;         /* 下管是否导通 */
};

typedef struct {
    PWMC_Handle_t _Super;   /* 基类 */
    uint32_t wPhaseAOffset; /* A相偏置 */
    uint32_t wPhaseBOffset; /* B相偏置 */
    uint32_t wPhaseCOffset; /* C相偏置 */

    uint32_t wADC1Channel;   /* ADC1采集的通道 */
    uint32_t wADC2Channel;   /* ADC2采集的通道 */
    uint16_t Half_PWMPeriod; /* 半个周期 */

    volatile uint8_t bIndex;   /* 滑动平均索引 */
    uint32_t wADCTriggerSet;   /* ADC触发使能和极性配置 */
    uint32_t wADCTriggerUnSet; /* ADC触发失能 */
    bool OverCurrentFlag;      /* 过流标志 */

    volatile uint8_t bSoFOC; /* FOC与TIM8更新中断频率标志 */
} PWMC_R3_F4_Handle_t;

/* 状态机 */
typedef enum {
    ICLWAIT = 12,
    IDLE = 0,
    IDLE_ALIGNMENT = 1,
    ALIGN_CHARGE_BOOT_CAP = 13,
    ALIGN_OFFSET_CALIB = 14,
    ALIGN_CLEAR = 15,
    ALIGNMENT = 2,
    IDLE_START = 3,
    CHARGE_BOOT_CAP = 16,
    OFFSET_CALIB = 17,
    CLEAR = 18,
    START = 4,
    START_RUN = 5,
    RUN = 6,
    ANY_STOP = 7,
    STOP = 8,
    STOP_IDLE = 9,
    FAULT_NOW = 10,
    FAULT_OVER = 11
} State_t;

/* 电机控制层状态 */
typedef enum {
    MCI_BUFFER_EMPTY,                  /* 缓冲区空 */
    MCI_COMMAND_NOT_ALREADY_EXECUTED,  /* 命令未执行 */
    MCI_COMMAND_EXECUTED_SUCCESFULLY,  /* 命令已执行 */
    MCI_COMMAND_EXECUTED_UNSUCCESFULLY /* 命令执行失败 */
} MCI_CommandState_t;

/* 用户命令 */
typedef enum {
    MCI_NOCOMMANDSYET,        /* 无命令 */
    MCI_EXECSPEEDRAMP,        /* 执行速度调节 */
    MCI_EXECTORQUERAMP,       /* 执行转矩调节 */
    MCI_SETCURRENTREFERENCES, /* 设置电流参考值 */
} MCI_UserCommands_t;

/* 动作状态 */
typedef enum CRCAction {
    CRC_START, /* 开始 */
    CRC_EXEC   /* 执行 */
} CRCAction_t;

typedef struct PID_Handle {
    int16_t hDefKpGain;           /* default Kp gain */
    int16_t hDefKiGain;           /* default Ki gain */
    int16_t hKpGain;              /* Kp gain */
    int16_t hKiGain;              /* Ki gain */
    int32_t wIntegralTerm;        /* integral term */
    int32_t wUpperIntegralLimit;  /* upper integral limit */
    int32_t wLowerIntegralLimit;  /* lower integral limit */
    int16_t hUpperOutputLimit;    /* upper output limit */
    int16_t hLowerOutputLimit;    /* lower output limit */
    uint16_t hKpDivisor;          /* Kp divisor */
    uint16_t hKiDivisor;          /* Ki divisor */
    uint16_t hKpDivisorPOW2;      /* Kp divisor power of 2,2^n=x,log2(x)=n */
    uint16_t hKiDivisorPOW2;      /* Ki divisor power of 2,2^n=x,log2(x)=n */
    int16_t hDefKdGain;           /* default Kd gain */
    int16_t hKdGain;              /* Kd gain */
    uint16_t hKdDivisor;          /* Kd divisor */
    uint16_t hKdDivisorPOW2;      /* Kd divisor power of 2,2^n=x,log2(x)=n */
    int32_t wPrevProcessVarError; /* previous process variable error */
} PID_Handle_t;

/* 位置传感器的基类 */
typedef struct
{
    int16_t hElAngle;                   /* 电角度 */
    int16_t hMecAngle;                  /* 机械角度 */
    int16_t hAvrMecSpeed01Hz;           /* 平均机械速度0.1Hz */
    int16_t hElSpeedDpp;                /* 电角度速度 */
    int16_t hMecAccel01HzP;             /* 机械加速度0.1HzP */
    uint8_t bSpeedErrorNumber;          /* 速度异常次数 */
    uint8_t bElToMecRatio;              /* 极对数 */
    uint16_t hMaxReliableMecSpeed01Hz;  /* 最大可靠机械速度0.1Hz */
    uint16_t hMinReliableMecSpeed01Hz;  /* 最小可靠机械速度0.1Hz */
    uint8_t bMaximumSpeedErrorsNumber;  /* 最大速度异常次数 */
    uint16_t hMaxReliableMecAccel01HzP; /* 最大可靠机械加速度0.1Hz  */
    uint16_t hMeasurementFrequency;     /* 测量频率16kHz */
} SpeednPosFdbk_Handle_t;

/* 速度转矩控制对象 */
typedef struct
{
    STC_Modality_t Mode;                 /* 电机模式:转矩、速度 */
    int16_t TargetFinal;                 /* 目标电流 */
    int32_t SpeedRef01HzExt;             /* 目标速度(当前Q16) */
    int32_t TorqueRef;                   /* 目标转矩(当前Q16) */
    uint32_t RampRemainingStep;          /* Ramp剩余步数 */
    PID_Handle_t *PISpeed;               /* 速度控制PID */
    SpeednPosFdbk_Handle_t *SPD;         /* 速度及位置反馈 */
    int32_t IncDecAmount;                /* 增量/减量值 */
    uint16_t STCFrequencyHz;             /* 速度转矩控制频率 */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* 最大正向机械速度0.1Hz */
    uint16_t MinAppPositiveMecSpeed01Hz; /* 最小正向机械速度0.1Hz */
    int16_t MaxAppNegativeMecSpeed01Hz;  /* 最大反向机械速度0.1Hz */
    int16_t MinAppNegativeMecSpeed01Hz;  /* 最小反向机械速度0.1Hz */
    uint16_t MaxPositiveTorque;          /* 最大正向转矩 */
    int16_t MinNegativeTorque;           /* 最小反向转矩 */
    STC_Modality_t ModeDefault;          /* 默认模式 */
    int16_t MecSpeedRef01HzDefault;      /* 默认目标机械速度0.1Hz */
    int16_t TorqueRefDefault;            /* 默认目标转矩 */
    int16_t IdrefDefault;                /* 默认电流Id */
} SpeednTorqCtrl_Handle_t;

/* 状态机对象 */
typedef struct
{
    State_t bState;          /* 状态 */
    uint16_t hFaultNow;      /* 故障 */
    uint16_t hFaultOccurred; /* 故障发生 */
    uint8_t bStop_Motor;     /* 1-启动 0-停止 */
    uint8_t bSenLess_Handle; /* 0-非转把 1-转把 */
} STM_Handle_t;

/* 电机控制层对象 */
typedef struct
{
    STM_Handle_t *pSTM;                   /* 状态机 */
    SpeednTorqCtrl_Handle_t *pSTC;        /* 速度和转矩控制 */
    pFOCVars_t pFOCVars;                  /* FOC变量 */
    MCI_UserCommands_t lastCommand;       /* 上一个命令 */
    int16_t hFinalSpeed;                  /* 最终速度 */
    int16_t hFinalTorque;                 /* 最终转矩 */
    Curr_Components Iqdref;               /* 电流参考值 */
    uint16_t hDurationms;                 /* 持续时间 */
    MCI_CommandState_t CommandState;      /* 命令状态 */
    STC_Modality_t LastModalitySetByUser; /* 上一次用户设置的模式 */
} MCI_Handle_t;

/* 虚拟速度传感器对象 */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super;     /* 位置传感器成员属基类 */
    int32_t wElAccDppP32;              /* 电加速度dpp 65536 */
    int32_t wElSpeedDpp32;             /* 电转速dpp 65536 */
    uint16_t hRemainingStep;           /* final speed剩余步数 */
    int16_t hFinalMecSpeed01Hz;        /* final speed 0.1Hz */
    bool bTransitionStarted;           /* 状态转换(斜坡)开始 */
    bool bTransitionEnded;             /* 状态转换结束 */
    int16_t hTransitionRemainingSteps; /* 状态转换剩余步数 */
    int16_t hElAngleAccu;              /* 电角度加速度 */
    bool bTransitionLocked;            /* 开始转换加速度 */
    bool bCopyObserver;
    uint16_t hSpeedSamplingFreqHz; /* 速度采样频率(Hz) */
    int16_t hTransitionSteps;      /* 状态转换步数 */
} VirtualSpeedSensor_Handle_t;

/* 编码器对象 */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super;     /* 位置传感器成员属基类 */
    uint16_t PulseNumber;              /* 编码器脉冲数(*4) */
    FunctionalState RevertSignal;      /* ENABLE/DISABLE */
    uint16_t SpeedSamplingFreq01Hz;    /* 速度采样频率(0.1Hz) */
    uint8_t SpeedBufferSize;           /* 速度缓冲区大小 */
    volatile uint16_t TimerOverflowNb; /* 溢出*/
    bool SensorIsReliable;             /* 编码器是否可靠 */
    uint16_t PreviousCapture;          /* 上一次采样的脉冲数 */

    int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /* 速度计算的差分脉冲数 */
    volatile uint8_t DeltaCapturesIndex;               /* 差分脉冲数的索引 */
    uint32_t U32MAXdivPulseNumber;                     /* U32MAX/hPulseNumber */
    uint16_t SpeedSamplingFreqHz;                      /* 速度采样频率(Hz) */
    bool TimerOverflowError;                           /* 定时器溢出错误 */
} ENCODER_Handle_t;

/* 编码器对齐对象 */
typedef struct
{
    SpeednTorqCtrl_Handle_t *pSTC;     /* 速度和转矩对象 */
    VirtualSpeedSensor_Handle_t *pVSS; /* 虚拟速度传感器对象 */
    ENCODER_Handle_t *pENC;            /* 编码器对象 */
    uint16_t hRemainingTicks;          /* 剩余滴答次数 */
    bool EncAligned;                   /* 编码器已对齐 */
    bool EncRestart;                   /* 编码器重启对齐 */
    uint16_t hEACFrequencyHz;          /* 频率 */
    int16_t hFinalTorque;              /* 最终转矩 */
    int16_t hElAngle;                  /* 对齐电角度 */
    uint16_t hDurationms;              /* 持续时间(ms) */
    uint8_t bElToMecRatio;             /* 极对数 */
} EncAlign_Handle_t;

/* Hall传感器对象 */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* 位置传感器成员属基类 */
    uint8_t SensorPlacement;       /* 120 or 60 degrees */
    int16_t PhaseShift;            /* H1上升沿与A相最大BEMF的偏移 */
    uint16_t SpeedSamplingFreqHz;  /* 计算电机转速的频率0.1HZ */
    uint8_t SpeedBufferSize;       /*滑动平均速度*/
    uint32_t TIMClockFreq;         /* 时钟频率 */
    bool SensorIsReliable;         /* 传感器是否可靠 */
    volatile bool RatioDec;        /* 递减psc */
    volatile bool RatioInc;        /* 递增psc */

    volatile uint8_t FirstCapt;                /* 第一次采样 */
    volatile uint8_t BufferFilled;             /* 测速数据数量,平滑/瞬时速度 */
    volatile uint8_t OVFCounter;               /* 溢出计数 */
    int16_t SensorSpeed[HALL_SPEED_FIFO_SIZE]; /* 速度缓冲区 */
    uint8_t SpeedFIFOIdx;                      /* 滑动index */
    int16_t CurrentSpeed;                      /* 最新速度 */
    int16_t CurrentSpeed_Last;                 /* 上一次速度 */
    int32_t ElSpeedSum;                        /* 速度累加器用于加快平均速度的计算过程 */
    int16_t PrevRotorFreq;                     /* 前一个转速r/min */
    int8_t Direction;                          /* 方向 */
    int8_t NewSpeedAcquisition;                /* 新速度采集标志 */
    int16_t AvrElSpeedDpp;                     /* 平均电转速 */

    uint8_t HallState;            /* Hall sensor state */
    int16_t DeltaAngle;           /* 电角度变化s16degrees.*/
    int16_t MeasuredElAngle;      /* 电角度测量值 */
    int16_t TargetElAngle;        /* 目标电角度 */
    int16_t CompSpeed;            /* 插补法dpp2=f_foc/f_speed */
    uint16_t HALLMaxRatio;        /* 最大psc对应最小速度 */
    uint16_t SatSpeed;            /* 限幅速度 */
    uint32_t PseudoFreqConv;      /* dpp计算的转换系数,dpp = wPseudoFreqConv / Delta */
    uint32_t MaxPeriod;           /* 最低运行频率 wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz) */
    uint32_t MinPeriod;           /* 最快频率 wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/
    uint16_t HallTimeout;         /* 2个HALL信号超时时间 */
    uint16_t OvfFreq;             /* 溢出频率hOvfFreq = CKTIM /65536.*/
    uint16_t PWMNbrPSamplingFreq; /* PWM脉冲数/采样频率 dpp2*/
} HALL_Handle_t;

/* 观测器状态变量 */
typedef struct
{
    Volt_Components Valfa_beta;
    Curr_Components Ialfa_beta;
    uint16_t Vbus;
} Observer_Inputs_t;

/* STO回调对象 */
struct STO_Handle {
    SpeednPosFdbk_Handle_t *_Super;
    STO_ForceConvergency1_Cb_t pFctForceConvergency1;
    STO_ForceConvergency2_Cb_t pFctForceConvergency2;
    STO_OtfResetPLL_Cb_t pFctStoOtfResetPLL;
    STO_SpeedReliabilityCheck_Cb_t pFctSTO_SpeedReliabilityCheck;
};

/* STO+PLL对象(静止坐标系-中高速) */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* 位置传感器成员基类 */

    int16_t hC1;     /* Rs/(Ls*Ts) */
    int16_t hC2;     /* I_est的增益->K1*Z */
    int16_t hC3;     /* Rs_bemf/(Ls*Ts) */
    int16_t hC4;     /* E_est的增益->K2*Z */
    int16_t hC5;     /* Rs_vs/(Ls*Ts) */
    int16_t hC6;     /* C6=F2*F3/C6_COMP_CONST2,与2*pi*f0信号的带宽和相角裕度有关 */
    int16_t hF1;     /* 标幺Q14->I */
    int16_t hF2;     /* 标幺Q11->E */
    int16_t hF3;     /* f0信号带宽的标幺化 */
    uint16_t F1LOG;  /* F1分频 */
    uint16_t F2LOG;  /* F2分频 */
    uint16_t F3POW2; /* F3分频 */

    PID_Handle_t PIRegulator;   /* PI */
    int32_t Ialfa_est;          /* 估计Ialfa */
    int32_t Ibeta_est;          /* 估计Ibeta */
    int32_t wBemf_alfa_est;     /* 估计B-emf alfa int32_t */
    int32_t wBemf_beta_est;     /* 估计B-emf beta int32_t */
    int16_t hBemf_alfa_est;     /* 估计B-emf alfa int16_t */
    int16_t hBemf_beta_est;     /* 估计B-emf beta int16_t */
    int16_t Speed_Buffer[64];   /* 估计速度缓冲区 */
    uint8_t Speed_Buffer_Index; /* 速度index */
    bool IsSpeedReliable;       /* 速度是否可靠 */
    uint8_t ConsistencyCounter; /* 收敛次数 */
    uint8_t ReliabilityCounter; /* 收敛计数 */
    bool IsAlgorithmConverged;  /* 观测器是否收敛:估计速度在当前速度收敛范围内->算法收敛 */
    bool IsBemfConsistent;      /* 估计Bemf和预期是否一致 */

    int32_t Obs_Bemf_Level;              /* 观测Bemf数值 */
    int32_t Est_Bemf_Level;              /* 估计Bemf数值 */
    bool EnableDualCheck;                /* 是否启用机械速度,观测与估计Es,判断算法收敛性*/
    int32_t DppBufferSum;                /* 速度缓存总和[dpp] */
    int16_t SpeedBufferOldestEl;         /* 速度缓存器的上一元素 */
    uint8_t SpeedBufferSize01Hz;         /* SPD_GetAvrgMecSpeed01Hz 函数所导出的估计速度0.1Hz */
    uint8_t SpeedBufferSizedpp;          /* SPD_GetElSpeedDpp 和状态观测器方程导出的估计速度dpp */
    uint16_t VariancePercentage;         /* 速度估计最大允许方差的参数 */
    uint8_t SpeedValidationBand_H;       /* 估计速度>强制定子电频率 程度 */
    uint8_t SpeedValidationBand_L;       /* 估计速度<定子电频率 程度 */
    uint16_t MinStartUpValidSpeed;       /* 启动时最小机械转速的绝对值0.1Hz */
    uint8_t StartUpConsistThreshold;     /* 持续收敛的次数 */
    uint8_t Reliability_hysteresys;      /* 测试中出现可靠性故障的次数 */
    uint8_t BemfConsistencyCheck;        /* 观测器的BEMF的置信度[1,64] */
    uint8_t BemfConsistencyGain;         /* 观测器BEMF增益的准确度[1,64,105] */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* 最大正向机械速度0.1Hz */

    uint16_t SpeedBufferSizedppLOG; /* bSpeedBufferSizedpp 的值表示为2的幂次方 */
    bool ForceConvergency;          /* 强制观测器收敛1 */
    bool ForceConvergency2;         /* 强制观测器收敛2 */
} STO_PLL_Handle_t;

/* STO+PLL对象(旋转坐标系下-低速及启动) */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* 位置传感器成员基类 */

    int16_t hC1;     /* 状态观测器常量C1 */
    int16_t hC2;     /* F1*K1/状态观测器执行频率[Hz]，其中K1是两个观测器增益之一 */
    int16_t hC3;     /* 状态观测器常量C3 */
    int16_t hC4;     /* K2*乘以最大可测量电流（安培）/（最大应用速度[转/分钟]*电机反电动势常数[Vllrms/krpm]*sqrt(2)*F2*状态观测器执行率[Hz]） */
    int16_t hC5;     /* 状态观测器常量C5 */
    int16_t hC6;     /* 状态观测器常量C6 */
    int16_t hF1;     /* 状态观测器缩放因子F1 */
    int16_t hF2;     /* 状态观测器缩放因子F2 */
    int16_t hF3;     /* 状态观测器缩放因子F3 */
    uint16_t F3POW2; /* F3的2的幂 */

    int32_t Ialfa_est;             /* 估计Ialfa */
    int32_t Ibeta_est;             /* 估计Ibeta */
    int32_t wBemf_alfa_est;        /* 估计B-emf alfa int32_t */
    int32_t wBemf_beta_est;        /* 估计B-emf beta int32_t */
    int16_t hBemf_alfa_est;        /* 估计B-emf alfa int16_t */
    int16_t hBemf_beta_est;        /* 估计B-emf beta int16_t */
    int16_t Speed_Buffer[64];      /* 估计速度缓冲区 */
    uint8_t Speed_Buffer_Index;    /* 速度index */
    bool IsSpeedReliable;          /* 速度是否可靠 */
    uint8_t ConsistencyCounter;    /* start-up启动是否通过计数 */
    uint8_t ReliabilityCounter;    /* 收敛计数 */
    bool IsAlgorithmConverged;     /* 观测器是否收敛 */
    int16_t Orig_Speed_Buffer[64]; /* 原始速度缓冲区 */
    int16_t Orig_ElSpeedDpp;       /* 原始速度dpp */
    bool IsBemfConsistent;         /* 估计Bemf和预期是否一致 */

    int32_t Obs_Bemf_Level;          /* 观测Bemf数值 */
    int32_t Est_Bemf_Level;          /* 估计Bemf数值 */
    bool EnableDualCheck;            /* 是否启用双检查 */
    int32_t DppBufferSum;            /* 速度缓存总和[dpp] */
    int32_t DppOrigBufferSum;        /* 原始速度缓存总和[dpp] */
    int16_t SpeedBufferOldestEl;     /* 速度缓存器的上一元素 */
    int16_t OrigSpeedBufferOldestEl; /* 原始速度缓存器的上一元素 */

    uint8_t SpeedBufferSize01Hz;         /* SPD_GetAvrgMecSpeed01Hz 函数所导出的估计速度0.1Hz */
    uint8_t SpeedBufferSizedpp;          /* SPD_GetElSpeedDpp 和状态观测器方程导出的估计速度dpp */
    uint16_t VariancePercentage;         /* 速度估计最大允许方差的参数 */
    uint8_t SpeedValidationBand_H;       /* 估计速度>强制定子电频率 程度 */
    uint8_t SpeedValidationBand_L;       /* 估计速度<定子电频率 程度 */
    uint16_t MinStartUpValidSpeed;       /* 启动时最小机械转速的绝对值0.1Hz */
    uint8_t StartUpConsistThreshold;     /* 测试次数->启动成功阈值 */
    uint8_t Reliability_hysteresys;      /* 测试中出现可靠性故障的次数 */
    int16_t MaxInstantElAcceleration;    /* 最大瞬时电加速度 */
    uint8_t BemfConsistencyCheck;        /* 观测器的BEMF的置信度[1,64] */
    uint8_t BemfConsistencyGain;         /* 观测器BEMF增益的准确度[1,64,105] */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* 最大正向机械速度0.1Hz */

    uint16_t F1LOG;                 /* F1增益分频器以2的幂次 */
    uint16_t F2LOG;                 /* F2增益分频器以2的幂次 */
    uint16_t SpeedBufferSizedppLOG; /* bSpeedBufferSizedpp 的值表示为2的幂次方 */
    bool ForceConvergency;          /* 观测器收敛1 */
    bool ForceConvergency2;         /* 观测器收敛2 */
} STO_CR_Handle_t;

/* 斜坡处理结构体 */
typedef struct
{
    uint16_t FrequencyHz;       /* 频率 */
    int16_t TargetFinal;        /* 目标值 */
    int32_t Ext;                /* 当前信号的放大值 */
    uint32_t RampRemainingStep; /* 剩余步数 */
    int32_t IncDecAmount;       /* 增量或减量 */
    uint32_t ScalingFactor;     /* 缩放因子 */
} RampExtMngr_Handle_t;

typedef enum {
    MOTOR_IDLE = 0, /*空闲状态*/
    MOTOR_IDLE_START,
    MOTOR_BOOT_CAP,     /* 电容充电 */
    MOTOR_OFFSET_CALIB, /* 校准电流 */
    MOTOR_IDENTIFY,     /*参数识别*/
    MOTOR_CLEAR,        /*清除参数*/
    MOTOR_START,        /*开始运行*/
    MOTOR_OPEN_RUN,     /*开环运行*/
    MOTOR_HALL_RUN,     /*霍尔运行*/
    MOTOR_ENC_RUN,      /*编码器运行*/
    MOTOR_SL_RUN,       /*无感运行*/
    MOTOR_ZERO_ALIGN,   /*零点对齐*/
} MOTOR_State_t;

/*转动方向*/
typedef enum {
    REVERSAL = -1,  /*反转*/
    COROTATION = 1, /*正转*/
} ROTATION_Dir_t;

/* 自然坐标系 */
typedef struct {
    float a;
    float b;
    float c;
} abc_t;

/* alphabeta 坐标系 */
typedef struct {
    float alpha;
    float beta;
} alphabeta_t;

/* qd坐标系 */
typedef struct {
    float q;
    float d;
} qd_t;

/* 速度结构体 */
typedef struct {
    float ElAngle;       /*当前电角度*/
    float PhaseShift;    /*相位的偏移:LFP的存在,需要进行补偿*/
    float MecAngle;      /*当前机械角度*/
    float AvrMecSpeed;   /*平均机械速度 r/min */
    float Max_MecSpeed;  /*最大机械速度限制*/
    uint32_t pole_pairs; /*极对数*/
    ROTATION_Dir_t dir;  /*速度方向*/
} SPEED_t;

/* FOC计算参数 */
typedef struct {
    /*采集电流*/
    abc_t I_abc;
    abc_t Eu; /*三相反电动势*/
    alphabeta_t I_alphabeta;
    qd_t I_qd;
    qd_t I_qd_ref; /*参考目标Iqd*/
    /*输出电压*/
    qd_t U_qd;
    alphabeta_t U_alphabeta;
    /*速度参数*/
    SPEED_t speed;
} FOC_VAR_t;

/* 滑动滤波器 */
typedef struct {
    int32_t buf[MOVE_FILTER_BUF_MAXSIZE];
    int32_t buf_sum;
    uint32_t buf_size;
    uint32_t buf_Index;
    uint32_t buf_fill;
    float val; /*计算的平均值*/
} MOVE_FILTER_t;

/* 锁相环 */
typedef struct {
    float Kp; /*锁相环PI控制器Kp*/
    float Ki; /*锁相环PI控制器Ki*/

    float P;
    float I;

    float Angle_Err;  /*角度误差*/
    float Omega;      /*电角速度*/
    float Omega_F;    /*滤波后的电角速度*/
    float Angle;      /*锁相环计算得到的电角度*/
    float Angle_last; /*上一次的电角度*/
    float Ts;         /*执行周期*/
    MOVE_FILTER_t move_filter;
} PLL_t;

#endif /* BSP_GLOBAL_H */
