#ifndef BSP_GLOBAL_H
#define BSP_GLOBAL_H

#include "main.h"
#include "mc_math.h"
#include "mc_type.h"
#include <arm_math.h>

#define OPEN_DEBUG_MODE   0 /* �Ƿ�ʹ��FOC��������ģʽ(����ѡ����һ�ִ�����) */
#define DAC_TEST_ENABLE   1 /* �Ƿ�ʹ��DACģʽ */
#define DAC_TEST          0 /* �Ƿ�ʹ��DAC����ģʽ */
#define PRINT_TEST        0 /* �Ƿ�ʹ�ܴ�ӡ���� */
#define ENCODER_ENABLE    0 /* �Ƿ�ʹ�ܱ����� */
#define HANDLE_BREAK      0 /* �Ƿ�ʹ��Uq��ת�ѵı��ۻ� */
#define HALL_ENABLE       1 /* �Ƿ�ʹ��HALL */
#define SENSORLESS_ENABLE 0 /* �Ƿ�ʹ���޸�IF+SMO+PLL */

#define HFI_SMO_ENABLE       0 /* �Ƿ�ʹ��HFI */
#define OPEN_SENSORLESS_MODE 0 /* �Ƿ�ʹ���޸п�������ģʽ */
#define FLUX_SENSOR_ENABLE   0 /* �Ƿ�ʹ�ܴ����۲��� */

/* ѡ������������ */
#define MAX_MODULATION_99_PER_CENT 1
#define MAX_MODULATION_96_PER_CENT 0
#define MAX_MODULATION_95_PER_CENT 0
#define MAX_MODULATION_94_PER_CENT 0

/* ���� */
#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u
#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* ����ͨ�� */
#define NB_CONVERSIONS 16u /* 16�β��� */
#define PHASE_A_MSK    (uint32_t)((uint32_t)(6) << 15)
#define PHASE_B_MSK    (uint32_t)((uint32_t)(8) << 15)
#define PHASE_C_MSK    (uint32_t)((uint32_t)(9) << 15)
#define ADC_SR_MASK    ((uint32_t)(0xC))

#define CONV_STARTED  ((uint32_t)(0x8))
#define CONV_FINISHED ((uint32_t)(0xC))
#define FLAGS_CLEARED ((uint32_t)(0x0))
#define MAX_TWAIT     0 /* ע��ADCת����־�����,�ȴ�Tw������ٴμ�� */

/* ������ */
// #define DEADTIME_NS 200
#define ADC_CLK_MHz          21
#define SAMPLING_TIME_NS     (3 * 1000uL / ADC_CLK_MHz)
#define TRIG_CONV_LATENCY_NS 150
#define MAX_TNTR_NS          150 /* rise time */
#define TW_AFTER             ((uint16_t)(((DEADTIME_NS + MAX_TNTR_NS) * ADV_TIM_CLK_MHz) / 1000ul))
#define TW_BEFORE            (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS + TRIG_CONV_LATENCY_NS))) * ADV_TIM_CLK_MHz) / 1000ul)) + 1u)

/* ʱ��δ� */
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

/* ��Դ */
#define MCU_SUPPLY_VOLTAGE 3.30

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE (uint16_t)((uint16_t)(PWM_FREQUENCY) / 1)
#define REP_COUNTER        (uint16_t)((1 * 2u) - 1u)

#define SPEED_LOOP_FREQUENCY_HZ    500 /* �ٶȻ�Ƶ��500hz */
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

/************************* ״̬�۲���+PLL���� **************************/
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM  580 /* Minimum speed for observer */
#define NB_CONSECUTIVE_TESTS   2
#define SPEED_BAND_UPPER_LIMIT 17
#define SPEED_BAND_LOWER_LIMIT 15
#define TRANSITION_DURATION    500 /* Switch over duration, ms */

#define RSHUNT             0.020 /* ����0.020 */
#define AMPLIFICATION_GAIN 4.02  /*  ISC���� */

#define MAX_BEMF_VOLTAGE (uint16_t)((MAX_APPLICATION_SPEED * 1.2 *      \
                                     MOTOR_VOLTAGE_CONSTANT * SQRT_2) / \
                                    (1000u * SQRT_3))

#define VBUS_PARTITIONING_FACTOR 0.0463 /* VBUS˥���̶� */
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR
#define MAX_VOLTAGE              (int16_t)((MCU_SUPPLY_VOLTAGE / 2) / BUS_ADC_CONV_RATIO) /* ������ѹ */
#define MAX_CURRENT              (MCU_SUPPLY_VOLTAGE / (2 * RSHUNT * AMPLIFICATION_GAIN)) /* ������ */

/******************** ��ֹ����ϵ ********************/
#define F1    16384 /* Q14 */
#define F2    2048  /* Q12 */
#define GAIN1 -23932
#define GAIN2 20828

#define VARIANCE_THRESHOLD    0.4 /* �ٶȹ���ֵ�����ɽ��ܷ���(�ٷֱ�) */
#define BEMF_CONSISTENCY_TOL  64  /* BEMF��׼ȷ�� */
#define BEMF_CONSISTENCY_GAIN 64  /* BEMF������׼ȷ�� */

/******************** ��ת����ϵ ********************/
#define C6_COMP_CONST1 (int32_t)1043038 /* 2*pi*f0�źŵĴ��� */
#define C6_COMP_CONST2 (int32_t)10430   /* 2*pi*f0�źŵ����ԣ�� */
#define CORD_F1        16384            /* ϵ��F1 */
#define CORD_F2        2048             /* ϵ��F2 */
#define CORD_GAIN1     -20932           /* ����K1 */
#define CORD_GAIN2     20828            /* ����K2 */

#define CORD_VARIANCE_THRESHOLD 4 /* �����ٶ����ɽ��ܵ�ƫ�� */

#define CORD_MEAS_ERRORS_BEFORE_FAULTS 16 /* ��ת����ϵ,�����ٶȷ�������֮ǰ���з������ʱ���ֵ������������ */
#define CORD_FIFO_DEPTH_DPP            64 /* �ٶ�ƽ�������С */
#define CORD_FIFO_DEPTH_01HZ           64 /* �ٶ�ƽ�������С */
#define CORD_MAX_ACCEL_DPPP            39 /* ���˲ʱ���ٶ� ��λ��dpp*/

#define CORD_BEMF_CONSISTENCY_TOL  64 /* BEMF��׼ȷ�� */
#define CORD_BEMF_CONSISTENCY_GAIN 64 /* BEMF������׼ȷ��  */

/* Default settings */
#define DEFAULT_CONTROL_MODE STC_SPEED_MODE /*!< STC_TORQUE_MODE or \
                                              STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM 0
#define DEFAULT_TORQUE_COMPONENT 0
#define DEFAULT_FLUX_COMPONENT   0

/* SensorLess parameters */
#define MOVE_FILTER_BUF_MAXSIZE 500 /*�����˲��������*/

/* FOC��صĻص������Ͳ��� */
typedef struct PWMC_Handle PWMC_Handle_t;

/* ��ַָ�뺯���������ͷ���ֵ */
typedef void (*PWMC_GetPhaseCurr_Cb_t)(PWMC_Handle_t *pHandle, Curr_Components *pStator_Currents);
typedef void (*PWMC_Generic_Cb_t)(PWMC_Handle_t *pHandle);
typedef uint16_t (*PWMC_OverCurr_Cb_t)(PWMC_Handle_t *pHandle);
typedef uint16_t (*PWMC_SetSampPointSectX_Cb_t)(PWMC_Handle_t *pHandle);
typedef void (*PWMC_SetOcpRefVolt_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDACVref);

/* STOָ�뺯�� */
typedef struct STO_Handle STO_Handle_t;
typedef void (*STO_ForceConvergency1_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_ForceConvergency2_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_OtfResetPLL_Cb_t)(STO_Handle_t *pHandle);
typedef bool (*STO_SpeedReliabilityCheck_Cb_t)(STO_Handle_t *pHandle);

/* FOC��صĻص������Ͳ��� */
struct PWMC_Handle {
    PWMC_GetPhaseCurr_Cb_t pFctGetPhaseCurrents; /* ��ȡ����� */
    PWMC_Generic_Cb_t pFctSwitchOffPwm;          /* �ر�PWM��� */
    PWMC_Generic_Cb_t pFctSwitchOnPwm;           /* ����PWM��� */
    PWMC_Generic_Cb_t pFctCurrReadingCalib;      /* ��������У׼ */
    PWMC_Generic_Cb_t pFctTurnOnLowSides;        /* �����¹� */

    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect1; /* ��������1��ADC������ */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect2; /* ��������2��ADC������ */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect3; /* ��������3��ADC������ */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect4; /* ��������4��ADC������ */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect5; /* ��������5��ADC������ */
    PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect6; /* ��������6��ADC������ */

    uint16_t hT_Sqrt3; /* sqrt(3)*T */
    uint16_t hSector;  /* ���� */
    uint16_t hCntPhA;  /* CCR1 */
    uint16_t hCntPhB;  /* CCR2 */
    uint16_t hCntPhC;  /* CCR3 */
    uint16_t SWerror;  /* ���ش��� */

    int16_t hIa; /* ����Ia */
    int16_t hIb; /* ����Ib */
    int16_t hIc; /* ����Ic */

    uint16_t hDeadTime; /* ���� */
    uint16_t hTafter;   /* �м��� */
    uint16_t hTbefore;  /* �м��ǰ */
    uint16_t Tw;        /* ע��ADCת����־�����,�ȴ�Tw������ٴμ�� */

    uint16_t hPWMperiod;                /* PWM���� */
    uint16_t hOffCalibrWaitTimeCounter; /* У׼�ȴ�ʱ����� */
    uint16_t hOffCalibrWaitTicks;       /* У׼�ȴ�ʱ�� */
    bool bTurnOnLowSidesAction;         /* �¹��Ƿ�ͨ */
};

typedef struct {
    PWMC_Handle_t _Super;   /* ���� */
    uint32_t wPhaseAOffset; /* A��ƫ�� */
    uint32_t wPhaseBOffset; /* B��ƫ�� */
    uint32_t wPhaseCOffset; /* C��ƫ�� */

    uint32_t wADC1Channel;   /* ADC1�ɼ���ͨ�� */
    uint32_t wADC2Channel;   /* ADC2�ɼ���ͨ�� */
    uint16_t Half_PWMPeriod; /* ������� */

    volatile uint8_t bIndex;   /* ����ƽ������ */
    uint32_t wADCTriggerSet;   /* ADC����ʹ�ܺͼ������� */
    uint32_t wADCTriggerUnSet; /* ADC����ʧ�� */
    bool OverCurrentFlag;      /* ������־ */

    volatile uint8_t bSoFOC; /* FOC��TIM8�����ж�Ƶ�ʱ�־ */
} PWMC_R3_F4_Handle_t;

/* ״̬�� */
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

/* ������Ʋ�״̬ */
typedef enum {
    MCI_BUFFER_EMPTY,                  /* �������� */
    MCI_COMMAND_NOT_ALREADY_EXECUTED,  /* ����δִ�� */
    MCI_COMMAND_EXECUTED_SUCCESFULLY,  /* ������ִ�� */
    MCI_COMMAND_EXECUTED_UNSUCCESFULLY /* ����ִ��ʧ�� */
} MCI_CommandState_t;

/* �û����� */
typedef enum {
    MCI_NOCOMMANDSYET,        /* ������ */
    MCI_EXECSPEEDRAMP,        /* ִ���ٶȵ��� */
    MCI_EXECTORQUERAMP,       /* ִ��ת�ص��� */
    MCI_SETCURRENTREFERENCES, /* ���õ����ο�ֵ */
} MCI_UserCommands_t;

/* ����״̬ */
typedef enum CRCAction {
    CRC_START, /* ��ʼ */
    CRC_EXEC   /* ִ�� */
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

/* λ�ô������Ļ��� */
typedef struct
{
    int16_t hElAngle;                   /* ��Ƕ� */
    int16_t hMecAngle;                  /* ��е�Ƕ� */
    int16_t hAvrMecSpeed01Hz;           /* ƽ����е�ٶ�0.1Hz */
    int16_t hElSpeedDpp;                /* ��Ƕ��ٶ� */
    int16_t hMecAccel01HzP;             /* ��е���ٶ�0.1HzP */
    uint8_t bSpeedErrorNumber;          /* �ٶ��쳣���� */
    uint8_t bElToMecRatio;              /* ������ */
    uint16_t hMaxReliableMecSpeed01Hz;  /* ���ɿ���е�ٶ�0.1Hz */
    uint16_t hMinReliableMecSpeed01Hz;  /* ��С�ɿ���е�ٶ�0.1Hz */
    uint8_t bMaximumSpeedErrorsNumber;  /* ����ٶ��쳣���� */
    uint16_t hMaxReliableMecAccel01HzP; /* ���ɿ���е���ٶ�0.1Hz  */
    uint16_t hMeasurementFrequency;     /* ����Ƶ��16kHz */
} SpeednPosFdbk_Handle_t;

/* �ٶ�ת�ؿ��ƶ��� */
typedef struct
{
    STC_Modality_t Mode;                 /* ���ģʽ:ת�ء��ٶ� */
    int16_t TargetFinal;                 /* Ŀ����� */
    int32_t SpeedRef01HzExt;             /* Ŀ���ٶ�(��ǰQ16) */
    int32_t TorqueRef;                   /* Ŀ��ת��(��ǰQ16) */
    uint32_t RampRemainingStep;          /* Rampʣ�ಽ�� */
    PID_Handle_t *PISpeed;               /* �ٶȿ���PID */
    SpeednPosFdbk_Handle_t *SPD;         /* �ٶȼ�λ�÷��� */
    int32_t IncDecAmount;                /* ����/����ֵ */
    uint16_t STCFrequencyHz;             /* �ٶ�ת�ؿ���Ƶ�� */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* ��������е�ٶ�0.1Hz */
    uint16_t MinAppPositiveMecSpeed01Hz; /* ��С�����е�ٶ�0.1Hz */
    int16_t MaxAppNegativeMecSpeed01Hz;  /* ������е�ٶ�0.1Hz */
    int16_t MinAppNegativeMecSpeed01Hz;  /* ��С�����е�ٶ�0.1Hz */
    uint16_t MaxPositiveTorque;          /* �������ת�� */
    int16_t MinNegativeTorque;           /* ��С����ת�� */
    STC_Modality_t ModeDefault;          /* Ĭ��ģʽ */
    int16_t MecSpeedRef01HzDefault;      /* Ĭ��Ŀ���е�ٶ�0.1Hz */
    int16_t TorqueRefDefault;            /* Ĭ��Ŀ��ת�� */
    int16_t IdrefDefault;                /* Ĭ�ϵ���Id */
} SpeednTorqCtrl_Handle_t;

/* ״̬������ */
typedef struct
{
    State_t bState;          /* ״̬ */
    uint16_t hFaultNow;      /* ���� */
    uint16_t hFaultOccurred; /* ���Ϸ��� */
    uint8_t bStop_Motor;     /* 1-���� 0-ֹͣ */
    uint8_t bSenLess_Handle; /* 0-��ת�� 1-ת�� */
} STM_Handle_t;

/* ������Ʋ���� */
typedef struct
{
    STM_Handle_t *pSTM;                   /* ״̬�� */
    SpeednTorqCtrl_Handle_t *pSTC;        /* �ٶȺ�ת�ؿ��� */
    pFOCVars_t pFOCVars;                  /* FOC���� */
    MCI_UserCommands_t lastCommand;       /* ��һ������ */
    int16_t hFinalSpeed;                  /* �����ٶ� */
    int16_t hFinalTorque;                 /* ����ת�� */
    Curr_Components Iqdref;               /* �����ο�ֵ */
    uint16_t hDurationms;                 /* ����ʱ�� */
    MCI_CommandState_t CommandState;      /* ����״̬ */
    STC_Modality_t LastModalitySetByUser; /* ��һ���û����õ�ģʽ */
} MCI_Handle_t;

/* �����ٶȴ��������� */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super;     /* λ�ô�������Ա������ */
    int32_t wElAccDppP32;              /* ����ٶ�dpp 65536 */
    int32_t wElSpeedDpp32;             /* ��ת��dpp 65536 */
    uint16_t hRemainingStep;           /* final speedʣ�ಽ�� */
    int16_t hFinalMecSpeed01Hz;        /* final speed 0.1Hz */
    bool bTransitionStarted;           /* ״̬ת��(б��)��ʼ */
    bool bTransitionEnded;             /* ״̬ת������ */
    int16_t hTransitionRemainingSteps; /* ״̬ת��ʣ�ಽ�� */
    int16_t hElAngleAccu;              /* ��Ƕȼ��ٶ� */
    bool bTransitionLocked;            /* ��ʼת�����ٶ� */
    bool bCopyObserver;
    uint16_t hSpeedSamplingFreqHz; /* �ٶȲ���Ƶ��(Hz) */
    int16_t hTransitionSteps;      /* ״̬ת������ */
} VirtualSpeedSensor_Handle_t;

/* ���������� */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super;     /* λ�ô�������Ա������ */
    uint16_t PulseNumber;              /* ������������(*4) */
    FunctionalState RevertSignal;      /* ENABLE/DISABLE */
    uint16_t SpeedSamplingFreq01Hz;    /* �ٶȲ���Ƶ��(0.1Hz) */
    uint8_t SpeedBufferSize;           /* �ٶȻ�������С */
    volatile uint16_t TimerOverflowNb; /* ���*/
    bool SensorIsReliable;             /* �������Ƿ�ɿ� */
    uint16_t PreviousCapture;          /* ��һ�β����������� */

    int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /* �ٶȼ���Ĳ�������� */
    volatile uint8_t DeltaCapturesIndex;               /* ��������������� */
    uint32_t U32MAXdivPulseNumber;                     /* U32MAX/hPulseNumber */
    uint16_t SpeedSamplingFreqHz;                      /* �ٶȲ���Ƶ��(Hz) */
    bool TimerOverflowError;                           /* ��ʱ��������� */
} ENCODER_Handle_t;

/* ������������� */
typedef struct
{
    SpeednTorqCtrl_Handle_t *pSTC;     /* �ٶȺ�ת�ض��� */
    VirtualSpeedSensor_Handle_t *pVSS; /* �����ٶȴ��������� */
    ENCODER_Handle_t *pENC;            /* ���������� */
    uint16_t hRemainingTicks;          /* ʣ��δ���� */
    bool EncAligned;                   /* �������Ѷ��� */
    bool EncRestart;                   /* �������������� */
    uint16_t hEACFrequencyHz;          /* Ƶ�� */
    int16_t hFinalTorque;              /* ����ת�� */
    int16_t hElAngle;                  /* �����Ƕ� */
    uint16_t hDurationms;              /* ����ʱ��(ms) */
    uint8_t bElToMecRatio;             /* ������ */
} EncAlign_Handle_t;

/* Hall���������� */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* λ�ô�������Ա������ */
    uint8_t SensorPlacement;       /* 120 or 60 degrees */
    int16_t PhaseShift;            /* H1��������A�����BEMF��ƫ�� */
    uint16_t SpeedSamplingFreqHz;  /* ������ת�ٵ�Ƶ��0.1HZ */
    uint8_t SpeedBufferSize;       /*����ƽ���ٶ�*/
    uint32_t TIMClockFreq;         /* ʱ��Ƶ�� */
    bool SensorIsReliable;         /* �������Ƿ�ɿ� */
    volatile bool RatioDec;        /* �ݼ�psc */
    volatile bool RatioInc;        /* ����psc */

    volatile uint8_t FirstCapt;                /* ��һ�β��� */
    volatile uint8_t BufferFilled;             /* ������������,ƽ��/˲ʱ�ٶ� */
    volatile uint8_t OVFCounter;               /* ������� */
    int16_t SensorSpeed[HALL_SPEED_FIFO_SIZE]; /* �ٶȻ����� */
    uint8_t SpeedFIFOIdx;                      /* ����index */
    int16_t CurrentSpeed;                      /* �����ٶ� */
    int16_t CurrentSpeed_Last;                 /* ��һ���ٶ� */
    int32_t ElSpeedSum;                        /* �ٶ��ۼ������ڼӿ�ƽ���ٶȵļ������ */
    int16_t PrevRotorFreq;                     /* ǰһ��ת��r/min */
    int8_t Direction;                          /* ���� */
    int8_t NewSpeedAcquisition;                /* ���ٶȲɼ���־ */
    int16_t AvrElSpeedDpp;                     /* ƽ����ת�� */

    uint8_t HallState;            /* Hall sensor state */
    int16_t DeltaAngle;           /* ��Ƕȱ仯s16degrees.*/
    int16_t MeasuredElAngle;      /* ��ǶȲ���ֵ */
    int16_t TargetElAngle;        /* Ŀ���Ƕ� */
    int16_t CompSpeed;            /* �岹��dpp2=f_foc/f_speed */
    uint16_t HALLMaxRatio;        /* ���psc��Ӧ��С�ٶ� */
    uint16_t SatSpeed;            /* �޷��ٶ� */
    uint32_t PseudoFreqConv;      /* dpp�����ת��ϵ��,dpp = wPseudoFreqConv / Delta */
    uint32_t MaxPeriod;           /* �������Ƶ�� wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz) */
    uint32_t MinPeriod;           /* ���Ƶ�� wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/
    uint16_t HallTimeout;         /* 2��HALL�źų�ʱʱ�� */
    uint16_t OvfFreq;             /* ���Ƶ��hOvfFreq = CKTIM /65536.*/
    uint16_t PWMNbrPSamplingFreq; /* PWM������/����Ƶ�� dpp2*/
} HALL_Handle_t;

/* �۲���״̬���� */
typedef struct
{
    Volt_Components Valfa_beta;
    Curr_Components Ialfa_beta;
    uint16_t Vbus;
} Observer_Inputs_t;

/* STO�ص����� */
struct STO_Handle {
    SpeednPosFdbk_Handle_t *_Super;
    STO_ForceConvergency1_Cb_t pFctForceConvergency1;
    STO_ForceConvergency2_Cb_t pFctForceConvergency2;
    STO_OtfResetPLL_Cb_t pFctStoOtfResetPLL;
    STO_SpeedReliabilityCheck_Cb_t pFctSTO_SpeedReliabilityCheck;
};

/* STO+PLL����(��ֹ����ϵ-�и���) */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* λ�ô�������Ա���� */

    int16_t hC1;     /* Rs/(Ls*Ts) */
    int16_t hC2;     /* I_est������->K1*Z */
    int16_t hC3;     /* Rs_bemf/(Ls*Ts) */
    int16_t hC4;     /* E_est������->K2*Z */
    int16_t hC5;     /* Rs_vs/(Ls*Ts) */
    int16_t hC6;     /* C6=F2*F3/C6_COMP_CONST2,��2*pi*f0�źŵĴ�������ԣ���й� */
    int16_t hF1;     /* ����Q14->I */
    int16_t hF2;     /* ����Q11->E */
    int16_t hF3;     /* f0�źŴ���ı��ۻ� */
    uint16_t F1LOG;  /* F1��Ƶ */
    uint16_t F2LOG;  /* F2��Ƶ */
    uint16_t F3POW2; /* F3��Ƶ */

    PID_Handle_t PIRegulator;   /* PI */
    int32_t Ialfa_est;          /* ����Ialfa */
    int32_t Ibeta_est;          /* ����Ibeta */
    int32_t wBemf_alfa_est;     /* ����B-emf alfa int32_t */
    int32_t wBemf_beta_est;     /* ����B-emf beta int32_t */
    int16_t hBemf_alfa_est;     /* ����B-emf alfa int16_t */
    int16_t hBemf_beta_est;     /* ����B-emf beta int16_t */
    int16_t Speed_Buffer[64];   /* �����ٶȻ����� */
    uint8_t Speed_Buffer_Index; /* �ٶ�index */
    bool IsSpeedReliable;       /* �ٶ��Ƿ�ɿ� */
    uint8_t ConsistencyCounter; /* �������� */
    uint8_t ReliabilityCounter; /* �������� */
    bool IsAlgorithmConverged;  /* �۲����Ƿ�����:�����ٶ��ڵ�ǰ�ٶ�������Χ��->�㷨���� */
    bool IsBemfConsistent;      /* ����Bemf��Ԥ���Ƿ�һ�� */

    int32_t Obs_Bemf_Level;              /* �۲�Bemf��ֵ */
    int32_t Est_Bemf_Level;              /* ����Bemf��ֵ */
    bool EnableDualCheck;                /* �Ƿ����û�е�ٶ�,�۲������Es,�ж��㷨������*/
    int32_t DppBufferSum;                /* �ٶȻ����ܺ�[dpp] */
    int16_t SpeedBufferOldestEl;         /* �ٶȻ���������һԪ�� */
    uint8_t SpeedBufferSize01Hz;         /* SPD_GetAvrgMecSpeed01Hz �����������Ĺ����ٶ�0.1Hz */
    uint8_t SpeedBufferSizedpp;          /* SPD_GetElSpeedDpp ��״̬�۲������̵����Ĺ����ٶ�dpp */
    uint16_t VariancePercentage;         /* �ٶȹ������������Ĳ��� */
    uint8_t SpeedValidationBand_H;       /* �����ٶ�>ǿ�ƶ��ӵ�Ƶ�� �̶� */
    uint8_t SpeedValidationBand_L;       /* �����ٶ�<���ӵ�Ƶ�� �̶� */
    uint16_t MinStartUpValidSpeed;       /* ����ʱ��С��еת�ٵľ���ֵ0.1Hz */
    uint8_t StartUpConsistThreshold;     /* ���������Ĵ��� */
    uint8_t Reliability_hysteresys;      /* �����г��ֿɿ��Թ��ϵĴ��� */
    uint8_t BemfConsistencyCheck;        /* �۲�����BEMF�����Ŷ�[1,64] */
    uint8_t BemfConsistencyGain;         /* �۲���BEMF�����׼ȷ��[1,64,105] */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* ��������е�ٶ�0.1Hz */

    uint16_t SpeedBufferSizedppLOG; /* bSpeedBufferSizedpp ��ֵ��ʾΪ2���ݴη� */
    bool ForceConvergency;          /* ǿ�ƹ۲�������1 */
    bool ForceConvergency2;         /* ǿ�ƹ۲�������2 */
} STO_PLL_Handle_t;

/* STO+PLL����(��ת����ϵ��-���ټ�����) */
typedef struct
{
    SpeednPosFdbk_Handle_t _Super; /* λ�ô�������Ա���� */

    int16_t hC1;     /* ״̬�۲�������C1 */
    int16_t hC2;     /* F1*K1/״̬�۲���ִ��Ƶ��[Hz]������K1�������۲�������֮һ */
    int16_t hC3;     /* ״̬�۲�������C3 */
    int16_t hC4;     /* K2*�������ɲ������������ࣩ/�����Ӧ���ٶ�[ת/����]*������綯�Ƴ���[Vllrms/krpm]*sqrt(2)*F2*״̬�۲���ִ����[Hz]�� */
    int16_t hC5;     /* ״̬�۲�������C5 */
    int16_t hC6;     /* ״̬�۲�������C6 */
    int16_t hF1;     /* ״̬�۲�����������F1 */
    int16_t hF2;     /* ״̬�۲�����������F2 */
    int16_t hF3;     /* ״̬�۲�����������F3 */
    uint16_t F3POW2; /* F3��2���� */

    int32_t Ialfa_est;             /* ����Ialfa */
    int32_t Ibeta_est;             /* ����Ibeta */
    int32_t wBemf_alfa_est;        /* ����B-emf alfa int32_t */
    int32_t wBemf_beta_est;        /* ����B-emf beta int32_t */
    int16_t hBemf_alfa_est;        /* ����B-emf alfa int16_t */
    int16_t hBemf_beta_est;        /* ����B-emf beta int16_t */
    int16_t Speed_Buffer[64];      /* �����ٶȻ����� */
    uint8_t Speed_Buffer_Index;    /* �ٶ�index */
    bool IsSpeedReliable;          /* �ٶ��Ƿ�ɿ� */
    uint8_t ConsistencyCounter;    /* start-up�����Ƿ�ͨ������ */
    uint8_t ReliabilityCounter;    /* �������� */
    bool IsAlgorithmConverged;     /* �۲����Ƿ����� */
    int16_t Orig_Speed_Buffer[64]; /* ԭʼ�ٶȻ����� */
    int16_t Orig_ElSpeedDpp;       /* ԭʼ�ٶ�dpp */
    bool IsBemfConsistent;         /* ����Bemf��Ԥ���Ƿ�һ�� */

    int32_t Obs_Bemf_Level;          /* �۲�Bemf��ֵ */
    int32_t Est_Bemf_Level;          /* ����Bemf��ֵ */
    bool EnableDualCheck;            /* �Ƿ�����˫��� */
    int32_t DppBufferSum;            /* �ٶȻ����ܺ�[dpp] */
    int32_t DppOrigBufferSum;        /* ԭʼ�ٶȻ����ܺ�[dpp] */
    int16_t SpeedBufferOldestEl;     /* �ٶȻ���������һԪ�� */
    int16_t OrigSpeedBufferOldestEl; /* ԭʼ�ٶȻ���������һԪ�� */

    uint8_t SpeedBufferSize01Hz;         /* SPD_GetAvrgMecSpeed01Hz �����������Ĺ����ٶ�0.1Hz */
    uint8_t SpeedBufferSizedpp;          /* SPD_GetElSpeedDpp ��״̬�۲������̵����Ĺ����ٶ�dpp */
    uint16_t VariancePercentage;         /* �ٶȹ������������Ĳ��� */
    uint8_t SpeedValidationBand_H;       /* �����ٶ�>ǿ�ƶ��ӵ�Ƶ�� �̶� */
    uint8_t SpeedValidationBand_L;       /* �����ٶ�<���ӵ�Ƶ�� �̶� */
    uint16_t MinStartUpValidSpeed;       /* ����ʱ��С��еת�ٵľ���ֵ0.1Hz */
    uint8_t StartUpConsistThreshold;     /* ���Դ���->�����ɹ���ֵ */
    uint8_t Reliability_hysteresys;      /* �����г��ֿɿ��Թ��ϵĴ��� */
    int16_t MaxInstantElAcceleration;    /* ���˲ʱ����ٶ� */
    uint8_t BemfConsistencyCheck;        /* �۲�����BEMF�����Ŷ�[1,64] */
    uint8_t BemfConsistencyGain;         /* �۲���BEMF�����׼ȷ��[1,64,105] */
    uint16_t MaxAppPositiveMecSpeed01Hz; /* ��������е�ٶ�0.1Hz */

    uint16_t F1LOG;                 /* F1�����Ƶ����2���ݴ� */
    uint16_t F2LOG;                 /* F2�����Ƶ����2���ݴ� */
    uint16_t SpeedBufferSizedppLOG; /* bSpeedBufferSizedpp ��ֵ��ʾΪ2���ݴη� */
    bool ForceConvergency;          /* �۲�������1 */
    bool ForceConvergency2;         /* �۲�������2 */
} STO_CR_Handle_t;

/* б�´���ṹ�� */
typedef struct
{
    uint16_t FrequencyHz;       /* Ƶ�� */
    int16_t TargetFinal;        /* Ŀ��ֵ */
    int32_t Ext;                /* ��ǰ�źŵķŴ�ֵ */
    uint32_t RampRemainingStep; /* ʣ�ಽ�� */
    int32_t IncDecAmount;       /* ��������� */
    uint32_t ScalingFactor;     /* �������� */
} RampExtMngr_Handle_t;

typedef enum {
    MOTOR_IDLE = 0, /*����״̬*/
    MOTOR_IDLE_START,
    MOTOR_BOOT_CAP,     /* ���ݳ�� */
    MOTOR_OFFSET_CALIB, /* У׼���� */
    MOTOR_IDENTIFY,     /*����ʶ��*/
    MOTOR_CLEAR,        /*�������*/
    MOTOR_START,        /*��ʼ����*/
    MOTOR_OPEN_RUN,     /*��������*/
    MOTOR_HALL_RUN,     /*��������*/
    MOTOR_ENC_RUN,      /*����������*/
    MOTOR_SL_RUN,       /*�޸�����*/
    MOTOR_ZERO_ALIGN,   /*������*/
} MOTOR_State_t;

/*ת������*/
typedef enum {
    REVERSAL = -1,  /*��ת*/
    COROTATION = 1, /*��ת*/
} ROTATION_Dir_t;

/* ��Ȼ����ϵ */
typedef struct {
    float a;
    float b;
    float c;
} abc_t;

/* alphabeta ����ϵ */
typedef struct {
    float alpha;
    float beta;
} alphabeta_t;

/* qd����ϵ */
typedef struct {
    float q;
    float d;
} qd_t;

/* �ٶȽṹ�� */
typedef struct {
    float ElAngle;       /*��ǰ��Ƕ�*/
    float PhaseShift;    /*��λ��ƫ��:LFP�Ĵ���,��Ҫ���в���*/
    float MecAngle;      /*��ǰ��е�Ƕ�*/
    float AvrMecSpeed;   /*ƽ����е�ٶ� r/min */
    float Max_MecSpeed;  /*����е�ٶ�����*/
    uint32_t pole_pairs; /*������*/
    ROTATION_Dir_t dir;  /*�ٶȷ���*/
} SPEED_t;

/* FOC������� */
typedef struct {
    /*�ɼ�����*/
    abc_t I_abc;
    abc_t Eu; /*���෴�綯��*/
    alphabeta_t I_alphabeta;
    qd_t I_qd;
    qd_t I_qd_ref; /*�ο�Ŀ��Iqd*/
    /*�����ѹ*/
    qd_t U_qd;
    alphabeta_t U_alphabeta;
    /*�ٶȲ���*/
    SPEED_t speed;
} FOC_VAR_t;

/* �����˲��� */
typedef struct {
    int32_t buf[MOVE_FILTER_BUF_MAXSIZE];
    int32_t buf_sum;
    uint32_t buf_size;
    uint32_t buf_Index;
    uint32_t buf_fill;
    float val; /*�����ƽ��ֵ*/
} MOVE_FILTER_t;

/* ���໷ */
typedef struct {
    float Kp; /*���໷PI������Kp*/
    float Ki; /*���໷PI������Ki*/

    float P;
    float I;

    float Angle_Err;  /*�Ƕ����*/
    float Omega;      /*����ٶ�*/
    float Omega_F;    /*�˲���ĵ���ٶ�*/
    float Angle;      /*���໷����õ��ĵ�Ƕ�*/
    float Angle_last; /*��һ�εĵ�Ƕ�*/
    float Ts;         /*ִ������*/
    MOVE_FILTER_t move_filter;
} PLL_t;

#endif /* BSP_GLOBAL_H */
