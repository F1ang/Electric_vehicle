/******************************************************************************
 *@brief  hall sensor motor control
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef MC_HALL_H
#define MC_HALL_H

#include "global_variable.h"

#define S16_360_PHASE_SHIFT (u16)(65535)
#define S16_330_PHASE_SHIFT (u16)(60075)
#define S16_318_PHASE_SHIFT (u16)(58000)
#define S16_315_PHASE_SHIFT (u16)(57344)
#define S16_300_PHASE_SHIFT (u16)(54613)
#define S16_270_PHASE_SHIFT (u16)(49152)
#define S16_240_PHASE_SHIFT (u16)(43691)
#define S16_210_PHASE_SHIFT (u16)(38220)
#define S16_180_PHASE_SHIFT (u16)(32768)
#define S16_150_PHASE_SHIFT (u16)(27300)
#define S16_120_PHASE_SHIFT (u16)(21845)
#define S16_90_PHASE_SHIFT  (u16)(16384)
#define S16_75_PHASE_SHIFT  (u16)(13653)
#define S16_70_PHASE_SHIFT  (u16)(12743)
#define S16_63_PHASE_SHIFT  (u16)(11468)
#define S16_60_PHASE_SHIFT  (u16)(10923)
#define S16_45_PHASE_SHIFT  (u16)(8192)
#define S16_30_PHASE_SHIFT  (u16)(5461)
#define S16_20_PHASE_SHIFT  (u16)(3641)
#define S16_15_PHASE_SHIFT  (u16)(2731)
#define S16_10_PHASE_SHIFT  (u16)(1820)
#define S16_5_PHASE_SHIFT   (u16)(910)
#define S16_1_PHASE_SHIFT   (u16)(182) /* 1度角归一化处理 1/360*65536, 角度0~360度变化， 对应归一化数据0~65535 */

#define REDUCE_TOQUE_PUSE  0x01 /* 电机速度较低时，正向降低换相引起的转矩波动标志 */
#define REDUCE_TOQUE_MINUS 0x02 /* 电机速度较低时，反向降低换相引起的转矩波动标志 */

/* ****************************************** HALL  ****************************************** */
#define HALL_SPEED_FIFO_SIZE ((u8)6) /* Hall信号处理Buffer大小，6次求平均 */

// Hall状态标志
typedef enum {
    HALL_COM_TIMEOUT = 0x01, /* 超时换相错误 */
    HALL_COM_FLG = 0x02,     /* 换相标志，Hall发生变化时置1 */
    HALL_COM_ERR = 0x04,     /* Hall变化和预期值不一致 */
    HALL_DIR_FLG = 0x80      /* 电机运行方向标志，0:正转 1:反转 */
} HALL_State_m;

// Hall控制结构体
typedef struct
{
    volatile u8 bHallState;                  /* 当前滤波后的Hall状态 */
    volatile u8 bHallState_Last;             /* 上一次滤波后的Hall状态 */
    u8 bHallState_Origin;                    /* Hall原始信号 */
    HALL_State_m bHallRunFlg;                /* Hall运行状态标志，故障、方向、超时等状态 */
    u8 bFstHallSigFlg;                       /* 第一次Hall变化标志，用来Hall信号去抖 */
    u32 wSensorPeriod[HALL_SPEED_FIFO_SIZE]; /* Hall信号周期变化数组，用来求信号平均值 */
    u32 wOldSensorPeriod;                    /* 上次Hall信号滤波后周期值 */
    u32 wMotorSpeedAvgCnt;                   /* Hall信号滤波后周期值, 以HAll模块时钟周期计数 */
    u8 bSpeedFIFO_Index;                     /* Hall信号周期变化数组指针 */
    volatile u32 wHallPWMTimerCnt;           /* Hall变化周期以PWM周期冗余计数，计数器值 */
    u8 bMotorDirtionCtrl;                    /* 期望控制的电机运行方向 */

    volatile u16 nElectrical_Angle;    /* 当前转子位置角度值，0~65535对应0~360度 */
    volatile u16 nOldElectrical_Angle; /* 上一次转子位置角度值，0~65535对应0~360度 */
    volatile u16 nMidElectrical_Angle; /* 扇区中间角度 */
    volatile s16 nEndElectricalAngle;  /* 最大角度增量限制值，在Hall处理错误时，用来快速找回正确角度 */
    volatile u16 nTabElectrical_Angle; /* 直接查表等到的当前转子电角度值 */
    volatile u32 wHallCapCnt;          /* Hall模块捕捉累计值 */
    u16 nPhaseShift;                   /* 当前Hall角度计算角度偏移值 */
    u16 nPhaseShiftOffset;             /* 当前Hall角度偏移值设置 */

    u8 bHallPossableTab[8];  /* Hall换相表，存储下一相Hall值 */
    u8 bHallPossableTab2[8]; /* Hall换相表，存储上一相Hall值 */
    u8 bHallCommTab[8];      /* Hall换相表 */
    u8 HallConversion[8];    /* Hall映射表 */
} HALL_Handle_t;

extern HALL_Handle_t Hall_handle;

u8 ReadHallState(void);
void Update_HallState(HALL_Handle_t *this);
void HALL_IRQProcess(HALL_Handle_t *this);

#endif
