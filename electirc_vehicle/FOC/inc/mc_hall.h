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
#define S16_1_PHASE_SHIFT   (u16)(182) /* 1�Ƚǹ�һ������ 1/360*65536, �Ƕ�0~360�ȱ仯�� ��Ӧ��һ������0~65535 */

#define REDUCE_TOQUE_PUSE  0x01 /* ����ٶȽϵ�ʱ�����򽵵ͻ��������ת�ز�����־ */
#define REDUCE_TOQUE_MINUS 0x02 /* ����ٶȽϵ�ʱ�����򽵵ͻ��������ת�ز�����־ */

/* ****************************************** HALL  ****************************************** */
#define HALL_SPEED_FIFO_SIZE ((u8)6) /* Hall�źŴ���Buffer��С��6����ƽ�� */

// Hall״̬��־
typedef enum {
    HALL_COM_TIMEOUT = 0x01, /* ��ʱ������� */
    HALL_COM_FLG = 0x02,     /* �����־��Hall�����仯ʱ��1 */
    HALL_COM_ERR = 0x04,     /* Hall�仯��Ԥ��ֵ��һ�� */
    HALL_DIR_FLG = 0x80      /* ������з����־��0:��ת 1:��ת */
} HALL_State_m;

// Hall���ƽṹ��
typedef struct
{
    volatile u8 bHallState;                  /* ��ǰ�˲����Hall״̬ */
    volatile u8 bHallState_Last;             /* ��һ���˲����Hall״̬ */
    u8 bHallState_Origin;                    /* Hallԭʼ�ź� */
    HALL_State_m bHallRunFlg;                /* Hall����״̬��־�����ϡ����򡢳�ʱ��״̬ */
    u8 bFstHallSigFlg;                       /* ��һ��Hall�仯��־������Hall�ź�ȥ�� */
    u32 wSensorPeriod[HALL_SPEED_FIFO_SIZE]; /* Hall�ź����ڱ仯���飬�������ź�ƽ��ֵ */
    u32 wOldSensorPeriod;                    /* �ϴ�Hall�ź��˲�������ֵ */
    u32 wMotorSpeedAvgCnt;                   /* Hall�ź��˲�������ֵ, ��HAllģ��ʱ�����ڼ��� */
    u8 bSpeedFIFO_Index;                     /* Hall�ź����ڱ仯����ָ�� */
    volatile u32 wHallPWMTimerCnt;           /* Hall�仯������PWM�������������������ֵ */
    u8 bMotorDirtionCtrl;                    /* �������Ƶĵ�����з��� */

    volatile u16 nElectrical_Angle;    /* ��ǰת��λ�ýǶ�ֵ��0~65535��Ӧ0~360�� */
    volatile u16 nOldElectrical_Angle; /* ��һ��ת��λ�ýǶ�ֵ��0~65535��Ӧ0~360�� */
    volatile u16 nMidElectrical_Angle; /* �����м�Ƕ� */
    volatile s16 nEndElectricalAngle;  /* ���Ƕ���������ֵ����Hall�������ʱ�����������һ���ȷ�Ƕ� */
    volatile u16 nTabElectrical_Angle; /* ֱ�Ӳ��ȵ��ĵ�ǰת�ӵ�Ƕ�ֵ */
    volatile u32 wHallCapCnt;          /* Hallģ�鲶׽�ۼ�ֵ */
    u16 nPhaseShift;                   /* ��ǰHall�Ƕȼ���Ƕ�ƫ��ֵ */
    u16 nPhaseShiftOffset;             /* ��ǰHall�Ƕ�ƫ��ֵ���� */

    u8 bHallPossableTab[8];  /* Hall������洢��һ��Hallֵ */
    u8 bHallPossableTab2[8]; /* Hall������洢��һ��Hallֵ */
    u8 bHallCommTab[8];      /* Hall����� */
    u8 HallConversion[8];    /* Hallӳ��� */
} HALL_Handle_t;

extern HALL_Handle_t Hall_handle;

u8 ReadHallState(void);
void Update_HallState(HALL_Handle_t *this);
void HALL_IRQProcess(HALL_Handle_t *this);

#endif
