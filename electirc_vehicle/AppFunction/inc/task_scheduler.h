/******************************************************************************
 *@brief  ���������
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include "global_variable.h"

#define TASK_SCHEDU_1MS   (2)    /* �������1ms����ʱ�� */
#define TASK_SCHEDU_10MS  (20)   /* �������1ms����ʱ�� */
#define TASK_SCHEDU_500MS (1000) /* �������500ms����ʱ�� */

/* ����������ṹ�� */
typedef struct
{
    u16 nTimeBaseFlg;     /* Timer �жϱ�ǣ�ȡֵ 0��1 */
    u8 bPWM_UpdateFlg;    /* PWM���ڸ��±�־��һ�μ��Ϊһ��PWM���� */
    u8 bTimeCnt1ms;       /* 1mS������ */
    u16 nTimeCnt10ms;     /* 10mS������ */
    u16 nTimeCnt500ms;    /* 500mS������ */
    const char *sVersion; /* ����汾�� */
} STR_TaskScheduler;

extern STR_TaskScheduler gS_TaskScheduler; /* ������Ƚṹ�� */

void Task_Scheduler(void);

#endif
