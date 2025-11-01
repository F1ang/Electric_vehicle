/******************************************************************************
 *@brief  任务调度器
 *@author By Spotted Owl
 *@date     2025.11.01
 ******************************************************************************/
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include "global_variable.h"

#define TASK_SCHEDU_1MS   (2)    /* 任务调度1ms计数时长 */
#define TASK_SCHEDU_10MS  (20)   /* 任务调度1ms计数时长 */
#define TASK_SCHEDU_500MS (1000) /* 任务调度500ms计数时长 */

/* 任务调度器结构体 */
typedef struct
{
    u16 nTimeBaseFlg;     /* Timer 中断标记，取值 0或1 */
    u8 bPWM_UpdateFlg;    /* PWM周期更新标志，一次间隔为一个PWM周期 */
    u8 bTimeCnt1ms;       /* 1mS计数器 */
    u16 nTimeCnt10ms;     /* 10mS计数器 */
    u16 nTimeCnt500ms;    /* 500mS计数器 */
    const char *sVersion; /* 程序版本号 */
} STR_TaskScheduler;

extern STR_TaskScheduler gS_TaskScheduler; /* 任务调度结构体 */

void Task_Scheduler(void);

#endif
