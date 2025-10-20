#include "task_scheduler.h"

STR_TaskScheduler gS_TaskScheduler; /* 任务调度结构体 */

/**
 * @brief 任务调度器
 * @retval None
 */
void Task_Scheduler(void)
{
    volatile u32 t_data;

    if (gS_TaskScheduler.bTimeCnt1ms >= TASK_SCHEDU_1MS) { /* 1毫秒事件，任务调度 */
        gS_TaskScheduler.bTimeCnt1ms = 0;
    }

    if (gS_TaskScheduler.nTimeCnt10ms >= TASK_SCHEDU_10MS) { /* 10毫秒事件，任务调度 */
        gS_TaskScheduler.nTimeCnt10ms = 0;
    }

    if (gS_TaskScheduler.nTimeCnt500ms >= TASK_SCHEDU_500MS) { /* 500毫秒事件，任务调度 */
        gS_TaskScheduler.nTimeCnt500ms = 0;
    }

    if (gS_TaskScheduler.bPWM_UpdateFlg) { /* 每个PWM周期更新一次 */
        gS_TaskScheduler.bPWM_UpdateFlg = 0;
    }
}

