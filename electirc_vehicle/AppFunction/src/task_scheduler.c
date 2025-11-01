#include "task_scheduler.h"
#include "mc_hall.h"

STR_TaskScheduler gS_TaskScheduler; /* 任务调度结构体 */

/**
 * @brief 任务调度器
 * @retval None
 */
void Task_Scheduler(void)
{
    volatile u32 t_data;

    if (gS_TaskScheduler.nTimeBaseFlg) {
        gS_TaskScheduler.nTimeBaseFlg = 0;

        if (++gS_TaskScheduler.bTimeCnt1ms == TASK_SCHEDU_1MS) { /* 1ms任务 */
            gS_TaskScheduler.bTimeCnt1ms = 0;
        }

        if (++gS_TaskScheduler.nTimeCnt10ms == TASK_SCHEDU_10MS) { /* 10ms任务 */
            gS_TaskScheduler.nTimeCnt10ms = 0;
        }
    } else {
        if (gS_TaskScheduler.bPWM_UpdateFlg) { /* 每个PWM周期更新一次 */
            gS_TaskScheduler.bPWM_UpdateFlg = 0;
            Update_HallState(&Hall_handle);
        }
    }
}
