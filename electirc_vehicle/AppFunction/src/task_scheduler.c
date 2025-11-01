#include "task_scheduler.h"
#include "mc_hall.h"

STR_TaskScheduler gS_TaskScheduler; /* ������Ƚṹ�� */

/**
 * @brief ���������
 * @retval None
 */
void Task_Scheduler(void)
{
    volatile u32 t_data;

    if (gS_TaskScheduler.nTimeBaseFlg) {
        gS_TaskScheduler.nTimeBaseFlg = 0;

        if (++gS_TaskScheduler.bTimeCnt1ms == TASK_SCHEDU_1MS) { /* 1ms���� */
            gS_TaskScheduler.bTimeCnt1ms = 0;
        }

        if (++gS_TaskScheduler.nTimeCnt10ms == TASK_SCHEDU_10MS) { /* 10ms���� */
            gS_TaskScheduler.nTimeCnt10ms = 0;
        }
    } else {
        if (gS_TaskScheduler.bPWM_UpdateFlg) { /* ÿ��PWM���ڸ���һ�� */
            gS_TaskScheduler.bPWM_UpdateFlg = 0;
            Update_HallState(&Hall_handle);
        }
    }
}
