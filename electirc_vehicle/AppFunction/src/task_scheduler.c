#include "task_scheduler.h"

STR_TaskScheduler gS_TaskScheduler; /* ������Ƚṹ�� */

/**
 * @brief ���������
 * @retval None
 */
void Task_Scheduler(void)
{
    volatile u32 t_data;

    if (gS_TaskScheduler.bTimeCnt1ms >= TASK_SCHEDU_1MS) { /* 1�����¼���������� */
        gS_TaskScheduler.bTimeCnt1ms = 0;
    }

    if (gS_TaskScheduler.nTimeCnt10ms >= TASK_SCHEDU_10MS) { /* 10�����¼���������� */
        gS_TaskScheduler.nTimeCnt10ms = 0;
    }

    if (gS_TaskScheduler.nTimeCnt500ms >= TASK_SCHEDU_500MS) { /* 500�����¼���������� */
        gS_TaskScheduler.nTimeCnt500ms = 0;
    }

    if (gS_TaskScheduler.bPWM_UpdateFlg) { /* ÿ��PWM���ڸ���һ�� */
        gS_TaskScheduler.bPWM_UpdateFlg = 0;
    }
}

