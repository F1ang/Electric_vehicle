#include "global_variable.h"
#include "hardware_init.h"
#include "task_scheduler.h"

int main(void)
{
    Hardware_init();

    foc_ctrl.nSys_TimerPWM = 0;
    while (foc_ctrl.nSys_TimerPWM < 1500) {
        ;
    }

    for (;;) {
        Task_Scheduler();
    }
}
