#include "global_variable.h"
#include "hardware_init.h"
#include "task_scheduler.h"

int main(void)
{
    Hardware_init();

    for (;;) {
        Task_Scheduler();
    }
}
