#include "global_variable.h"
#include "hardware_init.h"

int main(void)
{
    Hardware_init();
    for (;;) {
        LED1(1);
        SoftDelay(0xFFFFF);
        LED1(0);
        SoftDelay(0xFFFFF);
    }
}
