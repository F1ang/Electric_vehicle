#define _MAIN_C_

#include "platform.h"
#include "gpio_led_toggle.h"
#include "main.h"

int main(void)
{
    PLATFORM_Init();

    GPIO_LED_Toggle_Sample();

    while (1) {
    }
}
