#include "global_variable.h"
#include "segger_rtt.h"

#if (RTT_FUNCTION == FUNCTION_ON)
struct {
    s16 data0;
    s16 data1;
    s16 data2;
    s16 data3;
} Rttstru;
#endif

/**
 * @brief ADC0 interrupt handler
 * @retval None
 */
void ADC0_IRQHandler(void)
{
    ADC0_IF |= BIT1 | BIT0;

}

/**
 * @brief ADC1 interrupt handler
 * @retval None
 */
void ADC1_IRQHandler(void)
{
    ADC1_IF |= BIT1 | BIT0;
}

/**
 * @brief MCPWM0 interrupt handler
 * @retval None
 */
void MCPWM0_IRQHandler(void)
{
    MCPWM0_IF0 = BIT1 | BIT0;

    ADC0_IF |= BIT1 | BIT0;
}

/**
 * @brief HALL interrupt handler
 * @retval None
 */
void HALL_IRQHandler(void)
{
}

/**
 * @brief HALL0 interrupt handler
 * @retval None
 */
void HALL0_IRQHandler(void)
{
}

/**
 * @brief HALL1 interrupt handler
 * @retval None
 */
void HALL1_IRQHandler(void)
{
}

/**
 * @brief TIMER0 interrupt handler
 * @retval None
 */
void TIMER0_IRQHandler(void)
{
    /* Ê±»ù500us */
    UTIMER0_IF |= TIMER_IF_ZERO;
}

/**
 * @brief TIMER1 interrupt handler
 * @retval None
 */
void TIMER1_IRQHandler(void)
{
}

/**
 * @brief TIMER2 interrupt handler
 * @retval None
 */
void TIMER2_IRQHandler(void)
{
}

/**
 * @brief TIMER3 interrupt handler
 * @retval None
 */
void TIMER3_IRQHandler(void)
{
}

/**
 * @brief UTIMER1 interrupt handler
 * @retval None
 */
void UTIMER1_IRQHandler(void)
{
}

/**
 * @brief UTIMER2 interrupt handler
 * @retval None
 */
void UTIMER2_IRQHandler(void)
{
}

/**
 * @brief CMP interrupt handler
 * @retval None
 */
void CMP_IRQHandler(void)
{
}

/**
 * @brief UART0 interrupt handler
 * @retval None
 */
void UART0_IRQHandler(void)
{
}

/**
 * @brief UART1 interrupt handler
 * @retval None
 */
void UART1_IRQHandler(void)
{
    if (UART1_IF & 0x10) {
        UART1_IF |= 0x10;
    } else if (UART1_IF & 0x20) {
        UART1_IF |= 0x20;
    }
}

/**
 * @brief UART2 interrupt handler
 * @retval None
 */
void UART2_IRQHandler(void)
{
}

/**
 * @brief SysTick interrupt handler
 * @retval None
 */
void SysTick_Handler(void)
{
}

void HardFault_Handler(void)
{
}

void MemManageFault_Handler(void)
{
}

void BusFault_Handler(void)
{
}

void UsageFault_Handler(void)
{
}

void SPI0_IRQHandler(void)
{
}

void DSP0_IRQHandler(void)
{
}

void SIF0_IRQHandler(void)
{
}
void WAKE_IRQHandler(void)
{
}
void SW_IRQHandler(void)
{
}
void PWRDN_IRQHandler(void)
{
}
void CL0_IRQHandler(void)
{
}

