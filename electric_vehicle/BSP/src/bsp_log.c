/******************************************************************************
 *@brief  Log file
 *@author By Spotted Owl
 *@date     2025.05.07
 ******************************************************************************/
#include "usart.h"
#include "bsp_log.h"

/*-------------------- log packµÄÊ¹ÓÃprintf() --------------------*/
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
    return ch;
}
