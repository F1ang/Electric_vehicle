#include "hardware_init.h"

/**
 * @brief 系统时钟
 * @retval None
 */
void SystemInit(void)
{
    Clock_Init(); /* 时钟初始化 */
}

/**
 * @brief Software delay function
 * @param cnt Delay count
 * @retval None
 */
void SoftDelay(u32 cnt)
{
    volatile u32 t_cnt;

    for (t_cnt = 0; t_cnt < cnt; t_cnt++) {
        __NOP();
    }
}

/**
 * @brief 时钟初始化
 * @retval None
 */
void Clock_Init(void)
{
    SYS_WR_PROTECT = 0x7a83; /* 解除系统寄存器写保护 */
    SYS_SFT_RST = 0xffffffff;
    SYS_SFT_RST = 0;
    SYS_AFE_REG5 = BIT15;  /* BIT15:PLLPDN 开PLL */
    SYS_AFE_REG6 &= ~0300; /* 配置为4.0V监控，可根据实际配置其它档位：3.25V/3.5V/3.75V/4.0V */
    while (SYS_AFE_DBG & BIT15) {
        ;
    } /*  等待外部5VOK */
    SoftDelay(100);           /* 延时100us, 等待PLL稳定 21.4.17*/
    SYS_CLK_CFG = 0x000011ff; /* BIT8:0: CLK_HS,1:PLL  | BIT[7:0]CLK_DIV  | 1ff对应96M时钟 */
    SYS_WR_PROTECT = 0;       /* 关闭系统寄存器写操作*/
}

/**
 * @brief 硬件初始化
 * @retval None
 */
void Hardware_init(void)
{
    __disable_irq(); /* 关闭中断 中断总开关 */
    SYS_WR_PROTECT = 0x7a83;
    IWDG_DISABLE(); /* 关闭看门狗*/
    GPIO_init();    /* GPIO初始化 */
    SoftDelay(100); /* 延时等待硬件初始化稳定 */
    __enable_irq(); /* 开启总中断 */
}

/**
 * @brief GPIO初始化
 * @retval None
 */
void GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin =GPIO_Pin_3 |GPIO_Pin_6| GPIO_Pin_7;
    GPIO_Init(GPIO0, &GPIO_InitStruct);
}

