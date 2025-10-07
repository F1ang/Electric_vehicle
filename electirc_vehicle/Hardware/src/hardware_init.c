#include "hardware_init.h"

void SoftDelay(u32 cnt)
{
    volatile u32 t_cnt;

    for (t_cnt = 0; t_cnt < cnt; t_cnt++) {
        __NOP();
    }
}

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


void SystemInit(void)
{
    Clock_Init(); /* 时钟初始化 */
}
