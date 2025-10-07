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
    SYS_WR_PROTECT = 0x7a83; /* ���ϵͳ�Ĵ���д���� */
    SYS_SFT_RST = 0xffffffff;
    SYS_SFT_RST = 0;
    SYS_AFE_REG5 = BIT15;  /* BIT15:PLLPDN ��PLL */
    SYS_AFE_REG6 &= ~0300; /* ����Ϊ4.0V��أ��ɸ���ʵ������������λ��3.25V/3.5V/3.75V/4.0V */
    while (SYS_AFE_DBG & BIT15) {
        ;
    } /*  �ȴ��ⲿ5VOK */
    SoftDelay(100);           /* ��ʱ100us, �ȴ�PLL�ȶ� 21.4.17*/
    SYS_CLK_CFG = 0x000011ff; /* BIT8:0: CLK_HS,1:PLL  | BIT[7:0]CLK_DIV  | 1ff��Ӧ96Mʱ�� */
    SYS_WR_PROTECT = 0;       /* �ر�ϵͳ�Ĵ���д����*/
}


void SystemInit(void)
{
    Clock_Init(); /* ʱ�ӳ�ʼ�� */
}
