#include "hardware_init.h"

/**
 * @brief ϵͳʱ��
 * @retval None
 */
void SystemInit(void)
{
    Clock_Init(); /* ʱ�ӳ�ʼ�� */
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
 * @brief ʱ�ӳ�ʼ��
 * @retval None
 */
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

/**
 * @brief Ӳ����ʼ��
 * @retval None
 */
void Hardware_init(void)
{
    __disable_irq(); /* �ر��ж� �ж��ܿ��� */
    SYS_WR_PROTECT = 0x7a83;
    IWDG_DISABLE(); /* �رտ��Ź�*/
    GPIO_init();    /* GPIO��ʼ�� */
    SoftDelay(100); /* ��ʱ�ȴ�Ӳ����ʼ���ȶ� */
    __enable_irq(); /* �������ж� */
}

/**
 * @brief GPIO��ʼ��
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

