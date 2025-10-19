#include "hardware_init.h"
#include "mc_type.h"

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
 * @brief GPIO��ʼ��
 * @retval None
 */
void GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);

    // MCPWM0
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_4, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_5, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_6, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_7, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_8, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_9, AF3_MCPWM);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIO1, &GPIO_InitStruct);
}

/**
 * @brief UART0��ʼ��
 * @retval None
 */
void UART_init(void)
{
    UART_InitTypeDef UART_InitStruct;

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate = 115200;               /* ���ò�����38400 */
    UART_InitStruct.WordLength = UART_WORDLENGTH_8b; /* �������ݳ���8λ */
    UART_InitStruct.StopBits = UART_STOPBITS_1b;
    UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB; /* �ȷ���LSB */
    UART_InitStruct.ParityMode = UART_Parity_NO;    /* ����żУ�� */
    UART_InitStruct.IRQEna = UART_IF_RcvOver;
    UART_Init(UART0, &UART_InitStruct);
}

/**
 * @brief ���������ͨ������
 * @retval None
 */
void ADC_3Shunt_NormalModeCFG(void)
{
    ADC_CHN_GAIN_CFG(ADC0, CHN0, ADC0_CURRETN_A_CHANNEL, ADC_GAIN3V6); /*ADC��0�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN1, ADC0_CURRETN_B_CHANNEL, ADC_GAIN3V6); /*ADC��1�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN2, ADC0_CURRETN_A_CHANNEL, ADC_GAIN3V6); /*ADC��2�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN3, ADC0_CURRETN_B_CHANNEL, ADC_GAIN3V6); /*ADC��3�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN4, ADC_BUS_VOL_CHANNEL, ADC_GAIN3V6);    /*ADC��4�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN5, M0_ADC_BUS_CURR_CH, ADC_GAIN3V6);     /*ADC��5�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC0, CHN6, ADC0_TEMP_CHANNEL, ADC_GAIN3V6);      /*ADC��6�β���ͨ������*/

    // ADC1
    ADC_CHN_GAIN_CFG(ADC1, CHN0, BEMF_CH_A, ADC_GAIN3V6); /*ADC��0�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC1, CHN1, BEMF_CH_A, ADC_GAIN3V6); /*ADC��1�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC1, CHN2, BEMF_CH_B, ADC_GAIN3V6); /*ADC��2�β���ͨ������*/
    ADC_CHN_GAIN_CFG(ADC1, CHN3, BEMF_CH_C, ADC_GAIN3V6); /*ADC��3�β���ͨ������*/
}

/**
 * @brief ADC0��ʼ��
 * @retval None
 */
void ADC0_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    // ADC0����
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.RE = 0;               /* DMA����ʹ�� */
    ADC_InitStructure.DATA_ALIGN = DISABLE; /* DAT�Ҷ���ʹ��(�����) */
    ADC_InitStructure.CSMP = DISABLE;       /* ��������ʹ�� */
    ADC_InitStructure.TCNT = 0;             /* ����һ�β���������¼��� 0����ʾ��Ҫ���� 1 ���¼����ܴ���һ�β���  8KHZ  */
    //                                                                1����ʾ��Ҫ���� 2 ���¼����ܴ���һ�β���  16KHZ
    ADC_InitStructure.TROVS = DISABLE; /* �ֶ�����������ʹ�ܣ�������һ�β�����Ҫ��δ��� */
    ADC_InitStructure.OVSR = 0;        /* �������� */

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0 | ADC_TRIG_MCPWM0_T1; // �Ƚϵ�t0 ,t1�����ź�
    ADC_InitStructure.NSMP = 1;                                       // 0:���β����� 1:���β���
    ADC_InitStructure.IE = 0;                                         // �ڶ��γ����������ж�ʹ��  ��ʹ��ADC0

    ADC_InitStructure.S1 = 2;  // ��һ�γ���������� 2
    ADC_InitStructure.S2 = 4;  // �ڶ��γ���������� 4
    ADC_InitStructure.IS1 = 0; // ���в�������

#else
#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; // �����ź�
    ADC_InitStructure.NSMP = 0;                  // 0:���β����� 1:���β���
    ADC_InitStructure.IE = ADC_SF1_IE;           // ��һ�γ����������ж�ʹ��
    ADC_InitStructure.S1 = 6;                    // ��һ�γ����������
    ADC_InitStructure.S2 = 0;                    // �ڶ��γ����������
    ADC_InitStructure.IS1 = 0;                   // ���в�������

#else
#if ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; /* �����ź� */
    ADC_InitStructure.NSMP = 0;                  /* 0:���β����� 1:���β��� */
    ADC_InitStructure.IE = ADC_SF1_IE;           /* ��һ�γ����������ж�ʹ�� */
    ADC_InitStructure.S1 = 6;                    /* ��һ�γ���������� */
    ADC_InitStructure.S2 = 0;                    /* �ڶ��γ���������� */
    ADC_InitStructure.IS1 = 0;                   /* ���в������� */
#endif
#endif
#endif
    ADC_InitStructure.LTH = 0;       /* ADC ģ�⿴�Ź� 0 ����ֵ */
    ADC_InitStructure.HTH = 0;       /* ADC ģ�⿴�Ź� 0 ����ֵ */
    ADC_InitStructure.GEN = DISABLE; /* ADC ģ�⿴�Ź� 0 ��Ӧʹ��λ */
    ADC_Init(ADC0, &ADC_InitStructure);

    ADC_ClearIRQFlag(ADC0, ADC_ALL_IF); /* ��������жϱ�־λ */

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    ADC_1Shunt_NormalModeCFG();
#elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    ADC_2Shunt_NormalModeCFG();
#elif ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    ADC_3Shunt_NormalModeCFG();
#endif

    ADC0_STATE_RESET(); /* ��λһ�� */
}

/**
 * @brief  ADC1��ʼ��
 * @retval None
 */
void ADC1_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    // ADC0����
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.RE = 0;               /* DMA����ʹ�� */
    ADC_InitStructure.DATA_ALIGN = DISABLE; /* DAT�Ҷ���ʹ�� */
    ADC_InitStructure.CSMP = DISABLE;       /* ��������ʹ�� */
    ADC_InitStructure.TCNT = 0;             /* ����һ�β���������¼��� 0����ʾ��Ҫ���� 1 ���¼����ܴ���һ�β���  8KHZ */
    //                          							          1����ʾ��Ҫ���� 2 ���¼����ܴ���һ�β���  16KHZ
    ADC_InitStructure.TROVS = DISABLE; /* �ֶ�����������ʹ�ܣ�������һ�β�����Ҫ��δ��� */
    ADC_InitStructure.OVSR = 0;        /* �������� */

    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; /* �����ź� */
    ADC_InitStructure.NSMP = 0;                  /* 0:���β����� 1:���β��� */
    ADC_InitStructure.IE = ADC_SF1_IE;           /* ��һ�γ����������ж�ʹ�� */
    ADC_InitStructure.S1 = 4;                    /* ��һ�γ���������� */
    ADC_InitStructure.S2 = 0;                    /* �ڶ��γ���������� */
    ADC_InitStructure.IS1 = 0;                   /* ���в������� */

    ADC_InitStructure.LTH = 0;       /* ADC ģ�⿴�Ź� 0 ����ֵ */
    ADC_InitStructure.HTH = 0;       /* ADC ģ�⿴�Ź� 0 ����ֵ */
    ADC_InitStructure.GEN = DISABLE; /* ADC ģ�⿴�Ź� 0 ��Ӧʹ��λ */
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_ClearIRQFlag(ADC1, ADC_ALL_IF);

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    ADC_1Shunt_NormalModeCFG();
#elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    ADC_2Shunt_NormalModeCFG();
#elif ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    ADC_3Shunt_NormalModeCFG();
#endif

    ADC1_STATE_RESET();
}

/**
 * @brief Ӳ����ʼ��
 * @retval None
 */
void Hardware_init(void)
{
    __disable_irq(); /* �ر��ж� �ж��ܿ��� */
    SYS_WR_PROTECT = 0x7a83;
    SYS_DBG_CFG |= BIT14; /* �����λͨ�üĴ���*/
    IWDG_DISABLE();       /* �رտ��Ź�*/

    DSP_Init();  /* DSPģ���ʼ��*/
    GPIO_init(); /* GPIO��ʼ�� */
    UART_init(); /* UART0��ʼ�� */
    ADC0_init(); /* ADC0��ʼ�� */
    ADC1_init(); /* ADC1��ʼ�� */

    SoftDelay(100); /* ��ʱ�ȴ�Ӳ����ʼ���ȶ� */
    __enable_irq(); /* �������ж� */
}

