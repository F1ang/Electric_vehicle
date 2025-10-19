#include "hardware_init.h"
#include "mc_type.h"

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
 * @brief GPIO初始化
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
 * @brief UART0初始化
 * @retval None
 */
void UART_init(void)
{
    UART_InitTypeDef UART_InitStruct;

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate = 115200;               /* 设置波特率38400 */
    UART_InitStruct.WordLength = UART_WORDLENGTH_8b; /* 发送数据长度8位 */
    UART_InitStruct.StopBits = UART_STOPBITS_1b;
    UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB; /* 先发送LSB */
    UART_InitStruct.ParityMode = UART_Parity_NO;    /* 无奇偶校验 */
    UART_InitStruct.IRQEna = UART_IF_RcvOver;
    UART_Init(UART0, &UART_InitStruct);
}

/**
 * @brief 三电阻采样通道配置
 * @retval None
 */
void ADC_3Shunt_NormalModeCFG(void)
{
    ADC_CHN_GAIN_CFG(ADC0, CHN0, ADC0_CURRETN_A_CHANNEL, ADC_GAIN3V6); /*ADC第0次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN1, ADC0_CURRETN_B_CHANNEL, ADC_GAIN3V6); /*ADC第1次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN2, ADC0_CURRETN_A_CHANNEL, ADC_GAIN3V6); /*ADC第2次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN3, ADC0_CURRETN_B_CHANNEL, ADC_GAIN3V6); /*ADC第3次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN4, ADC_BUS_VOL_CHANNEL, ADC_GAIN3V6);    /*ADC第4次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN5, M0_ADC_BUS_CURR_CH, ADC_GAIN3V6);     /*ADC第5次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC0, CHN6, ADC0_TEMP_CHANNEL, ADC_GAIN3V6);      /*ADC第6次采样通道配置*/

    // ADC1
    ADC_CHN_GAIN_CFG(ADC1, CHN0, BEMF_CH_A, ADC_GAIN3V6); /*ADC第0次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC1, CHN1, BEMF_CH_A, ADC_GAIN3V6); /*ADC第1次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC1, CHN2, BEMF_CH_B, ADC_GAIN3V6); /*ADC第2次采样通道配置*/
    ADC_CHN_GAIN_CFG(ADC1, CHN3, BEMF_CH_C, ADC_GAIN3V6); /*ADC第3次采样通道配置*/
}

/**
 * @brief ADC0初始化
 * @retval None
 */
void ADC0_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    // ADC0设置
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.RE = 0;               /* DMA请求使能 */
    ADC_InitStructure.DATA_ALIGN = DISABLE; /* DAT右对齐使能(左对齐) */
    ADC_InitStructure.CSMP = DISABLE;       /* 连续采样使能 */
    ADC_InitStructure.TCNT = 0;             /* 触发一次采样所需的事件数 0：表示需要发生 1 次事件才能触发一次采样  8KHZ  */
    //                                                                1：表示需要发生 2 次事件才能触发一次采样  16KHZ
    ADC_InitStructure.TROVS = DISABLE; /* 手动触发过采样使能，开启后一次采样需要多次触发 */
    ADC_InitStructure.OVSR = 0;        /* 过采样率 */

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0 | ADC_TRIG_MCPWM0_T1; // 比较点t0 ,t1触发信号
    ADC_InitStructure.NSMP = 1;                                       // 0:单段采样， 1:两段采样
    ADC_InitStructure.IE = 0;                                         // 第二段常规采样完成中断使能  不使能ADC0

    ADC_InitStructure.S1 = 2;  // 第一段常规采样次数 2
    ADC_InitStructure.S2 = 4;  // 第二段常规采样次数 4
    ADC_InitStructure.IS1 = 0; // 空闲采样次数

#else
#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; // 触发信号
    ADC_InitStructure.NSMP = 0;                  // 0:单段采样， 1:两段采样
    ADC_InitStructure.IE = ADC_SF1_IE;           // 第一段常规采样完成中断使能
    ADC_InitStructure.S1 = 6;                    // 第一段常规采样次数
    ADC_InitStructure.S2 = 0;                    // 第二段常规采样次数
    ADC_InitStructure.IS1 = 0;                   // 空闲采样次数

#else
#if ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; /* 触发信号 */
    ADC_InitStructure.NSMP = 0;                  /* 0:单段采样， 1:两段采样 */
    ADC_InitStructure.IE = ADC_SF1_IE;           /* 第一段常规采样完成中断使能 */
    ADC_InitStructure.S1 = 6;                    /* 第一段常规采样次数 */
    ADC_InitStructure.S2 = 0;                    /* 第二段常规采样次数 */
    ADC_InitStructure.IS1 = 0;                   /* 空闲采样次数 */
#endif
#endif
#endif
    ADC_InitStructure.LTH = 0;       /* ADC 模拟看门狗 0 下阈值 */
    ADC_InitStructure.HTH = 0;       /* ADC 模拟看门狗 0 上阈值 */
    ADC_InitStructure.GEN = DISABLE; /* ADC 模拟看门狗 0 对应使能位 */
    ADC_Init(ADC0, &ADC_InitStructure);

    ADC_ClearIRQFlag(ADC0, ADC_ALL_IF); /* 清除所有中断标志位 */

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    ADC_1Shunt_NormalModeCFG();
#elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    ADC_2Shunt_NormalModeCFG();
#elif ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    ADC_3Shunt_NormalModeCFG();
#endif

    ADC0_STATE_RESET(); /* 复位一下 */
}

/**
 * @brief  ADC1初始化
 * @retval None
 */
void ADC1_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    // ADC0设置
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.RE = 0;               /* DMA请求使能 */
    ADC_InitStructure.DATA_ALIGN = DISABLE; /* DAT右对齐使能 */
    ADC_InitStructure.CSMP = DISABLE;       /* 连续采样使能 */
    ADC_InitStructure.TCNT = 0;             /* 触发一次采样所需的事件数 0：表示需要发生 1 次事件才能触发一次采样  8KHZ */
    //                          							          1：表示需要发生 2 次事件才能触发一次采样  16KHZ
    ADC_InitStructure.TROVS = DISABLE; /* 手动触发过采样使能，开启后一次采样需要多次触发 */
    ADC_InitStructure.OVSR = 0;        /* 过采样率 */

    ADC_InitStructure.TRIG = ADC_TRIG_MCPWM0_T0; /* 触发信号 */
    ADC_InitStructure.NSMP = 0;                  /* 0:单段采样， 1:两段采样 */
    ADC_InitStructure.IE = ADC_SF1_IE;           /* 第一段常规采样完成中断使能 */
    ADC_InitStructure.S1 = 4;                    /* 第一段常规采样次数 */
    ADC_InitStructure.S2 = 0;                    /* 第二段常规采样次数 */
    ADC_InitStructure.IS1 = 0;                   /* 空闲采样次数 */

    ADC_InitStructure.LTH = 0;       /* ADC 模拟看门狗 0 下阈值 */
    ADC_InitStructure.HTH = 0;       /* ADC 模拟看门狗 0 上阈值 */
    ADC_InitStructure.GEN = DISABLE; /* ADC 模拟看门狗 0 对应使能位 */
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
 * @brief 硬件初始化
 * @retval None
 */
void Hardware_init(void)
{
    __disable_irq(); /* 关闭中断 中断总开关 */
    SYS_WR_PROTECT = 0x7a83;
    SYS_DBG_CFG |= BIT14; /* 软件复位通用寄存器*/
    IWDG_DISABLE();       /* 关闭看门狗*/

    DSP_Init();  /* DSP模块初始化*/
    GPIO_init(); /* GPIO初始化 */
    UART_init(); /* UART0初始化 */
    ADC0_init(); /* ADC0初始化 */
    ADC1_init(); /* ADC1初始化 */

    SoftDelay(100); /* 延时等待硬件初始化稳定 */
    __enable_irq(); /* 开启总中断 */
}

