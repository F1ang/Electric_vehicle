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
    // TODO: 等待外部5VOK
    // while (SYS_AFE_DBG & BIT15) {
    //     ;
    // } /*  等待外部5VOK */
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

#if TEST_XH_CTRL_OUT_ENABLE
    // 巡航
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIO2, &GPIO_InitStruct);
#endif
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
 * @brief MCPWM0初始化
 * @retval None
 */
void MCPWM_ch012_init(void)
{
    MCPWM_InitTypeDef MCPWM_InitStructure;

    u16 prd;
    u16 deadTime;
    u8 clockDiv;
    // PSTR_DrvCfgPara pParaPtr;

    // pParaPtr = getCfgParaPtr(0);
    // prd = getPWMPrd(pParaPtr);
    // deadTime = getDeadTime(pParaPtr);
    // clockDiv = pParaPtr->mS_GlobalCfg.m_nuPWMPrsc & 0x03;

    prd = 3000;      /* 96000000/(2*3000) = 16000Hz */
    deadTime = 1300; /* ns */
    clockDiv = 0;

    MCPWM_StructInit(&MCPWM_InitStructure);

    MCPWM_InitStructure.CLK_DIV = clockDiv;  /* MCPWM时钟分频设置 */
    MCPWM_InitStructure.MCLK_EN = ENABLE;    /* 模块时钟开启 */
    MCPWM_InitStructure.IO_FLT_CLKDIV = 12;  /* 急停事件(来自IO口信号)数字滤波器时间设置 */
    MCPWM_InitStructure.CMP_FLT_CLKDIV = 12; /* 急停事件(来自比较器信号)数字滤波器时间设置 */

    MCPWM_InitStructure.AUEN = MCPWM0_ALL_AUPDAT; /*自动加载使能*/

    /* MCPWM0_CNT0	 第一组PWM设置*/
    MCPWM_InitStructure.BASE_CNT0_EN = ENABLE; /* 主计数器开始计数使能开关 */
    MCPWM_InitStructure.TH0 = prd;             /* 计数周期设置即MCPWM输出周期*/

    MCPWM_InitStructure.MCPWM_WorkModeCH0 = MCPWM0_CENTRAL_PWM_MODE; /* MCPWM CH0工作模式：中心对齐PWM模式 */
    MCPWM_InitStructure.MCPWM_WorkModeCH1 = MCPWM0_CENTRAL_PWM_MODE; /* 通道工作模式设置，中心对齐或边沿对齐 */
    MCPWM_InitStructure.MCPWM_WorkModeCH2 = MCPWM0_CENTRAL_PWM_MODE;
    MCPWM_InitStructure.DeadTimeCH012N = deadTime; /* 死区时间设置 */
    MCPWM_InitStructure.DeadTimeCH012P = deadTime;

    MCPWM_InitStructure.CMP_CTRL_CNT0 = DISABLE; /* CMP控制CNT0计数使能位 */
    MCPWM_InitStructure.EVT_CNT0_EN = DISABLE;   /* MCPWM_CNT1外部触发使能位 */
    MCPWM_InitStructure.EVT0 = DISABLE;          /* 外部触发 */

    MCPWM_InitStructure.TR0_UP_INTV = DISABLE;
    MCPWM_InitStructure.TR0_T0_UpdateEN = ENABLE; /*T0时刻更新使能*/
    MCPWM_InitStructure.TR0_T1_UpdateEN = DISABLE;
    MCPWM_InitStructure.TR0_AEC = DISABLE; /*自动清除MCPWM0_EIF标志位*/

    MCPWM_InitStructure.TMR0 = (u16)(40 - prd); /* MCPWM_TMR0  设置采样点 */
    MCPWM_InitStructure.TMR1 = (u16)(prd - 1);  /* MCPWM_TMR1 设置 */

#if (PRE_DRIVER_POLARITY == P_HIGH__N_LOW)           /* CHxP 高有效， CHxN低电平有效 */
    MCPWM_InitStructure.CH0N_Polarity_INV = ENABLE;  /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE; /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH1N_Polarity_INV = ENABLE;
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = ENABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

    MCPWM_InitStructure.Switch_CH0N_CH0P = DISABLE; /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
    MCPWM_InitStructure.Switch_CH1N_CH1P = DISABLE; /* 通道交换选择设置 */
    MCPWM_InitStructure.Switch_CH2N_CH2P = DISABLE; /* 通道交换选择设置 */

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = MCPWM0_LOW_LEVEL;
    MCPWM_InitStructure.CH0N_default_output = MCPWM0_HIGH_LEVEL;
    MCPWM_InitStructure.CH1P_default_output = MCPWM0_LOW_LEVEL;  /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH1N_default_output = MCPWM0_HIGH_LEVEL; /* CH1N对应引脚在空闲状态输出高电平 */
    MCPWM_InitStructure.CH2P_default_output = MCPWM0_LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = MCPWM0_HIGH_LEVEL;
#else
#if (PRE_DRIVER_POLARITY == P_HIGH__N_HIGH) /* CHxP 高有效， CHxN高电平有效 */
    MCPWM_InitStructure.CH0N_Polarity_INV = DISABLE; /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE; /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH1N_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

    MCPWM_InitStructure.Switch_CH0N_CH0P = DISABLE; /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
    MCPWM_InitStructure.Switch_CH1N_CH1P = DISABLE; /* 通道交换选择设置 */
    MCPWM_InitStructure.Switch_CH2N_CH2P = DISABLE; /* 通道交换选择设置 */

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = MCPWM0_LOW_LEVEL;
    MCPWM_InitStructure.CH0N_default_output = MCPWM0_LOW_LEVEL;
    MCPWM_InitStructure.CH1P_default_output = MCPWM0_LOW_LEVEL; /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH1N_default_output = MCPWM0_LOW_LEVEL; /* CH1N对应引脚在空闲状态输出高电平 */
    MCPWM_InitStructure.CH2P_default_output = MCPWM0_LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = MCPWM0_LOW_LEVEL;
#endif
#endif

#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    MCPWM_InitStructure.T0_Update0_INT_EN = DISABLE; /* T0更新事件 中断使能位 */
    MCPWM_InitStructure.T1_Update0_INT_EN = ENABLE;  /* T1更新事件 中断使能位*/
    MCPWM_InitStructure.Update0_INT_EN = DISABLE;    /* CNT0 更新事件 中断使能  */
#else
#if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    MCPWM_InitStructure.T0_Update0_INT_EN = DISABLE; /* T0更新事件 中断使能位 */
    MCPWM_InitStructure.T1_Update0_INT_EN = DISABLE; /* T1更新事件 中断使能位*/
    MCPWM_InitStructure.Update0_INT_EN = DISABLE;    /* CNT0更新事件中断使能	双电阻不使用PWM中断  */
#else
#if ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    MCPWM_InitStructure.T0_Update0_INT_EN = DISABLE; /* T0更新事件 中断使能位 */
    MCPWM_InitStructure.T1_Update0_INT_EN = DISABLE; /* T1更新事件 中断使能位*/
    MCPWM_InitStructure.Update0_INT_EN = DISABLE;    /* CNT0 更新事件 中断使能  双电阻不使用PWM中断  */
#endif
#endif
#endif

    MCPWM_InitStructure.CH0N_FAIL_EN = ENABLE; /* 刹车保护 */
    MCPWM_InitStructure.CH0P_FAIL_EN = ENABLE;
    MCPWM_InitStructure.CH1N_FAIL_EN = ENABLE;
    MCPWM_InitStructure.CH1P_FAIL_EN = ENABLE;
    MCPWM_InitStructure.CH2N_FAIL_EN = ENABLE;
    MCPWM_InitStructure.CH2P_FAIL_EN = ENABLE;

    MCPWM_InitStructure.FAIL0_INPUT_EN = DISABLE;
    MCPWM_InitStructure.FAIL0_INT_EN = DISABLE;
    MCPWM_InitStructure.FAIL0_Signal_Sel = MCPWM0_FAIL_SEL_CMP; /* FAIL_0CAP */
    MCPWM_InitStructure.FAIL0_Polarity = MCPWM0_HIGH_LEVEL_ACTIVE;

    MCPWM_InitStructure.FAIL1_INPUT_EN = ENABLE;                /* FAIL_1CAP，打开CAP1 */
    MCPWM_InitStructure.FAIL1_INT_EN = DISABLE;                 /* fail中断 */
    MCPWM_InitStructure.FAIL1_Signal_Sel = MCPWM0_FAIL_SEL_CMP; /* FAIL_0CAP */
    MCPWM_InitStructure.FAIL1_Polarity = MCPWM0_HIGH_LEVEL_ACTIVE;

    MCPWM_InitStructure.HALT_PRT0 = ENABLE; /* 在接上仿真器debug程序时，暂停MCU运行时，选择各PWM通道正常输出调制信号
                                             还是输出默认电平，保护功率器件 ENABLE:正常输出 DISABLE:输出默认电平*/
    MCPWM_InitStructure.FAIL_0CAP = ENABLE; /* FAIL01事件发生时 将MCPWM0_CNT0值存入MCPWM0_FCNT 使能 */

    MCPWM_Init(&MCPWM_InitStructure);

    g_hDriverPolarity[0] = MCPWM0_IO01;
}

/**
 * @brief 1000hz通用定时器
 * @retval None
 */
void UTimer_init(void)
{
    TIM_TimerInitTypeDef TIM_InitStruct;

    TIM_TimerStrutInit(&TIM_InitStruct);                  /* Timer结构体初始化*/
    TIM_InitStruct.Timer_CH0_WorkMode = TIMER_OPMode_CMP; /* 设置Timer CH0 为比较模式 */
    TIM_InitStruct.Timer_CH0Output = 0;                   /* 计数器回零时，比较模式输出极性控制 */
    TIM_InitStruct.Timer_CH1_WorkMode = TIMER_OPMode_CMP; /* 设置Timer CH1 为比较模式 */
    TIM_InitStruct.Timer_CH1Output = 0;                   /* 计数器回零时，比较模式输出极性控制 */
    TIM_InitStruct.Timer_TH = 48000;                      /* 定时器计数门限初始值48000*/
    TIM_InitStruct.Timer_CMP0 = 24000;                    /* 设置比较模式的CH0比较初始值24000 */
    TIM_InitStruct.Timer_CMP1 = 24000;                    /* 设置比较模式的CH1比较初始值24000 */
    TIM_InitStruct.Timer_FLT = 0;                         /* 设置捕捉模式或编码器模式下对应通道的数字滤波值 */
    TIM_InitStruct.Timer_ClockDiv = TIMER_CLK_DIV1;       /* 设置Timer模块时钟1分频系数 */
    TIM_InitStruct.Timer_IRQEna = Timer_IRQEna_ZC;        /* 开启Timer模块比较中断和过零中断 Timer_IRQEna_CH0 | Timer_IRQEna_CH1 | */
    TIM_TimerInit(UTIMER0, &TIM_InitStruct);
    TIM_TimerCmd(UTIMER0, ENABLE); /* Timer0 模块使能 */
}

/**
 * @brief DAC过流CMP设置
 * @retval None
 */
void DAC_init(void)
{
    s16 nDACRef;
    s32 wDACCmp;
    // PSTR_DrvCfgPara pParaPtr;

    DAC_InitTypeDef DAC_InitStre;
    DAC_StructInit(&DAC_InitStre); /* DAC结构体初始化 */

    // 默认 epwm0 ----- dac0
    // 双电机添加配置功能
    // pParaPtr = getCfgParaPtr(0);
    wDACCmp = (s32)700;

    DAC_InitStre.DAC_GAIN = DAC_RANGE_1V2;  /*DAC输出量程为1.2V*/
    DAC_InitStre.DACOUT_EN = DISABLE;       /*使能DAC输出*/
    DAC_InitStre.TIG_CH_EN = DISABLE;       /*是否使能UTIMER触发步进*/
    DAC_InitStre.DAC_STEP = 0;              /*步进值为0*/
    DAC_Init(DAC_Channel_0, &DAC_InitStre); /* DAC初始化 */

    if (DAC_InitStre.DAC_GAIN == DAC_RANGE_1V2) { /* 加载DAC 1.2V量程校正值 */
        nDACRef = 1200;
    } else if (DAC_InitStre.DAC_GAIN == DAC_RANGE_4V85) { /* 加载DAC 4.85V量程校正值 */
        nDACRef = 4850;
    }

    // 1.2V*700/1200*4096/0.05R=14A
    wDACCmp = (wDACCmp * 4096) / nDACRef;
    wDACCmp = sat(wDACCmp, 0, 4095);

    // 设置DAC的数值
    DAC_OutputValue(DAC_Channel_0, wDACCmp); /* 根据母线采样电阻计算保护值*/

    DAC_Cmd(DAC_Channel_0, ENABLE); /*使能DAC时钟*/
}

/**
 * @brief PGA初始化
 * @retval None
 */
void PGA_init(void)
{
    u16 OPA_mode;
    // PSTR_DrvCfgPara pParaPtr;

    OPA_InitTypeDef OPA_InitStruct;
    OPA_StructInit(&OPA_InitStruct);

    // pParaPtr = getCfgParaPtr(0);
    // OPA_mode = pParaPtr->mS_GlobalCfg.m_bOPAValue;

    OPA_mode = PGA_GAIN_32;
    OPA_InitStruct.OPA_IT = PGA_IT_1;  /*opa偏置电流调节*/
    OPA_InitStruct.OPA_CLEna = ENABLE; /*使能OPA*/
    // 选择160：10k，外接的是两个20k 放大倍数160/(10k+20k*2) = 3.2
    OPA_InitStruct.OPA_Gain = OPA_mode;

    OPA_Init(OPA0, &OPA_InitStruct);
    OPA_Init(OPA1, &OPA_InitStruct);
    OPA_Init(OPA2, &OPA_InitStruct);
    OPA_Init(OPA3, &OPA_InitStruct);
}

/**
 * @brief CMP0过流配置
 * @retval None
 */
void CMP_init(void)
{
    CMP_InitTypeDef CMP_InitStre;
    CMP_StructInit(&CMP_InitStre);

    // 数字模块时钟使能
    SYS_ModuleClockCmd(SYS_Module_CMP, ENABLE); // add

    CMP_InitStre.CLK_COM_DIV = 0;   /* 比较器共用滤波时钟分频*/
    CMP_InitStre.FT = DISABLE;      /* 比较器快速比较 30ns*/
    CMP_InitStre.HYS = CMP_HYS_0mV; // CMP_HYS_20mV;	/* 比较器滞回电压*/

    // TODO:CMP0_SELP_IP0  CMP_SELN_DAC0
    // CMP0 config
    CMP_InitStre.CMP0.SELP = CMP0_SELP_IP1;           /* 比较器0正端信号选择 */
    CMP_InitStre.CMP0.SELN = CMP_SELN_REF;            /* 比较器0负端信号选择 */
    CMP_InitStre.CMP0.RE = DISABLE;                   /* 比较器0DMA失能*/
    CMP_InitStre.CMP0.POL = CMP_HIGH_LEVEL;           /* 比较器0高电平输出有效*/
    CMP_InitStre.CMP0.IRQ_TRIG = IRQ_LEVEL_TRIG_MODE; /* 比较器0电平触发中断模式*/
    CMP_InitStre.CMP0.IN_EN = DISABLE;                /* 比较器0信号输入使能 */
    CMP_InitStre.CMP0.IE = DISABLE;                   /* 比较器0信号中断使能 */
    CMP_InitStre.CMP0.FIL_CLK_DIV16 = 2;              /* 即滤波宽度=tclk 周期*16*CMP_FltCnt (CMP_FltCnt分频系数,0~15)*/
    CMP_InitStre.CMP0.FIL_CLK_DIV2 = 2;               /* 比较器 2/1/0 滤波时钟使能 */
    CMP_InitStre.CMP0.CLK_EN = DISABLE;               /* 比较器时钟使能*/
    CMP_InitStre.CMP0.EN = DISABLE;                   /* 比较器0开关 操作SYS_AFE_REG5 */

    // CMP1 config
    CMP_InitStre.CMP1.SELP = CMP1_SELP_IP0;           /* 比较器1正端信号选择 */
    CMP_InitStre.CMP1.SELN = CMP_SELN_DAC0;           /* 比较器1负端信号选择 */
    CMP_InitStre.CMP0.RE = DISABLE;                   /* 比较器1DMA失能*/
    CMP_InitStre.CMP1.POL = CMP_HIGH_LEVEL;           /* 比较器1高电平输出有效*/
    CMP_InitStre.CMP1.IRQ_TRIG = IRQ_LEVEL_TRIG_MODE; /* 比较器1电平触发中断模式*/
    CMP_InitStre.CMP1.IN_EN = DISABLE;                /* 比较器1信号输入使能 */
    CMP_InitStre.CMP1.IE = DISABLE;                   /* 比较器1信号中断使能 */
    CMP_InitStre.CMP1.FIL_CLK_DIV16 = 2;              /* 即滤波宽度=tclk 周期*16*CMP_FltCnt (CMP_FltCnt分频系数,0~15)*/
    CMP_InitStre.CMP1.FIL_CLK_DIV2 = 2;               /* 比较器 2/1/0 滤波时钟使能 */
    CMP_InitStre.CMP1.CLK_EN = ENABLE;                /* 比较器时钟使能*/
    CMP_InitStre.CMP1.EN = ENABLE;                    /* 比较器0开关 操作SYS_AFE_REG5 */

    CMP_Init(&CMP_InitStre); /* 比较器初始化 */
    // CMP_Cmd(CMP_CHN_0, ENABLE); /* 比较器0时钟使能*/
    CMP_Cmd(CMP_CHN_1, ENABLE); /* 比较器1时钟使能*/
}

/**
 * @brief HALL计数器 1us计数
 * @retval None
 */
void HALL_init(void)
{
    HALL_InitTypeDef HALL_InitStruct;

    HALL_StructInit(&HALL_InitStruct);

    HALL_InitStruct.FilterLen = 512;               /* Hall信号数字滤波长度 512个时钟周期 */
    HALL_InitStruct.ClockDivision = HALL_CLK_DIV1; /* 设置Hall模块时钟分频系数 */
    HALL_InitStruct.Filter75_Ena = DISABLE;        /* Hall信号滤波方式，7判5模式或者全1有效模式 */
    HALL_InitStruct.HALL_Ena = ENABLE;             /* 模块使能 */

    HALL_InitStruct.Capture_IRQ_Ena = ENABLE;  /* 捕捉中断使能 */
    HALL_InitStruct.OverFlow_IRQ_Ena = ENABLE; /* 超时中断使能 */
    HALL_InitStruct.softIE = DISABLE;          /* 软件中断失能 */

    HALL_InitStruct.CountTH = 9600000; /* Hall模块计数模值，计数超过模值会产生超时中断 */

    HALL_Init(&HALL_InitStruct); /* HALL初化 */
    HALL_Cmd(ENABLE);            /* HALL使能 */
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

    DSP_Init();         /* DSP模块初始化*/
    GPIO_init();        /* GPIO初始化 */
    UART_init();        /* UART0初始化 */
    ADC0_init();        /* ADC0初始化 */
    ADC1_init();        /* ADC1初始化 */
    MCPWM_ch012_init(); /* MCPWM0初始化 */
    UTimer_init();      /* 500hz通用定时器初始化 */
    DAC_init();         /* DAC 初始化 */
    PGA_init();         /* PGA 初始化 */
    CMP_init();         /* 比较器初始化 */
    HALL_init();        /* HALL模块初始化 */
    TempSensor_Init();  /* 温度传感器初始化 */
    SoftDelay(100);     /* 延时等待硬件初始化稳定 */

    // 中断优先级设置
    NVIC_SetPriority(TIMER0_IRQn, 2); /*TIMER0中断优先级配置*/
    NVIC_SetPriority(HALL0_IRQn, 2);  /*HALL0_IRQn中断优先级配置*/
    NVIC_SetPriority(ADC0_IRQn, 1);   /*ADC0中断优先级配置*/
    NVIC_SetPriority(ADC1_IRQn, 1);   /*ADC0中断优先级配置*/
    NVIC_SetPriority(MCPWM0_IRQn, 1); /*MCPWM0中断优先级配置*/

    NVIC_EnableIRQ(ADC0_IRQn); /* enable the ADC0 interrupt */
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(HALL0_IRQn);

    __enable_irq(); /* 开启总中断 */
}
