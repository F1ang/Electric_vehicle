/***********************************************************************************************************************
    @file     hal_usart.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE USART FIRMWARE FUNCTIONS.
  **********************************************************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2023> <MindMotion></center></h2>

      Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
    following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
       the following disclaimer in the documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
       promote products derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#define _HAL_USART_C_

/* Files includes ------------------------------------------------------------*/
#include "hal_usart.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @addtogroup USART
  * @{
  */

/** @defgroup USART_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param usart: Select the USART peripheral.
  * @retval : None
  */
void USART_DeInit(USART_TypeDef *usart)
{
    if (usart == USART1)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART, DISABLE);
    }
}

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *   parameters in the USART_InitStruct .
  * @param usart: Select the USART peripheral.
  * @param init_struct: pointer to a USART_InitTypeDef structure
  *   that contains the configuration information for the specified USART peripheral.
  * @retval : None
  */
void USART_Init(USART_TypeDef *usart, USART_InitTypeDef *init_struct)
{
    uint32_t apbclock = 0x00;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    MODIFY_REG(usart->CR1, (USART_CR1_PS_Msk | USART_CR1_PCE_Msk), init_struct->USART_Parity);
    MODIFY_REG(usart->CR1, USART_CR1_DL_Msk, init_struct->USART_WordLength);
    MODIFY_REG(usart->CR1, USART_CR1_RE_Msk | USART_CR1_TE_Msk, init_struct->USART_Mode);

    MODIFY_REG(usart->CR2, USART_CR2_STOP_Msk, init_struct->USART_StopBits);

    MODIFY_REG(usart->CR3, USART_CR3_RTSE_Msk | USART_CR3_CTSE_Msk, init_struct->USART_HardwareFlowControl);

    /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    if (usart == USART1)
    {
        apbclock = RCC_ClocksStatus.PCLK2_Frequency;
    }

    /* Write to USART BRR */
    usart->BRR &= (~0x000fffff);
    usart->BRR |= (((apbclock / 16) / init_struct->USART_BaudRate) << 4);
    usart->BRR |= (((apbclock / (init_struct->USART_BaudRate)) % 16) << 0);
}

/**
  * @brief  USART_StructInit.
  * @param init_struct: pointer to a USART_InitTypeDef structure
  *   that contains the configuration information for the
  *   specified USART peripheral.
  * @retval : None
  */
void USART_StructInit(USART_InitTypeDef *init_struct)
{
    /* USART_InitStruct members default value */
    init_struct->USART_BaudRate   = 9600;
    init_struct->USART_WordLength = USART_WordLength_8b;
    init_struct->USART_StopBits   = USART_StopBits_1;
    init_struct->USART_Parity     = USART_Parity_No;
    init_struct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    init_struct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
}

/**
  * @brief  Half-duplex selection.
  * @param usart: Select the USART peripheral.
  * @param state: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_HalfDuplexCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                        \
    (usart->CR3 |= (0x01U << USART_CR3_HDSEL_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_HDSEL_Pos));
}

/**
  * @brief  Transmitter enable.
  * @param usart: Select the USART peripheral.
  * @param state: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_TxCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                     \
    (usart->CR1 |= (0x01U << USART_CR1_TE_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_TE_Pos));
}

/**
  * @brief  Receiver enable.
  * @param usart: Select the USART peripheral.
  * @param state: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_RxCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                     \
    (usart->CR1 |= (0x01U << USART_CR1_RE_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_RE_Pos));
}

/**
  * @brief  Enables or disables the specified USART peripheral.
  * @param usart: Select the USART peripheral.
  * @param state: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_Cmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                     \
    (usart->CR1 |= (0x01U << USART_CR1_UE_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_UE_Pos));
}

/**
  * @brief  Enables or disables the specified USART CR1 and CR3 interrupts.
  * @param usart: Select the USART or the USART peripheral.
  * @param usart_it: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  * @arg   USART_IT_IDLE
  * @arg   USART_IT_RXNE
  * @arg   USART_IT_TC
  * @arg   USART_IT_TXE
  * @arg   USART_IT_PE
  * @arg   USART_IT_BEIE
  * @arg   USART_IT_ATBRRCIE
  * @arg   USART_IT_LBDIE
  * @arg   USART_IT_ERR
  * @arg   USART_IT_CTS
  * @param state: New state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_ITConfig(USART_TypeDef *usart, uint32_t usart_it, FunctionalState state)
{
    uint32_t tmp1 = 0, tmp2 = 0;

    /* Get the usart_it position */
    tmp1 = usart_it & 0x1F;
    tmp2 = usart_it >> 5;

    if (tmp2 == 1)                     /* The usart_it to check is in CR1 register */
    {
        (state) ?                                 \
        (usart->CR1 |= ((uint32_t)(1 << tmp1))) : \
        (usart->CR1 &= ~((uint32_t)(1 << tmp1)));
    }
    else if (tmp2 == 2)                /* The usart_it to check is in CR2 register */
    {
        (state) ?                                 \
        (usart->CR2 |= ((uint32_t)(1 << tmp1))) : \
        (usart->CR2 &= ~((uint32_t)(1 << tmp1)));
    }
    else                               /* The usart_it to check is in CR3 register */
    {
        (state) ?                                 \
        (usart->CR3 |= ((uint32_t)(1 << tmp1))) : \
        (usart->CR3 &= ~((uint32_t)(1 << tmp1)));
    }
}

/**
  * @brief  Enables or disables the USART DMA interface.
  * @param usart: Select the USART or the USART peripheral.
  * @param NewState: new state of the DMA Request sources.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval : None
  */
void USART_DMACmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                          \
    (usart->CR3 |= (0x01U << USART_CR3_DMAMODE_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_DMAMODE_Pos));
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param usart: Select the USART or the USART peripheral.
  * @param data: the data to transmit.
  * @retval : None
  */
void USART_SendData(USART_TypeDef *usart, uint16_t data)
{
    /* Transmit Data */
    usart->DR = data & 0x1FFU;
}

/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param usart: Select the USART or the USART peripheral.
  * @retval : The received data.
  */
uint16_t USART_ReceiveData(USART_TypeDef *usart)
{
    /* Receive Data */
    return ((uint16_t)(usart->DR & (uint16_t)0x01FF));
}

/**
  * @brief  Checks whether the specified USART flag is set or not.
  * @param usart: Select the USART or the USART peripheral.
  * @param usart_flag: specifies the flag to check.
  *   This parameter can be one of the following values:
  * @arg   USART_FLAG_PE
  * @arg   USART_FLAG_FE
  * @arg   USART_FLAG_NF
  * @arg   USART_FLAG_ORE
  * @arg   USART_FLAG_IDLE
  * @arg   USART_FLAG_RXNE
  * @arg   USART_FLAG_TC
  * @arg   USART_FLAG_TXE
  * @arg   USART_FLAG_LBD
  * @arg   USART_FLAG_CTS
  * @arg   USART_FLAG_ATBRRC
  * @arg   USART_FLAG_MPRID
  * @arg   USART_FLAG_BE
  * @retval : The new state of USART_FLAG (SET or RESET)
  */
FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, uint32_t usart_flag)
{
    return ((usart->SR & usart_flag) ? SET : RESET);
}

/**
  * @brief  Clears the USARTx's pending flags.
  * @param usart: Select the USART or the USART peripheral.
  * @param usart_flag: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  * @arg   USART_FLAG_RXNE
  * @arg   USART_FLAG_LBD
  * @arg   USART_FLAG_CTS
  * @arg   USART_FLAG_ATBRRC
  * @arg   USART_FLAG_BE
  * @retval : None
  */
void USART_ClearFlag(USART_TypeDef *usart, uint32_t usart_flag)
{
    usart->SR = ~(usart_flag);
}

/**
  * @brief  Checks whether the specified USART  interrupt has occurred or not.
  * @param usart: Select the USART or the USART peripheral..
  * @param usart_it: specifies the flag to check.
  *   This parameter can be any combination of the following values:
  * @arg   USART_IT_IDLE
  * @arg   USART_IT_RXNE
  * @arg   USART_IT_TC
  * @arg   USART_IT_TXE
  * @arg   USART_IT_PE
  * @arg   USART_IT_CTS
  * @arg   USART_IT_ERR
  * @retval : The new state of USART_IT (SET or RESET).
  */
ITStatus USART_GetITStatus(USART_TypeDef *usart, uint32_t usart_it)
{
    ITStatus ret = RESET;

    if (usart_it == USART_IT_IDLE)
    {
        ret = ((usart->SR & USART_SR_IDLE_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_RXNE)
    {
        ret = ((usart->SR & USART_SR_RXNE_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_TC)
    {
        ret = ((usart->SR & USART_SR_TC_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_TXE)
    {
        ret = ((usart->SR & USART_SR_TXE_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_PE)
    {
        ret = ((usart->SR & USART_SR_PE_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_CTS)
    {
        ret = ((usart->SR & USART_SR_CTS_Msk) ? SET : RESET);
    }
    else if (usart_it == USART_IT_ERR)
    {
        ret = ((usart->SR & (USART_SR_FE_Msk | USART_SR_ORE_Msk | USART_SR_NF_Msk)) ? SET : RESET);
    }

    return (ret);
}

/**
  * @brief  Clears the USARTx interrupt pending bits.
  * @param usart: Select the USART or the USART peripheral.
  * @param usart_it: specifies the interrupt pending bit to clear.
  *   This parameter can be one of the following values:
  * @arg   USART_IT_RXNE
  * @arg   USART_IT_CTS
  * @retval : None
  */
void USART_ClearITPendingBit(USART_TypeDef *usart, uint32_t usart_it)
{
    usart->SR = ~(usart_it);
}

/**
  * @brief  Initializes the SYNC MASTER USARTx peripheral according to the specified
  *   parameters in the USART_InitStruct .
  * @param usart: Select the USART or the USART peripheral.
  * @param clock_polarity
  * @arg  USART_Clock_Idle_High
  * @arg  USART_Clock_Idle_Low
  * @param clock_phase
  * @arg  USART_Clock_Phase_1Edge
  * @arg  USART_Clock_Phase_2Edge
  * @param usart_bound: speed.
  * @retval : None
  */
void USART_SyncMasterConfig(USART_TypeDef *usart, uint32_t clock_polarity, uint32_t clock_phase, uint32_t usart_bound)
{
    uint32_t apbclock = 0x00;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    if (usart == USART1)
    {
        apbclock = RCC_ClocksStatus.PCLK2_Frequency;
    }

    /* Write to USART BRR */
    usart->BRR &= (~0x000fffff);
    usart->BRR |= (((apbclock / 4) / usart_bound) << 4);

    usart->CR1 |= (0x01U << USART_CR1_SAS_Pos);
    usart->CR1 |= (0x01U << USART_CR1_MLS_Pos);
    usart->CR3 &= ~(0x01U << USART_CR3_HDSEL_Pos);

    MODIFY_REG(usart->CR2, USART_CR2_CPHA_Msk, clock_phase);
    MODIFY_REG(usart->CR2, USART_CR2_CPOL_Msk, clock_polarity);

    usart->CR1 |= ((0x01U << USART_CR1_TE_Pos) | (0x01U << USART_CR1_RE_Pos));
}

/**
  * @brief  Initializes the SYNC SLAVE USARTx peripheral according to the specified
  *   parameters in the USART_InitStruct.
  * @param usart: Select the USART or the USART peripheral.
  * @param clock_polarity
  * @arg  USART_Clock_Idle_High
  * @arg  USART_Clock_Idle_Low
  * @param clock_phase
  * @arg  USART_Clock_Phase_1Edge
  * @arg  USART_Clock_Phase_2Edge
  * @retval None
  */
void USART_SyncSlaveConfig(USART_TypeDef *usart, uint32_t clock_polarity, uint32_t clock_phase)
{
    usart->CR1 |= (0x01U << USART_CR1_SAS_Pos);
    usart->CR1 |= (0x01U << USART_CR1_MLS_Pos);
    usart->CR3 &= ~(0x01U << USART_CR3_HDSEL_Pos);

    MODIFY_REG(usart->CR2, USART_CR2_CPHA_Msk, clock_phase);
    MODIFY_REG(usart->CR2, USART_CR2_CPOL_Msk, clock_polarity);

    usart->CR1 |= ((0x01U << USART_CR1_TE_Pos) | (0x01U << USART_CR1_RE_Pos));
}

/**
  * @brief  Enables or disables the USART's multiprocessor mode.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  NewState: new state of the USART's multiprocessor mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_MultiProcessorCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                      \
    (usart->CR1 |= (0x01U << USART_CR1_MPE_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_MPE_Pos));
}

/**
  * @brief  Selects the USART WakeUp method.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  method: specifies the USART wakeup method.
  *   This parameter can be one of the following values:
  * @arg    USART_WakeUp_IdleLine
  * @arg    USART_WakeUp_AddressMark
  * @retval None
  */
void USART_WakeUpConfig(USART_TypeDef *usart, uint32_t method)
{
    MODIFY_REG(usart->CR1, USART_CR1_WAKE_Msk, method);
}

/**
  * @brief  Determines if the USART is in mute mode or not.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  state: new state of the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ReceiverWakeUpCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                      \
    (usart->CR1 |= (0x01U << USART_CR1_RWU_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_RWU_Pos));
}

/**
  * @brief  Selects the USART WakeUp address.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  address: specifies the USART wakeup address.
  * @retval None
  */
void USART_WakeUpAddressConfig(USART_TypeDef *usart, uint8_t address)
{
    MODIFY_REG(usart->WADR, USART_WADR_WAD_Msk, address);
}

/**
  * @brief  Selects the USART WakeUp address mask.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  mask: specifies the USART wakeup address mask.
  * @retval None
  */
void USART_WakeUpAddressMaskConfig(USART_TypeDef *usart, uint8_t mask)
{
    MODIFY_REG(usart->WADR, USART_WADR_WADM_Pos, mask << USART_WADR_WADM_Pos);
}

/**
  * @brief  Transmits break characters.
  * @param  usart: Select the USART or the UART peripheral.
  * @retval None
  */
void USART_SendBreak(USART_TypeDef *usart)
{
    usart->CR1 |= (0x01U << USART_CR1_SBK_Pos);
}

/**
  * @brief  Sets the specified USART guard time.
  * @param  usart: Select the USART or the USART peripheral.
  * @param  time: Specifies the guard time.
  *    This value setting ranges from 0 to 255.
  * @retval None
  */
void USART_SetGuardTime(USART_TypeDef *usart, uint8_t time)
{
    MODIFY_REG(usart->GTPR, USART_GTPR_GCNT_Msk, time << USART_GTPR_GCNT_Pos);
}

/**
  * @brief  Sets the system clock prescaler.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  prescaler: specifies the prescaler clock.
  *    This value setting ranges from 0 to 255.
  * @retval None
  */
void USART_SetPrescaler(USART_TypeDef *usart, uint8_t prescaler)
{
    MODIFY_REG(usart->GTPR, USART_GTPR_PSC_Msk, prescaler << USART_GTPR_PSC_Pos);
}

/**
  * @brief  Enables or disables the USART Smart Card mode.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  state: new state of the Smart Card mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART_SmartCardCmd(USART_TypeDef *usart, FunctionalState state)
{
    usart->CR2 &= ~USART_CR2_LINEN_Msk;
    usart->CR3 &= ~USART_CR3_IRDAEN_Msk;

    (state) ?                                       \
    (usart->CR3 |= (0x01U << USART_CR3_SCEN_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_SCEN_Pos));
}

/**
  * @brief  Enables or disables NACK transmission.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  state: new state of the NACK transmission.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_SmartCardNACKCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                       \
    (usart->CR3 |= (0x01U << USART_CR3_NACK_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_NACK_Pos));
}

/**
  * @brief  Sets the USART LIN Break detection length.
  * @param  usart: Select the USART or the USART peripheral.
  * @param  length: Specifies the LIN break detection length.
  *    This parameter can be one of the following values:
  * @arg    USART_LINBreakDetectLength_10b
  * @arg    USART_LINBreakDetectLength_11b
  * @retval None
  */
void USART_LINBreakDetectLengthConfig(USART_TypeDef *usart, uint32_t length)
{
    MODIFY_REG(usart->CR2, USART_CR2_LBDL_Msk, length);
}

/**
  * @brief  Enables or disables the USART LIN mode.
  * @param  usart: Select the USART or the USART peripheral.
  * @param  state: new state of the USART LIN mode.
  *    This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_LINCmd(USART_TypeDef *usart, FunctionalState state)
{
    usart->CR2 &= ~(USART_CR2_CLKEN_Msk | USART_CR2_STOP_Msk);
    usart->CR3 &= ~(USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk | USART_CR3_IRDAEN_Msk);

    (state) ?                                        \
    (usart->CR2 |= (0x01U << USART_CR2_LINEN_Pos)) : \
    (usart->CR2 &= ~(0x01U << USART_CR2_LINEN_Pos));
}

/**
  * @brief  Enables or disables the USART's 8x oversampling mode.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  state: New state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @note
  *   This function has to be called before calling USART_Init()
  *   function in order to have correct baudrate Divider value.
  * @retval None
  */
void USART_OverSampling8Cmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                        \
    (usart->CR1 |= (0x01U << USART_CR1_OVER8_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_OVER8_Pos));
}

/**
  * @brief  Enables or disables the USART's one bit sampling method.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  state: New state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_OneBitMethodCmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ?                                         \
    (usart->CR3 |= (0x01U << USART_CR3_ONEBIT_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_ONEBIT_Pos));
}

/**
  * @brief  Configures the USART's IrDA interface.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  mode: specifies the IrDA mode.
  *   This parameter can be one of the following values:
  * @arg    USART_IrDAMode_LowPower
  * @arg    USART_IrDAMode_Normal
  * @retval None
  */
void USART_IrDAConfig(USART_TypeDef *usart, uint32_t mode)
{
    MODIFY_REG(usart->CR3, USART_CR3_IRLP_Msk, mode);
}

/**
  * @brief  Enables or disables the USART's IrDA interface.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  NewState: new state of the IrDA mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_IrDACmd(USART_TypeDef *usart, FunctionalState state)
{
    usart->CR2 &= ~(USART_CR2_LINEN_Msk | USART_CR2_STOP_Msk | USART_CR2_CLKEN_Msk);
    usart->CR3 &= ~(USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk);

    (state) ?                                         \
    (usart->CR3 |= (0x01U << USART_CR3_IRDAEN_Pos)) : \
    (usart->CR3 &= ~(0x01U << USART_CR3_IRDAEN_Pos));
}

/**
  * @brief  Enables or disables the USART's synchronous mode.
  * @param  usart: Select the USART or the UART peripheral.
  * @param  NewState: new state of the USART's synchronous mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_SyncCmd(USART_TypeDef *usart, FunctionalState state)
{
    usart->CR2 &= ~(USART_CR2_LINEN_Msk);
    usart->CR3 &= ~(USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk | USART_CR3_IRDAEN_Msk);

    (state) ?                                      \
    (usart->CR1 |= (0x01U << USART_CR1_SAS_Pos)) : \
    (usart->CR1 &= ~(0x01U << USART_CR1_SAS_Pos));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
