/***********************************************************************************************************************
    @file     hal_sram.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE SRAM FIRMWARE FUNCTIONS.
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
#define _HAL_SRAM_C_

/* Files includes ------------------------------------------------------------*/
#include "hal_sram.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @addtogroup SRAM
  * @{
  */

/** @defgroup SRAM_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup SRAM_Private_Functions
  * @{
  */

/**
  * @brief  Configure the SRAM read latency cycles.
  * @param  latency: specifies the SRAM read Latency value.
  *         This parameter can be one of the following values:
  * @arg    SRAM_READ_Latency_0
  * @arg    SRAM_READ_Latency_1
  * @arg    SRAM_READ_Latency_2
  * @arg    SRAM_READ_Latency_3
  * @arg    SRAM_READ_Latency_4
  * @arg    SRAM_READ_Latency_5
  * @arg    SRAM_READ_Latency_6
  * @arg    SRAM_READ_Latency_7
  * @retval None.
  */
void SRAM_ReadLatencyConfig(uint8_t latency)
{
    MODIFY_REG(SRAM->CFGR, SRAM_CFGR_RDLAT_Msk, latency);
}

/**
  * @brief  Configure the SRAM write waiting cycles.
  * @param  wait: specifies the SRAM write waiting value.
  *         This parameter can be one of the following values:
  * @arg    SRAM_WRITE_Wait_0
  * @arg    SRAM_WRITE_Wait_1
  * @arg    SRAM_WRITE_Wait_2
  * @arg    SRAM_WRITE_Wait_3
  * @arg    SRAM_WRITE_Wait_4
  * @arg    SRAM_WRITE_Wait_5
  * @arg    SRAM_WRITE_Wait_6
  * @arg    SRAM_WRITE_Wait_7
  * @retval None.
  */
void SRAM_WriteWaitingConfig(uint8_t wait)
{
    MODIFY_REG(SRAM->CFGR, SRAM_CFGR_WRWT_Msk, wait);
}

/**
  * @brief  Enables or disables the ECC One-bit error correction function.
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void SRAM_EccOneBitCorrectCmd(FunctionalState state)
{
    (state) ?                                             \
    (SRAM->ECCCR |= (0x01U << SRAM_ECCCR_ECCSECEN_Pos)) : \
    (SRAM->ECCCR &= ~(0x01U << SRAM_ECCCR_ECCSECEN_Pos));
}

/**
  * @brief  Enables or disables the ECC error information update function.
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void SRAM_EccErrorUpdateCmd(FunctionalState state)
{
    (state) ?                                             \
    (SRAM->ECCCR |= (0x01U << SRAM_ECCCR_ECCEUPEN_Pos)) : \
    (SRAM->ECCCR &= ~(0x01U << SRAM_ECCCR_ECCEUPEN_Pos));
}

/**
  * @brief  Configure the ECC error injection function.
  * @param  inject_msk: specifies the ECC error injection bit.
  *         Select bits ranging from 0~38 bits to enable error injection.
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void SRAM_EccErrorInjectConfig(uint64_t inject_msk)
{
    SRAM->ECCEINR0 = (uint32_t)(inject_msk & SRAM_ECCEINR0_ECCEIENL_Msk);
    SRAM->ECCEINR1 = (uint32_t)((inject_msk >> 32) & SRAM_ECCEINR1_ECCEIENH_Msk);
}

/**
  * @brief  Configure the ECC detection interrupt function.
  * @param  it: specifies the SRAM ECC detection interrupt sources.
  *         This parameter can be one of the following values:
  * @arg    SRAM_ECC_IT_SNE
  * @arg    SRAM_ECC_IT_DBE
  * @arg    SRAM_ECC_IT_RBDBE
  * @param  state: new state of the SRAM peripheral.
  * @retval None.
  */
void SRAM_EccITConfig(uint8_t it, FunctionalState state)
{
    (state) ?             \
    (SRAM->ECCCR |= it) : \
    (SRAM->ECCCR &= ~it);
}

/**
  * @brief  Checks whether the specified SRAM ECC detection flag is set or not.
  * @param  it: specifies the SRAM ECC detection status flag.
  *         This parameter can be one of the following values:
  * @arg    SRAM_ECC_IT_SNE
  * @arg    SRAM_ECC_IT_DBE
  * @arg    SRAM_ECC_IT_RBDBE
  * @retval The new state of SRAM ECC detection flag (SET or RESET).
  */
FlagStatus SRAM_GetEccITStatus(uint8_t it)
{
    return ((SRAM->ECCSR & (it >> 2)) ? SET : RESET);
}

/**
  * @brief  Clears the specified SRAM ECC detection flag.
  * @param  it: specifies the SRAM ECC detection status flag.
  *         This parameter can be one of the following values:
  * @arg    SRAM_ECC_IT_SNE
  * @arg    SRAM_ECC_IT_DBE
  * @arg    SRAM_ECC_IT_RBDBE
  * @retval None.
  */
void SRAM_ClearEccITPendingBit(uint8_t it)
{
    SRAM->ECCSR = (it >> 2);
}

/**
  * @brief  Returns the address of the word read error, when an ECC
  *         error occurred on the read SRAM.
  * @param  None.
  * @retval the address of the word read error.
  */
uint32_t SRAM_GetEccReadErrorAddress(void)
{
    return (SRAM->ECCEADRR);
}

/**
  * @brief  Returns the Syndrome Code, when an ECC error occurred on the read SRAM.
  * @param  None.
  * @retval the Syndrome Code.
  */
uint8_t SRAM_GetEccSyndromeCode(void)
{
    return ((uint8_t)SRAM->ECCESYNR);
}

/**
  * @brief  Returns the Read error data, when an ECC error occurred on the read SRAM.
  * @param  None.
  * @retval the Read error data.
  */
uint32_t SRAM_GetReadErrorData(void)
{
    return (SRAM->ECCEDATAR);
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
