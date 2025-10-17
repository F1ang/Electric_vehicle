/***********************************************************************************************************************
    @file     hal_flash.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE FLASH FIRMWARE FUNCTIONS.
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
#define _HAL_FLASH_C_

/* Files includes ------------------------------------------------------------*/
#include "hal_flash.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */

/** @defgroup FLASH_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup FLASH_Private_Functions
  * @{
  */

/**
  * @brief  Sets the code latency value.
  * @note   This function can be used for all MM32 devices.
  * @param  latency: specifies the FLASH Latency value.
  *         This parameter can be one of the following values:
  * @arg    FLASH_Latency_0: FLASH Zero Latency cycle
  * @arg    FLASH_Latency_1: FLASH One Latency cycle
  * @arg    FLASH_Latency_2: FLASH Two Latency cycles
  * @arg    FLASH_Latency_3: FLASH Three Latency cycles
  * @arg    FLASH_Latency_4: FLASH Four Latency cycles
  * @arg    FLASH_Latency_5: FLASH Five Latency cycles
  * @arg    FLASH_Latency_6: FLASH Six Latency cycles
  * @arg    FLASH_Latency_7: FLASH Seven Latency cycles
  * @retval None.
  */
void FLASH_SetLatency(uint32_t latency)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, latency);
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval None.
  */
void FLASH_Lock(void)
{
    FLASH->CR |= (0x01U << FLASH_CR_LOCK_Pos);
}

/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval None.
  */
void FLASH_Unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

/**
  * @brief  Enable to program the FLASH Option Byte.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval None.
  */
void FLASH_OPTB_Enable(void)
{
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;
}

/**
  * @brief  Erases a specified FLASH page.
  * @note   This function can be used for all MM32 devices.
  * @param  page_address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32_t page_address)
{
    FLASH->CR |= (0x01U << FLASH_CR_PER_Pos);
    FLASH->AR  = page_address;
    FLASH->CR |= (0x01U << FLASH_CR_STRT_Pos);
    return (FLASH_WaitForLastOperation(EraseTimeout));
}

/**
  * @brief  Erases all FLASH pages.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllPages(void)
{
    FLASH->AR  = FLASH_BASE;
    FLASH->CR |= (0x01U << FLASH_CR_MER_Pos);
    FLASH->CR |= (0x01U << FLASH_CR_STRT_Pos);

    return (FLASH_WaitForLastOperation(EraseTimeout));
}

/**
  * @brief  Erases the FLASH option bytes.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseOptionBytes(void)
{
    FLASH_OPTB_Enable();
    FLASH->AR  = OB_BASE;
    FLASH->CR |= (0x01U << FLASH_CR_OPTER_Pos);
    FLASH->CR |= (0x01U << FLASH_CR_STRT_Pos);

    return (FLASH_WaitForLastOperation(EraseTimeout));
}

/**
  * @brief  Programs a half word at a specified address.
  * @note   This function can be used for all MM32 devices.
  * @param  address: specifies the address to be programmed.
  * @param  data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramHalfWord(uint32_t address, uint16_t data)
{
    FLASH->CR |= (0x01U << FLASH_CR_PG_Pos);

    *(__IO uint16_t *)address = data;

    return (FLASH_WaitForLastOperation(ProgramTimeout));
}

/**
  * @brief  Programs two words at a specified address.
  * @note   This function can be used for all MM32 devices.
  * @param  address: specifies the address to be programmed.
  * @param  data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramDoubleWord(uint32_t address, uint64_t data)
{
    FLASH_Status ret = FLASH_ProgramHalfWord(address, (uint16_t)data);

    if (ret == FLASH_COMPLETE)
    {
        ret = FLASH_ProgramHalfWord(address + 2, (uint16_t)(data >> 16));

        if (ret == FLASH_COMPLETE)
        {
            ret = FLASH_ProgramHalfWord(address + 4, (uint16_t)(data >> 32));

            if (ret == FLASH_COMPLETE)
            {
                ret = FLASH_ProgramHalfWord(address + 6, (uint16_t)(data >> 48));
            }
        }
    }

    return (ret);
}


/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all MM32 devices.
  * @param  address: specifies the address to be programmed.
  *         This parameter can be 0x1FFFF804 or 0x1FFFF806.
  * @param  data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramOptionHalfWord(uint32_t address, uint16_t data)
{
    FLASH_Status ret;

    FLASH_OPTB_Enable();
    FLASH->CR |= (0x01U << FLASH_CR_OPTPG_Pos);

    *(__IO uint16_t *)address = data;

    ret = FLASH_WaitForLastOperation(ProgramTimeout);

    return (ret);
}

/**
  * @brief  Write protection for the specified address
  * @note   This function can be used for all MM32 devices.
  * @param  page: specifies the address of the pages to be write
  *         protected.
  *         This parameter is (0x01 << ((Absolute address - 0x08000000)/0x1000))
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EnableWriteProtection(uint32_t page)
{
    FLASH_Status ret;
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        ret = FLASH_ProgramOptionHalfWord((OB_BASE + 8 + i * 2), ~(page >> (i * 8)));

        if (ret != FLASH_COMPLETE)
        {
            break;
        }
    }

    return (ret);
}



/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The FLASH User Option Bytes values:IWDG_SW(Bit0), RST_STOP(Bit1)
  *         and RST_STDBY(Bit2).
  */
uint32_t FLASH_GetOptionByteRegistor(void)
{
    return (FLASH->OBR);
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes Register value.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The FLASH Write Protection  Option Bytes Register value.
  */
uint32_t FLASH_GetWriteProtectionOptionByte(void)
{
    return (FLASH->WRPR);
}

/**
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @note   This function can be used for all MM32 devices.
  * @param  flag: specifies the FLASH flags to clear.
  *         This parameter can be one of the following values:
  * @arg    FLASH_FLAG_BSY: FLASH Busy flag
  * @arg    FLASH_FLAG_PGERR: FLASH Program error flag
  * @arg    FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
  * @arg    FLASH_FLAG_EOP: FLASH End of Operation flag
  * @arg    FLASH_FLAG_ECCERR: ECC Programming error flag
  * @arg    FLASH_FLAG_OPTERR: FLASH Option Byte error flag
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetFlagStatus(uint16_t flag)
{
    return (((flag == FLASH_FLAG_OPTERR) ? (FLASH->OBR & FLASH_FLAG_OPTERR) : (FLASH->SR & flag)) ? SET : RESET);
}

/**
  * @brief  Clears the FLASH's pending flags.
  * @note   This function can be used for all MM32 devices.
  * @param  flag: specifies the FLASH flags to clear.
  *         This parameter can be any combination of the following values:
  * @arg    FLASH_FLAG_PGERR: FLASH Program error flag
  * @arg    FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
  * @arg    FLASH_FLAG_EOP: FLASH End of Operation flag
  * @arg    FLASH_FLAG_ECCERR: ECC Programming error flag
  * @retval None.
  */
void FLASH_ClearFlag(uint16_t flag)
{
    FLASH->SR = flag;
}

/**
  * @brief  Returns the FLASH Status.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY,
  *         FLASH_ERROR_ECC, FLASH_ERROR_PG, FLASH_ERROR_WRP or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
    return ((FLASH_Status)((FLASH->SR & FLASH_FLAG_BSY)) ? FLASH_BUSY
           : ((FLASH->SR & FLASH_FLAG_PGERR) ? FLASH_ERROR_PG
              : ((FLASH->SR & FLASH_FLAG_WRPRTERR) ? FLASH_ERROR_WRP
                  : ((FLASH->SR & FLASH_FLAG_ECCERR) ? FLASH_ERROR_ECC
                      : FLASH_COMPLETE))));
}

/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @note   This function can be used for all MM32 devices
  * @param  time_out: FLASH programming time_out
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_ECC, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32_t time_out)
{
    uint32_t i;
    FLASH_Status ret = FLASH_COMPLETE;

    do
    {
        ret = FLASH_GetStatus();
        time_out--;

        for (i = 0xFF; i != 0; i--)
        {
        }
    }
    while ((ret == FLASH_BUSY) && (time_out != 0x00));

    FLASH->CR &= ~(FLASH_CR_PG_Msk | FLASH_CR_PER_Msk | FLASH_CR_MER_Msk | FLASH_CR_OPTPG_Msk | FLASH_CR_OPTER_Msk | FLASH_CR_STRT_Msk);
    FLASH->SR  = (0x01U << FLASH_SR_ECCERR_Pos) | (0x01U << FLASH_SR_EOP_Pos) | (0x01U << FLASH_SR_WRPRTERR_Pos) | (0x01U << FLASH_SR_PGERR_Pos);

    return ((FLASH_Status)((time_out == 0x00) ? FLASH_TIMEOUT : ret));
}

/**
  * @brief  Enables or disables the ECC information update function.
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void FLASH_EccInformationUpdateCmd(FunctionalState state)
{
    (state) ?                                             \
    (FLASH->ECC_CR |= (0x01U << FLASH_ECC_CR_RERR_Pos)) : \
    (FLASH->ECC_CR &= ~(0x01U << FLASH_ECC_CR_RERR_Pos));
}

/**
  * @brief  Enables or disables the ECC_CR register write function.
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void FLASH_EccCrWriteCmd(FunctionalState state)
{
    (state) ?                                            \
    (FLASH->ECC_PR |= (0x01U << FLASH_ECC_PR_PRC_Pos)) : \
    (FLASH->ECC_PR &= ~(0x01U << FLASH_ECC_PR_PRC_Pos));
}

/**
  * @brief  Enables or disables the ECC error event request.
  * @param  ecc_sel: Select generate an event request when a 1-bit
  *         or 2-bit ECC error occurs
  *         This parameter can be one of the following values:
  * @arg    FLASH_ECC_1BIT
  * @arg    FLASH_ECC_2BIT
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void FLASH_EccErrorEventRequestCmd(uint8_t ecc_sel, FunctionalState state)
{
    (state) ?                                                   \
    (FLASH->ECC_EER |= (ecc_sel << FLASH_ECC_EER_1BITIE_Pos)) : \
    (FLASH->ECC_EER &= ~(ecc_sel << FLASH_ECC_EER_1BITIE_Pos));
}

/**
  * @brief  Checks the 1-bit or 2-bit ECC error occurs.
  * @note   This function can be used for all MM32 devices.
  * @param  ecc_flag: specifies the FLASH ECC status flag.
  *         This parameter can be one of the following values:
  * @arg    FLASH_FLAG_ECC_1BITF
  * @arg    FLASH_FLAG_ECC_2BITF
  * @retval The new state of FLASH_ECC_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetEccBitFlagStatus(uint8_t ecc_flag)
{
    return ((FLASH->ECC_SR & ecc_flag) ? SET : RESET);
}

/**
  * @brief  Clears the 1-bit or 2-bit ECC error flag.
  * @note   This function can be used for all MM32 devices.
  * @param  ecc_flag: specifies the FLASH ECC status flag.
  *         This parameter can be one of the following values:
  * @arg    FLASH_FLAG_ECC_1BITF
  * @arg    FLASH_FLAG_ECC_2BITF
  * @retval None.
  */
void FLASH_ClearEccBitFlagStatus(uint8_t ecc_flag)
{
    FLASH->ECC_SR &= ~ecc_flag;
}

/**
  * @brief  Get the data address that generated the ECC error.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The data address that generated the ECC error.
  */
uint32_t FLASH_GetEccErrorAddress(void)
{
    return (FLASH->ECC_ADDRR);
}

/**
  * @brief  Get the syndrome Code that generated the ECC error.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The syndrome Code that generated the ECC error.
  */
uint8_t FLASH_GetEccErrorSyndromeCode(void)
{
    return ((uint8_t)FLASH->ECC_SYNR);
}

/**
  * @brief  Get the raw data LSB portion that generated the ECC error.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The raw data LSB portion that generated the ECC error.
  */
uint32_t FLASH_GetEccErrorRawLSBData(void)
{
    return (FLASH->ECC_DLR);
}

/**
  * @brief  Get the raw data MSB portion that generated the ECC error.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval The raw data MSB portion that generated the ECC error.
  */
uint32_t FLASH_GetEccErrorRawMSBData(void)
{
    return (FLASH->ECC_DHR);
}

/**
  * @brief  Erases a specified DataArea page.
  * @note   This function can be used for all MM32 devices.
  * @param  page_address: The DataArea page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY,
  *         FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseDataAreaPage(uint32_t page_address)
{
    FLASH_OPTB_Enable();
    FLASH->AR  = page_address;
    FLASH->CR |= (0x01U << FLASH_CR_OPTER_Pos);
    FLASH->CR |= (0x01U << FLASH_CR_STRT_Pos);

    return (FLASH_WaitForLastOperation(EraseTimeout));
}

/**
  * @brief  Programs a half word at a specified DataArea address.
  * @note   This function can be used for all MM32 devices.
  * @param  address: specifies the DataArea address to be programmed.
  * @param  data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramDataAreaHalfWord(uint32_t address, uint16_t data)
{
    FLASH_Status ret;

    FLASH_OPTB_Enable();
    FLASH->CR |= (0x01U << FLASH_CR_OPTPG_Pos);

    *(__IO uint16_t *)address = data;

    ret = FLASH_WaitForLastOperation(ProgramTimeout);

    return (ret);
}

/**
  * @brief  Disable to program the FLASH Option Byte.
  * @note   This function can be used for all MM32 devices.
  * @param  None.
  * @retval None.
  */
void FLASH_OPTB_Disable(void)
{
    FLASH->CR &= ~(0x01U << FLASH_CR_OPTWRE_Pos);
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
