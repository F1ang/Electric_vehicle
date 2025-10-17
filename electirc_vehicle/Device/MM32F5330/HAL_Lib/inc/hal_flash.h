/***********************************************************************************************************************
    @file     hal_flash.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE FLASH FIRMWARE LIBRARY.
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
#ifndef __HAL_FLASH_H
#define __HAL_FLASH_H

/* Files includes ------------------------------------------------------------*/
#include "mm32_device.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH
  * @{
  */

/** @defgroup FLASH_Exported_Types
  * @{
  */

/**
  * @brief  FLASH Status
  */
typedef enum
{
    FLASH_BUSY = 1,                    /*!< FLASH busy status */
    FLASH_ERROR_PG,                    /*!< FLASH programming error status */
    FLASH_ERROR_WRP,                   /*!< FLASH write protection error status */
    FLASH_ERROR_ECC,                   /*!< FLASH ECC programming error status */
    FLASH_COMPLETE,                    /*!< FLASH end of operation status */
    FLASH_TIMEOUT                      /*!< FLASH Last operation timed out status */
} FLASH_Status;

/**
  * @}
  */

/** @defgroup FLASH_Exported_Constants
  * @{
  */
/**
  * @brief  FLASH Latency
  */
#define FLASH_Latency_0                 (0x00U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Zero Latency cycle */
#define FLASH_Latency_1                 (0x01U << FLASH_ACR_LATENCY_Pos) /*!< FLASH One Latency cycle */
#define FLASH_Latency_2                 (0x02U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Two Latency cycles */
#define FLASH_Latency_3                 (0x03U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Three Latency cycles */
#define FLASH_Latency_4                 (0x04U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Four Latency cycles */
#define FLASH_Latency_5                 (0x05U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Five Latency cycles */
#define FLASH_Latency_6                 (0x06U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Six Latency cycles */
#define FLASH_Latency_7                 (0x07U << FLASH_ACR_LATENCY_Pos) /*!< FLASH Seven Latency cycles */


/**
  * @brief  FLASH_Flags
  */
#define FLASH_FLAG_ECCERR               (0x01U << FLASH_SR_ECCERR_Pos)   /*!< FLASH ECC Program error flag */
#define FLASH_FLAG_EOP                  (0x01U << FLASH_SR_EOP_Pos)      /*!< FLASH End of Operation flag */
#define FLASH_FLAG_PGERR                (0x01U << FLASH_SR_PGERR_Pos)    /*!< FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR             (0x01U << FLASH_SR_WRPRTERR_Pos) /*!< FLASH Write protected error flag */
#define FLASH_FLAG_BSY                  (0x01U << FLASH_SR_BSY_Pos)      /*!< FLASH Busy flag */
#define FLASH_FLAG_OPTERR               (0x01U << FLASH_OBR_OPTERR_Pos)  /*!< FLASH Option Byte error flag */

/**
  * @brief  FLASH_ECC
  */
#define FLASH_ECC_1BIT                  0x01
#define FLASH_ECC_2BIT                  0x02

/**
  * @brief  FLASH_ECC_Flags
  */
#define FLASH_FLAG_ECC_1BITF            (0x01U << FLASH_ECC_SR_1BITF_Pos) /*!< Set when a 1-bit ECC error occurs */
#define FLASH_FLAG_ECC_2BITF            (0x01U << FLASH_ECC_SR_2BITF_Pos) /*!< Set when a 2-bit ECC error occurs */

#define FLASH_KEY1                      ((uint32_t)0x45670123)
#define FLASH_KEY2                      ((uint32_t)0xCDEF89AB)
#define EraseTimeout                    ((uint32_t)0x00002FFF) /*!< new Flash page/sector erase request 6ms, Mass erase 40ms */
#define ProgramTimeout                  ((uint32_t)0x0000012F) /*!< new Flash half word program request 155us */

/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions
  * @{
  */
void FLASH_SetLatency(uint32_t latency);
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_OPTB_Enable(void);
FLASH_Status FLASH_ErasePage(uint32_t page_address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramHalfWord(uint32_t address, uint16_t data);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t address, uint64_t data);
FLASH_Status FLASH_ProgramOptionHalfWord(uint32_t address, uint16_t data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t page);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t time_out);
void FLASH_ClearFlag(uint16_t flag);
uint32_t FLASH_GetOptionByteRegistor(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetFlagStatus(uint16_t flag);

void FLASH_EccInformationUpdateCmd(FunctionalState state);
void FLASH_EccCrWriteCmd(FunctionalState state);
void FLASH_EccErrorEventRequestCmd(uint8_t ecc_sel, FunctionalState state);
FlagStatus FLASH_GetEccBitFlagStatus(uint8_t ecc_flag);
void FLASH_ClearEccBitFlagStatus(uint8_t ecc_flag);
uint32_t FLASH_GetEccErrorAddress(void);
uint8_t FLASH_GetEccErrorSyndromeCode(void);
uint32_t FLASH_GetEccErrorRawLSBData(void);
uint32_t FLASH_GetEccErrorRawMSBData(void);
FLASH_Status FLASH_EraseDataAreaPage(uint32_t page_address);
FLASH_Status FLASH_ProgramDataAreaHalfWord(uint32_t address, uint16_t data);
void FLASH_OPTB_Disable(void);

#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
