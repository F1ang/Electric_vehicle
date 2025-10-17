/***********************************************************************************************************************
    @file     hal_sram.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE SRAM FIRMWARE LIBRARY.
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
#ifndef __HAL_SRAM_H
#define __HAL_SRAM_H

/* Files includes ------------------------------------------------------------*/
#include "mm32_device.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @defgroup SRAM
  * @{
  */

/** @defgroup SRAM_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup SRAM_Exported_Constants
  * @{
  */

/**
  * @brief  SRAM Read Latency
  */
#define SRAM_READ_Latency_0             (0x00U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 0 cycle */
#define SRAM_READ_Latency_1             (0x01U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 1 cycle */
#define SRAM_READ_Latency_2             (0x02U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 2 cycles */
#define SRAM_READ_Latency_3             (0x03U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 3 cycles */
#define SRAM_READ_Latency_4             (0x04U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 4 cycles */
#define SRAM_READ_Latency_5             (0x05U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 5 cycles */
#define SRAM_READ_Latency_6             (0x06U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 6 cycles */
#define SRAM_READ_Latency_7             (0x07U << SRAM_CFGR_RDLAT_Pos)       /*!< SRAM read latency 7 cycles */

/**
  * @brief  SRAM Write Wait
  */
#define SRAM_WRITE_Wait_0               (0x00U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 0 cycle */
#define SRAM_WRITE_Wait_1               (0x01U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 1 cycle */
#define SRAM_WRITE_Wait_2               (0x02U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 2 cycles */
#define SRAM_WRITE_Wait_3               (0x03U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 3 cycles */
#define SRAM_WRITE_Wait_4               (0x04U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 4 cycles */
#define SRAM_WRITE_Wait_5               (0x05U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 5 cycles */
#define SRAM_WRITE_Wait_6               (0x06U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 6 cycles */
#define SRAM_WRITE_Wait_7               (0x07U << SRAM_CFGR_WRWT_Pos)        /*!< SRAM write wait 7 cycles */

/**
  * @brief  SRAM ECC IT Config
  */
#define SRAM_ECC_IT_SNE                 (0x01U << SRAM_ECCCR_ECCSNEIE_Pos)   /*!< Enable the one-bit detection error interrupt function. */
#define SRAM_ECC_IT_DBE                 (0x01U << SRAM_ECCCR_ECCDBEIE_Pos)   /*!< Enable the interruption of a two-bit detection error when reading occur */
#define SRAM_ECC_IT_RBDBE               (0x01U << SRAM_ECCCR_ECCRBDBEIE_Pos) /*!< Enable the two-bit detection error interrupt in the read back */

/**
  * @}
  */

/** @defgroup SRAM_Exported_Functions
  * @{
  */
void SRAM_ReadLatencyConfig(uint8_t latency);
void SRAM_WriteWaitingConfig(uint8_t wait);

void SRAM_EccOneBitCorrectCmd(FunctionalState state);
void SRAM_EccErrorUpdateCmd(FunctionalState state);
void SRAM_EccErrorInjectConfig(uint64_t inject_msk);

void SRAM_EccITConfig(uint8_t it, FunctionalState state);
FlagStatus SRAM_GetEccITStatus(uint8_t it);
void SRAM_ClearEccITPendingBit(uint8_t it);

uint32_t SRAM_GetEccReadErrorAddress(void);
uint8_t SRAM_GetEccSyndromeCode(void);
uint32_t SRAM_GetReadErrorData(void);

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
