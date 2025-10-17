/***********************************************************************************************************************
    @file     reg_cordic.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE SERIES OF
              MM32 FIRMWARE LIBRARY.
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

#ifndef __REG_CORDIC_H
#define __REG_CORDIC_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__CC_ARM)
#pragma anon_unions
#elif defined(__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

/**
  * @brief CORDIC Base Address Definition
  */
#define CORDIC_BASE                     (AHB1PERIPH_BASE + 0xA000) /*!< Base Address: 0x4002A000 */

/**
  * @brief CORDIC Register Structure Definition
  */
typedef struct
{
    __IO uint32_t DXR;                 /*!< CORDIC X data register                         offset: 0x00 */
    __IO uint32_t DYR;                 /*!< CORDIC Y data register                         offset: 0x04 */
    __IO uint32_t DZR;                 /*!< CORDIC Z data register                         offset: 0x08 */
    __IO uint32_t RXR;                 /*!< CORDIC X result register                       offset: 0x0C */
    __IO uint32_t RYR;                 /*!< CORDIC Y result register                       offset: 0x10 */
    __IO uint32_t RZR;                 /*!< CORDIC Z result register                       offset: 0x14 */
    __IO uint32_t CR;                  /*!< CORDIC control register                        offset: 0x18 */
    __IO uint32_t SR;                  /*!< CORDIC status register                         offset: 0x1C */
} CORDIC_TypeDef;

/**
  * @brief CORDIC type pointer Definition
  */
#define CORDIC                          ((CORDIC_TypeDef *)CORDIC_BASE)

/**
  * @brief CORDIC_DXR Register Bit Definition
  */
#define CORDIC_DXR_DX_Pos               (0)
#define CORDIC_DXR_DX_Msk               (0x00FFFFFFU << CORDIC_DXR_DX_Pos) /*!< Initial X parameter data */

/**
  * @brief CORDIC_DYR Register Bit Definition
  */
#define CORDIC_DYR_DY_Pos               (0)
#define CORDIC_DYR_DY_Msk               (0x00FFFFFFU << CORDIC_DYR_DY_Pos) /*!< Initial Y parameter data */

/**
  * @brief CORDIC_DZR Register Bit Definition
  */
#define CORDIC_DZR_DZ_Pos               (0)
#define CORDIC_DZR_DZ_Msk               (0x00FFFFFFU << CORDIC_DZR_DZ_Pos) /*!< Initial Z parameter data */

/**
  * @brief CORDIC_RXR Register Bit Definition
  */
#define CORDIC_RXR_RX_Pos               (0)
#define CORDIC_RXR_RX_Msk               (0x01FFFFFFU << CORDIC_RXR_RX_Pos) /*!< X calculation result */

/**
  * @brief CORDIC_RYR Register Bit Definition
  */
#define CORDIC_RYR_RY_Pos               (0)
#define CORDIC_RYR_RY_Msk               (0x01FFFFFFU << CORDIC_RYR_RY_Pos) /*!< Y calculation result */

/**
  * @brief CORDIC_RZR Register Bit Definition
  */
#define CORDIC_RZR_RZ_Pos               (0)
#define CORDIC_RZR_RZ_Msk               (0x00FFFFFFU << CORDIC_RZR_RZ_Pos) /*!< Z calculation result */

/**
  * @brief CORDIC_CR Register Bit Definition
  */
#define CORDIC_CR_START_Pos             (0)
#define CORDIC_CR_START_Msk             (0x01U << CORDIC_CR_START_Pos)     /*!< set 1 to start calculation */
#define CORDIC_CR_WORKMODE_Pos          (1)
#define CORDIC_CR_WORKMODE_Msk          (0x01U << CORDIC_CR_WORKMODE_Pos)  /*!< Work mode selection for CORDIC calculation */
#define CORDIC_CR_STARTMODE_Pos         (2)
#define CORDIC_CR_STARTMODE_Msk         (0x01U << CORDIC_CR_STARTMODE_Pos) /*!< Start mode selection */
#define CORDIC_CR_ENABLE_Pos            (3)
#define CORDIC_CR_ENABLE_Msk            (0x01U << CORDIC_CR_ENABLE_Pos)    /*!< Module enable bit */

/**
  * @brief CORDIC_SR Register Bit Definition
  */
#define CORDIC_SR_BSY_Pos               (0)
#define CORDIC_SR_BSY_Msk               (0x01U << CORDIC_SR_BSY_Pos) /*!< CORDIC calculation Status bit */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif /* __REG_CORDIC_H ------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

