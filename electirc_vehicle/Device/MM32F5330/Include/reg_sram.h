/***********************************************************************************************************************
    @file     reg_sram.h
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

#ifndef __REG_SRAM_H
#define __REG_SRAM_H

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
/* anonymous unions are enabled by default -----------------------------------*/
#else
#warning Not supported compiler type
#endif

/**
  * @brief SRAM Base Address Definition
  */
#define SRAM_REG_BASE                   (AHB1PERIPH_BASE + 0xB000) /*!< Base Address: 0x4002B000 */
/**
  * @brief FLASH Registers Structure Definition
  */
typedef struct
{
    __IO uint32_t CFGR;                /*!< SRAM wait configuration register           offset: 0x00 */
    __IO uint32_t RESERVED0x04[15];    /*!< RESERVED                                   offset: 0x04-0x3C */
    __IO uint32_t ECCCR;               /*!< ECC control register                       offset: 0x40 */
    __IO uint32_t ECCEINR0;            /*!< ECC error was injected into register 0     offset: 0x44 */
    __IO uint32_t ECCEINR1;            /*!< ECC error was injected into register 1     offset: 0x48 */
    __IO uint32_t ECCSR;               /*!< ECC status register                        offset: 0x4C */
    __IO uint32_t ECCEADRR;            /*!< ECC error address register                 offset: 0x50 */
    __IO uint32_t RESERVED0x54[11];    /*!< RESERVED                                   offset: 0x54-0x7C */
    __IO uint32_t ECCESYNR;            /*!< ECC error check subregister                offset: 0x80 */
    __IO uint32_t ECCEDATAR;           /*!< ECC error data register                    offset: 0x84 */
} SRAM_TypeDef;

/**
  * @brief SRAM type pointer Definition
  */
#define SRAM                            ((SRAM_TypeDef *)SRAM_REG_BASE)

/**
  * @brief SRAM_CFGR Register Bit Definition
  */
#define SRAM_CFGR_RDLAT_Pos             (0)
#define SRAM_CFGR_RDLAT_Msk             (0x07U << SRAM_CFGR_RDLAT_Pos) /*!< SRAM read access delay. The ratio of HCLK period to SRAM write time. */
#define SRAM_CFGR_WRWT_Pos              (3)
#define SRAM_CFGR_WRWT_Msk              (0x07U << SRAM_CFGR_WRWT_Pos)  /*!< SRAM write wait time. The ratio of HCLK period to SRAM write time. */

/**
  * @brief SRAM_ECCCR Register Bit Definition
  */
#define SRAM_ECCCR_ECCSECEN_Pos         (0)
#define SRAM_ECCCR_ECCSECEN_Msk         (0x01U << SRAM_ECCCR_ECCSECEN_Pos)   /*!< Enable the ECC one-bit error correction function. */
#define SRAM_ECCCR_ECCEUPEN_Pos         (1)
#define SRAM_ECCCR_ECCEUPEN_Msk         (0x01U << SRAM_ECCCR_ECCEUPEN_Pos)   /*!< Enable the ECC error information update function. */
#define SRAM_ECCCR_ECCSNEIE_Pos         (2)
#define SRAM_ECCCR_ECCSNEIE_Msk         (0x01U << SRAM_ECCCR_ECCSNEIE_Pos)   /*!< Enable the one-bit detection error interrupt function. */
#define SRAM_ECCCR_ECCDBEIE_Pos         (3)
#define SRAM_ECCCR_ECCDBEIE_Msk         (0x01U << SRAM_ECCCR_ECCDBEIE_Pos)   /*!< Enable the interruption of a two-bit detection error when reading occurs. */
#define SRAM_ECCCR_ECCRBDBEIE_Pos       (4)
#define SRAM_ECCCR_ECCRBDBEIE_Msk       (0x01U << SRAM_ECCCR_ECCRBDBEIE_Pos) /*!< Enable the two-bit detection error interrupt in the read back. */

/**
  * @brief SRAM_ECCEINR0 Register Bit Definition
  */
#define SRAM_ECCEINR0_ECCEIENL_Pos      (0)
#define SRAM_ECCEINR0_ECCEIENL_Msk      (0xFFFFFFFFU << SRAM_ECCEINR0_ECCEIENL_Pos) /*!< ECC Error Read Data injection Enable (low bit) */

/**
  * @brief SRAM_ECCEINR1 Register Bit Definition
  */
#define SRAM_ECCEINR1_ECCEIENH_Pos      (0)
#define SRAM_ECCEINR1_ECCEIENH_Msk      (0x7FU << SRAM_ECCEINR1_ECCEIENH_Pos) /*!< ECC Error Read Data injection Enable (high bit) */

/**
  * @brief SRAM_ECCSR Register Bit Definition
  */
#define SRAM_ECCSR_ECCSNEF_Pos          (0)
#define SRAM_ECCSR_ECCSNEF_Msk          (0x01U << SRAM_ECCSR_ECCSNEF_Pos)   /*!< One bit detection error flag */
#define SRAM_ECCSR_ECCDBEF_Pos          (1)
#define SRAM_ECCSR_ECCDBEF_Msk          (0x01U << SRAM_ECCSR_ECCDBEF_Pos)   /*!< Two bit detection error flag */
#define SRAM_ECCSR_ECCRBDBEF_Pos        (2)
#define SRAM_ECCSR_ECCRBDBEF_Msk        (0x01U << SRAM_ECCSR_ECCRBDBEF_Pos) /*!< Read back detected a two-bit detection error */

/**
  * @brief SRAM_ECCEADRR Register Bit Definition
  */
#define SRAM_ECCEADRR_ECCEADR_Pos       (0)
#define SRAM_ECCEADRR_ECCEADR_Msk       (0xFFFFFFFFU << SRAM_ECCEADRR_ECCEADR_Pos) /*!< The ECC read an incorrect address */

/**
  * @brief SRAM_ECCESYNR Register Bit Definition
  */
#define SRAM_ECCESYNR_ECCESYN_Pos       (0)
#define SRAM_ECCESYNR_ECCESYN_Msk       (0x7FU << SRAM_ECCESYNR_ECCESYN_Pos) /*!< ECC Read error Syndrome Code */

/**
  * @brief SRAM_ECCEDATAR Register Bit Definition
  */
#define SRAM_ECCEDATAR_ECCEDATA_Pos     (0)
#define SRAM_ECCEDATAR_ECCEDATA_Msk     (0xFFFFFFFFU << SRAM_ECCEDATAR_ECCEDATA_Pos) /*!< ECC read error data */

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

/** --------------------------------------------------------------------------*/
#endif
/** --------------------------------------------------------------------------*/

