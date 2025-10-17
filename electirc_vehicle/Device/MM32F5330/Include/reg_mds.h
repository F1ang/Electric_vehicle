/***********************************************************************************************************************
    @file     reg_mds.h
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

#ifndef __REG_MDS_H
#define __REG_MDS_H

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
// anonymous unions are enabled by default
#else
#warning Not supported compiler type
#endif

/**
  * @brief MDS Base Address Definition
  */
#define MDS_BASE                        (APB2PERIPH_BASE + 0xFC00)
/**
  * @brief MDS Register Structure Definition
  */
typedef struct
{
    __IO uint32_t TRIGCR[8];           /*!< offset: 0x000 Trigger Channel x Control Register */
    __IO uint32_t RESERVED0x020[24];   /*!< offset: 0x020 Reserved */
    __IO uint32_t TRIGCLUSEL[4];       /*!< offset: 0x080 Trigger channel CLUx input select register */
    __IO uint32_t RESERVED0x090[12];   /*!< offset: 0x090 Reserved */
    __IO uint32_t TRIGCLUCFG[4];       /*!< offset: 0x0C0 Trigger channel CLUx input control register */
    __IO uint32_t RESERVED0x0D0[11];   /*!< offset: 0x0D0 Reserved */
    __IO uint32_t SWTRIG;              /*!< offset: 0x0FC Software Trigger Control Register */
    __IO uint32_t CONNCR[8];           /*!< offset: 0x100 Connect Channel x Control Register */
    __IO uint32_t RESERVED0x120[24];   /*!< offset: 0x120 Reserved */
    __IO uint32_t CONNCLUSEL[4];       /*!< offset: 0x180 Connection channel CLUx input control register */
    __IO uint32_t RESERVED0x190[12];   /*!< offset: 0x190 Reserved */
    __IO uint32_t CONNCLUCFG[4];       /*!< offset: 0x1C0 Connection Channel CLUx configuration register */
} MDS_TypeDef;

/**
  * @brief MDS type pointer Definition
  */
#define MDS                             ((MDS_TypeDef *)MDS_BASE)

/**
  * @addtogroup MDS_Register_Masks Register Masks
  * @{
  */

/**
  * @brief MDS_TRIGCR Register Bit Definition
  */
#define MDS_TRIGCR_TRGSEL_Pos           (0)
#define MDS_TRIGCR_TRGSEL_Msk           (0x3FU << MDS_TRIGCR_TRGSEL_Pos)
#define MDS_TRIGCR_CLUEN_Pos            (16)
#define MDS_TRIGCR_CLUEN_Msk            (0x01U << MDS_TRIGCR_CLUEN_Pos)
#define MDS_TRIGCR_CLUSEL_Pos           (17)
#define MDS_TRIGCR_CLUSEL_Msk           (0x03U << MDS_TRIGCR_CLUSEL_Pos)
#define MDS_TRIGCR_EDGESEL_Pos          (24)
#define MDS_TRIGCR_EDGESEL_Msk          (0x03U << MDS_TRIGCR_EDGESEL_Pos)

/**
  * @brief MDS_TRIG_CLUXSEL Register Bit Definition
  */
#define MDS_TRIGCLUSEL_CLUIN0SEL_Pos    (0)
#define MDS_TRIGCLUSEL_CLUIN0SEL_Msk    (0xFFU << MDS_TRIGCLUSEL_CLUIN0SEL_Pos)
#define MDS_TRIGCLUSEL_CLUIN1SEL_Pos    (8)
#define MDS_TRIGCLUSEL_CLUIN1SEL_Msk    (0xFFU << MDS_TRIGCLUSEL_CLUIN1SEL_Pos)
#define MDS_TRIGCLUSEL_CLUIN2SEL_Pos    (16)
#define MDS_TRIGCLUSEL_CLUIN2SEL_Msk    (0xFFU << MDS_TRIGCLUSEL_CLUIN2SEL_Pos)
#define MDS_TRIGCLUSEL_CLUIN3SEL_Pos    (24)
#define MDS_TRIGCLUSEL_CLUIN3SEL_Msk    (0xFFU << MDS_TRIGCLUSEL_CLUIN3SEL_Pos)

/**
  * @brief MDS_TRIG_CLUXCFG Register Bit Definition
  */
#define MDS_TRIGCLUCFG_CLUIN0ED_Pos     (0)
#define MDS_TRIGCLUCFG_CLUIN0ED_Msk     (0x03U << MDS_TRIGCLUCFG_CLUIN0ED_Pos)
#define MDS_TRIGCLUCFG_CLUIN1ED_Pos     (8)
#define MDS_TRIGCLUCFG_CLUIN1ED_Msk     (0x03U << MDS_TRIGCLUCFG_CLUIN1ED_Pos)
#define MDS_TRIGCLUCFG_CLUIN2ED_Pos     (16)
#define MDS_TRIGCLUCFG_CLUIN2ED_Msk     (0x03U << MDS_TRIGCLUCFG_CLUIN2ED_Pos)
#define MDS_TRIGCLUCFG_CLUIN3ED_Pos     (24)
#define MDS_TRIGCLUCFG_CLUIN3ED_Msk     (0x03U << MDS_TRIGCLUCFG_CLUIN3ED_Pos)

/**
  * @brief MDS_SWTRIG Register Bit Definition
  */
#define MDS_SWTRIG_SWTRIG_Pos           (0)
#define MDS_SWTRIG_SWTRIG_Msk           (0x01U << MDS_SWTRIG_SWTRIG_Pos)

/**
  * @brief MDS_CONNCR Register Bit Definition
  */
#define MDS_CONNCR_TRGSEL_Pos           (0)
#define MDS_CONNCR_TRGSEL_Msk           (0x3FU << MDS_CONNCR_TRGSEL_Pos)
#define MDS_CONNCR_CLUEN_Pos            (16)
#define MDS_CONNCR_CLUEN_Msk            (0x01U << MDS_CONNCR_CLUEN_Pos)
#define MDS_CONNCR_CLUSEL_Pos           (17)
#define MDS_CONNCR_CLUSEL_Msk           (0x03U << MDS_CONNCR_CLUSEL_Pos)

/**
  * @brief MDS_CONN_CLUXSEL Register Bit Definition
  */
#define MDS_CONNCLUSEL_CLUIN0SEL_Pos    (0)
#define MDS_CONNCLUSEL_CLUIN0SEL_Msk    (0xFFU << MDS_CONNCLUSEL_CLUIN0SEL_Pos)
#define MDS_CONNCLUSEL_CLUIN1SEL_Pos    (8)
#define MDS_CONNCLUSEL_CLUIN1SEL_Msk    (0xFFU << MDS_CONNCLUSEL_CLUIN1SEL_Pos)
#define MDS_CONNCLUSEL_CLUIN2SEL_Pos    (16)
#define MDS_CONNCLUSEL_CLUIN2SEL_Msk    (0xFFU << MDS_CONNCLUSEL_CLUIN2SEL_Pos)
#define MDS_CONNCLUSEL_CLUIN3SEL_Pos    (24)
#define MDS_CONNCLUSEL_CLUIN3SEL_Msk    (0xFFU << MDS_CONNCLUSEL_CLUIN3SEL_Pos)

/**
  * @brief MDS_CONN_CLUXCFG Register Bit Definition
  */
#define MDS_CONNCLUCFG_CLMODE_Pos       (0)
#define MDS_CONNCLUCFG_CLMODE_Msk       (0x03U << MDS_CONNCLUCFG_CLMODE_Pos)
#define MDS_CONNCLUCFG_INV0_Pos         (8)
#define MDS_CONNCLUCFG_INV0_Msk         (0x01U << MDS_CONNCLUCFG_INV0_Pos)
#define MDS_CONNCLUCFG_INV1_Pos         (9)
#define MDS_CONNCLUCFG_INV1_Msk         (0x01U << MDS_CONNCLUCFG_INV1_Pos)
#define MDS_CONNCLUCFG_INV2_Pos         (10)
#define MDS_CONNCLUCFG_INV2_Msk         (0x01U << MDS_CONNCLUCFG_INV2_Pos)
#define MDS_CONNCLUCFG_INV3_Pos         (11)
#define MDS_CONNCLUCFG_INV3_Msk         (0x01U << MDS_CONNCLUCFG_INV3_Pos)

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
#endif
/*----------------------------------------------------------------------------*/

