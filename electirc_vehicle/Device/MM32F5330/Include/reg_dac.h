/***********************************************************************************************************************
    @file     reg_dac.h
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

#ifndef __REG_DAC_H
#define __REG_DAC_H

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
  * @brief DAC Base Address Definition
  */
#define DAC_BASE                        (APB1PERIPH_BASE + 0x7400) /*!< Base Address: 0x40007400           */

/**
  * @brief Digital to analog converter register
  */
typedef struct
{
    __IO uint32_t CR;                  /*!< DAC control register,                                       offset: 0x00      */
    __IO uint32_t SWTRIGR;             /*!< DAC software trigger register,                              offset: 0x04      */
    __IO uint32_t DHR12R1;             /*!< DAC channel 1 12-bit right-aligned data holding register,   offset: 0x08      */
    __IO uint32_t DHR12L1;             /*!< DAC channel 1 12-bit left-aligned data holding register,    offset: 0x0C      */
    __IO uint32_t DHR8R1;              /*!< DAC channel 1 8-bit right-aligned data holding register,    offset: 0x10      */
    __IO uint16_t RESERVED0x14[6];     /*!< RESERVED                                                    offset: 0x14-0x28 */
    __IO uint32_t DOR1;                /*!< DAC channel 1 data output register,                         offset: 0x2C      */
} DAC_TypeDef;

/**
  * @brief DAC type pointer Definition
  */
#define DAC                             ((DAC_TypeDef *)DAC_BASE)

/**
  * @brief DAC_CR Register Bit Definition
  */
#define DAC_CR_EN1_Pos                  (0)
#define DAC_CR_EN1                      (0x01U << DAC_CR_EN1_Pos)    /*!< DAC Channel 1 Enable       */
#define DAC_CR_BOFF1_Pos                (1)
#define DAC_CR_BOFF1_Msk                (0x01U << DAC_CR_BOFF1_Pos)  /*!< DAC Channel 1 Output Buffer Disable        */
#define DAC_CR_TEN1_Pos                 (2)
#define DAC_CR_TEN1                     (0x01U << DAC_CR_TEN1_Pos)   /*!< DAC Channel 1 Trigger Enable       */
#define DAC_CR_WAVE1_Pos                (6)
#define DAC_CR_WAVE1_Msk                (0x03U << DAC_CR_WAVE1_Pos)  /*!< DAC Channel 1 Noise/Triangle Wave Generation Enable  */
#define DAC_CR_WAVE1_NONE               (0x00U << DAC_CR_WAVE1_Pos)  /*!< Wave generation disabled                             */
#define DAC_CR_WAVE1_NOISE              (0x01U << DAC_CR_WAVE1_Pos)  /*!< Noise wave generation enabled                        */
#define DAC_CR_WAVE1_TRIANGLE           (0x02U << DAC_CR_WAVE1_Pos)  /*!< Triangle wave generation enabled                     */
#define DAC_CR_MAMP1_Pos                (8)
#define DAC_CR_MAMP1_Msk                (0x0FU << DAC_CR_MAMP1_Pos)  /*!< DAC channel 1 Mask/Amplitude selector                       */
#define DAC_CR_MAMP1_1                  (0x00U << DAC_CR_MAMP1_Pos)  /*!< Unmask bit 0 of LSFR/triangle amplitude equal to 1          */
#define DAC_CR_MAMP1_3                  (0x01U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [1:0] of LSFR/triangle amplitude equal to 3     */
#define DAC_CR_MAMP1_7                  (0x02U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [2:0] of LSFR/triangle amplitude equal to 7     */
#define DAC_CR_MAMP1_15                 (0x03U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [3:0] of LSFR/triangle amplitude equal to 15    */
#define DAC_CR_MAMP1_31                 (0x04U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [4:0] of LSFR/triangle amplitude equal to 31    */
#define DAC_CR_MAMP1_63                 (0x05U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [5:0] of LSFR/triangle amplitude equal to 63    */
#define DAC_CR_MAMP1_127                (0x06U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [6:0] of LSFR/triangle amplitude equal to 127   */
#define DAC_CR_MAMP1_255                (0x07U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [7:0] of LSFR/triangle amplitude equal to 255   */
#define DAC_CR_MAMP1_511                (0x08U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [8:0] of LSFR/triangle amplitude equal to 511   */
#define DAC_CR_MAMP1_1023               (0x09U << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [9:0] of LSFR/triangle amplitude equal to 1023  */
#define DAC_CR_MAMP1_2047               (0x0AU << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [10:0] of LSFR/triangle amplitude equal to 2047 */
#define DAC_CR_MAMP1_4095               (0x0BU << DAC_CR_MAMP1_Pos)  /*!< Unmask bits [11:0] of LSFR/triangle amplitude equal to 4095 */
#define DAC_CR_DMAEN1_Pos               (12)
#define DAC_CR_DMAEN1                   (0x01U << DAC_CR_DMAEN1_Pos) /*!< DAC channel1 DMA enable        */
#define DAC_CR_OE_EN1_Pos               (13)
#define DAC_CR_OE_EN1                   (0x01U << DAC_CR_OE_EN1_Pos) /*!< DAC channel 1 output enable    */

/**
  * @brief DAC_SWTRIGR Register Bit Definition
  */
#define DAC_SWTRIGR_SW_TRIG1_Pos        (0)
#define DAC_SWTRIGR_SW_TRIG1            (0x01U << DAC_SWTRIGR_SW_TRIG1_Pos) /*!< DAC channel 1 software trigger     */

/**
  * @brief DAC_DHR12R1 Register Bit Definition
  */
#define DAC_DHR12R1_DACC1DHR_Pos        (0)
#define DAC_DHR12R1_DACC1DHR            (0xFFFU << DAC_DHR12R1_DACC1DHR_Pos) /*!< DAC Channel 1 12-bit Right Aligned Data        */

/**
  * @brief DAC_DHR12L1 Register Bit Definition
  */
#define DAC_DHR12L1_DACC1DHR_Pos        (4)
#define DAC_DHR12L1_DACC1DHR            (0xFFFU << DAC_DHR12L1_DACC1DHR_Pos) /*!< DAC Channel 1 12-bit Left Aligned Data     */

/**
  * @brief DAC_DHR8R1 Register Bit Definition
  */
#define DAC_DHR8R1_DACC1DHR_Pos         (0)
#define DAC_DHR8R1_DACC1DHR             (0xFFU << DAC_DHR8R1_DACC1DHR_Pos) /*!< DAC Channel 1 8-bit Right Aligned Data     */

/**
  * @brief DAC_DOR1 Register Bit Definition
  */
#define DAC_DOR1_DACC1DOR_Pos           (0)
#define DAC_DOR1_DACC1DOR_Msk           (0xFFFU << DAC_DOR1_DACC1DOR_Pos) /*!< DAC Channel 1 Data Output      */

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

////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////

