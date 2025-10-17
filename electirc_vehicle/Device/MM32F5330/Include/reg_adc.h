/***********************************************************************************************************************
    @file     reg_adc.h
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

#ifndef __REG_ADC_H
#define __REG_ADC_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

#if defined(__CC_ARM)
#pragma anon_unions
#endif

/**
  * @brief ADC Base Address Definition
  */
#define ADC1_BASE                       (APB2PERIPH_BASE + 0x2400) /*!< Base Address: 0x40012400 */
#define ADC2_BASE                       (APB2PERIPH_BASE + 0x2800) /*!< Base Address: 0x40012800 */

/**
  * @brief Analog-to-Digital Converter register
  */

typedef struct
{
    __IO uint32_t ADDATA;              /*!< ADC data register,                             offset: 0x00 */
    __IO uint32_t ADCFG;               /*!< ADC configuration register,                    offset: 0x04 */
    __IO uint32_t ADCR;                /*!< ADC control register,                          offset: 0x08 */
    __IO uint32_t ADCHS;               /*!< ADC channel selection register,                offset: 0x0C */
    __IO uint32_t ADCMPR;              /*!< ADC window compare register,                   offset: 0x10 */
    __IO uint32_t ADSTA;               /*!< ADC status register,                           offset: 0x14 */
    __IO uint32_t ADDR0;               /*!< ADC channel0 data register,                    offset: 0x18 */
    __IO uint32_t ADDR1;               /*!< ADC channel1 data register,                    offset: 0x1C */
    __IO uint32_t ADDR2;               /*!< ADC channel2 data register,                    offset: 0x20 */
    __IO uint32_t ADDR3;               /*!< ADC channel3 data register,                    offset: 0x24 */
    __IO uint32_t ADDR4;               /*!< ADC channel4 data register,                    offset: 0x28 */
    __IO uint32_t ADDR5;               /*!< ADC channel5 data register,                    offset: 0x2C */
    __IO uint32_t ADDR6;               /*!< ADC channel6 data register,                    offset: 0x30 */
    __IO uint32_t ADDR7;               /*!< ADC channel7 data register,                    offset: 0x34 */
    __IO uint32_t ADDR8;               /*!< ADC channel8 data register,                    offset: 0x38 */
    __IO uint32_t ADDR9;               /*!< ADC channel9 data register,                    offset: 0x3C */
    __IO uint32_t ADDR10;              /*!< ADC channel10 data register,                   offset: 0x40 */
    __IO uint32_t ADDR11;              /*!< ADC channel11 data register,                   offset: 0x44 */
    __IO uint32_t ADDR12;              /*!< ADC channel12 data register,                   offset: 0x48 */
    __IO uint32_t ADDR13;              /*!< ADC channel13 data register,                   offset: 0x4C */
    __IO uint32_t ADDR14;              /*!< ADC channel14 data register,                   offset: 0x50 */
    __IO uint32_t ADDR15;              /*!< ADC channel15 data register,                   offset: 0x54 */
    __IO uint32_t ADSTA_EXT;           /*!< ADC Extended Status Register,                  offset: 0x58 */
    __IO uint32_t CHANY0;              /*!< ADC any Chan Select Register 0,                offset: 0x5C */
    __IO uint32_t CHANY1;              /*!< ADC any Chan Select Register 1,                offset: 0x60 */
    __IO uint32_t ANY_CFG;             /*!< ADC any Chan config Register,                  offset: 0x64 */
    __IO uint32_t ANY_CR;              /*!< ADC any Chan control Register,                 offset: 0x68 */
    __IO uint32_t ADCFG2;              /*!< ADC configuration register 2,                  offset: 0x6C */
    __IO uint32_t SMPR1;               /*!< Sampling configuration register 1              offset: 0x70 */
    __IO uint32_t SMPR2;               /*!< Sampling configuration register 2              offset: 0x74 */
    __IO uint32_t SMPR3;               /*!< Sampling configuration register 3              offset: 0x78 */
    __IO uint32_t JOFR0;               /*!< Injection channel data compensation register 0 offset: 0x7C */
    __IO uint32_t JOFR1;               /*!< Injection channel data compensation register 1 offset: 0x80 */
    __IO uint32_t JOFR2;               /*!< Injection channel data compensation register 2 offset: 0x84 */
    __IO uint32_t JOFR3;               /*!< Injection channel data compensation register 3 offset: 0x88 */
    __IO uint32_t JSQR;                /*!< Injection sequence register                    offset: 0x8C */
    __IO uint32_t JADDATA;             /*!< Inject data register                           offset: 0x90 */
    __IO uint32_t RESERVED0x94[7];     /*!< RESERVED                                       offset: 0x94 - 0xAC */
    __IO uint32_t JDR0;                /*!< Injection channel data register 0              offset: 0xB0 */
    __IO uint32_t JDR1;                /*!< Injection channel data register 1              offset: 0xB4 */
    __IO uint32_t JDR2;                /*!< Injection channel data register 2              offset: 0xB8 */
    __IO uint32_t JDR3;                /*!< Injection channel data register 3              offset: 0xBC */
} ADC_TypeDef;

/**
  * @brief ADC type pointer Definition
  */
#define ADC1                             ((ADC_TypeDef *)ADC1_BASE)
#define ADC2                             ((ADC_TypeDef *)ADC2_BASE)

/**
  * @brief ADC_ADDATA Register Bit Definition
  */
#define ADC_ADDATA_DATA_Pos              (0)
#define ADC_ADDATA_DATA_Msk              (0xFFFFU << ADC_DR_DATA_Pos)          /*!< ADC 12bit convert data */

#define ADC_ADDATA_CHANNELSELL_Pos       (16)
#define ADC_ADDATA_CHANNELSELL_Msk       (0x0FU << ADC_ADDATA_CHANNELSELL_Pos) /*!< CHANNELSEL[19:16] (ADC current channel convert data) */

#define ADC_ADDATA_OVERRUN_Pos           (20)
#define ADC_ADDATA_OVERRUN_Msk           (0x01U << ADC_ADDATA_OVERRUN_Pos)     /*!< ADC data will be cover */
#define ADC_ADDATA_VALID_Pos             (21)
#define ADC_ADDATA_VALID_Msk             (0x01U << ADC_ADDATA_VALID_Pos)       /*!< ADC data[11:0] is valid */
#define ADC_ADDATA_CHANNELSELH_Pos       (22)
#define ADC_ADDATA_CHANNELSELH_Msk       (0x01U << ADC_ADDATA_CHANNELSELH_Pos) /*!< Channel selection  For High Bits */

/**
  * @brief ADC_ADCFG Register Bit Definition
  */
#define ADC_ADCFG_ADEN_Pos               (0)
#define ADC_ADCFG_ADEN_Msk               (0x01U << ADC_ADCFG_ADEN_Pos)    /*!< Enable ADC convert */
#define ADC_ADCFG_AWDEN_Pos              (1)
#define ADC_ADCFG_AWDEN_Msk              (0x01U << ADC_ADCFG_AWDEN_Pos)   /*!< Enable ADC window compare */

#define ADC_ADCFG_TSEN_Pos               (2)
#define ADC_ADCFG_TSEN_Msk               (0x01U << ADC_ADCFG_TSEN_Pos)    /*!< Enable ADC temperature sensor */
#define ADC_ADCFG_VSEN_Pos               (3)
#define ADC_ADCFG_VSEN_Msk               (0x01U << ADC_ADCFG_VSEN_Pos)    /*!< Enable ADC voltage reference */

#define ADC_ADCFG_RSLTCTL_Pos            (7)
#define ADC_ADCFG_RSLTCTL_Msk            (0x07U << ADC_ADCFG_RSLTCTL_Pos) /*!< ADC resolution select */

#define ADC_ADCFG_JAWDEN_Pos             (16)
#define ADC_ADCFG_JAWDEN_Msk             (0x01U << ADC_ADCFG_JAWDEN_Pos)  /*!< Inject ADC conversion window comparison enable */

/**
  * @brief ADC_ADCR Register Bit Definition
  */
#define ADC_ADCR_EOSIE_Pos               (0)
#define ADC_ADCR_EOSIE_Msk               (0x01U << ADC_ADCR_EOSIE_Pos)    /*!< ADC interrupt enable */
#define ADC_ADCR_AWDIE_Pos               (1)
#define ADC_ADCR_AWDIE_Msk               (0x01U << ADC_ADCR_AWDIE_Pos)    /*!< ADC window compare interrupt enable */
#define ADC_ADCR_TRGEN_Pos               (2)
#define ADC_ADCR_TRGEN_Msk               (0x01U << ADC_ADCR_TRGEN_Pos)    /*!< extranal trigger single start AD convert */
#define ADC_ADCR_DMAEN_Pos               (3)
#define ADC_ADCR_DMAEN_Msk               (0x01U << ADC_ADCR_DMAEN_Pos)    /*!< ADC DMA enable */

#define ADC_ADCR_ADST_Pos                (8)
#define ADC_ADCR_ADST_Msk                (0x01U << ADC_ADCR_ADST_Pos)     /*!< ADC start convert data */
#define ADC_ADCR_ADMD_Pos                (9)
#define ADC_ADCR_ADMD_Msk                (0x03U << ADC_ADCR_ADMD_Pos)     /*!< ADC convert mode */
#define ADC_ADCR_IMM                     (0x00U << ADC_ADCR_ADMD_Pos)     /*!< ADC imm convert mode */
#define ADC_ADCR_SCAN                    (0x01U << ADC_ADCR_ADMD_Pos)     /*!< ADC scan convert mode */
#define ADC_ADCR_CONTINUE                (0x02U << ADC_ADCR_ADMD_Pos)     /*!< ADC continue scan convert mode */
#define ADC_ADCR_ALIGN_Pos               (11)
#define ADC_ADCR_ALIGN_Msk               (0x01U << ADC_ADCR_ALIGN_Pos)    /*!< ADC data align */
#define ADC_ADCR_LEFT                    (0x01U << ADC_ADCR_ALIGN_Pos)    /*!< ADC data left align */
#define ADC_ADCR_RIGHT                   (0x00U << ADC_ADCR_ALIGN_Pos)    /*!< ADC data right align */
#define ADC_ADCR_CMPCHL_Pos              (12)
#define ADC_ADCR_CMPCHL_Msk              (0x0FU << ADC_ADCR_CMPCHL_Pos)   /*!< CMPCH[15:12] ADC window compare channel0 convert data */

#define ADC_ADCR_SCANDIR_Pos             (16)
#define ADC_ADCR_SCANDIR_Msk             (0x01U << ADC_ADCR_SCANDIR_Pos)  /*!< ADC scan direction */

#define ADC_ADCR_TRGSHIFT_Pos            (19)
#define ADC_ADCR_TRGSHIFT_Msk            (0x07U << ADC_ADCR_TRGSHIFT_Pos) /*!< External trigger shift sample */

#define ADC_ADCR_TRG_EDGE_Pos            (24)
#define ADC_ADCR_TRG_EDGE_Msk            (0x03U << ADC_ADCR_TRG_EDGE_Pos) /*!< ADC trig edge config */
#define ADC_ADCR_TRG_EDGE_UP             (0x02U << ADC_ADCR_TRG_EDGE_Pos) /*!< ADC up   edge trig mode */

#define ADC_ADCR_EOSMPIE_Pos             (26)
#define ADC_ADCR_EOSMPIE_Msk             (0X01U << ADC_ADCR_EOSMPIE_Pos)  /*!< ADC end sampling flag interrupt enable */
#define ADC_ADCR_EOCIE_Pos               (27)
#define ADC_ADCR_EOCIE_Msk               (0X01U << ADC_ADCR_EOCIE_Pos)    /*!< ADC end of conversion interrupt enable */

#define ADC_ADCR_CMPCHH_Pos              (28)
#define ADC_ADCR_CMPCHH_Msk              (0X01U << ADC_ADCR_CMPCHH_Pos)   /*!< Compare Channel Selection For Analog Watchdog For High Bits */

/**
  * @brief ADC_ADCHS Register Bit Definition
  */
#define ADC_ADCHS_CHEN0_Pos              (0)
#define ADC_ADCHS_CHEN0_Msk              (0x01U << ADC_ADCHS_CHEN0_Pos)  /*!< Enable ADC channel 0 */
#define ADC_ADCHS_CHEN1_Pos              (1)
#define ADC_ADCHS_CHEN1_Msk              (0x01U << ADC_ADCHS_CHEN1_Pos)  /*!< Enable ADC channel 1 */
#define ADC_ADCHS_CHEN2_Pos              (2)
#define ADC_ADCHS_CHEN2_Msk              (0x01U << ADC_ADCHS_CHEN2_Pos)  /*!< Enable ADC channel 2 */
#define ADC_ADCHS_CHEN3_Pos              (3)
#define ADC_ADCHS_CHEN3_Msk              (0x01U << ADC_ADCHS_CHEN3_Pos)  /*!< Enable ADC channel 3 */
#define ADC_ADCHS_CHEN4_Pos              (4)
#define ADC_ADCHS_CHEN4_Msk              (0x01U << ADC_ADCHS_CHEN4_Pos)  /*!< Enable ADC channel 4 */
#define ADC_ADCHS_CHEN5_Pos              (5)
#define ADC_ADCHS_CHEN5_Msk              (0x01U << ADC_ADCHS_CHEN5_Pos)  /*!< Enable ADC channel 5 */
#define ADC_ADCHS_CHEN6_Pos              (6)
#define ADC_ADCHS_CHEN6_Msk              (0x01U << ADC_ADCHS_CHEN6_Pos)  /*!< Enable ADC channel 6 */
#define ADC_ADCHS_CHEN7_Pos              (7)
#define ADC_ADCHS_CHEN7_Msk              (0x01U << ADC_ADCHS_CHEN7_Pos)  /*!< Enable ADC channel 7 */
#define ADC_ADCHS_CHEN8_Pos              (8)
#define ADC_ADCHS_CHEN8_Msk              (0x01U << ADC_ADCHS_CHEN8_Pos)  /*!< Enable ADC channel 8 */
#define ADC_ADCHS_CHEN9_Pos              (9)
#define ADC_ADCHS_CHEN9_Msk              (0x01U << ADC_ADCHS_CHEN9_Pos)  /*!< Enable ADC channel 9 */
#define ADC_ADCHS_CHEN10_Pos             (10)
#define ADC_ADCHS_CHEN10_Msk             (0x01U << ADC_ADCHS_CHEN10_Pos) /*!< Enable ADC channel 10 */
#define ADC_ADCHS_CHEN11_Pos             (11)
#define ADC_ADCHS_CHEN11_Msk             (0x01U << ADC_ADCHS_CHEN11_Pos) /*!< Enable ADC channel 11 */
#define ADC_ADCHS_CHEN12_Pos             (12)
#define ADC_ADCHS_CHEN12_Msk             (0x01U << ADC_ADCHS_CHEN12_Pos) /*!< Enable ADC channel 12 */
#define ADC_ADCHS_CHEN13_Pos             (13)
#define ADC_ADCHS_CHEN13_Msk             (0x01U << ADC_ADCHS_CHEN13_Pos) /*!< Enable ADC channel 13 */
#define ADC_ADCHS_CHEN14_Pos             (14)
#define ADC_ADCHS_CHEN14_Msk             (0x01U << ADC_ADCHS_CHEN14_Pos) /*!< Enable ADC channel 14 */
#define ADC_ADCHS_CHEN15_Pos             (15)
#define ADC_ADCHS_CHEN15_Msk             (0x01U << ADC_ADCHS_CHEN15_Pos) /*!< Enable ADC channel 15 */

/**
  * @brief ADC_ADCMPR Register Bit Definition
  */
#define ADC_ADCMPR_CMPLDATA_Pos          (0)
#define ADC_ADCMPR_CMPLDATA_Msk          (0x0FFFU << ADC_ADCMPR_CMPLDATA_Pos) /*!< ADC 12bit window compare DOWN LEVEL DATA */
#define ADC_ADCMPR_CMPHDATA_Pos          (16)
#define ADC_ADCMPR_CMPHDATA_Msk          (0x0FFFU << ADC_ADCMPR_CMPHDATA_Pos) /*!< ADC 12bit window compare UP LEVEL DATA */

/**
  * @brief ADC_ADSTA Register Bit Definition
  */
#define ADC_ADSTA_EOSIF_Pos              (0)
#define ADC_ADSTA_EOSIF_Msk              (0x01U << ADC_ADSTA_EOSIF_Pos)     /*!< ADC convert complete flag */
#define ADC_ADSTA_AWDIF_Pos              (1)
#define ADC_ADSTA_AWDIF_Msk              (0x01U << ADC_ADSTA_AWDIF_Pos)     /*!< ADC compare flag */
#define ADC_ADSTA_BUSY_Pos               (2)
#define ADC_ADSTA_BUSY_Msk               (0x01U << ADC_ADSTA_BUSY_Pos)      /*!< ADC busy flag */
#define ADC_ADSTA_CHANNELH_Pos           (3)
#define ADC_ADSTA_CHANNELH_Msk           (0x01U << ADC_ADSTA_CHANNELH_Pos)  /*!< Current Convert Channel For High Bits */
#define ADC_ADSTA_CHANNELL_Pos           (4)
#define ADC_ADSTA_CHANNELL_Msk           (0x0FU << ADC_ADSTA_CHANNELL_Pos)  /*!< CHANNEL[7:4] ADC current channel */

#define ADC_ADSTA_VALID_Pos              (8)
#define ADC_ADSTA_VALID_Msk              (0x0FFFU << ADC_ADSTA_VALID_Pos)   /*!< VALID[19:8] ADC channel 0..11 valid flag */
#define ADC_ADSTA_OVERRUN_Pos            (20)
#define ADC_ADSTA_OVERRUN_Msk            (0x0FFFU << ADC_ADSTA_OVERRUN_Pos) /*!< OVERRUN[31:20] ADC channel 0..11 data covered flag */

/**
  * @brief ADC_ADDRn Register Bit Definition
  */
#define ADC_ADDR_DATA_Pos                (0)
#define ADC_ADDR_DATA_Msk                (0xFFFFU << ADC_ADDR_DATA_Pos)  /*!< ADC channel convert data */
#define ADC_ADDR_OVERRUN_Pos             (20)
#define ADC_ADDR_OVERRUN_Msk             (0x01U << ADC_ADDR_OVERRUN_Pos) /*!< ADC data covered flag */
#define ADC_ADDR_VALID_Pos               (21)
#define ADC_ADDR_VALID_Msk               (0x01U << ADC_ADDR_VALID_Pos)   /*!< ADC data valid flag */

/**
  * @brief ADC_ADSTA_EXT Register Bit Definition
  */
#define ADC_ADSTA_EXT_VALID_Pos          (0)
#define ADC_ADSTA_EXT_VALID_Msk          (0x0FU << ADC_ADSTA_EXT_VALID_Pos)    /*!< VALID[3:0] ADC channel 12,14..15 valid flag */
#define ADC_ADSTA_EXT_OVERRUN_Pos        (4)
#define ADC_ADSTA_EXT_OVERRUN_Msk        (0x0FU << ADC_ADSTA_EXT_OVERRUN_Pos)  /*!< OVERRUN[7:4] ADC channel 12,14..15 data covered flag */

#define ADC_ADSTA_EXT_EOSMPIF_Pos        (16)
#define ADC_ADSTA_EXT_EOSMPIF_Msk        (0x01U << ADC_ADSTA_EXT_EOSMPIF_Pos)  /*!< End of sampling interrupt flag */
#define ADC_ADSTA_EXT_EOCIF_Pos          (17)
#define ADC_ADSTA_EXT_EOCIF_Msk          (0x01U << ADC_ADSTA_EXT_EOCIF_Pos)    /*!< End of conversion interrupt flag */
#define ADC_ADSTA_EXT_JEOSMPIF_Pos       (18)
#define ADC_ADSTA_EXT_JEOSMPIF_Msk       (0x01U << ADC_ADSTA_EXT_JEOSMPIF_Pos) /*! Injected channel end of sampling interrupt flag */
#define ADC_ADSTA_EXT_JEOCIF_Pos         (19)
#define ADC_ADSTA_EXT_JEOCIF_Msk         (0x01U << ADC_ADSTA_EXT_JEOCIF_Pos)   /*!< Injected channel end of conversion interrupt flag */
#define ADC_ADSTA_EXT_JEOSIF_Pos         (20)
#define ADC_ADSTA_EXT_JEOSIF_Msk         (0x01U << ADC_ADSTA_EXT_JEOSIF_Pos)   /*!< Injected channel end of sequential conversion interrupt flag */
#define ADC_ADSTA_EXT_JBUSY_Pos          (21)
#define ADC_ADSTA_EXT_JBUSY_Msk          (0x01U << ADC_ADSTA_EXT_JBUSY_Pos)    /*!< Injection mode busy/idle */
#define ADC_ADSTA_EXT_EOCALIF_Pos        (24)
#define ADC_ADSTA_EXT_EOCALIF_Msk        (0x01U << ADC_ADSTA_EXT_EOCALIF_Pos)  /*!< End of calibration interrupt flag */
#define ADC_ADSTA_EXT_CALBUSY_Pos        (25)
#define ADC_ADSTA_EXT_CALBUSY_Msk        (0x01U << ADC_ADSTA_EXT_CALBUSY_Pos)  /*!< Busy */
#define ADC_ADSTA_EXT_FREOCIF_Pos        (26)
#define ADC_ADSTA_EXT_FREOCIF_Msk        (0x01U << ADC_ADSTA_EXT_FREOCIF_Pos)  /*!< Calibration factor reading and writing data register end flag */

/**
  * @brief ADC_CHANY0 select Register Bit Definition
  */
#define ADC_CHANY0_SELL0_Pos             (0)                                   /*!< CHANY_SEL0 (Bit 0) */
#define ADC_CHANY0_SELL0_Msk             (0x0FU << ADC_CHANY0_SELL0_Pos)       /*!< CHANY_SEL0 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL1_Pos             (4)                                   /*!< CHANY_SEL1 (Bit 4) */
#define ADC_CHANY0_SELL1_Msk             (0x0FU << ADC_CHANY0_SELL1_Pos)       /*!< CHANY_SEL1 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL2_Pos             (8)                                   /*!< CHANY_SEL2 (Bit 8) */
#define ADC_CHANY0_SELL2_Msk             (0x0FU << ADC_CHANY0_SELL2_Pos)       /*!< CHANY_SEL2 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL3_Pos             (12)                                  /*!< CHANY_SEL3 (Bit 12) */
#define ADC_CHANY0_SELL3_Msk             (0x0FU << ADC_CHANY0_SELL3_Pos)       /*!< CHANY_SEL3 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL4_Pos             (16)                                  /*!< CHANY_SEL4 (Bit 16) */
#define ADC_CHANY0_SELL4_Msk             (0x0FU << ADC_CHANY0_SELL4_Pos)       /*!< CHANY_SEL4 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL5_Pos             (20)                                  /*!< CHANY_SEL5 (Bit 20) */
#define ADC_CHANY0_SELL5_Msk             (0x0FU << ADC_CHANY0_SELL5_Pos)       /*!< CHANY_SEL5 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL6_Pos             (24)                                  /*!< CHANY_SEL6 (Bit 24) */
#define ADC_CHANY0_SELL6_Msk             (0x0FU << ADC_CHANY0_SELL6_Pos)       /*!< CHANY_SEL6 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY0_SELL7_Pos             (28)                                  /*!< CHANY_SEL7 (Bit 28) */
#define ADC_CHANY0_SELL7_Msk             (0x0FU << ADC_CHANY0_SELL7_Pos)       /*!< CHANY_SEL7 (Bitfield-Mask: 0x0f) */

/**
  * @brief ADC_CHANY1 select Register Bit Definition
  */
#define ADC_CHANY1_SELL8_Pos             (0)                                   /*!< CHANY_SEL8 (Bit 0) */
#define ADC_CHANY1_SELL8_Msk             (0x0FU << ADC_CHANY1_SELL8_Pos)       /*!< CHANY_SEL8 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL9_Pos             (4)                                   /*!< CHANY_SEL9 (Bit 4) */
#define ADC_CHANY1_SELL9_Msk             (0x0FU << ADC_CHANY1_SELL9_Pos)       /*!< CHANY_SEL9 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL10_Pos            (8)                                   /*!< CHANY_SEL10 (Bit 8) */
#define ADC_CHANY1_SELL10_Msk            (0x0FU << ADC_CHANY1_SELL10_Pos)      /*!< CHANY_SEL10 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL11_Pos            (12)                                  /*!< CHANY_SEL11 (Bit 12) */
#define ADC_CHANY1_SELL11_Msk            (0x0FU << ADC_CHANY1_SELL11_Pos)      /*!< CHANY_SEL11 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL12_Pos            (16)                                  /*!< CHANY_SEL12 (Bit 16) */
#define ADC_CHANY1_SELL12_Msk            (0x0FU << ADC_CHANY1_SELL12_Pos)      /*!< CHANY_SEL12 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL13_Pos            (20)                                  /*!< CHANY_SEL13 (Bit 20) */
#define ADC_CHANY1_SELL13_Msk            (0x0FU << ADC_CHANY1_SELL13_Pos)      /*!< CHANY_SEL13 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL14_Pos            (24)                                  /*!< CHANY_SEL14 (Bit 24) */
#define ADC_CHANY1_SELL14_Msk            (0x0FU << ADC_CHANY1_SELL14_Pos)      /*!< CHANY_SEL14 (Bitfield-Mask: 0x0f) */
#define ADC_CHANY1_SELL15_Pos            (28)                                  /*!< CHANY_SEL15 (Bit 28) */
#define ADC_CHANY1_SELL15_Msk            (0x0FU << ADC_CHANY1_SELL15_Pos)      /*!< CHANY_SEL15 (Bitfield-Mask: 0x0f) */

/**
  * @brief ADC_ANY_CFG config number Register Bit Definition
  */
#define ADC_ANY_CFG_CHANY_NUM_Pos        (0)
#define ADC_ANY_CFG_CHANY_NUM_Msk        (0x0FU << ADC_ANY_CFG_CHANY_NUM_Pos)    /*!< Number of Any Channel Mode */
#define ADC_ANY_CFG_CHANY_SELH0_Pos      (8)
#define ADC_ANY_CFG_CHANY_SELH0_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH0_Pos)  /*!< 0th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH1_Pos      (9)
#define ADC_ANY_CFG_CHANY_SELH1_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH1_Pos)  /*!< 1th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH2_Pos      (10)
#define ADC_ANY_CFG_CHANY_SELH2_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH2_Pos)  /*!< 2th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH3_Pos      (11)
#define ADC_ANY_CFG_CHANY_SELH3_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH3_Pos)  /*!< 3th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH4_Pos      (12)
#define ADC_ANY_CFG_CHANY_SELH4_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH4_Pos)  /*!< 4th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH5_Pos      (13)
#define ADC_ANY_CFG_CHANY_SELH5_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH5_Pos)  /*!< 5th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH6_Pos      (14)
#define ADC_ANY_CFG_CHANY_SELH6_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH6_Pos)  /*!< 6th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH7_Pos      (15)
#define ADC_ANY_CFG_CHANY_SELH7_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH7_Pos)  /*!< 7th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH8_Pos      (16)
#define ADC_ANY_CFG_CHANY_SELH8_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH8_Pos)  /*!< 8th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH9_Pos      (17)
#define ADC_ANY_CFG_CHANY_SELH9_Msk      (0x01U << ADC_ANY_CFG_CHANY_SELH9_Pos)  /*!< 9th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH10_Pos     (18)
#define ADC_ANY_CFG_CHANY_SELH10_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH10_Pos) /*!< 10th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH11_Pos     (19)
#define ADC_ANY_CFG_CHANY_SELH11_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH11_Pos) /*!< 11th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH12_Pos     (20)
#define ADC_ANY_CFG_CHANY_SELH12_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH12_Pos) /*!< 12th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH13_Pos     (21)
#define ADC_ANY_CFG_CHANY_SELH13_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH13_Pos) /*!< 13th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH14_Pos     (22)
#define ADC_ANY_CFG_CHANY_SELH14_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH14_Pos) /*!< 14th Conversion Select for Any Channel sequence For High Bits */
#define ADC_ANY_CFG_CHANY_SELH15_Pos     (23)
#define ADC_ANY_CFG_CHANY_SELH15_Msk     (0x01U << ADC_ANY_CFG_CHANY_SELH15_Pos) /*!< 15th Conversion Select for Any Channel sequence For High Bits */

/**
  * @brief ADC_ANY_CR mode enable Register Bit Definition
  */
#define ADC_ANY_CR_CHANY_MDEN_Pos        (0)                                     /*!< CHANY_MDEN (Bit 0) */
#define ADC_ANY_CR_CHANY_MDEN_Msk        (0x01U << ADC_ANY_CR_CHANY_MDEN_Pos)    /*!< CHANY_MDEN (Bitfield-Mask: 0x01) */
#define ADC_ANY_CR_JCEN_Pos              (1)
#define ADC_ANY_CR_JCEN_Msk              (0x01U << ADC_ANY_CR_JCEN_Pos)          /*!< Injected channel enable */
#define ADC_ANY_CR_JEOSMPIE_Pos          (2)
#define ADC_ANY_CR_JEOSMPIE_Msk          (0x01U << ADC_ANY_CR_JEOSMPIE_Pos)      /*!< Interrupt enable the end of sequence conversion for injected channel */
#define ADC_ANY_CR_JEOCIE_Pos            (3)
#define ADC_ANY_CR_JEOCIE_Msk            (0x01U << ADC_ANY_CR_JEOCIE_Pos)        /*!< Interrupt enable the end of conversion for injected channel */
#define ADC_ANY_CR_JEOSIE_Pos            (4)
#define ADC_ANY_CR_JEOSIE_Msk            (0x01U << ADC_ANY_CR_JEOSIE_Pos)        /*!< Interrupt enable the end of sequence conversion for injected channel */
#define ADC_ANY_CR_JAUTO_Pos             (5)
#define ADC_ANY_CR_JAUTO_Msk             (0x01U << ADC_ANY_CR_JAUTO_Pos)         /*!<Automatic Injected group conversion */
#define ADC_ANY_CR_JADST_Pos             (6)
#define ADC_ANY_CR_JADST_Msk             (0x01U << ADC_ANY_CR_JADST_Pos)         /*!< Start conversion of injected channels */
#define ADC_ANY_CR_JTRGEN_Pos            (7)
#define ADC_ANY_CR_JTRGEN_Msk            (0x01U << ADC_ANY_CR_JTRGEN_Pos)        /*!< External trigger conversion mode for injected channels */

#define ADC_ANY_CR_JTRGSHIFT_Pos         (13)                                    /*!< Injection mode external trigger delay sampling */
#define ADC_ANY_CR_JTRGSHIFT_Msk         (0x07U << ADC_ANY_CR_JTRGSHIFT_Pos)

#define ADC_ANY_CR_JTRGEDGE_Pos          (16)                                    /*!< Injection mode triggers edge selection */
#define ADC_ANY_CR_JTRGEDGE_Msk          (0x03U << ADC_ANY_CR_JTRGEDGE_Pos)
#define ADC_ANY_CR_JTRGEDGE_R            (0x02U << ADC_ANY_CR_JTRGEDGE_Pos)      /*!< Rising edge trigger */

#define ADC_ANY_CR_ADCAL_Pos             (20)
#define ADC_ANY_CR_ADCAL_Msk             (0x01U << ADC_ANY_CR_ADCAL_Pos)         /*!< A/D Calibration */
#define ADC_ANY_CR_EOCALIE_Pos           (21)
#define ADC_ANY_CR_EOCALIE_Msk           (0x01U << ADC_ANY_CR_EOCALIE_Pos)       /*!< End of calibration interrupt enable */

/**
  * @brief ADC_ADCFG2 Register Bit Definition
  */
#define ADC_ADCFG2_CORREN_Pos            (0)
#define ADC_ADCFG2_CORREN_Msk            (0x01U << ADC_ADCFG2_CORREN_Pos)  /*!< Correct Enable */
#define ADC_ADCFG2_ADCCR_Pos             (1)
#define ADC_ADCFG2_ADCCR_Msk             (0x01U << ADC_ADCFG2_ADCCR_Pos)   /*!< ADC Control */

#define ADC_ADCFG2_ADCSREF_Pos           (2)
#define ADC_ADCFG2_ADCSREF_Msk           (0x01U << ADC_ADCFG2_ADCSREF_Pos) /*!< ADC Select Reference Voltage */

#define ADC_ADCFG2_DC_Pos                (4)
#define ADC_ADCFG2_DC_Msk                (0x1FU << ADC_ADCFG2_DC_Pos)      /*!< Differential Conversion */
#define ADC_ADCFG2_DC_0_1                (0x01U << ADC_ADCFG2_DC_Pos)      /*!< Channels 0 and 1 are a group of differential channels */
#define ADC_ADCFG2_DC_2_3                (0x02U << ADC_ADCFG2_DC_Pos)      /*!< Channels 2 and 3 are a group of differential channels */
#define ADC_ADCFG2_DC_4_5                (0x04U << ADC_ADCFG2_DC_Pos)      /*!< Channels 4 and 5 are a group of differential channels */
#define ADC_ADCFG2_DC_6_7                (0x08U << ADC_ADCFG2_DC_Pos)      /*!< Channels 6 and 7 are a group of differential channels */

#define ADC_ADCFG2_PSDC_Pos              (10)
#define ADC_ADCFG2_PSDC_Msk              (0x1FU << ADC_ADCFG2_PSDC_Pos)    /*!< Pseudo-differential Conversion */
#define ADC_ADCFG2_PSDC_0_1              (0x01U << ADC_ADCFG2_PSDC_Pos)    /*!< Channels 0 and 1 are a group of pseudo-differential channels */
#define ADC_ADCFG2_PSDC_2_3              (0x02U << ADC_ADCFG2_PSDC_Pos)    /*!< Channels 2 and 3 are a group of pseudo-differential channels */
#define ADC_ADCFG2_PSDC_4_5              (0x04U << ADC_ADCFG2_PSDC_Pos)    /*!< Channels 4 and 5 are a group of pseudo-differential channels */
#define ADC_ADCFG2_PSDC_6_7              (0x08U << ADC_ADCFG2_PSDC_Pos)    /*!< Channels 6 and 7 are a group of pseudo-differential channels */

#define ADC_ADCFG2_ROVSE_Pos             (16)
#define ADC_ADCFG2_ROVSE_Msk             (0x01U << ADC_ADCFG2_ROVSE_Pos)   /*!< Regular Oversampler Enable */

#define ADC_ADCFG2_JOVSE_Pos             (17)
#define ADC_ADCFG2_JOVSE_Msk             (0x01U << ADC_ADCFG2_JOVSE_Pos)   /*!< Injected Oversampler Enable */

#define ADC_ADCFG2_OVSR_Pos              (18)
#define ADC_ADCFG2_OVSR_Msk              (0x07U << ADC_ADCFG2_OVSR_Pos)    /*!< Oversampling Ratio */

#define ADC_ADCFG2_OVSS_Pos              (21)
#define ADC_ADCFG2_OVSS_Msk              (0x0FU << ADC_ADCFG2_OVSS_Pos)    /*!< Oversampling Shift Bits */

#define ADC_ADCFG2_TROVS_Pos             (25)
#define ADC_ADCFG2_TROVS_Msk             (0x01U << ADC_ADCFG2_TROVS_Pos)   /*!< Trigger Oversampling Select */

/**
  * @brief ADC_SMPR1 mode enable Register Bit Definition
  */
#define ADC_SMPR_SAMCTL_Pos              (0)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR_SAMCTL_Msk              (0x0FU << ADC_SMPR_SAMCTL_Pos)    /*!< Injection mode external trigger delay sampling mask for Value */
#define ADC_SMPR_SAMCTL_3_5              (0x00U << ADC_SMPR_SAMCTL_Pos)    /*!< 3.5    cycle */
#define ADC_SMPR_SAMCTL_4_5              (0x01U << ADC_SMPR_SAMCTL_Pos)    /*!< 4.5    cycle */
#define ADC_SMPR_SAMCTL_5_5              (0x02U << ADC_SMPR_SAMCTL_Pos)    /*!< 5.5   cycle */
#define ADC_SMPR_SAMCTL_6_5              (0x03U << ADC_SMPR_SAMCTL_Pos)    /*!< 6.5   cycle */
#define ADC_SMPR_SAMCTL_7_5              (0x04U << ADC_SMPR_SAMCTL_Pos)    /*!< 7.5   cycle */
#define ADC_SMPR_SAMCTL_11_5             (0x05U << ADC_SMPR_SAMCTL_Pos)    /*!< 11.5   cycle */
#define ADC_SMPR_SAMCTL_13_5             (0x06U << ADC_SMPR_SAMCTL_Pos)    /*!< 13.5   cycle */
#define ADC_SMPR_SAMCTL_15_5             (0x07U << ADC_SMPR_SAMCTL_Pos)    /*!< 15.5   cycle */
#define ADC_SMPR_SAMCTL_19_5             (0x08U << ADC_SMPR_SAMCTL_Pos)    /*!< 19.5    cycle */
#define ADC_SMPR_SAMCTL_29_5             (0x09U << ADC_SMPR_SAMCTL_Pos)    /*!< 29.5    cycle */
#define ADC_SMPR_SAMCTL_39_5             (0x0AU << ADC_SMPR_SAMCTL_Pos)    /*!< 39.5    cycle */
#define ADC_SMPR_SAMCTL_59_5             (0x0BU << ADC_SMPR_SAMCTL_Pos)    /*!< 59.5    cycle */
#define ADC_SMPR_SAMCTL_79_5             (0x0CU << ADC_SMPR_SAMCTL_Pos)    /*!< 79.5    cycle */
#define ADC_SMPR_SAMCTL_119_5            (0x0DU << ADC_SMPR_SAMCTL_Pos)    /*!< 119.5    cycle */
#define ADC_SMPR_SAMCTL_159_5            (0x0EU << ADC_SMPR_SAMCTL_Pos)    /*!< 159.5    cycle */
#define ADC_SMPR_SAMCTL_240_5            (0x0FU << ADC_SMPR_SAMCTL_Pos)    /*!< 240.5    cycle */

#define ADC_SMPR1_SAMCTL7_Pos            (28)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL7_Msk            (0x0FU << ADC_SMPR1_SAMCTL7_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL6_Pos            (24)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL6_Msk            (0x0FU << ADC_SMPR1_SAMCTL6_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL5_Pos            (20)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL5_Msk            (0x0FU << ADC_SMPR1_SAMCTL5_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL4_Pos            (16)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL4_Msk            (0x0FU << ADC_SMPR1_SAMCTL4_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL3_Pos            (12)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL3_Msk            (0x0FU << ADC_SMPR1_SAMCTL3_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL2_Pos            (8)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL2_Msk            (0x0FU << ADC_SMPR1_SAMCTL2_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL1_Pos            (4)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL1_Msk            (0x0FU << ADC_SMPR1_SAMCTL1_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR1_SAMCTL0_Pos            (0)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR1_SAMCTL0_Msk            (0x0FU << ADC_SMPR1_SAMCTL0_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

/**
  * @brief ADC_SMPR2 mode enable Register Bit Definition
  */
#define ADC_SMPR2_SAMCTL15_Pos           (28)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL15_Msk           (0x0FU << ADC_SMPR2_SAMCTL15_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL15_240_5         (0x0FU << ADC_SMPR2_SAMCTL15_Pos) /*!< 240.5    cycle */
#define ADC_SMPR2_SAMCTL14_Pos           (24)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL14_Msk           (0x0FU << ADC_SMPR2_SAMCTL14_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL13_Pos           (20)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL13_Msk           (0x0FU << ADC_SMPR2_SAMCTL13_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL12_Pos           (16)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL12_Msk           (0x0FU << ADC_SMPR2_SAMCTL12_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL11_Pos           (12)                              /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL11_Msk           (0x0FU << ADC_SMPR2_SAMCTL11_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL10_Pos           (8)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL10_Msk           (0x0FU << ADC_SMPR2_SAMCTL10_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL9_Pos            (4)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL9_Msk            (0x0FU << ADC_SMPR2_SAMCTL9_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR2_SAMCTL8_Pos            (0)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR2_SAMCTL8_Msk            (0x0FU << ADC_SMPR2_SAMCTL8_Pos)  /*!< Injection mode external trigger delay sampling mask for Value */

/**
  * @brief ADC_SMPR3 mode enable Register Bit Definition
  */
#define ADC_SMPR3_SAMCTL18_Pos           (8)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR3_SAMCTL18_Msk           (0x0FU << ADC_SMPR3_SAMCTL18_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR3_SAMCTL17_Pos           (4)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR3_SAMCTL17_Msk           (0x0FU << ADC_SMPR3_SAMCTL17_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

#define ADC_SMPR3_SAMCTL16_Pos           (0)                               /*!< Injection mode external trigger delay sampling off_set Position */
#define ADC_SMPR3_SAMCTL16_Msk           (0x0FU << ADC_SMPR3_SAMCTL16_Pos) /*!< Injection mode external trigger delay sampling mask for Value */

/**
  * @brief ADC_JOFR0 mode enable Register Bit Definition
  */
#define ADC_JOFR0_JOFFSET_Pos            (0)
#define ADC_JOFR0_JOFFSET_Msk            (0x0FFFU << ADC_JOFR0_JOFFSET_Pos) /*!< Compensates for the A/D conversion results of the injected channel 0 */

/**
  * @brief ADC_JOFR1 mode enable Register Bit Definition
  */
#define ADC_JOFR1_JOFFSET_Pos            (0)
#define ADC_JOFR1_JOFFSET_Msk            (0x0FFFU << ADC_JOFR1_JOFFSET_Pos) /*!< Compensates for the A/D conversion results of the injected channel 1 */
/**
  * @brief ADC_JOFR2 mode enable Register Bit Definition
  */
#define ADC_JOFR2_JOFFSET_Pos            (0)
#define ADC_JOFR2_JOFFSET_Msk            (0x0FFFU << ADC_JOFR2_JOFFSET_Pos) /*!< Compensates for the A/D conversion results of the injected channel 2 */

/**
  * @brief ADC_JOFR3 mode enable Register Bit Definition
  */
#define ADC_JOFR3_JOFFSET_Pos            (0)
#define ADC_JOFR3_JOFFSET_Msk            (0x0FFFU << ADC_JOFR3_JOFFSET_Pos) /*!< Compensates for the A/D conversion results of the injected channel 3 */

/**
  * @brief ADC_JSQR mode enable Register Bit Definition
  */
#define ADC_JSQR_JSQ0_Pos                (0)                                /*!< 1st conversion in injected sequence */
#define ADC_JSQR_JSQ0_Msk                (0x1FU << ADC_JSQR_JSQ0_Pos)

#define ADC_JSQR_JSQ1_Pos                (5)                                /*!< 2st conversion in injected sequence */
#define ADC_JSQR_JSQ1_Msk                (0x1FU << ADC_JSQR_JSQ1_Pos)

#define ADC_JSQR_JSQ2_Pos                (10)                               /*!< 3st conversion in injected sequence */
#define ADC_JSQR_JSQ2_Msk                (0x1FU << ADC_JSQR_JSQ2_Pos)

#define ADC_JSQR_JSQ3_Pos                (15)                               /*!< 4st conversion in injected sequence */
#define ADC_JSQR_JSQ3_Msk                (0x1FU << ADC_JSQR_JSQ3_Pos)

#define ADC_JSQR_JNUM_Pos                (20)                               /*!< Injected Sequence length */
#define ADC_JSQR_JNUM_Msk                (0x03U << ADC_JSQR_JNUM_Pos)

/**
  * @brief ADC_JADDATA mode enable Register Bit Definition
  */
#define ADC_JADDATA_JDATA_Pos            (0)
#define ADC_JADDATA_JDATA_Msk            (0xFFFFU << ADC_JADDATA_JDATA_Pos)     /*!< Transfer data */
#define ADC_JADDATA_JCHANNELSEL_Pos      (16)
#define ADC_JADDATA_JCHANNELSEL_Msk      (0x1FU << ADC_JADDATA_JCHANNELSEL_Pos) /*!< Channel selection */
#define ADC_JADDATA_JOVERRUN_Pos         (21)
#define ADC_JADDATA_JOVERRUN_Msk         (0x01U << ADC_JADDATA_JOVERRUN_Pos)    /*!< Overrun flag */
#define ADC_JADDATA_JVALID_Pos           (22)
#define ADC_JADDATA_JVALID_Msk           (0x01U << ADC_JADDATA_JVALID_Pos)      /*!< Valid flag */

/**
  * @brief ADC_JDR0 mode enable Register Bit Definition
  */
#define ADC_JDR0_JVALID_Pos              (22)
#define ADC_JDR0_JVALID_Msk              (0x01U << ADC_JDR0_JVALID_Pos)   /*!< Valid flag */
#define ADC_JDR0_JOVERRUN_Pos            (21)
#define ADC_JDR0_JOVERRUN_Msk            (0x01U << ADC_JDR0_JOVERRUN_Pos) /*!< Overrun flag */
#define ADC_JDR0_JDATA_Pos               (0)
#define ADC_JDR0_JDATA_Msk               (0xFFFFU << ADC_JDR0_JDATA_Pos)  /*!< Transfer data */

/**
  * @brief ADC_JDR1 mode enable Register Bit Definition
  */
#define ADC_JDR1_JVALID_Pos              (22)
#define ADC_JDR1_JVALID_Msk              (0x01U << ADC_JDR1_JVALID_Pos)   /*!< Valid flag */
#define ADC_JDR1_JOVERRUN_Pos            (21)
#define ADC_JDR1_JOVERRUN_Msk            (0x01U << ADC_JDR1_JOVERRUN_Pos) /*!< Overrun flag */
#define ADC_JDR1_JDATA_Pos               (0)
#define ADC_JDR1_JDATA_Msk               (0xFFFFU << ADC_JDR1_JDATA_Pos)  /*!< Transfer data */

/**
  * @brief ADC_JDR2 mode enable Register Bit Definition
  */
#define ADC_JDR2_JVALID_Pos              (22)
#define ADC_JDR2_JVALID_Msk              (0x01U << ADC_JDR2_JVALID_Pos)   /*!< Valid flag */
#define ADC_JDR2_JOVERRUN_Pos            (21)
#define ADC_JDR2_JOVERRUN_Msk            (0x01U << ADC_JDR2_JOVERRUN_Pos) /*!< Overrun flag */
#define ADC_JDR2_JDATA_Pos               (0)
#define ADC_JDR2_JDATA_Msk               (0xFFFFU << ADC_JDR2_JDATA_Pos)  /*!< Transfer data */

/**
  * @brief ADC_JDR3 mode enable Register Bit Definition
  */
#define ADC_JDR3_JVALID_Pos              (22)
#define ADC_JDR3_JVALID_Msk              (0x01U << ADC_JDR3_JVALID_Pos)   /*!< Valid flag */
#define ADC_JDR3_JOVERRUN_Pos            (21)
#define ADC_JDR3_JOVERRUN_Msk            (0x01U << ADC_JDR3_JOVERRUN_Pos) /*!< Overrun flag */
#define ADC_JDR3_JDATA_Pos               (0)
#define ADC_JDR3_JDATA_Msk               (0xFFFFU << ADC_JDR3_JDATA_Pos)  /*!< Transfer data */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/** --------------------------------------------------------------------------*/
#endif
/** --------------------------------------------------------------------------*/

