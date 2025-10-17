/***********************************************************************************************************************
    @file     reg_rcc.h
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

#ifndef __REG_RCC_H
#define __REG_RCC_H

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
  * @brief RCC Base Address Definition
  */
#define RCC_BASE                        (AHB1PERIPH_BASE + 0x1000) /*!< Base Address: 0x40021000 */

/**
  * @brief RCC Register Structure Definition
  */
typedef struct
{
    __IO uint32_t CR;                  /*!< Control Register                               offset: 0x00 */
    __IO uint32_t CFGR;                /*!< Configuration Register                         offset: 0x04 */
    __IO uint32_t CIR;                 /*!< Clock Interrupt Register                       offset: 0x08 */
    __IO uint32_t RESERVED0x0C[2];     /*!< Reserved,                                      offset: 0x0C~0x10*/
    __IO uint32_t AHBRSTR;             /*!< AHB Peripheral Reset Register,                 offset: 0x14 */
    __IO uint32_t APB2RSTR;            /*!< APB2 Peripheral Reset Register,                offset: 0x18 */
    __IO uint32_t APB1RSTR;            /*!< APB1 Peripheral Reset Register,                offset: 0x1C */
    __IO uint32_t RESERVED0x20[2];     /*!< Reserved,                                      offset: 0x20~0x24 */
    __IO uint32_t AHBENR;              /*!< AHB Peripheral Clock Enable Register,          offset: 0x28 */
    __IO uint32_t APB2ENR;             /*!< APB2 Peripheral Clock Enable Register,         offset: 0x2C */
    __IO uint32_t APB1ENR;             /*!< APB1 Peripheral Clock Enable Register,         offset: 0x30 */
    __IO uint32_t BDCR;                /*!< Backup Domain Control Register,                offset: 0x34 */
    __IO uint32_t CSR;                 /*!< Control Status Register,                       offset: 0x38 */
    __IO uint32_t SYSCFGR;             /*!< System Configuration Register,                 offset: 0x3C */
    __IO uint32_t CFGR2;               /*!< Clock Configuration Register 2,                offset: 0x40 */
    __IO uint32_t ICSCR;               /*!< Internal Clock Source Calibration Register,    offset: 0x44 */
    __IO uint32_t PLL1CFGR;            /*!< PLL1 Configuration Register,                   offset: 0x48 */
    __IO uint32_t PLL2CFGR;            /*!< PLL2 Configuration Register,                   offset: 0x4C */
    __IO uint32_t RESERVED0x50[4];     /*!< Reserved,                                      offset: 0x50~0x5C */
    __IO uint32_t ADC1CFGR;            /*!< ADC1 Configuration Register,                   offset: 0x60 */
    __IO uint32_t ADC2CFGR;            /*!< ADC2 Configuration Register,                   offset: 0x64 */
    __IO uint32_t RESERVED0x68[2];     /*!< Reserved,                                      offset: 0x68~0x6C */
    __IO uint32_t DACCFGR;             /*!< DAC Configuration Register,                    offset: 0x70 */
    __IO uint32_t RESERVED0x74[2];     /*!< Reserved,                                      offset: 0x74~0x78 */
    __IO uint32_t TPIUCFGR;            /*!< TPIU Configuration Register,                   offset: 0x7C */
} RCC_TypeDef;

/**
  * @brief RCC type pointer Definition
  */
#define RCC                             ((RCC_TypeDef *)RCC_BASE)

/**
  * @brief RCC_CR Register Bit Definition
  */
#define RCC_CR_HSION_Pos                (0)
#define RCC_CR_HSION_Msk                (0x01U << RCC_CR_HSION_Pos)          /*!< Internal High Speed clock enable */

#define RCC_CR_HSIRDY_Pos               (1)
#define RCC_CR_HSIRDY_Msk               (0x01U << RCC_CR_HSIRDY_Pos)         /*!< Internal High Speed clock ready flag */

#define RCC_CR_HSELPFBYP_Pos            (4)
#define RCC_CR_HSELPFBYP_Msk            (0x01U << RCC_CR_HSELPFBYP_Pos)      /*!< LPF_IN direct output */
#define RCC_CR_HSELPFSEL_Pos            (5)
#define RCC_CR_HSELPFSEL_Msk            (0x01U << RCC_CR_HSELPFSEL_Pos)      /*!< Output after LPF filtering */
#define RCC_CR_HSEDEGLITCHBYP_Pos       (6)
#define RCC_CR_HSEDEGLITCHBYP_Msk       (0x01U << RCC_CR_HSEDEGLITCHBYP_Pos) /*!< Bypass deburring function */
#define RCC_CR_HSEDEGLITCHSEL_Pos       (7)
#define RCC_CR_HSEDEGLITCHSEL_Msk       (0x01U << RCC_CR_HSEDEGLITCHSEL_Pos) /*!< Deburring width 5nS */
#define RCC_CR_HSEOUTPUTSEL_Pos         (8)
#define RCC_CR_HSEOUTPUTSEL_Msk         (0x01U << RCC_CR_HSEOUTPUTSEL_Pos)   /*!< Filtered output */
#define RCC_CR_HSEDR_Pos                (9)
#define RCC_CR_HSEDR_Msk                (0x03U << RCC_CR_HSEDR_Pos)          /*!< HSE Driver Ability */

#define RCC_CR_HSIDIV_Pos               (11)
#define RCC_CR_HSIDIV_Msk               (0x07U << RCC_CR_HSIDIV_Pos)
#define RCC_CR_HSIDIV_1                 (0x00U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 1   factor  */
#define RCC_CR_HSIDIV_2                 (0x01U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 2   factor  */
#define RCC_CR_HSIDIV_4                 (0x02U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 4   factor  */
#define RCC_CR_HSIDIV_8                 (0x03U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 8   factor  */
#define RCC_CR_HSIDIV_16                (0x04U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 16  factor  */
#define RCC_CR_HSIDIV_32                (0x05U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 32  factor  */
#define RCC_CR_HSIDIV_64                (0x06U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 64  factor  */
#define RCC_CR_HSIDIV_128               (0x07U << RCC_CR_HSIDIV_Pos)    /*!< HSI clock division 128 factor  */

#define RCC_CR_HSEON_Pos                (16)
#define RCC_CR_HSEON_Msk                (0x01U << RCC_CR_HSEON_Pos)     /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos               (17)
#define RCC_CR_HSERDY_Msk               (0x01U << RCC_CR_HSERDY_Pos)    /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP_Pos               (18)
#define RCC_CR_HSEBYP_Msk               (0x01U << RCC_CR_HSEBYP_Pos)    /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON_Pos                (19)
#define RCC_CR_CSSON_Msk                (0x01U << RCC_CR_CSSON_Pos)     /*!< Clock Security System enable */

#define RCC_CR_HSEAACSEL_Pos            (21)
#define RCC_CR_HSEAACSEL_Msk            (0x01U << RCC_CR_HSEAACSEL_Pos) /*!< HSE Amplitude Auto Calibration Selece */
#define RCC_CR_HSEIB_Pos                (22)
#define RCC_CR_HSEIB_Msk                (0x03U << RCC_CR_HSEIB_Pos)     /*!< HSE Current Mode Select */
#define RCC_CR_HSEIB_LOW                (0x00U << RCC_CR_HSEIB_Pos)     /*!< Low power mode */
#define RCC_CR_HSEIB_MID                (0x01U << RCC_CR_HSEIB_Pos)     /*!< Medium power mode */
#define RCC_CR_HSEIB_MID_HIGH           (0x02U << RCC_CR_HSEIB_Pos)     /*!< Medium-high power mode */
#define RCC_CR_HSEIB_HIGH               (0x03U << RCC_CR_HSEIB_Pos)     /*!< High power mode */

#define RCC_CR_PLL1ON_Pos               (24)
#define RCC_CR_PLL1ON_Msk               (0x01U << RCC_CR_PLL1ON_Pos)    /*!< PLL1 enable */
#define RCC_CR_PLL1RDY_Pos              (25)
#define RCC_CR_PLL1RDY_Msk              (0x01U << RCC_CR_PLL1RDY_Pos)   /*!< PLL1 clock ready flag */

#define RCC_CR_PLL2ON_Pos               (28)
#define RCC_CR_PLL2ON_Msk               (0x01U << RCC_CR_PLL2ON_Pos)    /*!< PLL2 enable */
#define RCC_CR_PLL2RDY_Pos              (29)
#define RCC_CR_PLL2RDY_Msk              (0x01U << RCC_CR_PLL2RDY_Pos)   /*!< PLL2 clock ready flag */

/**
  * @brief RCC_CFGR Register Bit Definition
  */
#define RCC_CFGR_SW_Pos                 (0)
#define RCC_CFGR_SW_Msk                 (0x03U << RCC_CFGR_SW_Pos)         /*!< SW[1:0] bits (System clock Switch) */

#define RCC_CFGR_SWS_Pos                (2)
#define RCC_CFGR_SWS_Msk                (0x03U << RCC_CFGR_SWS_Pos)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI                (0x00U << RCC_CFGR_SWS_Pos)        /*!< HSI/6 oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                (0x01U << RCC_CFGR_SWS_Pos)        /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                (0x02U << RCC_CFGR_SWS_Pos)        /*!< PLL used as system clock */
#define RCC_CFGR_SWS_LSI                (0x03U << RCC_CFGR_SWS_Pos)        /*!< LSI used as system clock */

#define RCC_CFGR_HPRE_Pos               (4)
#define RCC_CFGR_HPRE_Msk               (0x0FU << RCC_CFGR_HPRE_Pos)       /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_DIV1              (0x00U << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2              (0x08U << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4              (0x09U << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8              (0x0AU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16             (0x0BU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64             (0x0CU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128            (0x0DU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256            (0x0EU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512            (0x0FU << RCC_CFGR_HPRE_Pos)       /*!< AHB = FCLK = SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_Pos              (8)
#define RCC_CFGR_PPRE1_Msk              (0x07U << RCC_CFGR_PPRE1_Pos)      /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_DIV1             (0x00U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK not divided */
#define RCC_CFGR_PPRE1_DIV32            (0x01U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK divided by 32 */
#define RCC_CFGR_PPRE1_DIV2             (0x04U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4             (0x05U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8             (0x06U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16            (0x07U << RCC_CFGR_PPRE1_Pos)      /*!< APB1 = HCLK divided by 16 */

#define RCC_CFGR_PPRE2_Pos              (11)
#define RCC_CFGR_PPRE2_Msk              (0x07U << RCC_CFGR_PPRE2_Pos)      /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_DIV1             (0x00U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK not divided */
#define RCC_CFGR_PPRE2_DIV32            (0x01U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK divided by 32 */
#define RCC_CFGR_PPRE2_DIV2             (0x04U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4             (0x05U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8             (0x06U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16            (0x07U << RCC_CFGR_PPRE2_Pos)      /*!< APB2 = HCLK divided by 16 */

#define RCC_CFGR_ADC1CLKSEL_Pos         (16)
#define RCC_CFGR_ADC1CLKSEL_Msk         (0x01U << RCC_CFGR_ADC1CLKSEL_Pos) /*!< Select ADC1CLK Clock Source */
#define RCC_CFGR_ADC2CLKSEL_Pos         (17)
#define RCC_CFGR_ADC2CLKSEL_Msk         (0x01U << RCC_CFGR_ADC2CLKSEL_Pos) /*!< Select ADC2CLK Clock Source */

#define RCC_CFGR_USBCLKSEL_Pos          (19)
#define RCC_CFGR_USBCLKSEL_Msk          (0x01U << RCC_CFGR_USBCLKSEL_Pos)  /*!< Select USBCLK Clock Source */

#define RCC_CFGR_USBPRE_Pos             (22)
#define RCC_CFGR_USBPRE_Msk             (0x03U << RCC_CFGR_USBPRE_Pos)     /*!< USB Prescaler */
#define RCC_CFGR_USBPRE_1               (0x00U << RCC_CFGR_USBPRE_Pos)     /*!< PLL1/PLL2 clock directly as USB_FS clock */
#define RCC_CFGR_USBPRE_2               (0x01U << RCC_CFGR_USBPRE_Pos)     /*!< PLL1/PLL2 clock 2 division as USB_FS clock */
#define RCC_CFGR_USBPRE_3               (0x02U << RCC_CFGR_USBPRE_Pos)     /*!< PLL1/PLL2 clock 3 division as USB_FS clock */
#define RCC_CFGR_USBPRE_4               (0x03U << RCC_CFGR_USBPRE_Pos)     /*!< PLL1/PLL2 clock 4 division as USB_FS clock */

#define RCC_CFGR_MCO_Pos                (24)
#define RCC_CFGR_MCO_Msk                (0x0FU << RCC_CFGR_MCO_Pos)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_NOCLOCK            (0x00U << RCC_CFGR_MCO_Pos)        /*!< No clock */
#define RCC_CFGR_MCO_LSI                (0x02U << RCC_CFGR_MCO_Pos)        /*!< LSI clock */
#define RCC_CFGR_MCO_LSE                (0x03U << RCC_CFGR_MCO_Pos)        /*!< LSE clock */
#define RCC_CFGR_MCO_SYSCLK             (0x04U << RCC_CFGR_MCO_Pos)        /*!< System clock selected */
#define RCC_CFGR_MCO_HSI                (0x05U << RCC_CFGR_MCO_Pos)        /*!< Internal 48 MHz RC oscillator clock selected */
#define RCC_CFGR_MCO_HSE                (0x06U << RCC_CFGR_MCO_Pos)        /*!< External 1-25 MHz oscillator clock selected */
#define RCC_CFGR_MCO_PLL1               (0x07U << RCC_CFGR_MCO_Pos)        /*!< PLL clock divided by 2 selected */
#define RCC_CFGR_MCO_PLL2               (0x08U << RCC_CFGR_MCO_Pos)        /*!< PLL clock divided by 2 selected */

/**
  * @brief RCC_CIR Register Bit Definition
  */
#define RCC_CIR_LSIRDYF_Pos             (0)
#define RCC_CIR_LSIRDYF_Msk             (0x01U << RCC_CIR_LSIRDYF_Pos)   /*!< LSI Ready Interrupt flag */

#define RCC_CIR_LSERDYF_Pos             (1)
#define RCC_CIR_LSERDYF_Msk             (0x01U << RCC_CIR_LSERDYF_Pos)   /*!< LSE Ready Interrupt flag */

#define RCC_CIR_HSIRDYF_Pos             (2)
#define RCC_CIR_HSIRDYF_Msk             (0x01U << RCC_CIR_HSIRDYF_Pos)   /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF_Pos             (3)
#define RCC_CIR_HSERDYF_Msk             (0x01U << RCC_CIR_HSERDYF_Pos)   /*!< HSE Ready Interrupt flag */

#define RCC_CIR_PLL1RDYF_Pos            (4)
#define RCC_CIR_PLL1RDYF_Msk            (0x01U << RCC_CIR_PLL1RDYF_Pos)  /*!< PLL1 Ready Interrupt flag */
#define RCC_CIR_PLL2RDYF_Pos            (5)
#define RCC_CIR_PLL2RDYF_Msk            (0x01U << RCC_CIR_PLL2RDYF_Pos)  /*!< PLL2 Ready Interrupt flag */

#define RCC_CIR_CSSF_Pos                (7)
#define RCC_CIR_CSSF_Msk                (0x01U << RCC_CIR_CSSF_Pos)      /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE_Pos            (8)
#define RCC_CIR_LSIRDYIE_Msk            (0x01U << RCC_CIR_LSIRDYIE_Pos)  /*!< LSI Ready Interrupt Enable */

#define RCC_CIR_LSERDYIE_Pos            (9)
#define RCC_CIR_LSERDYIE_Msk            (0x01U << RCC_CIR_LSERDYIE_Pos)  /*!< LSE Ready Interrupt Enable */

#define RCC_CIR_HSIRDYIE_Pos            (10)
#define RCC_CIR_HSIRDYIE_Msk            (0x01U << RCC_CIR_HSIRDYIE_Pos)  /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE_Pos            (11)
#define RCC_CIR_HSERDYIE_Msk            (0x01U << RCC_CIR_HSERDYIE_Pos)  /*!< HSE Ready Interrupt Enable */

#define RCC_CIR_PLL1RDYIE_Pos           (12)
#define RCC_CIR_PLL1RDYIE_Msk           (0x01U << RCC_CIR_PLL1RDYIE_Pos) /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_PLL2RDYIE_Pos           (13)
#define RCC_CIR_PLL2RDYIE_Msk           (0x01U << RCC_CIR_PLL2RDYIE_Pos) /*!< PLL Ready Interrupt Enable */

#define RCC_CIR_LSIRDYC_Pos             (16)
#define RCC_CIR_LSIRDYC_Msk             (0x01U << RCC_CIR_LSIRDYC_Pos)   /*!< LSI Ready Interrupt Clear */

#define RCC_CIR_LSERDYC_Pos             (17)
#define RCC_CIR_LSERDYC_Msk             (0x01U << RCC_CIR_LSERDYC_Pos)   /*!< LSE Ready Interrupt Clear */

#define RCC_CIR_HSIRDYC_Pos             (18)
#define RCC_CIR_HSIRDYC_Msk             (0x01U << RCC_CIR_HSIRDYC_Pos)   /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC_Pos             (19)
#define RCC_CIR_HSERDYC_Msk             (0x01U << RCC_CIR_HSERDYC_Pos)   /*!< HSE Ready Interrupt Clear */

#define RCC_CIR_PLL1RDYC_Pos            (20)
#define RCC_CIR_PLL1RDYC_Msk            (0x01U << RCC_CIR_PLL1RDYC_Pos)  /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_PLL2RDYC_Pos            (21)
#define RCC_CIR_PLL2RDYC_Msk            (0x01U << RCC_CIR_PLL2RDYC_Pos)  /*!< PLL Ready Interrupt Clear */

#define RCC_CIR_CSSC_Pos                (23)
#define RCC_CIR_CSSC_Msk                (0x01U << RCC_CIR_CSSC_Pos)      /*!< Clock Security System Interrupt Clear */

/**
  * @brief RCC_AHBRSTR Register Bit Definition
  */
#define RCC_AHBRSTR_GPIOA_Pos           (0)
#define RCC_AHBRSTR_GPIOA_Msk           (0x01U << RCC_AHBRSTR_GPIOA_Pos)  /*!< GPIOA Reset */
#define RCC_AHBRSTR_GPIOB_Pos           (1)
#define RCC_AHBRSTR_GPIOB_Msk           (0x01U << RCC_AHBRSTR_GPIOB_Pos)  /*!< GPIOB Reset */
#define RCC_AHBRSTR_GPIOC_Pos           (2)
#define RCC_AHBRSTR_GPIOC_Msk           (0x01U << RCC_AHBRSTR_GPIOC_Pos)  /*!< GPIOC Reset */
#define RCC_AHBRSTR_GPIOD_Pos           (3)
#define RCC_AHBRSTR_GPIOD_Msk           (0x01U << RCC_AHBRSTR_GPIOD_Pos)  /*!< GPIOD Reset */

#define RCC_AHBRSTR_GPIOH_Pos           (7)
#define RCC_AHBRSTR_GPIOH_Msk           (0x01U << RCC_AHBRSTR_GPIOH_Pos)  /*!< GPIOH Reset */
#define RCC_AHBRSTR_GPIOI_Pos           (8)
#define RCC_AHBRSTR_GPIOI_Msk           (0x01U << RCC_AHBRSTR_GPIOI_Pos)  /*!< GPIOI Reset */

#define RCC_AHBRSTR_CRC_Pos             (12)
#define RCC_AHBRSTR_CRC_Msk             (0x01U << RCC_AHBRSTR_CRC_Pos)    /*!< CRC Reset */

#define RCC_AHBRSTR_CORDIC_Pos          (15)
#define RCC_AHBRSTR_CORDIC_Msk          (0x01U << RCC_AHBRSTR_CORDIC_Pos) /*!< CORDIC Reset */

#define RCC_AHBRSTR_DMA1_Pos            (21)
#define RCC_AHBRSTR_DMA1_Msk            (0x01U << RCC_AHBRSTR_DMA1_Pos)   /*!< DMA1 Reset */
#define RCC_AHBRSTR_DMA2_Pos            (22)
#define RCC_AHBRSTR_DMA2_Msk            (0x01U << RCC_AHBRSTR_DMA2_Pos)   /*!< DMA2 Reset */

#define RCC_AHBRSTR_USB_FS_Pos          (24)
#define RCC_AHBRSTR_USB_FS_Msk          (0x01U << RCC_AHBRSTR_USB_FS_Pos) /*!< USB_FS Reset */

/**
  * @brief RCC_APB2RSTR Register Bit Definition
  */
#define RCC_APB2RSTR_TIM1_Pos           (0)
#define RCC_APB2RSTR_TIM1_Msk           (0x01U << RCC_APB2RSTR_TIM1_Pos)   /*!< TIM1 Reset */
#define RCC_APB2RSTR_TIM8_Pos           (1)
#define RCC_APB2RSTR_TIM8_Msk           (0x01U << RCC_APB2RSTR_TIM8_Pos)   /*!< TIM8 Reset */

#define RCC_APB2RSTR_UART1_Pos          (4)
#define RCC_APB2RSTR_UART1_Msk          (0x01U << RCC_APB2RSTR_UART1_Pos)  /*!< UART1 Reset */

#define RCC_APB2RSTR_ADC1_Pos           (8)
#define RCC_APB2RSTR_ADC1_Msk           (0x01U << RCC_APB2RSTR_ADC1_Pos)   /*!< ADC1 interface reset */
#define RCC_APB2RSTR_ADC2_Pos           (9)
#define RCC_APB2RSTR_ADC2_Msk           (0x01U << RCC_APB2RSTR_ADC2_Pos)   /*!< ADC2 interface reset */

#define RCC_APB2RSTR_SPI1_Pos           (12)
#define RCC_APB2RSTR_SPI1_Msk           (0x01U << RCC_APB2RSTR_SPI1_Pos)   /*!< SPI 1 reset */
#define RCC_APB2RSTR_SYSCFG_Pos         (14)
#define RCC_APB2RSTR_SYSCFG_Msk         (0x01U << RCC_APB2RSTR_SYSCFG_Pos) /*!< SYSCFG Reset */

#define RCC_APB2RSTR_COMP_Pos           (15)
#define RCC_APB2RSTR_COMP_Msk           (0x01U << RCC_APB2RSTR_COMP_Pos)   /*!< COMP interface reset */

#define RCC_APB2RSTR_USART_Pos          (20)
#define RCC_APB2RSTR_USART_Msk          (0x01U << RCC_APB2RSTR_USART_Pos)  /*!< RCC_APB2RSTR_USART_Pos */

#define RCC_APB2RSTR_MDS_Pos            (28)
#define RCC_APB2RSTR_MDS_Msk            (0x01U << RCC_APB2RSTR_MDS_Pos)    /*!< MDS Reset */

#define RCC_APB2RSTR_LPTIM_Pos          (30)
#define RCC_APB2RSTR_LPTIM_Msk          (0x01U << RCC_APB2RSTR_LPTIM_Pos)  /*!< LPTIM Reset */
#define RCC_APB2RSTR_LPUART_Pos         (31)
#define RCC_APB2RSTR_LPUART_Msk         (0x01U << RCC_APB2RSTR_LPUART_Pos) /*!< LPUART Reset */

/**
  * @brief RCC_APB1RSTR Register Bit Definition
  */
#define RCC_APB1RSTR_TIM2_Pos           (0)
#define RCC_APB1RSTR_TIM2_Msk           (0x01U << RCC_APB1RSTR_TIM2_Pos)    /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3_Pos           (1)
#define RCC_APB1RSTR_TIM3_Msk           (0x01U << RCC_APB1RSTR_TIM3_Pos)    /*!< Timer 3 reset */
#define RCC_APB1RSTR_TIM4_Pos           (2)
#define RCC_APB1RSTR_TIM4_Msk           (0x01U << RCC_APB1RSTR_TIM4_Pos)    /*!< Timer 4 reset */
#define RCC_APB1RSTR_TIM5_Pos           (3)
#define RCC_APB1RSTR_TIM5_Msk           (0x01U << RCC_APB1RSTR_TIM5_Pos)    /*!< Timer 5 reset */
#define RCC_APB1RSTR_TIM6_Pos           (4)
#define RCC_APB1RSTR_TIM6_Msk           (0x01U << RCC_APB1RSTR_TIM6_Pos)    /*!< Timer 6 reset */
#define RCC_APB1RSTR_TIM7_Pos           (5)
#define RCC_APB1RSTR_TIM7_Msk           (0x01U << RCC_APB1RSTR_TIM7_Pos)    /*!< Timer 7 reset */

#define RCC_APB1RSTR_I3C_Pos            (8)
#define RCC_APB1RSTR_I3C_Msk            (0x01U << RCC_APB1RSTR_I3C_Pos)     /*!< TI3C reset */

#define RCC_APB1RSTR_WWDG_Pos           (11)
#define RCC_APB1RSTR_WWDG_Msk           (0x01U << RCC_APB1RSTR_WWDG_Pos)    /*!< Window Watchdog reset */

#define RCC_APB1RSTR_SPI2_Pos           (14)
#define RCC_APB1RSTR_SPI2_Msk           (0x01U << RCC_APB1RSTR_SPI2_Pos)    /*!< SPI 2 reset */
#define RCC_APB1RSTR_SPI3_Pos           (15)
#define RCC_APB1RSTR_SPI3_Msk           (0x01U << RCC_APB1RSTR_SPI3_Pos)    /*!< SPI 3 reset */

#define RCC_APB1RSTR_UART2_Pos          (17)
#define RCC_APB1RSTR_UART2_Msk          (0x01U << RCC_APB1RSTR_UART2_Pos)   /*!< UART 2 reset */
#define RCC_APB1RSTR_UART3_Pos          (18)
#define RCC_APB1RSTR_UART3_Msk          (0x01U << RCC_APB1RSTR_UART3_Pos)   /*!< UART 3 reset */
#define RCC_APB1RSTR_UART4_Pos          (19)
#define RCC_APB1RSTR_UART4_Msk          (0x01U << RCC_APB1RSTR_UART4_Pos)   /*!< UART 4 reset */

#define RCC_APB1RSTR_I2C1_Pos           (21)
#define RCC_APB1RSTR_I2C1_Msk           (0x01U << RCC_APB1RSTR_I2C1_Pos)    /*!< I2C 1 reset */
#define RCC_APB1RSTR_I2C2_Pos           (22)
#define RCC_APB1RSTR_I2C2_Msk           (0x01U << RCC_APB1RSTR_I2C2_Pos)    /*!< I2C 2 reset */

#define RCC_APB1RSTR_CRS_Pos            (24)
#define RCC_APB1RSTR_CRS_Msk            (0x01U << RCC_APB1RSTR_CRS_Pos)     /*!< CRS Reset */
#define RCC_APB1RSTR_FLEXCAN_Pos        (25)
#define RCC_APB1RSTR_FLEXCAN_Msk        (0x01U << RCC_APB1RSTR_FLEXCAN_Pos) /*!< FLEXCAN Reset */

#define RCC_APB1RSTR_DBG_Pos            (26)
#define RCC_APB1RSTR_DBG_Msk            (0x01U << RCC_APB1RSTR_DBG_Pos)     /*!< DBG Reset */
#define RCC_APB1RSTR_BKP_Pos            (27)
#define RCC_APB1RSTR_BKP_Msk            (0x01U << RCC_APB1RSTR_BKP_Pos)     /*!< BKP Reset */
#define RCC_APB1RSTR_PWR_Pos            (28)
#define RCC_APB1RSTR_PWR_Msk            (0x01U << RCC_APB1RSTR_PWR_Pos)     /*!< Power interface reset */
#define RCC_APB1RSTR_DAC_Pos            (29)
#define RCC_APB1RSTR_DAC_Msk            (0x01U << RCC_APB1RSTR_DAC_Pos)     /*!< DAC Reset */

/**
  * @brief RCC_AHBENR Register Bit Definition
  */
#define RCC_AHBENR_GPIOA_Pos            (0)
#define RCC_AHBENR_GPIOA_Msk            (0x01U << RCC_AHBENR_GPIOA_Pos)  /*!< GPIOA CLOCK ENABLE */
#define RCC_AHBENR_GPIOB_Pos            (1)
#define RCC_AHBENR_GPIOB_Msk            (0x01U << RCC_AHBENR_GPIOB_Pos)  /*!< GPIOB CLOCK ENABLE */
#define RCC_AHBENR_GPIOC_Pos            (2)
#define RCC_AHBENR_GPIOC_Msk            (0x01U << RCC_AHBENR_GPIOC_Pos)  /*!< GPIOC CLOCK ENABLE */
#define RCC_AHBENR_GPIOD_Pos            (3)
#define RCC_AHBENR_GPIOD_Msk            (0x01U << RCC_AHBENR_GPIOD_Pos)  /*!< GPIOD CLOCK ENABLE */

#define RCC_AHBENR_GPIOH_Pos            (7)
#define RCC_AHBENR_GPIOH_Msk            (0x01U << RCC_AHBENR_GPIOH_Pos)  /*!< GPIOH CLOCK ENABLE */
#define RCC_AHBENR_GPIOI_Pos            (8)
#define RCC_AHBENR_GPIOI_Msk            (0x01U << RCC_AHBENR_GPIOI_Pos)  /*!< GPIOI CLOCK ENABLE */

#define RCC_AHBENR_CRC_Pos              (12)
#define RCC_AHBENR_CRC_Msk              (0x01U << RCC_AHBENR_CRC_Pos)    /*!< CRC CLOCK ENABLE */
#define RCC_AHBENR_FLASH_Pos            (13)
#define RCC_AHBENR_FLASH_Msk            (0x01U << RCC_AHBENR_FLASH_Pos)  /*!< FLASH CLOCK ENABLE */
#define RCC_AHBENR_SRAM_Pos             (14)
#define RCC_AHBENR_SRAM_Msk             (0x01U << RCC_AHBENR_SRAM_Pos)   /*!< SRAM CLOCK ENABLE */
#define RCC_AHBENR_CORDIC_Pos           (15)
#define RCC_AHBENR_CORDIC_Msk           (0x01U << RCC_AHBENR_CORDIC_Pos) /*!< CORDIC CLOCK ENABLE */

#define RCC_AHBENR_DMA1_Pos             (21)
#define RCC_AHBENR_DMA1_Msk             (0x01U << RCC_AHBENR_DMA1_Pos)   /*!< DMA1 CLOCK ENABLE */
#define RCC_AHBENR_DMA2_Pos             (22)
#define RCC_AHBENR_DMA2_Msk             (0x01U << RCC_AHBENR_DMA2_Pos)   /*!< DMA2 CLOCK ENABLE */

#define RCC_AHBENR_USB_FS_Pos           (24)
#define RCC_AHBENR_USB_FS_Msk           (0x01U << RCC_AHBENR_USB_FS_Pos) /*!< USB_FS CLOCK ENABLE */

/**
  * @brief RCC_APB2ENR Register Bit Definition
  */
#define RCC_APB2ENR_TIM1_Pos            (0)
#define RCC_APB2ENR_TIM1_Msk            (0x01U << RCC_APB2ENR_TIM1_Pos)   /*!< TIM1 Clock enable */
#define RCC_APB2ENR_TIM8_Pos            (1)
#define RCC_APB2ENR_TIM8_Msk            (0x01U << RCC_APB2ENR_TIM8_Pos)   /*!< TIM8 CLOCK ENABLE */

#define RCC_APB2ENR_UART1_Pos           (4)
#define RCC_APB2ENR_UART1_Msk           (0x01U << RCC_APB2ENR_UART1_Pos)  /*!< UART1 CLOCK ENABLE */

#define RCC_APB2ENR_ADC1_Pos            (8)
#define RCC_APB2ENR_ADC1_Msk            (0x01U << RCC_APB2ENR_ADC1_Pos)   /*!< ADC1 Interface CLOCK ENABLE */
#define RCC_APB2ENR_ADC2_Pos            (9)
#define RCC_APB2ENR_ADC2_Msk            (0x01U << RCC_APB2ENR_ADC2_Pos)   /*!< ADC2 Interface CLOCK ENABLE */

#define RCC_APB2ENR_SPI1_Pos            (12)
#define RCC_APB2ENR_SPI1_Msk            (0x01U << RCC_APB2ENR_SPI1_Pos)   /*!< SPI1 CLOCK ENABLE */

#define RCC_APB2ENR_SYSCFG_Pos          (14)
#define RCC_APB2ENR_SYSCFG_Msk          (0x01U << RCC_APB2ENR_SYSCFG_Pos) /*!< SYSCFG CLOCK ENABLE */
#define RCC_APB2ENR_COMP_Pos            (15)
#define RCC_APB2ENR_COMP_Msk            (0x01U << RCC_APB2ENR_COMP_Pos)   /*!< COMP CLOCK ENABLE */

#define RCC_APB2ENR_USART_Pos           (20)
#define RCC_APB2ENR_USART_Msk           (0x01U << RCC_APB2ENR_USART_Pos)  /*!< USART CLOCK ENABLE */

#define RCC_APB2ENR_MDS_Pos             (28)
#define RCC_APB2ENR_MDS_Msk             (0x01U << RCC_APB2ENR_MDS_Pos)    /*!< MDS CLOCK ENABLE */

#define RCC_APB2ENR_LPTIM_Pos           (30)
#define RCC_APB2ENR_LPTIM_Msk           (0x01U << RCC_APB2ENR_LPTIM_Pos)  /*!< LPTIM CLOCK ENABLE */
#define RCC_APB2ENR_LPUART_Pos          (31)
#define RCC_APB2ENR_LPUART_Msk          (0x01U << RCC_APB2ENR_LPUART_Pos) /*!< LPUART CLOCK ENABLE */

/**
  * @brief RCC_APB1ENR Register Bit Definition
  */
#define RCC_APB1ENR_TIM2_Pos            (0)
#define RCC_APB1ENR_TIM2_Msk            (0x01U << RCC_APB1ENR_TIM2_Pos)    /*!< TIM2 CLOCK ENABLE */
#define RCC_APB1ENR_TIM3_Pos            (1)
#define RCC_APB1ENR_TIM3_Msk            (0x01U << RCC_APB1ENR_TIM3_Pos)    /*!< TIM3 CLOCK ENABLE */
#define RCC_APB1ENR_TIM4_Pos            (2)
#define RCC_APB1ENR_TIM4_Msk            (0x01U << RCC_APB1ENR_TIM4_Pos)    /*!< TIM4 CLOCK ENABLE */
#define RCC_APB1ENR_TIM5_Pos            (3)
#define RCC_APB1ENR_TIM5_Msk            (0x01U << RCC_APB1ENR_TIM5_Pos)    /*!< TIM5 CLOCK ENABLE */
#define RCC_APB1ENR_TIM6_Pos            (4)
#define RCC_APB1ENR_TIM6_Msk            (0x01U << RCC_APB1ENR_TIM6_Pos)    /*!< TIM6 CLOCK ENABLE */
#define RCC_APB1ENR_TIM7_Pos            (5)
#define RCC_APB1ENR_TIM7_Msk            (0x01U << RCC_APB1ENR_TIM7_Pos)    /*!< TIM7 CLOCK ENABLE */

#define RCC_APB1ENR_I3C_Pos             (8)
#define RCC_APB1ENR_I3C_Msk             (0x01U << RCC_APB1ENR_I3C_Pos)     /*!< I3C CLOCK ENABLE */

#define RCC_APB1ENR_WWDG_Pos            (11)
#define RCC_APB1ENR_WWDG_Msk            (0x01U << RCC_APB1ENR_WWDG_Pos)    /*!< Window Watchdog CLOCK ENABLE*/

#define RCC_APB1ENR_SPI2_Pos            (14)
#define RCC_APB1ENR_SPI2_Msk            (0x01U << RCC_APB1ENR_SPI2_Pos)    /*!< SPI2 CLOCK ENABLE */
#define RCC_APB1ENR_SPI3_Pos            (15)
#define RCC_APB1ENR_SPI3_Msk            (0x01U << RCC_APB1ENR_SPI3_Pos)    /*!< SPI3 CLOCK ENABLE */

#define RCC_APB1ENR_UART2_Pos           (17)
#define RCC_APB1ENR_UART2_Msk           (0x01U << RCC_APB1ENR_UART2_Pos)   /*!< UART2 CLOCK ENABLE */
#define RCC_APB1ENR_UART3_Pos           (18)
#define RCC_APB1ENR_UART3_Msk           (0x01U << RCC_APB1ENR_UART3_Pos)   /*!< UART3 CLOCK ENABLE */
#define RCC_APB1ENR_UART4_Pos           (19)
#define RCC_APB1ENR_UART4_Msk           (0x01U << RCC_APB1ENR_UART4_Pos)   /*!< UART4 CLOCK ENABLE*/

#define RCC_APB1ENR_I2C1_Pos            (21)
#define RCC_APB1ENR_I2C1_Msk            (0x01U << RCC_APB1ENR_I2C1_Pos)    /*!< I2C CLOCK ENABLE */
#define RCC_APB1ENR_I2C2_Pos            (22)
#define RCC_APB1ENR_I2C2_Msk            (0x01U << RCC_APB1ENR_I2C2_Pos)    /*!, I2C2 CLOCK ENANE */

#define RCC_APB1ENR_CRS_Pos             (24)
#define RCC_APB1ENR_CRS_Msk             (0x01U << RCC_APB1ENR_CRS_Pos)     /*!< CRS CLOCK ENABEE */
#define RCC_APB1ENR_FLEXCAN_Pos         (25)
#define RCC_APB1ENR_FLEXCAN_Msk         (0x01U << RCC_APB1ENR_FLEXCAN_Pos) /*!< FLEXCAN CLOCK ENABLE */

#define RCC_APB1ENR_BKP_Pos             (27)
#define RCC_APB1ENR_BKP_Msk             (0x01U << RCC_APB1ENR_BKP_Pos)     /*!< BKP CLOCK ENABLE */
#define RCC_APB1ENR_PWR_DBG_Pos         (28)
#define RCC_APB1ENR_PWR_DBG_Msk         (0x01U << RCC_APB1ENR_PWR_DBG_Pos) /*!< PWR/DBG CLOCK ENABLE */
#define RCC_APB1ENR_DAC_Pos             (29)
#define RCC_APB1ENR_DAC_Msk             (0x01U << RCC_APB1ENR_DAC_Pos)     /*!< DAC CLOCK ENABLE */

/**
  * @brief RCC_BDCR Register Bit Definition
  */
#define RCC_BDCR_LSEON_Pos              (0)
#define RCC_BDCR_LSEON_Msk              (0x01U << RCC_BDCR_LSEON_Pos)  /*!< External Low-speed Oscillator Enable */
#define RCC_BDCR_LSERDY_Pos             (1)
#define RCC_BDCR_LSERDY_Msk             (0x01U << RCC_BDCR_LSERDY_Pos) /*!< External Low-speed Oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos             (2)
#define RCC_BDCR_LSEBYP_Msk             (0x01U << RCC_BDCR_LSEBYP_Pos) /*!< External Low-speed Oscillator Bypass */

#define RCC_BDCR_RTCSEL_Pos             (8)
#define RCC_BDCR_RTCSEL_Msk             (0x03U << RCC_BDCR_RTCSEL_Pos) /*!< RTC Clock Source Selection */

#define RCC_BDCR_RTCEN_Pos              (15)
#define RCC_BDCR_RTCEN_Msk              (0x01U << RCC_BDCR_RTCEN_Pos)  /*!< RTC Clock Enable */
#define RCC_BDCR_BDRST_Pos              (16)
#define RCC_BDCR_BDRST_Msk              (0x01U << RCC_BDCR_BDRST_Pos)  /*!< Backup Domain Software Reset */

#define RCC_BDCR_DBP_Pos                (24)
#define RCC_BDCR_DBP_Msk                (0x01U << RCC_BDCR_DBP_Pos)    /*!< Disable backup area write protection */

/**
  * @brief RCC_CSR Register Bit Definition
  */
#define RCC_CSR_LSION_Pos               (0)
#define RCC_CSR_LSION_Msk               (0x01U << RCC_CSR_LSION_Pos)    /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos              (1)
#define RCC_CSR_LSIRDY_Msk              (0x01U << RCC_CSR_LSIRDY_Pos)   /*!< Internal Low Speed oscillator Ready */

#define RCC_CSR_LSIOE_Pos               (5)
#define RCC_CSR_LSIOE_Msk               (0x01U << RCC_CSR_LSIOE_Pos)    /*!< LSI output enable */
#define RCC_CSR_PVDRSTEN_Pos            (6)
#define RCC_CSR_PVDRSTEN_Msk            (0x01U << RCC_CSR_PVDRSTEN_Pos) /*!< PVD reset enable */
#define RCC_CSR_LOCKUPEN_Pos            (7)
#define RCC_CSR_LOCKUPEN_Msk            (0x01U << RCC_CSR_LOCKUPEN_Pos) /*!< CPU LOCK reset enable */

#define RCC_CSR_PVDRSTF_Pos             (22)
#define RCC_CSR_PVDRSTF_Msk             (0x01U << RCC_CSR_PVDRSTF_Pos)  /*!< PVD reset flag */
#define RCC_CSR_LOCKUPF_Pos             (23)
#define RCC_CSR_LOCKUPF_Msk             (0x01U << RCC_CSR_LOCKUPF_Pos)  /*!< CPU lockup reset flag */

#define RCC_CSR_RMVF_Pos                (24)
#define RCC_CSR_RMVF_Msk                (0x01U << RCC_CSR_RMVF_Pos)     /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos             (26)
#define RCC_CSR_PINRSTF_Msk             (0x01U << RCC_CSR_PINRSTF_Pos)  /*!< PIN reset flag */

#define RCC_CSR_PORRSTF_Pos             (27)
#define RCC_CSR_PORRSTF_Msk             (0x01U << RCC_CSR_PORRSTF_Pos)  /*!< POR/PDR reset flag */

#define RCC_CSR_SFTRSTF_Pos             (28)
#define RCC_CSR_SFTRSTF_Msk             (0x01U << RCC_CSR_SFTRSTF_Pos)  /*!< Software Reset flag */

#define RCC_CSR_IWDGRSTF_Pos            (29)
#define RCC_CSR_IWDGRSTF_Msk            (0x01U << RCC_CSR_IWDGRSTF_Pos) /*!< Independent Watchdog reset flag */

#define RCC_CSR_WWDGRSTF_Pos            (30)
#define RCC_CSR_WWDGRSTF_Msk            (0x01U << RCC_CSR_WWDGRSTF_Pos) /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos            (31)
#define RCC_CSR_LPWRRSTF_Msk            (0x01U << RCC_CSR_LPWRRSTF_Pos) /*!< Low power reset flag */

/**
  * @brief RCC_SYSCFG Register Bit Definition
  */
#define RCC_SYSCFG_PROGCHECKEN_Pos      (0)
#define RCC_SYSCFG_PROGCHECKEN_Msk      (0x01U << RCC_SYSCFG_PROGCHECKEN_Pos) /*!< Whether to check the number in Flash when writing to Flash */

#define RCC_SYSCFG_HSERFBSEL_Pos        (8)
#define RCC_SYSCFG_HSERFBSEL_Msk        (0x03U << RCC_SYSCFG_HSERFBSEL_Pos)   /*!< Oscillator feedback resistance trimming */
#define RCC_SYSCFG_HSELPFEN_Pos         (14)
#define RCC_SYSCFG_HSELPFEN_Msk         (0x01U << RCC_SYSCFG_HSELPFEN_Pos)    /*!< Oscillator low pass filtering enable */

/**
  * @brief RCC_CFGR2 Register Bit Definition
  */
#define RCC_CFGR2_TIMADV_CKSEL_Pos      (0)
#define RCC_CFGR2_TIMADV_CKSEL_Msk      (0x01U << RCC_CFGR2_TIMADV_CKSEL_Pos) /*!< TIMADV clock selection */
#define RCC_CFGR2_TIMADV_PRE_Pos        (1)
#define RCC_CFGR2_TIMADV_PRE_Msk        (0x07U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK prescaler */
#define RCC_CFGR2_TIMADV_PRE_0          (0x00U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK no division */
#define RCC_CFGR2_TIMADV_PRE_2          (0x04U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK 2 division */
#define RCC_CFGR2_TIMADV_PRE_4          (0x05U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK 4 division */
#define RCC_CFGR2_TIMADV_PRE_8          (0x06U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK 8 division */
#define RCC_CFGR2_TIMADV_PRE_16         (0x07U << RCC_CFGR2_TIMADV_PRE_Pos)   /*!< TIMADV_CLK 16 division */

#define RCC_CFGR2_CANCLKSEL_Pos         (8)
#define RCC_CFGR2_CANCLKSEL_Msk         (0x03U << RCC_CFGR2_CANCLKSEL_Pos)    /*!< FLEXCAN clock selection */
#define RCC_CFGR2_CANCLKSEL_PLL1        (0x00U << RCC_CFGR2_CANCLKSEL_Pos)    /*!< PLL1 */
#define RCC_CFGR2_CANCLKSEL_PLL2        (0x01U << RCC_CFGR2_CANCLKSEL_Pos)    /*!< PLL2 */
#define RCC_CFGR2_CANCLKSEL_HSE         (0x02U << RCC_CFGR2_CANCLKSEL_Pos)    /*!< HSE */

#define RCC_CFGR2_CANPRE_Pos            (12)
#define RCC_CFGR2_CANPRE_Msk            (0x03U << RCC_CFGR2_CANPRE_Pos)       /*!< FLEXCAN prescaler */
#define RCC_CFGR2_CANPRE_1              (0x00U << RCC_CFGR2_CANPRE_Pos)       /*!< FLEXCAN no division */
#define RCC_CFGR2_CANPRE_2              (0x01U << RCC_CFGR2_CANPRE_Pos)       /*!< FLEXCAN 2 division */
#define RCC_CFGR2_CANPRE_3              (0x02U << RCC_CFGR2_CANPRE_Pos)       /*!< FLEXCAN 3 division */
#define RCC_CFGR2_CANPRE_4              (0x03U << RCC_CFGR2_CANPRE_Pos)       /*!< FLEXCAN 4 division */

#define RCC_CFGR2_APB1CLKHVPRE_Pos      (16)
#define RCC_CFGR2_APB1CLKHVPRE_Msk      (0x0FU << RCC_CFGR2_APB1CLKHVPRE_Pos) /*!< APB1_HV output clock division coefficient */
#define RCC_CFGR2_APB1CLKHVPRE_0        (0x01U << RCC_CFGR2_APB1CLKHVPRE_Pos) /*!< Bit 0 */
#define RCC_CFGR2_APB1CLKHVPRE_1        (0x02U << RCC_CFGR2_APB1CLKHVPRE_Pos) /*!< Bit 1 */
#define RCC_CFGR2_APB1CLKHVPRE_2        (0x04U << RCC_CFGR2_APB1CLKHVPRE_Pos) /*!< Bit 2 */
#define RCC_CFGR2_APB1CLKHVPRE_3        (0x08U << RCC_CFGR2_APB1CLKHVPRE_Pos) /*!< Bit 3 */

#define RCC_CFGR2_MCO_PRE_Pos           (20)
#define RCC_CFGR2_MCO_PRE_Msk           (0x0FU << RCC_CFGR2_MCO_PRE_Pos)      /*!< MCO Clock Prescaler Coefficient */
#define RCC_CFGR2_MCO_PRE_0             (0x01U << RCC_CFGR2_MCO_PRE_Pos)      /*!< Bit 0 */
#define RCC_CFGR2_MCO_PRE_1             (0x02U << RCC_CFGR2_MCO_PRE_Pos)      /*!< Bit 1 */
#define RCC_CFGR2_MCO_PRE_2             (0x04U << RCC_CFGR2_MCO_PRE_Pos)      /*!< Bit 2 */
#define RCC_CFGR2_MCO_PRE_3             (0x08U << RCC_CFGR2_MCO_PRE_Pos)      /*!< Bit 3 */

#define RCC_CFGR2_LPUARTCLKSEL_Pos      (26)
#define RCC_CFGR2_LPUARTCLKSEL_Msk      (0x03U << RCC_CFGR2_LPUARTCLKSEL_Pos) /*!< LPUART Clock Source Select */
#define RCC_CFGR2_LPUARTCLKSEL_LSE      (0x00U << RCC_CFGR2_LPUARTCLKSEL_Pos) /*!< LSE_CLK */
#define RCC_CFGR2_LPUARTCLKSEL_LSI      (0x01U << RCC_CFGR2_LPUARTCLKSEL_Pos) /*!< LSI_CLK */
#define RCC_CFGR2_LPUARTCLKSEL_PCLK     (0x02U << RCC_CFGR2_LPUARTCLKSEL_Pos) /*!< PCLK_LPUART */
#define RCC_CFGR2_LPUARTCLKSEL_HSI      (0x03U << RCC_CFGR2_LPUARTCLKSEL_Pos) /*!< HSI_CLK */

#define RCC_CFGR2_LPTIMCLKSEL_Pos       (29)
#define RCC_CFGR2_LPTIMCLKSEL_Msk       (0x03U << RCC_CFGR2_LPTIMCLKSEL_Pos)  /*!< LPUART Clock Source Select */
#define RCC_CFGR2_LPTIMCLKSEL_LSE       (0x00U << RCC_CFGR2_LPTIMCLKSEL_Pos)  /*!< LSE_CLK */
#define RCC_CFGR2_LPTIMCLKSEL_LSI       (0x01U << RCC_CFGR2_LPTIMCLKSEL_Pos)  /*!< LSI_CLK */
#define RCC_CFGR2_LPTIMCLKSEL_PCLK      (0x02U << RCC_CFGR2_LPTIMCLKSEL_Pos)  /*!< PCLK_LPTIM */
#define RCC_CFGR2_LPTIMCLKSEL_HSI       (0x03U << RCC_CFGR2_LPTIMCLKSEL_Pos)  /*!< HSI_CLK */

/**
  * @brief RCC_ICSCR Register Bit Definition
  */
#define RCC_ICSCR_TRIMCRSSEL_Pos        (0)
#define RCC_ICSCR_TRIMCRSSEL_Msk        (0x01U << RCC_ICSCR_TRIMCRSSEL_Pos) /*!< Will HSITRIM value use CRS module as source */

/**
  * @brief RCC_PLL1CFGR Register Bit Definition
  */
#define RCC_PLL1CFGR_PLL1SRC_Pos        (0)
#define RCC_PLL1CFGR_PLL1SRC_Msk        (0x01U << RCC_PLL1CFGR_PLL1SRC_Pos)   /*!< PLL1 Entry Clock Source Selection */
#define RCC_PLL1CFGR_PLL1XTPRE_Pos      (1)
#define RCC_PLL1CFGR_PLL1XTPRE_Msk      (0x01U << RCC_PLL1CFGR_PLL1XTPRE_Pos) /*!< Division selection when HSE clock is used as PLL1 input */
#define RCC_PLL1CFGR_PLL1ICTRL_Pos      (2)
#define RCC_PLL1CFGR_PLL1ICTRL_Msk      (0x03U << RCC_PLL1CFGR_PLL1ICTRL_Pos) /*!< PLL1 CP current control signals */
#define RCC_PLL1CFGR_PLL1LDS_Pos        (4)
#define RCC_PLL1CFGR_PLL1LDS_Msk        (0x07U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< PLL1 Lock Detector Accuracy Select */
#define RCC_PLL1CFGR_PLL1LDS_OFF        (0x00U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Turn off accuracy */
#define RCC_PLL1CFGR_PLL1LDS_HIGH       (0x01U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< High accuracy */
#define RCC_PLL1CFGR_PLL1LDS_SUBHIGH    (0x02U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Sub-high accuracy */
#define RCC_PLL1CFGR_PLL1LDS_MIDHIGH    (0x03U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Medium high accuracy */
#define RCC_PLL1CFGR_PLL1LDS_MID        (0x04U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Medium accuracy */
#define RCC_PLL1CFGR_PLL1LDS_MIDLOW     (0x05U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Medium-low accuracy */
#define RCC_PLL1CFGR_PLL1LDS_SUBLOW     (0x06U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Sub-low accuracy */
#define RCC_PLL1CFGR_PLL1LDS_LOW        (0x07U << RCC_PLL1CFGR_PLL1LDS_Pos)   /*!< Low accuracy */

#define RCC_PLL1CFGR_PLL1DIV_Pos        (8)
#define RCC_PLL1CFGR_PLL1DIV_Msk        (0x07U << RCC_PLL1CFGR_PLL1DIV_Pos)   /*!< PLL1 Divide Factor */

#define RCC_PLL1CFGR_PLL1MUL_Pos        (16)
#define RCC_PLL1CFGR_PLL1MUL_Msk        (0xFFU << RCC_PLL1CFGR_PLL1MUL_Pos)   /*!< PLL1 Multiplication Factor */

#define RCC_PLL1CFGR_PLL1PDIV_Pos       (24)
#define RCC_PLL1CFGR_PLL1PDIV_Msk       (0x07U << RCC_PLL1CFGR_PLL1PDIV_Pos)  /*!< PLL1 Pre-divide Factor */

/**
  * @brief RCC_PLL2CFGR Register Bit Definition
  */
#define RCC_PLL2CFGR_PLL2SRC_Pos        (0)
#define RCC_PLL2CFGR_PLL2SRC_Msk        (0x01U << RCC_PLL2CFGR_PLL2SRC_Pos)   /*!< PLL2 Entry Clock Source Selection */
#define RCC_PLL2CFGR_PLL2XTPRE_Pos      (1)
#define RCC_PLL2CFGR_PLL2XTPRE_Msk      (0x01U << RCC_PLL2CFGR_PLL2XTPRE_Pos) /*!< Division selection when HSE clock is used as PLL2 input */
#define RCC_PLL2CFGR_PLL2ICTRL_Pos      (2)
#define RCC_PLL2CFGR_PLL2ICTRL_Msk      (0x03U << RCC_PLL2CFGR_PLL2ICTRL_Pos) /*!< PLL2 CP current control signals */
#define RCC_PLL2CFGR_PLL2LDS_Pos        (4)
#define RCC_PLL2CFGR_PLL2LDS_Msk        (0x07U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< PLL2 Lock Detector Accuracy Select */
#define RCC_PLL2CFGR_PLL2LDS_OFF        (0x00U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Turn off accuracy */
#define RCC_PLL2CFGR_PLL2LDS_HIGH       (0x01U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< High accuracy */
#define RCC_PLL2CFGR_PLL2LDS_SUBHIGH    (0x02U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Sub-high accuracy */
#define RCC_PLL2CFGR_PLL2LDS_MIDHIGH    (0x03U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Medium high accuracy */
#define RCC_PLL2CFGR_PLL2LDS_MID        (0x04U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Medium accuracy */
#define RCC_PLL2CFGR_PLL2LDS_MIDLOW     (0x05U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Medium-low accuracy */
#define RCC_PLL2CFGR_PLL2LDS_SUBLOW     (0x06U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Sub-low accuracy */
#define RCC_PLL2CFGR_PLL2LDS_LOW        (0x07U << RCC_PLL2CFGR_PLL2LDS_Pos)   /*!< Low accuracy */

#define RCC_PLL2CFGR_PLL2DIV_Pos        (8)
#define RCC_PLL2CFGR_PLL2DIV_Msk        (0x07U << RCC_PLL2CFGR_PLL2DIV_Pos)   /*!< PLL2 Divide Factor */

#define RCC_PLL2CFGR_PLL2MUL_Pos        (16)
#define RCC_PLL2CFGR_PLL2MUL_Msk        (0xFFU << RCC_PLL2CFGR_PLL2MUL_Pos)   /*!< PLL2 Multiplication Factor */

#define RCC_PLL2CFGR_PLL2PDIV_Pos       (24)
#define RCC_PLL2CFGR_PLL2PDIV_Msk       (0x07U << RCC_PLL2CFGR_PLL2PDIV_Pos)  /*!< PLL2 Pre-divide Factor */

/**
  * @brief  RCC_ADC1CFGR Register Bit Definition
  */
#define RCC_ADC1CFGR_PRE_Pos            (0)
#define RCC_ADC1CFGR_PRE_Msk            (0x0FU << RCC_ADC1CFGR_PRE_Pos)     /*!< ADC1 prescaler */
#define RCC_ADC1CFGR_PRECAL_Pos         (8)
#define RCC_ADC1CFGR_PRECAL_Msk         (0x1FFU << RCC_ADC1CFGR_PRECAL_Pos) /*!< ADC1 calibration clock division configuration */

/**
  * @brief  RCC_ADC2CFGR Register Bit Definition
  */
#define RCC_ADC2CFGR_PRE_Pos            (0)
#define RCC_ADC2CFGR_PRE_Msk            (0x0FU << RCC_ADC2CFGR_PRE_Pos)     /*!< ADC2 prescaler */
#define RCC_ADC2CFGR_PRECAL_Pos         (8)
#define RCC_ADC2CFGR_PRECAL_Msk         (0x1FFU << RCC_ADC2CFGR_PRECAL_Pos) /*!< ADC2 calibration clock division configuration */

/**
  * @brief  RCC_DACCFGR Register Bit Definition
  */
#define RCC_DACCFGR_PRE_Pos             (0)
#define RCC_DACCFGR_PRE_Msk             (0x7FU << RCC_DACCFGR_PRE_Pos) /*!< DAC prescaler */

/**
  * @brief  RCC_TPIUCFGR Register Bit Definition
  */
#define RCC_TPIUCFGR_PRE_Pos            (0)
#define RCC_TPIUCFGR_PRE_Msk            (0x03U << RCC_TPIUCFGR_PRE_Pos) /*!< TPIU prescaler */
#define RCC_TPIUCFGR_PRE_0              (0x00U << RCC_TPIUCFGR_PRE_Pos) /*!< No division */
#define RCC_TPIUCFGR_PRE_1              (0x01U << RCC_TPIUCFGR_PRE_Pos) /*!< 2 division */
#define RCC_TPIUCFGR_PRE_2              (0x02U << RCC_TPIUCFGR_PRE_Pos) /*!< 3 division */
#define RCC_TPIUCFGR_PRE_3              (0x03U << RCC_TPIUCFGR_PRE_Pos) /*!< 4 division */

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
/** --------------------------------------------------------------------------*/

