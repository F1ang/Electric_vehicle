/***********************************************************************************************************************
    @file     reg_syscfg.h
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

#ifndef __REG_SYSCFG_H
#define __REG_SYSCFG_H

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

#define SYSCFG_BASE                     (APB2PERIPH_BASE + 0x0000) /*!< Base Address: 0x40010000 */

/**
  * @brief SysTem Configuration Register Structure Definition
  */
typedef struct
{
    __IO uint32_t CFGR1;               /*!< SYSCFG configuration register                  offset: 0x00 */
    __IO uint32_t RESERVED0x04;        /*!< RESERVED register                              offset: 0x04 */
    __IO uint32_t EXTICR[4];           /*!< EXTI Control register                          offset: 0x08-0x14 */
    __IO uint32_t CFGR2;               /*!< SYSCFG configuration 2 register                offset: 0x18 */
    __IO uint32_t PDETCSR;             /*!< Power detection configuration status register  offset: 0x1C */
    __IO uint32_t VOSDLY;              /*!< VOSDLY configuration register                  offset: 0x20 */
    __IO uint32_t DMARMP;              /*!< DMA Remapping register                         offset: 0x24 */
    __IO uint32_t BUS_PRI;             /*!< Bus priority configuration register            offset: 0x28 */
} SYSCFG_TypeDef;

#define SYSCFG                                        ((SYSCFG_TypeDef *)SYSCFG_BASE)

/**
  * @brief System Configuration (SYSCFG)
  */

/**
  * @brief SYSCFG_CFGR1 Register Bit definition
  */
#define SYSCFG_CFGR1_CAN1_SPV_Pos                     (16)
#define SYSCFG_CFGR1_CAN1_SPV_Msk                     (0x01U << SYSCFG_CFGR1_CAN1_SPV_Pos)  /*!< FCAN1 SUPERVISOR ACCESS */
#define SYSCFG_CFGR1_CAN1_TEST_Pos                    (17)
#define SYSCFG_CFGR1_CAN1_TEST_Msk                    (0x01U << SYSCFG_CFGR1_CAN1_TEST_Pos) /*!< FCAN1 TEST ACCESS */
#define SYSCFG_CFGR1_CAN1_STOP_Pos                    (18)
#define SYSCFG_CFGR1_CAN1_STOP_Msk                    (0x01U << SYSCFG_CFGR1_CAN1_STOP_Pos) /*!< FCAN1 DEBUG STOP */

/**
  * @brief SYSCFG_EXTICR1 Bit mask
  */
#define SYSCFG_EXTICR1_EXTI0                          (0x000FU)                             /*!< EXTI  0 configuration mask */
#define SYSCFG_EXTICR1_EXTI1                          (0x00F0U)                             /*!< EXTI  1 configuration mask */
#define SYSCFG_EXTICR1_EXTI2                          (0x0F00U)                             /*!< EXTI  2 configuration mask */
#define SYSCFG_EXTICR1_EXTI3                          (0xF000U)                             /*!< EXTI  3 configuration mask */

/**
  * @brief SYSCFG_EXTICR2 Bit mask
  */
#define SYSCFG_EXTICR2_EXTI4                          (0x000FU)                             /*!< EXTI  4 configuration mask */
#define SYSCFG_EXTICR2_EXTI5                          (0x00F0U)                             /*!< EXTI  5 configuration mask */
#define SYSCFG_EXTICR2_EXTI6                          (0x0F00U)                             /*!< EXTI  6 configuration mask */
#define SYSCFG_EXTICR2_EXTI7                          (0xF000U)                             /*!< EXTI  7 configuration mask */

/**
  * @brief SYSCFG_EXTICR3 Bit mask
  */
#define SYSCFG_EXTICR3_EXTI8                          (0x000FU)                             /*!< EXTI  8 configuration mask */
#define SYSCFG_EXTICR3_EXTI9                          (0x00F0U)                             /*!< EXTI  9 configuration mask */
#define SYSCFG_EXTICR3_EXTI10                         (0x0F00U)                             /*!< EXTI 10 configuration mask */
#define SYSCFG_EXTICR3_EXTI11                         (0xF000U)                             /*!< EXTI 11 configuration mask */

/**
  * @brief SYSCFG_EXTICR4 Bit mask
  */
#define SYSCFG_EXTICR4_EXTI12                         (0x000FU)                             /*!< EXTI 12 configuration mask */
#define SYSCFG_EXTICR4_EXTI13                         (0x00F0U)                             /*!< EXTI 13 configuration mask */
#define SYSCFG_EXTICR4_EXTI14                         (0x0F00U)                             /*!< EXTI 14 configuration mask */
#define SYSCFG_EXTICR4_EXTI15                         (0xF000U)                             /*!< EXTI 15 configuration mask */

/**
  * @brief EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA                       (0x0000U)                             /*!< PA[0] pin  for EXTI[0] */
#define SYSCFG_EXTICR1_EXTI0_PB                       (0x0001U)                             /*!< PB[0] pin  for EXTI[0] */
#define SYSCFG_EXTICR1_EXTI0_PC                       (0x0002U)                             /*!< PC[0] pin  for EXTI[0] */
#define SYSCFG_EXTICR1_EXTI0_PH                       (0x0007U)                             /*!< PH[0] pin  for EXTI[0] */
#define SYSCFG_EXTICR1_EXTI0_PI                       (0x0008U)                             /*!< PI[0] pin  for EXTI[0] */

/**
  * @brief EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA                       (0x0000U)                             /*!< PA[1] pin  for EXTI[1] */
#define SYSCFG_EXTICR1_EXTI1_PB                       (0x0010U)                             /*!< PB[1] pin  for EXTI[1] */
#define SYSCFG_EXTICR1_EXTI1_PC                       (0x0020U)                             /*!< PC[1] pin  for EXTI[1] */
#define SYSCFG_EXTICR1_EXTI1_PH                       (0x0070U)                             /*!< PH[1] pin  for EXTI[1] */
#define SYSCFG_EXTICR1_EXTI1_PI                       (0x0080U)                             /*!< PI[1] pin  for EXTI[1] */

/**
  * @brief EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA                       (0x0000U)                             /*!< PA[2] pin  for EXTI[2] */
#define SYSCFG_EXTICR1_EXTI2_PB                       (0x0100U)                             /*!< PB[2] pin  for EXTI[2] */
#define SYSCFG_EXTICR1_EXTI2_PC                       (0x0200U)                             /*!< PC[2] pin  for EXTI[2] */
#define SYSCFG_EXTICR1_EXTI2_PD                       (0x0300U)                             /*!< PD[2] pin  for EXTI[2] */

/**
  * @brief EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA                       (0x0000U)                             /*!< PA[3] pin  for EXTI[3] */
#define SYSCFG_EXTICR1_EXTI3_PB                       (0x1000U)                             /*!< PB[3] pin  for EXTI[3] */
#define SYSCFG_EXTICR1_EXTI3_PC                       (0x2000U)                             /*!< PC[3] pin  for EXTI[3] */
#define SYSCFG_EXTICR1_EXTI3_PH                       (0x7000U)                             /*!< PH[3] pin  for EXTI[3] */

/**
  * @brief EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA                       (0x0000U)                             /*!< PA[4] pin  for EXTI[4] */
#define SYSCFG_EXTICR2_EXTI4_PB                       (0x0001U)                             /*!< PB[4] pin  for EXTI[4] */
#define SYSCFG_EXTICR2_EXTI4_PC                       (0x0002U)                             /*!< PC[4] pin  for EXTI[4] */

/**
  * @brief EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA                       (0x0000U)                             /*!< PA[5] pin  for EXTI[5] */
#define SYSCFG_EXTICR2_EXTI5_PB                       (0x0010U)                             /*!< PB[5] pin  for EXTI[5] */
#define SYSCFG_EXTICR2_EXTI5_PC                       (0x0020U)                             /*!< PC[5] pin  for EXTI[5] */

/**
  * @brief EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA                       (0x0000U)                             /*!< PA[6] pin  for EXTI[6] */
#define SYSCFG_EXTICR2_EXTI6_PB                       (0x0100U)                             /*!< PB[6] pin  for EXTI[6] */
#define SYSCFG_EXTICR2_EXTI6_PC                       (0x0200U)                             /*!< PC[6] pin  for EXTI[6] */

/**
  * @brief EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA                       (0x0000U)                             /*!< PA[7] pin  for EXTI[7] */
#define SYSCFG_EXTICR2_EXTI7_PB                       (0x1000U)                             /*!< PB[7] pin  for EXTI[7] */
#define SYSCFG_EXTICR2_EXTI7_PC                       (0x2000U)                             /*!< PC[7] pin  for EXTI[7] */

/**
  * @brief EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA                       (0x0000U)                             /*!< PA[8] pin  for EXTI[8] */
#define SYSCFG_EXTICR3_EXTI8_PB                       (0x0001U)                             /*!< PB[8] pin  for EXTI[8] */
#define SYSCFG_EXTICR3_EXTI8_PC                       (0x0002U)                             /*!< PC[8] pin  for EXTI[8] */

/**
  * @brief EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA                       (0x0000U)                             /*!< PA[9] pin  for EXTI[9] */
#define SYSCFG_EXTICR3_EXTI9_PB                       (0x0010U)                             /*!< PB[9] pin  for EXTI[9] */
#define SYSCFG_EXTICR3_EXTI9_PC                       (0x0020U)                             /*!< PC[9] pin  for EXTI[9] */

/**
  * @brief EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA                      (0x0000U)                             /*!< PA[10] pin for EXTI[10] */
#define SYSCFG_EXTICR3_EXTI10_PB                      (0x0100U)                             /*!< PB[10] pin for EXTI[10] */
#define SYSCFG_EXTICR3_EXTI10_PC                      (0x0200U)                             /*!< PC[10] pin for EXTI[10] */

/**
  * @brief EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA                      (0x0000U)                             /*!< PA[11] pin for EXTI[11] */
#define SYSCFG_EXTICR3_EXTI11_PB                      (0x1000U)                             /*!< PB[11] pin for EXTI[11] */
#define SYSCFG_EXTICR3_EXTI11_PC                      (0x2000U)                             /*!< PC[11] pin for EXTI[11] */

/**
  * @brief EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA                      (0x0000U)                             /*!< PA[12] pin for EXTI[12] */
#define SYSCFG_EXTICR4_EXTI12_PB                      (0x0001U)                             /*!< PB[12] pin for EXTI[12] */
#define SYSCFG_EXTICR4_EXTI12_PC                      (0x0002U)                             /*!< PC[12] pin for EXTI[12] */

/**
  * @brief EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA                      (0x0000U)                             /*!< PA[13] pin for EXTI[13] */
#define SYSCFG_EXTICR4_EXTI13_PB                      (0x0010U)                             /*!< PB[13] pin for EXTI[13] */
#define SYSCFG_EXTICR4_EXTI13_PC                      (0x0020U)                             /*!< PC[13] pin for EXTI[13] */

/**
  * @brief EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA                      (0x0000U)                             /*!< PA[14] pin for EXTI[14] */
#define SYSCFG_EXTICR4_EXTI14_PB                      (0x0100U)                             /*!< PB[14] pin for EXTI[14] */
#define SYSCFG_EXTICR4_EXTI14_PC                      (0x0200U)                             /*!< PC[14] pin for EXTI[14] */

/**
  * @brief EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA                      (0x0000U)                             /*!< PA[15] pin for EXTI[15] */
#define SYSCFG_EXTICR4_EXTI15_PB                      (0x1000U)                             /*!< PB[15] pin for EXTI[15] */
#define SYSCFG_EXTICR4_EXTI15_PC                      (0x2000U)                             /*!< PC[15] pin for EXTI[15] */

/**
  * @brief SYSCFG_CFGR2 Register Bit definition
  */
#define SYSCFG_CFGR2_I2C1_MODE_SEL_Pos                (16)
#define SYSCFG_CFGR2_I2C1_MODE_SEL_Msk                (0x01U << SYSCFG_CFGR2_I2C1_MODE_SEL_Pos) /*!< I2C1 Enable PushPull mode */
#define SYSCFG_CFGR2_I2C2_MODE_SEL_Pos                (17)
#define SYSCFG_CFGR2_I2C2_MODE_SEL_Msk                (0x01U << SYSCFG_CFGR2_I2C2_MODE_SEL_Pos) /*!< I2C2 Enable PushPull mode */

#define SYSCFG_CFGR2_PB10FMP_Pos                      (24)
#define SYSCFG_CFGR2_PB10FMP_Msk                      (0x01U << SYSCFG_CFGR2_PB10FMP_Pos)       /*!< I2C Enable PB10 FM+ */
#define SYSCFG_CFGR2_PB11FMP_Pos                      (25)
#define SYSCFG_CFGR2_PB11FMP_Msk                      (0x01U << SYSCFG_CFGR2_PB11FMP_Pos)       /*!< I2C Enable PB11 FM+ */

/**
  * @brief SYSCFG_PDETCSR Register Bit definition
  */
#define SYSCFG_PDETCSR_PVDE_Pos                       (0)
#define SYSCFG_PDETCSR_PVDE_Msk                       (0x01U << SYSCFG_PDETCSR_PVDE_Pos) /*!< PVD Enable */
#define SYSCFG_PDETCSR_PLS_Pos                        (1)
#define SYSCFG_PDETCSR_PLS_Msk                        (0x0FU << SYSCFG_PDETCSR_PLS_Pos)  /*!< PVD threshold selection */
#define SYSCFG_PDETCSR_PLS_0                          (0x01U << SYSCFG_PDETCSR_PLS_Pos)  /*!< Bit 0 */
#define SYSCFG_PDETCSR_PLS_1                          (0x02U << SYSCFG_PDETCSR_PLS_Pos)  /*!< Bit 1 */
#define SYSCFG_PDETCSR_PLS_2                          (0x04U << SYSCFG_PDETCSR_PLS_Pos)  /*!< Bit 2 */
#define SYSCFG_PDETCSR_PLS_3                          (0x08U << SYSCFG_PDETCSR_PLS_Pos)  /*!< Bit 3 */

#define SYSCFG_PDETCSR_PVDO_Pos                       (5)
#define SYSCFG_PDETCSR_PVDO_Msk                       (0x01U << SYSCFG_PDETCSR_PVDO_Pos) /*!< PVD output status */

/**
  * @brief SYSCFG_VOSDLY Register Bit definition
  */
#define SYSCFG_VOSDLY_CNT_Pos                         (0)
#define SYSCFG_VOSDLY_CNT_Msk                         (0x3FFU << SYSCFG_VOSDLY_CNT_Pos) /*!< VOS delay time */

/**
  * @brief SYSCFG_DMARMP Register Bit definition
  */
#define SYSCFG_DMARMP_TIM1_TRIG_RMP_Pos               (0)
#define SYSCFG_DMARMP_TIM1_TRIG_RMP_Msk               (0x01U << SYSCFG_DMARMP_TIM1_TRIG_RMP_Pos) /*!< The DMA function of TIM1 TRIG is remapped to DMA1 channel7 */
#define SYSCFG_DMARMP_TIM1_COM_RMP_Pos                (1)
#define SYSCFG_DMARMP_TIM1_COM_RMP_Msk                (0x01U << SYSCFG_DMARMP_TIM1_COM_RMP_Pos)  /*!< The DMA function of TIM1 COM is remapped to DMA1 channel8 */

#define SYSCFG_DMARMP_TIM2_UP_RMP_Pos                 (2)
#define SYSCFG_DMARMP_TIM2_UP_RMP_Msk                 (0x03U << SYSCFG_DMARMP_TIM2_UP_RMP_Pos)
#define SYSCFG_DMARMP_TIM2_UP_RMP_0                   (0x00U << SYSCFG_DMARMP_TIM2_UP_RMP_Pos)   /*!< The DMA function of TIM2 UP is remapped to DMA1 channel2 */
#define SYSCFG_DMARMP_TIM2_UP_RMP_1                   (0x01U << SYSCFG_DMARMP_TIM2_UP_RMP_Pos)   /*!< The DMA function of TIM2 UP is remapped to DMA1 channel3 */
#define SYSCFG_DMARMP_TIM2_UP_RMP_2                   (0x02U << SYSCFG_DMARMP_TIM2_UP_RMP_Pos)   /*!< The DMA function of TIM2 UP is remapped to DMA1 channel8 */
#define SYSCFG_DMARMP_TIM2_UP_RMP_3                   (0x03U << SYSCFG_DMARMP_TIM2_UP_RMP_Pos)   /*!< reserved */

#define SYSCFG_DMARMP_TIM2_CC3_RMP_Pos                (4)
#define SYSCFG_DMARMP_TIM2_CC3_RMP_Msk                (0x01U << SYSCFG_DMARMP_TIM2_CC3_RMP_Pos)  /*!< The DMA function of TIM2 CC3 is remapped to DMA1 channel3 */
#define SYSCFG_DMARMP_TIM2_CC4_RMP_Pos                (5)
#define SYSCFG_DMARMP_TIM2_CC4_RMP_Msk                (0x01U << SYSCFG_DMARMP_TIM2_CC4_RMP_Pos)  /*!< The DMA function of TIM2 CC4 is remapped to DMA1 channel8 */

#define SYSCFG_DMARMP_TIM5_UP_RMP_Pos                 (8)
#define SYSCFG_DMARMP_TIM5_UP_RMP_Msk                 (0x01U << SYSCFG_DMARMP_TIM5_UP_RMP_Pos)   /*!< The DMA function of TIM5 UP is remapped to DMA1 channel6 */

#define SYSCFG_DMARMP_TIM6_UP_RMP_Pos                 (10)
#define SYSCFG_DMARMP_TIM6_UP_RMP_Msk                 (0x01U << SYSCFG_DMARMP_TIM6_UP_RMP_Pos)   /*!< The DMA function of TIM6 UP is remapped to DMA1 channel6 */

#define SYSCFG_DMARMP_TIM7_UP_RMP_Pos                 (12)
#define SYSCFG_DMARMP_TIM7_UP_RMP_Msk                 (0x01U << SYSCFG_DMARMP_TIM7_UP_RMP_Pos)   /*!< The DMA function of TIM7 UP is remapped to DMA1 channel6 */
#define SYSCFG_DMARMP_TIM8_TRIG_RMP_Pos               (13)
#define SYSCFG_DMARMP_TIM8_TRIG_RMP_Msk               (0x01U << SYSCFG_DMARMP_TIM8_TRIG_RMP_Pos) /*!< The DMA function of TIM8 TRIG is remapped to DMA1 channel6 */
#define SYSCFG_DMARMP_TIM8_UP_RMP_Pos                 (14)
#define SYSCFG_DMARMP_TIM8_UP_RMP_Msk                 (0x01U << SYSCFG_DMARMP_TIM8_UP_RMP_Pos)   /*!< The DMA function of TIM8 UP is remapped to DMA1 channel6 */
#define SYSCFG_DMARMP_TIM8_COM_RMP_Pos                (15)
#define SYSCFG_DMARMP_TIM8_COM_RMP_Msk                (0x01U << SYSCFG_DMARMP_TIM8_COM_RMP_Pos)  /*!< The DMA function of TIM8 COM is remapped to DMA1 channel8 */
#define SYSCFG_DMARMP_UART4_RX_RMP_Pos                (16)
#define SYSCFG_DMARMP_UART4_RX_RMP_Msk                (0x01U << SYSCFG_DMARMP_UART4_RX_RMP_Pos)  /*!< The DMA function of UART4 RX is remapped to DMA1 channel7 */
#define SYSCFG_DMARMP_UART4_TX_RMP_Pos                (17)
#define SYSCFG_DMARMP_UART4_TX_RMP_Msk                (0x01U << SYSCFG_DMARMP_UART4_TX_RMP_Pos)  /*!< The DMA function of UART4 TX is remapped to DMA1 channel8 */

#define SYSCFG_DMARMP_I3C1_RX_RMP_Pos                 (24)
#define SYSCFG_DMARMP_I3C1_RX_RMP_Msk                 (0x01U << SYSCFG_DMARMP_I3C1_RX_RMP_Pos)   /*!< The DMA function of I3C RX is remapped to DMA1 channel8 */
#define SYSCFG_DMARMP_I3C1_TX_RMP_Pos                 (25)
#define SYSCFG_DMARMP_I3C1_TX_RMP_Msk                 (0x01U << SYSCFG_DMARMP_I3C1_TX_RMP_Pos)   /*!< The DMA function of I3C TX is remapped to DMA1 channel7 */

#define SYSCFG_DMARMP_DAC_CH1_RMP_Pos                 (28)
#define SYSCFG_DMARMP_DAC_CH1_RMP_Msk                 (0x01U << SYSCFG_DMARMP_DAC_CH1_RMP_Pos)   /*!< The DMA function of DAC CH1 is remapped to DMA1 channel7 */

#define SYSCFG_DMARMP_ADC1_RMP_Pos                    (30)
#define SYSCFG_DMARMP_ADC1_RMP_Msk                    (0x01U << SYSCFG_DMARMP_ADC1_RMP_Pos)      /*!< The DMA function of ADC1 is remapped to DMA1 channel6 */
#define SYSCFG_DMARMP_ADC2_RMP_Pos                    (31)
#define SYSCFG_DMARMP_ADC2_RMP_Msk                    (0x01U << SYSCFG_DMARMP_ADC2_RMP_Pos)      /*!< The DMA function of ADC2 is remapped to DMA1 channel8 */

/**
  * @brief SYSCFG_BUS_PRI Register Bit definition
  */
#define SYSCFG_BUS_PRI_M1_Pos                         (0)
#define SYSCFG_BUS_PRI_M1_Msk                         (0x07U << SYSCFG_BUS_PRI_M1_Pos)
#define SYSCFG_BUS_PRI_M2_Pos                         (3)
#define SYSCFG_BUS_PRI_M2_Msk                         (0x07U << SYSCFG_BUS_PRI_M2_Pos)
#define SYSCFG_BUS_PRI_M3_Pos                         (6)
#define SYSCFG_BUS_PRI_M3_Msk                         (0x07U << SYSCFG_BUS_PRI_M3_Pos)
#define SYSCFG_BUS_PRI_M4_Pos                         (9)
#define SYSCFG_BUS_PRI_M4_Msk                         (0x07U << SYSCFG_BUS_PRI_M4_Pos)
#define SYSCFG_BUS_PRI_M5_Pos                         (12)
#define SYSCFG_BUS_PRI_M5_Msk                         (0x07U << SYSCFG_BUS_PRI_M5_Pos)

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

