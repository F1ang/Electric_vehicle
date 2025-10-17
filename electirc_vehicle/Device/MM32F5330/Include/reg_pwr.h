/***********************************************************************************************************************
    @file     reg_pwr.h
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

#ifndef __REG_PWR_H
#define __REG_PWR_H

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
  * @brief PWR Base Address Definition
  */
#define PWR_BASE                        (APB1PERIPH_BASE + 0x7000) /*!< Base Address: 0x40007000 */

/**
  * @brief PWR Register Structure Definition
  */
typedef struct
{
    __IO uint32_t CR1;                 /*!< Power control register 1,                                    offset: 0x00 */
    __IO uint32_t CSR;                 /*!< Power control status register,                               offset: 0x04 */
    __IO uint32_t CR2;                 /*!< Power control register 2,                                    offset: 0x08 */
    __IO uint32_t CR3;                 /*!< Power control register 3,                                    offset: 0x0C */
    __IO uint32_t CR4;                 /*!< Power control register 4,                                    offset: 0x10 */
    __IO uint32_t CR5;                 /*!< Power control register 5,                                    offset: 0x14 */
    __IO uint32_t CR6;                 /*!< Power control register 6,                                    offset: 0x18 */
    __IO uint32_t SR;                  /*!< Power status register,                                       offset: 0x1C */
    __IO uint32_t SCR;                 /*!< Power status clear register,                                 offset: 0x20 */
    __IO uint32_t CFGR;                /*!< Power configuration register,                                offset: 0x24 */
    __IO uint32_t RESERVED0x28[2];     /*!< Reserved,                                                    offset: 0x28~0x2C */
    __IO uint32_t MEMCR;               /*!< Power supply storage control register,                       offset: 0x30 */
    __IO uint32_t PWCFGR1;             /*!< Power is waiting for the source configuration register 1     offset: 0x34 */
    __IO uint32_t PWCFGR2;             /*!< Power is waiting for the source configuration register 2     offset: 0x38 */
} PWR_TypeDef;

/**
  * @brief PWR type pointer Definition
  */
#define PWR                                 ((PWR_TypeDef *)PWR_BASE)

/**
  * @brief PWR_CR1 register Bit definition
  */
#define PWR_CR1_LPDS_Pos                    (0)
#define PWR_CR1_LPDS_Msk                    (0x01U << PWR_CR1_LPDS_Pos) /*!< Domain Write Protction */

#define PWR_CR1_PDDS_Pos                    (1)
#define PWR_CR1_PDDS_Msk                    (0x01U << PWR_CR1_PDDS_Pos) /*!< Power Down Deepsleep */

#define PWR_CR1_CSBF_Pos                    (3)
#define PWR_CR1_CSBF_Msk                    (0x01U << PWR_CR1_CSBF_Pos) /*!< Clear Standby Flag */

#define PWR_CR1_VOSH_Pos                    (8)
#define PWR_CR1_VOSH_Msk                    (0x01U << PWR_CR1_VOSH_Pos) /*!< VOS high selection bit */

#define PWR_CR1_LPR_Pos                     (13)
#define PWR_CR1_LPR_Msk                     (0x01U << PWR_CR1_LPR_Pos)  /*!< Low power configuration bit */
#define PWR_CR1_VOSL_Pos                    (14)
#define PWR_CR1_VOSL_Msk                    (0x03U << PWR_CR1_VOSL_Pos) /*!< VOS low selection bit */
#define PWR_CR1_VOSL_0_1V2                  (0x00U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 10 : 1.2V */
#define PWR_CR1_VOSL_0_1V3                  (0x01U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 10 : 1.3V */
#define PWR_CR1_VOSL_0_1V4                  (0x02U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 10 : 1.4V */
#define PWR_CR1_VOSL_1_1V2                  (0x00U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 11 : 1.2V */
#define PWR_CR1_VOSL_1_1V1                  (0x01U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 11 : 1.1V */
#define PWR_CR1_VOSL_1_1V0                  (0x02U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 11 : 1.0V */
#define PWR_CR1_VOSL_1_0V9                  (0x03U << PWR_CR1_VOSL_Pos) /*!< Voltage output selection of the voltage regulator when VOSH is 11 : 0.9V */

/**
  * @brief PWR_CSR register Bit definition
  */
#define PWR_CSR_SBF_Pos                     (1)
#define PWR_CSR_SBF_Msk                     (0x01U << PWR_CSR_SBF_Pos)    /*!< Standby Flag */
#define PWR_CSR_VOSRDY_Pos                  (14)
#define PWR_CSR_VOSRDY_Msk                  (0x01U << PWR_CSR_VOSRDY_Pos) /*!< Voltage regulator output preparation is finished flag */

/**
  * @brief PWR_CR2 register Bit definition
  */
#define PWR_CR2_EWUP_Pos                    (0)
#define PWR_CR2_EWUP_Msk                    (0x3FU << PWR_CR2_EWUP_Pos)  /*!< Enable WakeUp Pin */

#define PWR_CR2_EWUP0_Pos                   (0)
#define PWR_CR2_EWUP0_Msk                   (0x01U << PWR_CR2_EWUP0_Pos) /*!< Enable WakeUp Pin0 */
#define PWR_CR2_EWUP1_Pos                   (1)
#define PWR_CR2_EWUP1_Msk                   (0x01U << PWR_CR2_EWUP1_Pos) /*!< Enable WakeUp Pin1 */
#define PWR_CR2_EWUP2_Pos                   (2)
#define PWR_CR2_EWUP2_Msk                   (0x01U << PWR_CR2_EWUP2_Pos) /*!< Enable WakeUp Pin2 */
#define PWR_CR2_EWUP3_Pos                   (3)
#define PWR_CR2_EWUP3_Msk                   (0x01U << PWR_CR2_EWUP3_Pos) /*!< Enable WakeUp Pin3 */
#define PWR_CR2_EWUP4_Pos                   (4)
#define PWR_CR2_EWUP4_Msk                   (0x01U << PWR_CR2_EWUP4_Pos) /*!< Enable WakeUp Pin4 */
#define PWR_CR2_EWUP5_Pos                   (5)
#define PWR_CR2_EWUP5_Msk                   (0x01U << PWR_CR2_EWUP5_Pos) /*!< Enable WakeUp Pin5 */

/**
  * @brief PWR_CR3 register Bit definition
  */
#define PWR_CR3_WP_Pos                      (0)
#define PWR_CR3_WP_Msk                      (0x3FU << PWR_CR3_WP_Pos)  /*!< Polarity detection of wakeup pin */

#define PWR_CR3_WP0_Pos                     (0)
#define PWR_CR3_WP0_Msk                     (0x01U << PWR_CR3_WP0_Pos) /*!< Polarity detection of wakeup pin0 */
#define PWR_CR3_WP1_Pos                     (1)
#define PWR_CR3_WP1_Msk                     (0x01U << PWR_CR3_WP1_Pos) /*!< Polarity detection of wakeup pin1 */
#define PWR_CR3_WP2_Pos                     (2)
#define PWR_CR3_WP2_Msk                     (0x01U << PWR_CR3_WP2_Pos) /*!< Polarity detection of wakeup pin2 */
#define PWR_CR3_WP3_Pos                     (3)
#define PWR_CR3_WP3_Msk                     (0x01U << PWR_CR3_WP3_Pos) /*!< Polarity detection of wakeup pin3 */
#define PWR_CR3_WP4_Pos                     (4)
#define PWR_CR3_WP4_Msk                     (0x01U << PWR_CR3_WP4_Pos) /*!< Polarity detection of wakeup pin4 */
#define PWR_CR3_WP5_Pos                     (5)
#define PWR_CR3_WP5_Msk                     (0x01U << PWR_CR3_WP5_Pos) /*!< Polarity detection of wakeup pin5 */

/**
  * @brief PWR_CR4 register Bit definition
  */
#define PWR_CR4_FILTSEL0_Pos                (0)
#define PWR_CR4_FILTSEL0_Msk                (0x03U << PWR_CR4_FILTSEL0_Pos) /*!< WakeUp Pin Filter Select 0 */
#define PWR_CR4_FILTSEL0_0                  (0x00U << PWR_CR4_FILTSEL0_Pos) /*!< Select WKUP pin 0 */
#define PWR_CR4_FILTSEL0_1                  (0x01U << PWR_CR4_FILTSEL0_Pos) /*!< Select WKUP pin 1 */
#define PWR_CR4_FILTSEL0_2                  (0x02U << PWR_CR4_FILTSEL0_Pos) /*!< Select WKUP pin 2 */

#define PWR_CR4_FILTE0_Pos                  (2)
#define PWR_CR4_FILTE0_Msk                  (0x03U << PWR_CR4_FILTE0_Pos)   /*!< Wakeup pin filter 0 enable */
#define PWR_CR4_FILTE0_INVALID              (0x00U << PWR_CR4_FILTE0_Pos)   /*!< Enable filter invalid */
#define PWR_CR4_FILTE0_HIGH                 (0x01U << PWR_CR4_FILTE0_Pos)   /*!< Enable filter0 high level filter */
#define PWR_CR4_FILTE0_LOW                  (0x02U << PWR_CR4_FILTE0_Pos)   /*!< Enable filter0 low level filter */

#define PWR_CR4_FILTF0_Pos                  (4)
#define PWR_CR4_FILTF0_Msk                  (0x01U << PWR_CR4_FILTF0_Pos)   /*!< Filter valid flag bit of wakeup pin filter 0 */

#define PWR_CR4_FILTCNT0_Pos                (8)
#define PWR_CR4_FILTCNT0_Msk                (0xFFU << PWR_CR4_FILTCNT0_Pos) /*!< Counter value of wakeup pin filter 0 */

/**
  * @brief PWR_CR5 register Bit definition
  */
#define PWR_CR5_FILTSEL1_Pos                (0)
#define PWR_CR5_FILTSEL1_Msk                (0x03U << PWR_CR5_FILTSEL1_Pos) /*!< WakeUp Pin Filter Select 1 */
#define PWR_CR5_FILTSEL1_3                  (0x00U << PWR_CR5_FILTSEL1_Pos) /*!< Select WKUP pin 3 */
#define PWR_CR5_FILTSEL1_4                  (0x01U << PWR_CR5_FILTSEL1_Pos) /*!< Select WKUP pin 4 */
#define PWR_CR5_FILTSEL1_5                  (0x02U << PWR_CR5_FILTSEL1_Pos) /*!< Select WKUP pin 5 */

#define PWR_CR5_FILTE1_Pos                  (2)
#define PWR_CR5_FILTE1_Msk                  (0x03U << PWR_CR5_FILTE1_Pos)   /*!< Wakeup pin filter 1 enable */
#define PWR_CR5_FILTE1_INVALID              (0x00U << PWR_CR5_FILTE1_Pos)   /*!< Enable filter invalid */
#define PWR_CR5_FILTE1_HIGH                 (0x01U << PWR_CR5_FILTE1_Pos)   /*!< Enable filter1 high level filter */
#define PWR_CR5_FILTE1_LOW                  (0x02U << PWR_CR5_FILTE1_Pos)   /*!< Enable filter1 low level filter */

#define PWR_CR5_FILTF1_Pos                  (4)
#define PWR_CR5_FILTF1_Msk                  (0x01U << PWR_CR5_FILTF1_Pos)   /*!< Filter valid flag bit of wakeup pin filter 1 */

#define PWR_CR5_FILTCNT1_Pos                (8)
#define PWR_CR5_FILTCNT1_Msk                (0xFFU << PWR_CR5_FILTCNT1_Pos) /*!< Counter value of wakeup pin filter 1 */

/**
  * @brief PWR_CR6 register Bit definition
  */
#define PWR_CR6_STDBY_FS_WK_Pos             (0)
#define PWR_CR6_STDBY_FS_WK_Msk             (0x07U << PWR_CR6_STDBY_FS_WK_Pos) /*!< STDBY_FS_WK: Rapidly wakeup standby mode selection bit */
#define PWR_CR6_STDBY_FS_WK_3               (0x02U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 3 LSI period wakeup */
#define PWR_CR6_STDBY_FS_WK_4               (0x03U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 4 LSI period wakeup */
#define PWR_CR6_STDBY_FS_WK_5               (0x04U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 5 LSI period wakeup */
#define PWR_CR6_STDBY_FS_WK_6               (0x05U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 6 LSI period wakeup */
#define PWR_CR6_STDBY_FS_WK_7               (0x06U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 7 LSI period wakeup */
#define PWR_CR6_STDBY_FS_WK_8               (0x07U << PWR_CR6_STDBY_FS_WK_Pos) /*!< 8 LSI period wakeup */

/**
  * @brief PWR_SR register Bit definition
  */
#define PWR_SR_WUF_Pos                      (0)
#define PWR_SR_WUF_Msk                      (0x3FU << PWR_SR_WUF_Pos)  /*!<  wake-up flag */

#define PWR_SR_WUF0_Pos                     (0)
#define PWR_SR_WUF0_Msk                     (0x01U << PWR_SR_WUF0_Pos) /*!<  wake-up flag 0 */
#define PWR_SR_WUF1_Pos                     (1)
#define PWR_SR_WUF1_Msk                     (0x01U << PWR_SR_WUF1_Pos) /*!<  wake-up flag 1 */
#define PWR_SR_WUF2_Pos                     (2)
#define PWR_SR_WUF2_Msk                     (0x01U << PWR_SR_WUF2_Pos) /*!<  wake-up flag 2 */
#define PWR_SR_WUF3_Pos                     (3)
#define PWR_SR_WUF3_Msk                     (0x01U << PWR_SR_WUF3_Pos) /*!<  wake-up flag 3 */
#define PWR_SR_WUF4_Pos                     (4)
#define PWR_SR_WUF4_Msk                     (0x01U << PWR_SR_WUF4_Pos) /*!<  wake-up flag 4 */
#define PWR_SR_WUF5_Pos                     (5)
#define PWR_SR_WUF5_Msk                     (0x01U << PWR_SR_WUF5_Pos) /*!<  wake-up flag 5 */

/**
  * @brief PWR_SCR register Bit definition
  */
#define PWR_SCR_CWUF_Pos                    (0)
#define PWR_SCR_CWUF_Msk                    (0x3FU << PWR_SCR_CWUF_Pos)  /*!< clear wake-up flag */

#define PWR_SCR_CWUF0_Pos                   (0)
#define PWR_SCR_CWUF0_Msk                   (0x01U << PWR_SCR_CWUF0_Pos) /*!< clear wake-up flag 0 */
#define PWR_SCR_CWUF1_Pos                   (1)
#define PWR_SCR_CWUF1_Msk                   (0x01U << PWR_SCR_CWUF1_Pos) /*!< clear wake-up flag 1 */
#define PWR_SCR_CWUF2_Pos                   (2)
#define PWR_SCR_CWUF2_Msk                   (0x01U << PWR_SCR_CWUF2_Pos) /*!< clear wake-up flag 2 */
#define PWR_SCR_CWUF3_Pos                   (3)
#define PWR_SCR_CWUF3_Msk                   (0x01U << PWR_SCR_CWUF3_Pos) /*!< clear wake-up flag 3 */
#define PWR_SCR_CWUF4_Pos                   (4)
#define PWR_SCR_CWUF4_Msk                   (0x01U << PWR_SCR_CWUF4_Pos) /*!< clear wake-up flag 4 */
#define PWR_SCR_CWUF5_Pos                   (5)
#define PWR_SCR_CWUF5_Msk                   (0x01U << PWR_SCR_CWUF5_Pos) /*!< clear wake-up flag 5 */

/**
  * @brief PWR_CFGR register Bit definition
  */
#define PWR_CFGR_LSICALSEL_Pos              (0)
#define PWR_CFGR_LSICALSEL_Msk              (0x1FU << PWR_CFGR_LSICALSEL_Pos) /*!< Enable internal clock calibration  */
#define PWR_CFGR_LSICAL_Pos                 (5)
#define PWR_CFGR_LSICAL_Msk                 (0x1FU << PWR_CFGR_LSICAL_Pos)    /*!< Internal high-speed clock calibration */

/**
  * @brief PWR_MEMCR register Bit definition
  */
#define PWR_MEMCR_MEM_LOCK_Pos              (0)
#define PWR_MEMCR_MEM_LOCK_Msk              (0x1FU << PWR_MEMCR_MEM_LOCK_Pos)      /*!< Configure the unlock bit for PWR_MEMCR */

#define PWR_MEMCR_FFWUP_EN_Pos              (5)
#define PWR_MEMCR_FFWUP_EN_Msk              (0x01U << PWR_MEMCR_FFWUP_EN_Pos)      /*!< The Flash Quick wake-up enables */
#define PWR_MEMCR_FDP_STOP_EN_Pos           (6)
#define PWR_MEMCR_FDP_STOP_EN_Msk           (0x01U << PWR_MEMCR_FDP_STOP_EN_Pos)   /*!< Flash DPSTDB enables efficiently in Stop mode */

#define PWR_MEMCR_SRAM_RET_EN_Pos           (13)
#define PWR_MEMCR_SRAM_RET_EN_Msk           (0x01U << PWR_MEMCR_SRAM_RET_EN_Pos)   /*!< The SRAM Retention is enabled efficiently in both Stop and Deepstop mode */
#define PWR_MEMCR_CCRAM_RET_EN_Pos          (14)
#define PWR_MEMCR_CCRAM_RET_EN_Msk          (0x01U << PWR_MEMCR_CCRAM_RET_EN_Pos)  /*!< The Cache RAM Retention is enabled efficiently in both Stop and Deepstop mode */
#define PWR_MEMCR_CANRAM_RET_EN_Pos         (15)
#define PWR_MEMCR_CANRAM_RET_EN_Msk         (0x01U << PWR_MEMCR_CANRAM_RET_EN_Pos) /*!< The CAN RAM Retention is enabled efficiently in both Stop and Deepstop mode */

/**
  * @brief PWR_PWCFGR1 register Bit definition
  */
#define PWR_PWCFGR1_LDO_WAIT_LOCK_Pos       (0)
#define PWR_PWCFGR1_LDO_WAIT_LOCK_Msk       (0x1FU << PWR_PWCFGR1_LDO_WAIT_LOCK_Pos) /*!< LDO_WAIT configuration */
#define PWR_PWCFGR1_LDO_WAIT_Pos            (5)
#define PWR_PWCFGR1_LDO_WAIT_Msk            (0xFFU << PWR_PWCFGR1_LDO_WAIT_Pos)      /*!< In Stop and Deepstop mode, waiting for the flash LDO wake-up time */

/**
  * @brief PWR_PWCFGR2 register Bit definition
  */
#define PWR_PWCFGR2_LDO_FWUP_LOCK_Pos       (0)
#define PWR_PWCFGR2_LDO_FWUP_LOCK_Msk       (0x1FU << PWR_PWCFGR2_LDO_FWUP_LOCK_Pos) /*!< LDO_FWUP_WAIT configuration */
#define PWR_PWCFGR2_LDO_FWUP_Pos            (5)
#define PWR_PWCFGR2_LDO_FWUP_Msk            (0xFFU << PWR_PWCFGR2_LDO_FWUP_Pos)      /*!< In Stop and Deepstop mode, waiting for the flash LDO wake-up time */

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
