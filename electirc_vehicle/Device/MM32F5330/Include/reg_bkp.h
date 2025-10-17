/***********************************************************************************************************************
    @file     reg_bkp.h
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

#ifndef __REG_BKP_H
#define __REG_BKP_H

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
  * @brief BKP Base Address Definition
  */
#define BKP_BASE                        (APB1PERIPH_BASE + 0x2840) /*!< Base Address: 0x40002840 */

/**
  * @brief BKP Register Structure Definition
  */
#define BKP_NUMBER                      20

typedef struct
{
    __IO uint32_t RTCCR;               /*!< RTC clock calibration register,        offset: 0x00 */
    __IO uint32_t CR;                  /*!< BKP control register,                  offset: 0x04 */
    __IO uint32_t CSR;                 /*!< BKP control/status register,           offset: 0x08 */
    __IO uint32_t RESERVED0;           /*!< Reserved,                              offset: 0x0C */
    __IO uint32_t DRn[20];             /*!< BKP data register 1 ~ 20               offset: 0x10~0x5C */
} BKP_TypeDef;

/**
  * @brief BKP type pointer Definition
  */
#define BKP                             ((BKP_TypeDef *)BKP_BASE)

/**
  * @brief BKP_RTCCR Register Bit Definition
  */
#define BKP_RTCCR_CAL_Pos               (0)
#define BKP_RTCCR_CAL_Msk               (0x7FU << BKP_RTCCR_CAL_Pos)  /*!< Calibration value */
#define BKP_RTCCR_CCO_Pos               (7)
#define BKP_RTCCR_CCO                   (0x01U << BKP_RTCCR_CCO_Pos)  /*!< Calibration Clock Output */
#define BKP_RTCCR_ASOE_Pos              (8)
#define BKP_RTCCR_ASOE                  (0x01U << BKP_RTCCR_ASOE_Pos) /*!< Alarm or Second Output Enable */
#define BKP_RTCCR_ASOS_Pos              (9)
#define BKP_RTCCR_ASOS                  (0x01U << BKP_RTCCR_ASOS_Pos) /*!< Alarm or Second Output Selection */

/**
  * @brief BKP_CR Register Bit Definition
  */
#define BKP_CR_TPE_Pos                  (0)
#define BKP_CR_TPE                      (0x01U << BKP_CR_TPE_Pos)  /*!< TAMPER pin enable */
#define BKP_CR_TPAL_Pos                 (1)
#define BKP_CR_TPAL_Msk                 (0x01U << BKP_CR_TPAL_Pos) /*!< TAMPER pin active level */

/**
  * @brief BKP_CSR Register Bit Definition
  */
#define BKP_CSR_CTE_Pos                 (0)
#define BKP_CSR_CTE                     (0x01U << BKP_CSR_CTE_Pos)  /*!< Clear Tamper event */
#define BKP_CSR_CTI_Pos                 (1)
#define BKP_CSR_CTI                     (0x01U << BKP_CSR_CTI_Pos)  /*!< Clear Tamper Interrupt */
#define BKP_CSR_TPIE_Pos                (2)
#define BKP_CSR_TPIE                    (0x01U << BKP_CSR_TPIE_Pos) /*!< TAMPER Pin interrupt enable */
#define BKP_CSR_TEF_Pos                 (8)
#define BKP_CSR_TEF_Msk                 (0x01U << BKP_CSR_TEF_Pos)  /*!< Tamper Event Flag */
#define BKP_CSR_TIF_Pos                 (9)
#define BKP_CSR_TIF_Msk                 (0x01U << BKP_CSR_TIF_Pos)  /*!< Tamper Interrupt Flag */

/**
  * @brief BKP_DR1 Register Bit Definition
  */
#define BKP_DR1_BKP_Pos                 (0)
#define BKP_DR1_BKP                     (0xFFFFU << BKP_DR1_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR2 Register Bit Definition
  */
#define BKP_DR2_BKP_Pos                 (0)
#define BKP_DR2_BKP                     (0xFFFFU << BKP_DR2_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR3 Register Bit Definition
  */
#define BKP_DR3_BKP_Pos                 (0)
#define BKP_DR3_BKP                     (0xFFFFU << BKP_DR3_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR4 Register Bit Definition
  */
#define BKP_DR4_BKP_Pos                 (0)
#define BKP_DR4_BKP                     (0xFFFFU << BKP_DR4_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR5 Register Bit Definition
  */
#define BKP_DR5_BKP_Pos                 (0)
#define BKP_DR5_BKP                     (0xFFFFU << BKP_DR5_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR6 Register Bit Definition
  */
#define BKP_DR6_BKP_Pos                 (0)
#define BKP_DR6_BKP                     (0xFFFFU << BKP_DR6_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR7 Register Bit Definition
  */
#define BKP_DR7_BKP_Pos                 (0)
#define BKP_DR7_BKP                     (0xFFFFU << BKP_DR7_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR8 Register Bit Definition
  */
#define BKP_DR8_BKP_Pos                 (0)
#define BKP_DR8_BKP                     (0xFFFFU << BKP_DR8_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR9 Register Bit Definition
  */
#define BKP_DR9_BKP_Pos                 (0)
#define BKP_DR9_BKP                     (0xFFFFU << BKP_DR9_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR10 Register Bit Definition
  */
#define BKP_DR10_BKP_Pos                (0)
#define BKP_DR10_BKP                    (0xFFFFU << BKP_DR10_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR11 Register Bit Definition
  */
#define BKP_DR11_BKP_Pos                (0)
#define BKP_DR11_BKP                    (0xFFFFU << BKP_DR11_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR12 Register Bit Definition
  */
#define BKP_DR12_BKP_Pos                (0)
#define BKP_DR12_BKP                    (0xFFFFU << BKP_DR12_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR13 Register Bit Definition
  */
#define BKP_DR13_BKP_Pos                (0)
#define BKP_DR13_BKP                    (0xFFFFU << BKP_DR13_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR14 Register Bit Definition
  */
#define BKP_DR14_BKP_Pos                (0)
#define BKP_DR14_BKP                    (0xFFFFU << BKP_DR14_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR15 Register Bit Definition
  */
#define BKP_DR15_BKP_Pos                (0)
#define BKP_DR15_BKP                    (0xFFFFU << BKP_DR15_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR16 Register Bit Definition
  */
#define BKP_DR16_BKP_Pos                (0)
#define BKP_DR16_BKP                    (0xFFFFU << BKP_DR16_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR17 Register Bit Definition
  */
#define BKP_DR17_BKP_Pos                (0)
#define BKP_DR17_BKP                    (0xFFFFU << BKP_DR17_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR18 Register Bit Definition
  */
#define BKP_DR18_BKP_Pos                (0)
#define BKP_DR18_BKP                    (0xFFFFU << BKP_DR18_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR19 Register Bit Definition
  */
#define BKP_DR19_BKP_Pos                (0)
#define BKP_DR19_BKP                    (0xFFFFU << BKP_DR19_BKP_Pos) /*!< Backup data */

/**
  * @brief BKP_DR20 Register Bit Definition
  */
#define BKP_DR20_BKP_Pos                (0)
#define BKP_DR20_BKP                    (0xFFFFU << BKP_DR20_BKP_Pos) /*!< Backup data */

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

