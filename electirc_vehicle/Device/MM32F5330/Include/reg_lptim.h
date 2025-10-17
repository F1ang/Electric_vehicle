/***********************************************************************************************************************
    @file     reg_lptim.h
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

#ifndef __REG_LPTIM_H
#define __REG_LPTIM_H

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
  * @brief LPTIM Base Address Definition
  */
#define LPTIM_BASE                      (APB2PERIPH_BASE + 0xD000)

/**
  * @brief LPTIM Register Structure Definition
  */
typedef struct
{
    __IO uint32_t  CFG;                /*!< configuration register                                       offset: 0x00 */
    __IO uint32_t  IE;                 /*!< interrupt enable register                                    offset: 0x04 */
    __IO uint32_t  IF;                 /*!< interrupt flag register                                      offset: 0x08 */
    __IO uint32_t  CTRL;               /*!< control register                                             offset: 0x0C */
    __IO uint32_t  CNT;                /*!< count register                                               offset: 0x10 */
    __IO uint32_t  CMP;                /*!< compare value register                                       offset: 0x14 */
    __IO uint32_t  TARGET;             /*!< target value register                                        offset: 0x18 */
} LPTIM_TypeDef;

/**
  * @brief LPTIM type pointer Definition
  */
#define LPTIM1                          ((LPTIM_TypeDef *)LPTIM_BASE)

/**
  * @brief LPTCFG Register Bit Definition
  */
#define LPTCFG_MODE_Pos                 (0)
#define LPTCFG_MODE_Msk                 (0x01U << LPTCFG_MODE_Pos)     /*!< Counting mode */
#define LPTCFG_TMODE_Pos                (1)
#define LPTCFG_TMODE_Msk                (0x03U << LPTCFG_TMODE_Pos)    /*!< Workting mode */
#define LPTCFG_PWM_Pos                  (3)
#define LPTCFG_PWM_Msk                  (0x01U << LPTCFG_PWM_Pos)      /*!< PWM mode output */
#define LPTCFG_POLARITY_Pos             (4)
#define LPTCFG_POLARITY_Msk             (0x01U << LPTCFG_POLARITY_Pos) /*!< Compare waveform polarity select */
#define LPTCFG_TRIGSEL_Pos              (5)
#define LPTCFG_TRIGSEL_Msk              (0x01U << LPTCFG_TRIGSEL_Pos)  /*!< Trigger input source selection */
#define LPTCFG_TRIGCFG_Pos              (6)
#define LPTCFG_TRIGCFG_Msk              (0x03U << LPTCFG_TRIGCFG_Pos)
#define LPTCFG_TRIGCFG_Rise             (0x00U << LPTCFG_TRIGCFG_Pos)  /*!< External input signal rising edge trigger */
#define LPTCFG_TRIGCFG_Fall             (0x01U << LPTCFG_TRIGCFG_Pos)  /*!< External input signal falling edge trigger */
#define LPTCFG_TRIGCFG_Both             (0x02U << LPTCFG_TRIGCFG_Pos)  /*!< External input signal rising and falling edge trigger */

#define LPTCFG_DIVSEL_Pos               (8)
#define LPTCFG_DIVSEL_Msk               (0x07U << LPTCFG_DIVSEL_Pos)

#define LPTCFG_FLTEN_Pos                (15)
#define LPTCFG_FLTEN_Msk                (0x01U << LPTCFG_FLTEN_Pos) /*!< Input signal filtering enable */

/**
  * @brief LPTIE Register Bit Definition
  */
#define LPTIE_OVIE_Pos                  (0)
#define LPTIE_OVIE_Msk                  (0x01U << LPTIE_OVIE_Pos)   /*!< Counter overflow interrupt enable */
#define LPTIE_TRIGIE_Pos                (1)
#define LPTIE_TRIGIE_Msk                (0x01U << LPTIE_TRIGIE_Pos) /*!< Counter value and comparison value match interrupt enable */
#define LPTIE_COMPIE_Pos                (2)
#define LPTIE_COMPIE_Msk                (0x01U << LPTIE_COMPIE_Pos) /*!< External trigger arrival interrupt enable */

/**
  * @brief LPTIF Register Bit Definition
  */
#define LPTIF_OVIF_Pos                  (0)
#define LPTIF_OVIF_Msk                  (0x01U << LPTIF_OVIF_Pos)   /*!< Counter overflow interrupt flag */
#define LPTIF_TRIGIF_Pos                (1)
#define LPTIF_TRIGIF_Msk                (0x01U << LPTIF_TRIGIF_Pos) /*!< Counter value and comparison value match interrupt flag */
#define LPTIF_COMPIF_Pos                (2)
#define LPTIF_COMPIF_Msk                (0x01U << LPTIF_COMPIF_Pos) /*!< External trigger arrival interrupt flag */

/**
  * @brief LPTCTRL Register Bit Definition
  */
#define LPTCTRL_LPTEN_Pos               (0)
#define LPTCTRL_LPTEN_Msk               (0x01U << LPTCTRL_LPTEN_Pos) /*!< Enable counter count */

/**
  * @brief LPTCNT Register Bit Definition
  */
#define LPTCNT_CNT_Pos                  (0)
#define LPTCNT_CNT_Msk                  (0xFFFFU << LPTCNT_CNT_Pos) /*!< counter count value */

/**
  * @brief LPTCMP Register Bit Definition
  */
#define LPTCMP_COMPARE_REG_Pos          (0)
#define LPTCMP_COMPARE_REG_Msk          (0xFFFFU << LPTCMP_COMPARE_REG_Pos) /*!< compare value */

/**
  * @brief LPTTARGET Register Bit Definition
  */
#define LPTTARGET_TARGET_REG_Pos        (0)
#define LPTTARGET_TARGET_REG_Msk        (0xFFFFU << LPTTARGET_TARGET_REG_Pos) /*!< target value */

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

