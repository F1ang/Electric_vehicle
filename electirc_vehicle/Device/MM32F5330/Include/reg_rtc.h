/***********************************************************************************************************************
    @file     reg_rtc.h
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
#ifndef __REG_RTC_H
#define __REG_RTC_H

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
  * @brief RTC Base Address Definition
  */
#define RTC_BASE                        (APB1PERIPH_BASE + 0x2800) /*!< Base Address: 0x40002800 */

/**
  * @brief RTC Registers Structure Definition
  */
#define COMPITABLE_REG

typedef struct
{
    __IO uint32_t CRH;                 /*!< RTC control register high,             offset:0x00 */
    __IO uint32_t CRL;                 /*!< RTC control register low,              offset:0x04 */
    __IO uint32_t PRLH;                /*!< RTC prescaler load register high,      offset:0x08 */
    __IO uint32_t PRLL;                /*!< RTC prescaler load register low,       offset:0x0C */
    __IO uint32_t DIVH;                /*!< RTC prescaler divider register high,   offset:0x10 */
    __IO uint32_t DIVL;                /*!< RTC prescaler divider register low,    offset:0x14 */
    __IO uint32_t CNTH;                /*!< RTC counter register high,             offset:0x18 */
    __IO uint32_t CNTL;                /*!< RTC counter register low,              offset:0x1C */
    __IO uint32_t ALRH;                /*!< RTC alarm register high,               offset:0x20 */
    __IO uint32_t ALRL;                /*!< RTC alarm register low,                offset:0x24 */
    __IO uint32_t MSRH;                /*!< RTC millisecond register high,         offset:0x28 */
    __IO uint32_t MSRL;                /*!< RTC millisecond register low,          offset:0x2C */
    __IO uint32_t RESERVED0x30[3];     /*!< Reserved,                              offset:0x30~0x38 */
    __IO uint32_t LSE_CFG;             /*!< RTC LSE configuration register,        offset:0x3C */
} RTC_TypeDef;

/**
  * @brief RTC type pointer Definition
  */
#define RTC                             ((RTC_TypeDef *)RTC_BASE)

/**
  * @brief RTC_CRH Register Bit Definition
  */
#define RTC_CRH_SECIE_Pos               (0)
#define RTC_CRH_SECIE_Msk               (0x01U << RTC_CRH_SECIE_Pos) /*!< Second Interrupt Enable */
#define RTC_CRH_ALRIE_Pos               (1)
#define RTC_CRH_ALRIE_Msk               (0x01U << RTC_CRH_ALRIE_Pos) /*!< Alarm Interrupt Enable */
#define RTC_CRH_OWIE_Pos                (2)
#define RTC_CRH_OWIE_Msk                (0x01U << RTC_CRH_OWIE_Pos)  /*!< Overflow Interrupt Enable */

/**
  * @brief RTC_CRL Register Bit Definition
  */
#define RTC_CRL_SECF_Pos                (0)
#define RTC_CRL_SECF_Msk                (0x01U << RTC_CRL_SECF_Pos)  /*!< Second Flag */
#define RTC_CRL_ALRF_Pos                (1)
#define RTC_CRL_ALRF_Msk                (0x01U << RTC_CRL_ALRF_Pos)  /*!< Alarm Flag */
#define RTC_CRL_OWF_Pos                 (2)
#define RTC_CRL_OWF_MsK                 (0x01U << RTC_CRL_OWF_Pos)   /*!< Overflow Flag */
#define RTC_CRL_RSF_Pos                 (3)
#define RTC_CRL_RSF_Msk                 (0x01U << RTC_CRL_RSF_Pos)   /*!< Registers Synchronized Flag */
#define RTC_CRL_CNF_Pos                 (4)
#define RTC_CRL_CNF_Msk                 (0x01U << RTC_CRL_CNF_Pos)   /*!< Configuration Flag */
#define RTC_CRL_RTOFF_Pos               (5)
#define RTC_CRL_RTOFF_Msk               (0x01U << RTC_CRL_RTOFF_Pos) /*!< RTC Operation OFF */
#define RTC_CRL_ALPEN_Pos               (6)
#define RTC_CRL_ALPEN_Msk               (0x01U << RTC_CRL_ALPEN_Pos) /*!< RTC Alarm Loop Enable */

/**
  * @brief RTC_PRLH Register Bit Definition
  */
#define RTC_PRLH_PRL_Pos                (0)
#define RTC_PRLH_PRL_Msk                (0x0FU << RTC_PRLH_PRL_Pos) /*!< RTC Prescaler Reload Value High */

/**
  * @brief RTC_PRLL Register Bit Definition
  */
#define RTC_PRLL_PRL_Pos                (0)
#define RTC_PRLL_PRL_Msk                (0xFFFFU << RTC_PRLL_PRL_Pos) /*!< RTC Prescaler Reload Value Low */

/**
  * @brief RTC_DIVH Register Bit Definition
  */
#define RTC_DIVH_DIV_Pos                (0)
#define RTC_DIVH_DIV_Msk                (0x0FU << RTC_DIVH_DIV_Pos) /*!< RTC Clock Divider High */

/**
  * @brief RTC_DIVL Register Bit Definition
  */
#define RTC_DIVL_DIV_Pos                (0)
#define RTC_DIVL_DIV_Msk                (0xFFFFU << RTC_DIVL_DIV_Pos) /*!< RTC Clock Divider Low */

/**
  * @brief RTC_CNTH Register Bit Definition
  */
#define RTC_CNTH_CNT_Pos                (0)
#define RTC_CNTH_CNT_Msk                (0xFFFFU << RTC_CNTH_CNT_Pos) /*!< RTC Counter High */

/**
  * @brief RTC_CNTL Register Bit Definition
  */
#define RTC_CNTL_CNT_Pos                (0)
#define RTC_CNTL_CNT_Msk                (0xFFFFU << RTC_CNTL_CNT_Pos) /*!< RTC Counter Low */

/**
  * @brief RTC_ALRH Register Bit Definition
  */
#define RTC_ALRH_ALR_Pos                (0)
#define RTC_ALRH_ALR_Msk                (0xFFFFU << RTC_ALRH_ALR_Pos) /*!< RTC Alarm High */

/**
  * @brief RTC_ALRL Register Bit Definition
  */
#define RTC_ALRL_ALR_Pos                (0)
#define RTC_ALRL_ALR_Msk                (0xFFFFU << RTC_ALRL_ALR_Pos) /*!< RTC Alarm Low */

/**
  * @brief RTC_MSRH Register Bit Definition
  */
#define RTC_MSRH_MSR_Pos                (0)
#define RTC_MSRH_MSR_Msk                (0x0FU << RTC_MSRH_MSR_Pos) /*!< RTC Msec High */

/**
  * @brief RTC_MSRL Register Bit Definition
  */
#define RTC_MSRL_MSR_Pos                (0)
#define RTC_MSRL_MSR_Msk                (0xFFFFU << RTC_MSRL_MSR_Pos) /*!< RTC Msec Low */

/**
  * @brief RTC_LSE_CFG Register Bit Definition
  */
#define RTC_LSE_CFG_LSE_TC_Pos          (0)
#define RTC_LSE_CFG_LSE_TC_Msk          (0x03U << RTC_LSE_CFG_LSE_TC_Pos)      /*!< LSE test compensation mode */
#define RTC_LSE_CFG_LSE_OUTENH_Pos      (2)
#define RTC_LSE_CFG_LSE_OUTENH_Msk      (0x01U << RTC_LSE_CFG_LSE_OUTENH_Pos)  /*!< LSE enhanced output mode */

#define RTC_LSE_CFG_LSE_DR_Pos          (4)
#define RTC_LSE_CFG_LSE_DR_Msk          (0x03U << RTC_LSE_CFG_LSE_DR_Pos)      /*!< LSE drive capability selection */
#define RTC_LSE_CFG_LSE_RFB_SEL_Pos     (6)
#define RTC_LSE_CFG_LSE_RFB_SEL_Msk     (0x03U << RTC_LSE_CFG_LSE_RFB_SEL_Pos) /*!< Feedback resistor selection */
#define RTC_LSE_CFG_LSE_IB_Pos          (8)
#define RTC_LSE_CFG_LSE_IB_Msk          (0x03U << RTC_LSE_CFG_LSE_IB_Pos)      /*!< LSE bias current adjustment */
#define RTC_LSE_CFG_LSE_AAC_Pos         (10)
#define RTC_LSE_CFG_LSE_AAC_Msk         (0x01U << RTC_LSE_CFG_LSE_AAC_Pos)     /*!< LSE oscillation amplitude select bit */

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

