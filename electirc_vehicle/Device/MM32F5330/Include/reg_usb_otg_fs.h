/***********************************************************************************************************************
    @file     reg_usb_otg_fs.h
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

#ifndef __REG_USB_OTG_FS_H
#define __REG_USB_OTG_FS_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

/**
  * @brief USB Base Address Definition
  */
#define USB_OTG_FS_BASE                 (AHB2PERIPH_BASE + 0x0000) /*!< Base Address: 0x50000000 */

/**
  * @brief USB Register Structure Definition
  */
typedef struct
{
    __IO uint32_t RESERVED0x00[4];     /*!< Reserved                                       offset: 0x00 */
    __IO uint32_t OTG_ISTAT;           /*!< OTG Interrupt Status Register                  offset: 0x10 */
    __IO uint32_t OTG_ICTRL;           /*!< OTG Interrupt Control Register                 offset: 0x14 */
    __IO uint32_t OTG_STAT;            /*!< OTG Status Register                            offset: 0x18 */
    __IO uint32_t OTG_CTRL;            /*!< OTG Control register                           offset: 0x1C */
    __IO uint32_t RESERVED0x20[24];    /*!< Reserved                                       offset: 0x20 */
    __IO uint32_t INT_STAT;            /*!< Interrupt status register                      offset: 0x80 */
    __IO uint32_t INT_ENB;             /*!< Interrupt enable register                      offset: 0x84 */
    __IO uint32_t ERR_STAT;            /*!< Error interrupt status register                offset: 0x88 */
    __IO uint32_t ERR_ENB;             /*!< Error interrupt enable register                offset: 0x8C */
    __IO uint32_t STAT;                /*!< Status register                                offset: 0x90 */
    __IO uint32_t CTL;                 /*!< Control register                               offset: 0x94 */
    __IO uint32_t ADDR;                /*!< Address register                               offset: 0x98 */
    __IO uint32_t BDT_PAGE_01;         /*!< BDT page register 1                            offset: 0x9C */
    __IO uint32_t FRM_NUML;            /*!< Frame number register                          offset: 0xA0 */
    __IO uint32_t FRM_NUMH;            /*!< Frame number register                          offset: 0xA4 */
    __IO uint32_t TOKEN;               /*!< Token register                                 offset: 0xA8 */
    __IO uint32_t SOF_THLD;            /*!< SOF threshold register                         offset: 0xAC */
    __IO uint32_t BDT_PAGE_02;         /*!< BDT page register 2                            offset: 0xB0 */
    __IO uint32_t BDT_PAGE_03;         /*!< BDT page register 3                            offset: 0xB4 */
    __IO uint32_t RESERVED0xB8;        /*!< Reserved                                       offset: 0xB8 */
    __IO uint32_t RESERVED0xBC;        /*!< Reserved                                       offset: 0xBC */
    __IO uint32_t EP_CTL[16];          /*!< Endpoint control register                      offset: 0xC0 */
    __IO uint32_t USB_CTRL;            /*!< USB Control Register                           offset: 0x100 */
} USB_OTG_FS_TypeDef;

/**
  * @brief USBD type pointer Definition
  */
#define USB_OTG_FS                              ((USB_OTG_FS_TypeDef *)USB_OTG_FS_BASE)

/**
  * @brief OTG_FS_OTG_ISTAT Register Bit Definition
  */
#define OTG_FS_OTG_ISTAT_A_VBUS_VLD_CHG_Pos     (0)
#define OTG_FS_OTG_ISTAT_A_VBUS_VLD_CHG_Msk     (0x01U << OTG_FS_OTG_ISTAT_A_VBUS_VLD_CHG_Pos)
#define OTG_FS_OTG_ISTAT_B_SESS_END_CHG_Pos     (2)
#define OTG_FS_OTG_ISTAT_B_SESS_END_CHG_Msk     (0x01U << OTG_FS_OTG_ISTAT_B_SESS_END_CHG_Pos)
#define OTG_FS_OTG_ISTAT_SESS_VLD_CHG_Pos       (3)
#define OTG_FS_OTG_ISTAT_SESS_VLD_CHG_Msk       (0x01U << OTG_FS_OTG_ISTAT_SESS_VLD_CHG_Pos)
#define OTG_FS_OTG_ISTAT_LINE_STATE_CHG_Pos     (5)
#define OTG_FS_OTG_ISTAT_LINE_STATE_CHG_Msk     (0x01U << OTG_FS_OTG_ISTAT_LINE_STATE_CHG_Pos)
#define OTG_FS_OTG_ISTAT_1_MSEC_Pos             (6)
#define OTG_FS_OTG_ISTAT_1_MSEC_Msk             (0x01U << OTG_FS_OTG_ISTAT_1_MSEC_Pos)
#define OTG_FS_OTG_ISTAT_ID_CHG_Pos             (7)
#define OTG_FS_OTG_ISTAT_ID_CHG_Msk             (0x01U << OTG_FS_OTG_ISTAT_ID_CHG_Pos)

/**
  * @brief OTG_FS_OTG_ICTRL Register Bit Definition
  */
#define OTG_FS_OTG_ICTRL_A_VBUS_VLD_EN_Pos      (0)
#define OTG_FS_OTG_ICTRL_A_VBUS_VLD_EN_Msk      (0x01U << OTG_FS_OTG_ICTRL_A_VBUS_VLD_EN_Pos)
#define OTG_FS_OTG_ICTRL_B_SESS_END_EN_Pos      (2)
#define OTG_FS_OTG_ICTRL_B_SESS_END_EN_Msk      (0x01U << OTG_FS_OTG_ICTRL_B_SESS_END_EN_Pos)
#define OTG_FS_OTG_ICTRL_SESS_VLD_EN_Pos        (3)
#define OTG_FS_OTG_ICTRL_SESS_VLD_EN_Msk        (0x01U << OTG_FS_OTG_ICTRL_SESS_VLD_EN_Pos)
#define OTG_FS_OTG_ICTRL_LINE_STATE_EN_Pos      (5)
#define OTG_FS_OTG_ICTRL_LINE_STATE_EN_Msk      (0x01U << OTG_FS_OTG_ICTRL_LINE_STATE_EN_Pos)
#define OTG_FS_OTG_ICTRL_1_MSEC_EN_Pos          (6)
#define OTG_FS_OTG_ICTRL_1_MSEC_EN_Msk          (0x01U << OTG_FS_OTG_ICTRL_1_MSEC_EN_Pos)
#define OTG_FS_OTG_ICTRL_ID_EN_Pos              (7)
#define OTG_FS_OTG_ICTRL_ID_EN_Msk              (0x01U << OTG_FS_OTG_ICTRL_ID_EN_Pos)

/**
  * @brief OTG_FS_OTG_STAT Register Bit Definition
  */
#define OTG_FS_OTG_STAT_A_VBUS_VLD_Pos          (0)
#define OTG_FS_OTG_STAT_A_VBUS_VLD_Msk          (0x01U << OTG_FS_OTG_STAT_A_VBUS_VLD_Pos)
#define OTG_FS_OTG_STAT_B_SESS_END_Pos          (2)
#define OTG_FS_OTG_STAT_B_SESS_END_Msk          (0x01U << OTG_FS_OTG_STAT_B_SESS_END_Pos)
#define OTG_FS_OTG_STAT_SESS_VLD_Pos            (3)
#define OTG_FS_OTG_STAT_SESS_VLD_Msk            (0x01U << OTG_FS_OTG_STAT_SESS_VLD_Pos)
#define OTG_FS_OTG_STAT_LINESTATE_STABLE_Pos    (5)
#define OTG_FS_OTG_STAT_LINESTATE_STABLE_Msk    (0x01U << OTG_FS_OTG_STAT_LINESTATE_STABLE_Pos)
#define OTG_FS_OTG_STAT_ID_Pos                  (7)
#define OTG_FS_OTG_STAT_ID_Msk                  (0x01U << OTG_FS_OTG_STAT_ID_Pos)

/**
  * @brief OTG_FS_OTG_CTRL Register Bit Definition
  */
#define OTG_FS_OTG_CTRL_VBUS_DSCHG_Pos          (0)
#define OTG_FS_OTG_CTRL_VBUS_DSCHG_Msk          (0x01U << OTG_FS_OTG_CTRL_VBUS_DSCHG_Pos)
#define OTG_FS_OTG_CTRL_VBUS_CHG_Pos            (1)
#define OTG_FS_OTG_CTRL_VBUS_CHG_Msk            (0x01U << OTG_FS_OTG_CTRL_VBUS_CHG_Pos)
#define OTG_FS_OTG_CTRL_OTG_EN_Pos              (2)
#define OTG_FS_OTG_CTRL_OTG_EN_Msk              (0x01U << OTG_FS_OTG_CTRL_OTG_EN_Pos)
#define OTG_FS_OTG_CTRL_VBUS_ON_Pos             (3)
#define OTG_FS_OTG_CTRL_VBUS_ON_Msk             (0x01U << OTG_FS_OTG_CTRL_VBUS_ON_Pos)
#define OTG_FS_OTG_CTRL_DM_LOW_Pos              (4)
#define OTG_FS_OTG_CTRL_DM_LOW_Msk              (0x01U << OTG_FS_OTG_CTRL_DM_LOW_Pos)
#define OTG_FS_OTG_CTRL_DP_LOW_Pos              (5)
#define OTG_FS_OTG_CTRL_DP_LOW_Msk              (0x01U << OTG_FS_OTG_CTRL_DP_LOW_Pos)
#define OTG_FS_OTG_CTRL_DM_HIGH_Pos             (6)
#define OTG_FS_OTG_CTRL_DM_HIGH_Msk             (0x01U << OTG_FS_OTG_CTRL_DM_HIGH_Pos)
#define OTG_FS_OTG_CTRL_DP_HIGH_Pos             (7)
#define OTG_FS_OTG_CTRL_DP_HIGH_Msk             (0x01U << OTG_FS_OTG_CTRL_DP_HIGH_Pos)

/**
  * @brief OTG_FS_INT_STAT Register Bit Definition
  */
#define OTG_FS_INT_STAT_USB_RST_Pos             (0)
#define OTG_FS_INT_STAT_USB_RST_Msk             (0x01U << OTG_FS_INT_STAT_USB_RST_Pos)
#define OTG_FS_INT_STAT_ERROR_Pos               (1)
#define OTG_FS_INT_STAT_ERROR_Msk               (0x01U << OTG_FS_INT_STAT_ERROR_Pos)
#define OTG_FS_INT_STAT_SOF_TOK_Pos             (2)
#define OTG_FS_INT_STAT_SOF_TOK_Msk             (0x01U << OTG_FS_INT_STAT_SOF_TOK_Pos)
#define OTG_FS_INT_STAT_TOK_DNE_Pos             (3)
#define OTG_FS_INT_STAT_TOK_DNE_Msk             (0x01U << OTG_FS_INT_STAT_TOK_DNE_Pos)
#define OTG_FS_INT_STAT_SLEEP_Pos               (4)
#define OTG_FS_INT_STAT_SLEEP_Msk               (0x01U << OTG_FS_INT_STAT_SLEEP_Pos)
#define OTG_FS_INT_STAT_RESUME_Pos              (5)
#define OTG_FS_INT_STAT_RESUME_Msk              (0x01U << OTG_FS_INT_STAT_RESUME_Pos)
#define OTG_FS_INT_STAT_ATTACH_Pos              (6)
#define OTG_FS_INT_STAT_ATTACH_Msk              (0x01U << OTG_FS_INT_STAT_ATTACH_Pos)
#define OTG_FS_INT_STAT_STALL_Pos               (7)
#define OTG_FS_INT_STAT_STALL_Msk               (0x01U << OTG_FS_INT_STAT_STALL_Pos)

/**
  * @brief OTG_FS_INT_ENB Register Bit Definition
  */
#define OTG_FS_INT_ENB_USB_RST_Pos              (0)
#define OTG_FS_INT_ENB_USB_RST_Msk              (0x01U << OTG_FS_INT_ENB_USB_RST_Pos)
#define OTG_FS_INT_ENB_ERROR_Pos                (1)
#define OTG_FS_INT_ENB_ERROR_Msk                (0x01U << OTG_FS_INT_ENB_ERROR_Pos)
#define OTG_FS_INT_ENB_SOF_TOK_Pos              (2)
#define OTG_FS_INT_ENB_SOF_TOK_Msk              (0x01U << OTG_FS_INT_ENB_SOF_TOK_Pos)
#define OTG_FS_INT_ENB_TOK_DNE_Pos              (3)
#define OTG_FS_INT_ENB_TOK_DNE_Msk              (0x01U << OTG_FS_INT_ENB_TOK_DNE_Pos)
#define OTG_FS_INT_ENB_SLEEP_Pos                (4)
#define OTG_FS_INT_ENB_SLEEP_Msk                (0x01U << OTG_FS_INT_ENB_SLEEP_Pos)
#define OTG_FS_INT_ENB_RESUME_Pos               (5)
#define OTG_FS_INT_ENB_RESUME_Msk               (0x01U << OTG_FS_INT_ENB_RESUME_Pos)
#define OTG_FS_INT_ENB_ATTACH_Pos               (6)
#define OTG_FS_INT_ENB_ATTACH_Msk               (0x01U << OTG_FS_INT_ENB_ATTACH_Pos)
#define OTG_FS_INT_ENB_STALL_Pos                (7)
#define OTG_FS_INT_ENB_STALL_Msk                (0x01U << OTG_FS_INT_ENB_STALL_Pos)

/**
  * @brief OTG_FS_ERR_STAT Register Bit Definition
  */
#define OTG_FS_ERR_STAT_PID_ERR_Pos             (0)
#define OTG_FS_ERR_STAT_PID_ERR_Msk             (0x01U << OTG_FS_ERR_STAT_PID_ERR_Pos)
#define OTG_FS_ERR_STAT_CRC5_EOF_Pos            (1)
#define OTG_FS_ERR_STAT_CRC5_EOF_Msk            (0x01U << OTG_FS_ERR_STAT_CRC5_EOF_Pos)
#define OTG_FS_ERR_STAT_CRC16_Pos               (2)
#define OTG_FS_ERR_STAT_CRC16_Msk               (0x01U << OTG_FS_ERR_STAT_CRC16_Pos)
#define OTG_FS_ERR_STAT_DFN8_Pos                (3)
#define OTG_FS_ERR_STAT_DFN8_Msk                (0x01U << OTG_FS_ERR_STAT_DFN8_Pos)
#define OTG_FS_ERR_STAT_BTO_ERR_Pos             (4)
#define OTG_FS_ERR_STAT_BTO_ERR_Msk             (0x01U << OTG_FS_ERR_STAT_BTO_ERR_Pos)
#define OTG_FS_ERR_STAT_DMA_ERR_Pos             (5)
#define OTG_FS_ERR_STAT_DMA_ERR_Msk             (0x01U << OTG_FS_ERR_STAT_DMA_ERR_Pos)
#define OTG_FS_ERR_STAT_BTS_ERR_Pos             (7)
#define OTG_FS_ERR_STAT_BTS_ERR_Msk             (0x01U << OTG_FS_ERR_STAT_BTS_ERR_Pos)

/**
  * @brief OTG_FS_ERR_ENB Register Bit Definition
  */
#define OTG_FS_ERR_ENB_PID_ERR_Pos              (0)
#define OTG_FS_ERR_ENB_PID_ERR_Msk              (0x01U << OTG_FS_ERR_ENB_PID_ERR_Pos)
#define OTG_FS_ERR_ENB_CRC5_EOF_Pos             (1)
#define OTG_FS_ERR_ENB_CRC5_EOF_Msk             (0x01U << OTG_FS_ERR_ENB_CRC5_EOF_Pos)
#define OTG_FS_ERR_ENB_CRC16_Pos                (2)
#define OTG_FS_ERR_ENB_CRC16_Msk                (0x01U << OTG_FS_ERR_ENB_CRC16_Pos)
#define OTG_FS_ERR_ENB_DFN8_Pos                 (3)
#define OTG_FS_ERR_ENB_DFN8_Msk                 (0x01U << OTG_FS_ERR_ENB_DFN8_Pos)
#define OTG_FS_ERR_ENB_BTO_ERR_Pos              (4)
#define OTG_FS_ERR_ENB_BTO_ERR_Msk              (0x01U << OTG_FS_ERR_ENB_BTO_ERR_Pos)
#define OTG_FS_ERR_ENB_DMA_ERR_Pos              (5)
#define OTG_FS_ERR_ENB_DMA_ERR_Msk              (0x01U << OTG_FS_ERR_ENB_DMA_ERR_Pos)
#define OTG_FS_ERR_ENB_BTS_ERR_Pos              (7)
#define OTG_FS_ERR_ENB_BTS_ERR_Msk              (0x01U << OTG_FS_ERR_ENB_BTS_ERR_Pos)

/**
  * @brief OTG_FS_STAT Register Bit Definition
  */
#define OTG_FS_STAT_ODD_Pos                     (2)
#define OTG_FS_STAT_ODD_Msk                     (0x01U << OTG_FS_STAT_ODD_Pos)
#define OTG_FS_STAT_TX_Pos                      (3)
#define OTG_FS_STAT_TX_Msk                      (0x01U << OTG_FS_STAT_TX_Pos)
#define OTG_FS_STAT_ENDP_Pos                    (4)
#define OTG_FS_STAT_ENDP_Msk                    (0x0FU << OTG_FS_STAT_ENDP_Pos)

/**
  * @brief OTG_FS_CTL Register Bit Definition
  */
#define OTG_FS_CTL_USB_EN_Pos                   (0)
#define OTG_FS_CTL_USB_EN_Msk                   (0x01U << OTG_FS_CTL_USB_EN_Pos)
#define OTG_FS_CTL_ODD_RST_Pos                  (1)
#define OTG_FS_CTL_ODD_RST_Msk                  (0x01U << OTG_FS_CTL_ODD_RST_Pos)
#define OTG_FS_CTL_RESUME_Pos                   (2)
#define OTG_FS_CTL_RESUME_Msk                   (0x01U << OTG_FS_CTL_RESUME_Pos)
#define OTG_FS_CTL_HOST_MODE_EN_Pos             (3)
#define OTG_FS_CTL_HOST_MODE_EN_Msk             (0x01U << OTG_FS_CTL_HOST_MODE_EN_Pos)
#define OTG_FS_CTL_RESET_Pos                    (4)
#define OTG_FS_CTL_RESET_Msk                    (0x01U << OTG_FS_CTL_RESET_Pos)
#define OTG_FS_CTL_TXDSUSPEND_TOKENBUSY_Pos     (5)
#define OTG_FS_CTL_TXDSUSPEND_TOKENBUSY_Msk     (0x01U << OTG_FS_CTL_TXDSUSPEND_TOKENBUSY_Pos)
#define OTG_FS_CTL_SE0_Pos                      (6)
#define OTG_FS_CTL_SE0_Msk                      (0x01U << OTG_FS_CTL_SE0_Pos)
#define OTG_FS_CTL_JSTATE_Pos                   (7)
#define OTG_FS_CTL_JSTATE_Msk                   (0x01U << OTG_FS_CTL_JSTATE_Pos)

/**
  * @brief OTG_FS_ADDR Register Bit Definition
  */
#define OTG_FS_ADDR_ADDR_Pos                    (0)
#define OTG_FS_ADDR_ADDR_Msk                    (0x7FU << OTG_FS_ADDR_ADDR_Pos)
#define OTG_FS_ADDR_LS_EN_Pos                   (7)
#define OTG_FS_ADDR_LS_EN_Msk                   (0x01U << OTG_FS_ADDR_LS_EN_Pos)

/**
  * @brief OTG_FS_BDT_PAGE_01 Register Bit Definition
  */
#define OTG_FS_BDT_PAGE_01_BDT_BA_Pos           (1)
#define OTG_FS_BDT_PAGE_01_BDT_BA_Msk           (0x7FU << OTG_FS_BDT_PAGE_01_BDT_BA_15_9_Pos)

/**
  * @brief OTG_FS_FRM_NUML Register Bit Definition
  */
#define OTG_FS_FRM_NUML_FRM_Pos                 (0)
#define OTG_FS_FRM_NUML_FRM_Msk                 (0xFFU << OTG_FS_FRM_NUML_FRM_Pos)

/**
  * @brief OTG_FS_FRM_NUMH Register Bit Definition
  */
#define OTG_FS_FRM_NUMH_FRM_Pos                 (0)
#define OTG_FS_FRM_NUMH_FRM_Msk                 (0x07U << OTG_FS_FRM_NUMH_FRM_Pos)

/**
  * @brief OTG_FS_TOKEN Register Bit Definition
  */
#define OTG_FS_TOKEN_TOKEN_ENDPT_Pos            (0)
#define OTG_FS_TOKEN_TOKEN_ENDPT_Msk            (0x0FU << OTG_FS_TOKEN_TOKEN_ENDPT_Pos)
#define OTG_FS_TOKEN_TOKEN_PID_Pos              (4)
#define OTG_FS_TOKEN_TOKEN_PID_Msk              (0x0FU << OTG_FS_TOKEN_TOKEN_PID_Pos)

/**
  * @brief OTG_FS_SOF_THLD Register Bit Definition
  */
#define OTG_FS_SOF_THLD_CNT_Pos                 (0)
#define OTG_FS_SOF_THLD_CNT_Msk                 (0xFFU << OTG_FS_SOF_THLD_CNT_Pos)

/**
  * @brief OTG_FS_BDT_PAGE_02 Register Bit Definition
  */
#define OTG_FS_BDT_PAGE_02_BDT_BA_Pos           (0)
#define OTG_FS_BDT_PAGE_02_BDT_BA_Msk           (0xFFU << OTG_FS_BDT_PAGE_02_BDT_BA_Pos)

/**
  * @brief OTG_FS_BDT_PAGE_03 Register Bit Definition
  */
#define OTG_FS_BDT_PAGE_03_BDT_BA_Pos           (0)
#define OTG_FS_BDT_PAGE_03_BDT_BA_Msk           (0xFFU << OTG_FS_BDT_PAGE_03_BDT_BA_Pos)

/**
  * @brief OTG_FS_EP_CTL Register Bit Definition
  */
#define OTG_FS_EP_CTL_EP_HSHK_Pos               (0)
#define OTG_FS_EP_CTL_EP_HSHK_Msk               (0x01U << OTG_FS_EP_CTL_EP_HSHK_Pos)
#define OTG_FS_EP_CTL_EP_STALL_Pos              (1)
#define OTG_FS_EP_CTL_EP_STALL_Msk              (0x01U << OTG_FS_EP_CTL_EP_STALL_Pos)

#define OTG_FS_EP_CTL_EP_TX_EN_Pos              (2)
#define OTG_FS_EP_CTL_EP_TX_EN_Msk              (0x01U << OTG_FS_EP_CTL_EP_TX_EN_Pos)
#define OTG_FS_EP_CTL_EP_RX_EN_Pos              (3)
#define OTG_FS_EP_CTL_EP_RX_EN_Msk              (0x01U << OTG_FS_EP_CTL_EP_RX_EN_Pos)
#define OTG_FS_EP_CTL_EP_CTL_DIS_Pos            (4)
#define OTG_FS_EP_CTL_EP_CTL_DIS_Msk            (0x01U << OTG_FS_EP_CTL_EP_CTL_DIS_Pos)

#define OTG_FS_EP_CTL_RETRY_DIS_Pos             (6)
#define OTG_FS_EP_CTL_RETRY_DIS_Msk             (0x01U << OTG_FS_EP_CTL_RETRY_DIS_Pos)
#define OTG_FS_EP_CTL_HOST_WO_HUB_Pos           (7)
#define OTG_FS_EP_CTL_HOST_WO_HUB_Msk           (0x01U << OTG_FS_EP_CTL_HOST_WO_HUB_Pos)

/**
  * @brief  USB_FS_USB_CTRL Register Descriptor Bit Definition
  */
#define USB_FS_USB_CTRL_SUSPE_Pos               (7)
#define USB_FS_USB_CTRL_SUSPE_Msk               (0x01U << USB_FS_USB_CTRL_SUSPE_Pos)

/** --------------------------------------------------------------------------*/
#endif /* __REG_USB_OTG_FS_H --------------------------------------------------*/
/** --------------------------------------------------------------------------*/

