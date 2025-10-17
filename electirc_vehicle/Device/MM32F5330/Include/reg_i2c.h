/***********************************************************************************************************************
    @file     reg_i2c.h
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

#ifndef __REG_I2C_H
#define __REG_I2C_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

#if defined(__CC_ARM)
#pragma anon_unions
#endif

/**
  * @brief I2C Base Address Definition
  */
#define I2C1_BASE                       (APB1PERIPH_BASE + 0x5400) /*!< Base Address: 0x40005400 */
#define I2C2_BASE                       (APB1PERIPH_BASE + 0x5800) /*!< Base Address: 0x40005800 */

/**
  * @brief I2C Register Structure Definition
  */
typedef struct
{
    __IO uint32_t CR;                  /*!< Control Register                               offset: 0x00 */
    __IO uint32_t TAR;                 /*!< Target Address Register                        offset: 0x04 */
    __IO uint32_t SAR;                 /*!< Slave Address Register                         offset: 0x08 */
    __IO uint32_t RESERVED;            /*!< Reserved Register                              offset: 0x0C */
    __IO uint32_t DR;                  /*!< Data Command Register                          offset: 0x10 */
    __IO uint32_t SSHR;                /*!< SCL High Period Count for Std. Speed Register  offset: 0x14 */
    __IO uint32_t SSLR;                /*!< SCL Low Period Count for Std. Speed Register   offset: 0x18 */
    __IO uint32_t FSHR;                /*!< SCL High Period Count for Fast Speed Register  offset: 0x1C */
    __IO uint32_t FSLR;                /*!< SCL Low Period Count for Fast Speed Register   offset: 0x20 */
    __IO uint32_t RESERVED0x24;        /*!< Reserved Register                              offset: 0x24 */
    __IO uint32_t RESERVED0x28;        /*!< Reserved Register                              offset: 0x28 */
    __IO uint32_t ISR;                 /*!< Interrupt Status Register                      offset: 0x2C */
    __IO uint32_t IMR;                 /*!< Interrupt Mask Register                        offset: 0x30 */
    __IO uint32_t RAWISR;              /*!< RAW Interrupt Status Register                  offset: 0x34 */
    __IO uint32_t RXTLR;               /*!< Receive FIFO Threshold Level Register          offset: 0x38 */
    __IO uint32_t TXTLR;               /*!< Transmit FIFO Threshold Level Register         offset: 0x3C */
    __IO uint32_t ICR;                 /*!< Clear All Interrupt Register                   offset: 0x40 */
    __IO uint32_t RX_UNDER;            /*!< Clear RX_UNDER Interrupt Register              offset: 0x44 */
    __IO uint32_t RX_OVER;             /*!< Clear RX_OVER Interrupt Register               offset: 0x48 */
    __IO uint32_t TX_OVER;             /*!< Clear TX_OVER Interrupt Register               offset: 0x4C */
    __IO uint32_t RD_REQ;              /*!< Clear RD_REQ Interrupt Register                offset: 0x50 */
    __IO uint32_t TX_ABRT;             /*!< Clear TX_ABRT Interrupt Register               offset: 0x54 */
    __IO uint32_t RX_DONE;             /*!< Clear RX_DONE Interrupt Register               offset: 0x58 */
    __IO uint32_t ACTIV;               /*!< Clear ACTIVITY Interrupt Register              offset: 0x5C */
    __IO uint32_t STOP;                /*!< Clear STOP_DET Interrupt Register              offset: 0x60 */
    __IO uint32_t START;               /*!< Clear START_DET Interrupt Register             offset: 0x64 */
    __IO uint32_t GC;                  /*!< Clear GEN_CALL Interrupt Register              offset: 0x68 */
    __IO uint32_t ENR;                 /*!< Enable Register                                offset: 0x6C */
    __IO uint32_t SR;                  /*!< Status Register                                offset: 0x70 */
    __IO uint32_t TXFLR;               /*!< Transmit FIFO Level Register                   offset: 0x74 */
    __IO uint32_t RXFLR;               /*!< Receive FIFO Level Register                    offset: 0x78 */
    __IO uint32_t HOLD;                /*!< SDA Hold Time Register                         offset: 0x7C */
    __IO uint32_t TX_ABRT_SRC;         /*!< IC_TX_ABRT_SOURCE_RESERVED;                    offset: 0x80 */
    __IO uint32_t SLV_NACK;            /*!< IC_SLV_DATA_NACK_ONLY_RESERVED;                offset: 0x84 */
    __IO uint32_t DMA;                 /*!< DMA Control Register                           offset: 0x88 */
    __IO uint32_t RESERVED30;          /*!< IC_DMA_TDLR_RESERVED; */
    __IO uint32_t RESERVED31;          /*!< IC_DMA_RDLR_RESERVED; */
    __IO uint32_t SETUP;               /*!< SDA Setup Time Register                        offset: 0x94 */
    __IO uint32_t GCR;                 /*!< ACK General Call Register                      offset: 0x98 */
    __IO uint32_t EN_SR;               /*!< ENABLE status register;                        offset: 0x9C */
    __IO uint32_t SPKLEN;              /*!< Filter register;                               offset: 0xA0 */
    __IO uint32_t RESERVED34;          /*!< _RESERVED;                                     offset: 0xA4 */
    __IO uint32_t RESERVED35;          /*!< _RESERVED;                                     offset: 0xA8 */
    __IO uint32_t SCL_TMO;             /*!< SCL low-level timeout register                 offset: 0xAC */
    __IO uint32_t SDA_TMO;             /*!< SDA low-level timeout register                 offset: 0xB0 */
    __IO uint32_t SCL_STUCK;           /*!< Clear SCL_ STUCK interrupt register            offset: 0xB4 */
    __IO uint32_t RESERVED0xB8;        /*!<                                                offset: 0xB8 */
    __IO uint32_t SMB_SEXT;            /*!< SMBus slave device clock extension timeout register  offset: 0xBC */
    __IO uint32_t SMB_MEXT;            /*!< SMBus master device clock extension timeout register offset: 0xC0 */
    __IO uint32_t SMB_IDLE;            /*!< SMBus bus idle count register                  offset: 0xC4 */
    __IO uint32_t SMB_ISR;             /*!< SMBus interrupt status register                offset: 0xC8 */
    __IO uint32_t SMB_IMR;             /*!< SMBus interrupt mask register                  offset: 0xCC */
    __IO uint32_t SMB_RAWISR;          /*!< SMBus RAW interrupt register                   offset: 0xD0 */
    __IO uint32_t SMB_ICR;             /*!< SMBus combination and independent interrupt clear register offset: 0xD4 */
    __IO uint32_t OPT_SAR;             /*!< Optional slave address register                offset: 0xD8 */
    __IO uint32_t SMB_UDID_LSB;        /*!< SMBus UDID LSB register                        offset: 0xDC */
    __IO uint32_t SMB_UDID_MSB0;       /*!< SMBus UDID MSB register 0                      offset: 0xE0 */
    __IO uint32_t SMB_UDID_MSB1;       /*!< SMBus UDID MSB register 1                      offset: 0xE4 */
    __IO uint32_t SMB_UDID_MSB2;       /*!< SMBus UDID MSB register 2                      offset: 0xE8 */
    __IO uint32_t SLVMASK;             /*!< Slave address mask register                    offset: 0xEC */
    __IO uint32_t SLVRCVADDR;          /*!< Receive address register from device           offset: 0xF0 */
} I2C_TypeDef;

/**
  * @brief I2C type pointer Definition
  */
#define I2C1                                ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                                ((I2C_TypeDef *)I2C2_BASE)

/**
  * @brief I2C_CR Register Bit Definition
  */
#define I2C_CR_MASTER_Pos                   (0)
#define I2C_CR_MASTER_Msk                   (0x01U << I2C_CR_MASTER_Pos)           /*!< I2C master mode enable */
#define I2C_CR_SPEED_Pos                    (1)
#define I2C_CR_SPEED_Msk                    (0x03U << I2C_CR_SPEED_Pos)            /*!< I2C speed mode */
#define I2C_CR_STD                          (0x01U << I2C_CR_SPEED_Pos)            /*!< I2C standard speed mode */
#define I2C_CR_FAST                         (0x02U << I2C_CR_SPEED_Pos)            /*!< I2C fast speed mode */
#define I2C_CR_SLAVE10_Pos                  (3)
#define I2C_CR_SLAVE10_Msk                  (0x01U << I2C_CR_SLAVE10_Pos)          /*!< I2C slave mode responds to 10-bit address */
#define I2C_CR_MASTER10_Pos                 (4)
#define I2C_CR_MASTER10_Msk                 (0x01U << I2C_CR_MASTER10_Pos)         /*!< I2C master mode responds to 10-bit address */
#define I2C_CR_REPEN_Pos                    (5)
#define I2C_CR_REPEN_Msk                    (0x01U << I2C_CR_REPEN_Pos)            /*!< Enable send RESTART */
#define I2C_CR_DISSLAVE_Pos                 (6)
#define I2C_CR_DISSLAVE_Msk                 (0x01U << I2C_CR_DISSLAVE_Pos)         /*!< I2C slave mode disable */
#define I2C_CR_STOPINT_Pos                  (7)
#define I2C_CR_STOPINT_Msk                  (0x01U << I2C_CR_STOPINT_Pos)          /*!< Generate STOP interrupt in slave mode */
#define I2C_CR_EMPINT_Pos                   (8)
#define I2C_CR_EMPINT_Msk                   (0x01U << I2C_CR_EMPINT_Pos)           /*!< I2C TX_EMPTY interrupt */

#define I2C_CR_STOP_Pos                     (9)
#define I2C_CR_STOP_Msk                     (0x01U << I2C_CR_STOP_Pos)             /*!< STOP signal enable */
#define I2C_CR_RESTART_Pos                  (10)
#define I2C_CR_RESTART_Msk                  (0x01U << I2C_CR_RESTART_Pos)          /*!< RESTART signal enable */
#define I2C_CR_SLV_TX_ABRT_DIS_Pos          (11)
#define I2C_CR_SLV_TX_ABRT_DIS_Msk          (0x01U << I2C_CR_SLV_TX_ABRT_DIS_Pos)  /*!< Prohibit clearing TXFIFO after receiving RD_REQ signal */
#define I2C_CR_PAD_SEL_Pos                  (12)
#define I2C_CR_PAD_SEL_Msk                  (0x01U << I2C_CR_PAD_SEL_Pos)          /*!< pad0 select sda;pad1 select scl */
#define I2C_CR_OPT_SAR_EN_Pos               (16)
#define I2C_CR_OPT_SAR_EN_Msk               (0x01U << I2C_CR_OPT_SAR_EN_Pos)       /*!< Enable/DISABLE OPT_ SAR register */
#define I2C_CR_SMB_SLV_QC_EN_Pos            (17)
#define I2C_CR_SMB_SLV_QC_EN_Msk            (0x01U << I2C_CR_SMB_SLV_QC_EN_Pos)    /*!< Receive all commands except fast commands from the device in SMBus mode */
#define I2C_CR_SMB_ARP_EN_Pos               (18)
#define I2C_CR_SMB_ARP_EN_Msk               (0x01U << I2C_CR_SMB_ARP_EN_Pos)       /*!< Decode and respond to address resolution protocol (ARP) commands in SMBus mode */
#define I2C_CR_SMB_PSA_EN_Pos               (19)
#define I2C_CR_SMB_PSA_EN_Msk               (0x01U << I2C_CR_SMB_PSA_EN_Pos)       /*!<  persistent slave */
#define I2C_CR_STOP_DET_MST_ACT_Pos         (20)
#define I2C_CR_STOP_DET_MST_ACT_Msk         (0x01U << I2C_CR_STOP_DET_MST_ACT_Pos) /*!< Stop interrupt generated */
#define I2C_CR_BUS_CLR_Pos                  (21)
#define I2C_CR_BUS_CLR_Msk                  (0x01U << I2C_CR_BUS_CLR_Pos)          /*!< Bus clear enable */
#define I2C_CR_RX_FULL_HLD_Pos              (22)
#define I2C_CR_RX_FULL_HLD_Msk              (0x01U << I2C_CR_RX_FULL_HLD_Pos)      /*!< RX FIFO MODE */

/**
  * @brief I2C_TAR Register Bit Definition
  */
#define I2C_TAR_ADDR_Pos                    (0)
#define I2C_TAR_ADDR_Msk                    (0x03FFU << I2C_TAR_ADDR_Pos)  /*!< Target address for master mode */
#define I2C_TAR_GC_Pos                      (10)
#define I2C_TAR_GC_Msk                      (0x01U << I2C_TAR_GC_Pos)      /*!< General Call or START byte */
#define I2C_TAR_SPECIAL_Pos                 (11)
#define I2C_TAR_SPECIAL_Msk                 (0x01U << I2C_TAR_SPECIAL_Pos) /*!< Special command enable like General Call or START byte */
#define I2C_TAR_SMB_QC_Pos                  (16)
#define I2C_TAR_SMB_QC_Msk                  (0x01U << I2C_TAR_SMB_QC_Pos)  /*!< This bit indicates whether the I2C executes a fast command */

/**
  * @brief I2C_SAR Register Bit Definition
  */
#define I2C_SAR_ADDR_Pos                    (0)
#define I2C_SAR_ADDR_Msk                    (0x03FFU << I2C_SAR_ADDR_Pos) /*!< Slave address */

/**
  * @brief I2C_DR Register Bit Definition
  */
#define I2C_DR_DAT_Pos                      (0)
#define I2C_DR_DAT_Msk                      (0xFFU << I2C_DR_DAT_Pos)        /*!< The data to be transmitted or received */
#define I2C_DR_CMD_Pos                      (8)
#define I2C_DR_CMD_Msk                      (0x01U << I2C_DR_CMD_Pos)        /*!< Read or write command */
#define I2C_DR_FIRST_DATA_Pos               (11)
#define I2C_DR_FIRST_DATA_Msk               (0x01U << I2C_DR_FIRST_DATA_Pos) /*!< Set when the first data byte is received after address transmission */

/**
  * @brief I2C_SSHR Register Bit Definition
  */
#define I2C_SSHR_CNT_Pos                    (0)
#define I2C_SSHR_CNT_Msk                    (0xFFFFU << I2C_SSHR_CNT_Pos) /*!< SCL clock high period count for standard speed */

/**
  * @brief I2C_SSLR Register Bit Definition
  */
#define I2C_SSLR_CNT_Pos                    (0)
#define I2C_SSLR_CNT_Msk                    (0xFFFFU << I2C_SSLR_CNT_Pos) /*!< SCL clock low period count for standard speed */

/**
  * @brief I2C_FSHR Register Bit Definition
  */
#define I2C_FSHR_CNT_Pos                    (0)
#define I2C_FSHR_CNT_Msk                    (0xFFFFU << I2C_FSHR_CNT_Pos) /*!< SCL clock high period count for fast speed */

/**
  * @brief I2C_FSLR Register Bit Definition
  */
#define I2C_FSLR_CNT_Pos                    (0)
#define I2C_FSLR_CNT_Msk                    (0xFFFFU << I2C_FSLR_CNT_Pos) /*!< SCL clock low period count for fast speed */

/**
  * @brief I2C_ISR Register Bit Definition
  */
#define I2C_ISR_R_RX_UNDER_Pos              (0)
#define I2C_ISR_R_RX_UNDER_Msk              (0x01U << I2C_ISR_R_RX_UNDER_Pos)         /*!< RX_UNDER interrupt status */
#define I2C_ISR_R_RX_OVER_Pos               (1)
#define I2C_ISR_R_RX_OVER_Msk               (0x01U << I2C_ISR_R_RX_OVER_Pos)          /*!< RX_OVER interrupt status */
#define I2C_ISR_R_RX_FULL_Pos               (2)
#define I2C_ISR_R_RX_FULL_Msk               (0x01U << I2C_ISR_R_RX_FULL_Pos)          /*!< RX_FULL interrupt status */
#define I2C_ISR_R_TX_OVER_Pos               (3)
#define I2C_ISR_R_TX_OVER_Msk               (0x01U << I2C_ISR_R_TX_OVER_Pos)          /*!< TX_OVER interrupt status */
#define I2C_ISR_R_TX_EMPTY_Pos              (4)
#define I2C_ISR_R_TX_EMPTY_Msk              (0x01U << I2C_ISR_R_TX_EMPTY_Pos)         /*!< TX_EMPTY interrupt status */
#define I2C_ISR_R_RD_REQ_Pos                (5)
#define I2C_ISR_R_RD_REQ_Msk                (0x01U << I2C_ISR_R_RD_REQ_Pos)           /*!< RD_REQ interrupt status */
#define I2C_ISR_R_TX_ABRT_Pos               (6)
#define I2C_ISR_R_TX_ABRT_Msk               (0x01U << I2C_ISR_R_TX_ABRT_Pos)          /*!< TX_ABRT interrupt status */
#define I2C_ISR_R_RX_DONE_Pos               (7)
#define I2C_ISR_R_RX_DONE_Msk               (0x01U << I2C_ISR_R_RX_DONE_Pos)          /*!< RX_DONE interrupt status */
#define I2C_ISR_R_ACTIV_Pos                 (8)
#define I2C_ISR_R_ACTIV_Msk                 (0x01U << I2C_ISR_R_ACTIV_Pos)            /*!< ACTIVITY interrupt status */
#define I2C_ISR_R_STOP_Pos                  (9)
#define I2C_ISR_R_STOP_Msk                  (0x01U << I2C_ISR_R_STOP_Pos)             /*!< STOP_DET interrupt status */
#define I2C_ISR_R_START_Pos                 (10)
#define I2C_ISR_R_START_Msk                 (0x01U << I2C_ISR_R_START_Pos)            /*!< START_DET interrupt status */
#define I2C_ISR_R_GC_Pos                    (11)
#define I2C_ISR_R_GC_Msk                    (0x01U << I2C_ISR_R_GC_Pos)               /*!< GEN_CALL interrupt status */
#define I2C_ISR_R_MST_ON_HOLD_Pos           (13)
#define I2C_ISR_R_MST_ON_HOLD_Msk           (0x01U << I2C_ISR_R_MST_ON_HOLD_Pos)      /*!< MST_ON_HOLD interrupt status */
#define I2C_ISR_R_SCL_STUCK_AT_LOW_Pos      (14)
#define I2C_ISR_R_SCL_STUCK_AT_LOW_Msk      (0x01U << I2C_ISR_R_SCL_STUCK_AT_LOW_Pos) /*!< SCL_STUCK_AT_LOW interrupt status */

/**
  * @brief I2C_IMR Register Bit Definition
  */
#define I2C_IMR_M_RX_UNDER_Pos              (0)
#define I2C_IMR_M_RX_UNDER_Msk              (0x01U << I2C_IMR_M_RX_UNDER_Pos)    /*!< RX_UNDER interrupt status */
#define I2C_IMR_M_RX_OVER_Pos               (1)
#define I2C_IMR_M_RX_OVER_Msk               (0x01U << I2C_IMR_M_RX_OVER_Pos)     /*!< RX_OVER interrupt status */
#define I2C_IMR_M_RX_FULL_Pos               (2)
#define I2C_IMR_M_RX_FULL_Msk               (0x01U << I2C_IMR_M_RX_FULL_Pos)     /*!< RX_FULL interrupt status */
#define I2C_IMR_M_TX_OVER_Pos               (3)
#define I2C_IMR_M_TX_OVER_Msk               (0x01U << I2C_IMR_M_TX_OVER_Pos)     /*!< TX_OVER interrupt status */
#define I2C_IMR_M_TX_EMPTY_Pos              (4)
#define I2C_IMR_M_TX_EMPTY_Msk              (0x01U << I2C_IMR_M_TX_EMPTY_Pos)    /*!< TX_EMPTY interrupt status */
#define I2C_IMR_M_RD_REQ_Pos                (5)
#define I2C_IMR_M_RD_REQ_Msk                (0x01U << I2C_IMR_M_RD_REQ_Pos)      /*!< RD_REQ interrupt status */
#define I2C_IMR_M_TX_ABRT_Pos               (6)
#define I2C_IMR_M_TX_ABRT_Msk               (0x01U << I2C_IMR_M_TX_ABRT_Pos)     /*!< TX_ABRT interrupt status */
#define I2C_IMR_M_RX_DONE_Pos               (7)
#define I2C_IMR_M_RX_DONE_Msk               (0x01U << I2C_IMR_M_RX_DONE_Pos)     /*!< RX_DONE interrupt status */
#define I2C_IMR_M_ACTIV_Pos                 (8)
#define I2C_IMR_M_ACTIV_Msk                 (0x01U << I2C_IMR_M_ACTIV_Pos)       /*!< ACTIVITY interrupt status */
#define I2C_IMR_M_STOP_Pos                  (9)
#define I2C_IMR_M_STOP_Msk                  (0x01U << I2C_IMR_M_STOP_Pos)        /*!< STOP_DET interrupt status */
#define I2C_IMR_M_START_Pos                 (10)
#define I2C_IMR_M_START_Msk                 (0x01U << I2C_IMR_M_START_Pos)       /*!< START_DET interrupt status */
#define I2C_IMR_M_GC_Pos                    (11)
#define I2C_IMR_M_GC_Msk                    (0x01U << I2C_IMR_M_GC_Pos)          /*!< GEN_CALL interrupt status */
#define I2C_IMR_M_MST_ON_HOLD_Pos           (13)
#define I2C_IMR_M_MST_ON_HOLD_Msk           (0x01U << I2C_IMR_M_MST_ON_HOLD_Pos) /*!< MST_ON_HOLD interrupt status */
#define I2C_IMR_M_SCL_STUCK_Pos             (14)
#define I2C_IMR_M_SCL_STUCK_Msk             (0x01U << I2C_IMR_M_SCL_STUCK_Pos)   /*!< SCL_STUCK_AT_LOW interrupt status */

/**
  * @brief I2C_RAWISR Register Bit Definition
  */
#define I2C_RAWISR_RX_UNDER_Pos             (0)
#define I2C_RAWISR_RX_UNDER_Msk             (0x01U << I2C_RAWISR_RX_UNDER_Pos)    /*!< RX_UNDER raw interrupt status */
#define I2C_RAWISR_RX_OVER_Pos              (1)
#define I2C_RAWISR_RX_OVER_Msk              (0x01U << I2C_RAWISR_RX_OVER_Pos)     /*!< RX_OVER raw interrupt status */
#define I2C_RAWISR_RX_FULL_Pos              (2)
#define I2C_RAWISR_RX_FULL_Msk              (0x01U << I2C_RAWISR_RX_FULL_Pos)     /*!< RX_FULL raw interrupt status */
#define I2C_RAWISR_TX_OVER_Pos              (3)
#define I2C_RAWISR_TX_OVER_Msk              (0x01U << I2C_RAWISR_TX_OVER_Pos)     /*!< TX_OVER raw interrupt status */
#define I2C_RAWISR_TX_EMPTY_Pos             (4)
#define I2C_RAWISR_TX_EMPTY_Msk             (0x01U << I2C_RAWISR_TX_EMPTY_Pos)    /*!< TX_EMPTY raw interrupt status */
#define I2C_RAWISR_RD_REQ_Pos               (5)
#define I2C_RAWISR_RD_REQ_Msk               (0x01U << I2C_RAWISR_RD_REQ_Pos)      /*!< RD_REQ raw interrupt status */
#define I2C_RAWISR_TX_ABRT_Pos              (6)
#define I2C_RAWISR_TX_ABRT_Msk              (0x01U << I2C_RAWISR_TX_ABRT_Pos)     /*!< TX_ABRT raw interrupt status */
#define I2C_RAWISR_RX_DONE_Pos              (7)
#define I2C_RAWISR_RX_DONE_Msk              (0x01U << I2C_RAWISR_RX_DONE_Pos)     /*!< RX_DONE raw interrupt status */

#define I2C_RAWISR_ACTIV_Pos                (8)
#define I2C_RAWISR_ACTIV_Msk                (0x01U << I2C_RAWISR_ACTIV_Pos)       /*!< ACTIVITY interrupt status */
#define I2C_RAWISR_STOP_Pos                 (9)
#define I2C_RAWISR_STOP_Msk                 (0x01U << I2C_RAWISR_STOP_Pos)        /*!< STOP_DET interrupt status */
#define I2C_RAWISR_START_Pos                (10)
#define I2C_RAWISR_START_Msk                (0x01U << I2C_RAWISR_START_Pos)       /*!< START_DET interrupt status */
#define I2C_RAWISR_GC_Pos                   (11)
#define I2C_RAWISR_GC_Msk                   (0x01U << I2C_RAWISR_GC_Pos)          /*!< GEN_CALL interrupt status */
#define I2C_RAWISR_MST_ON_HOLD_Pos          (13)
#define I2C_RAWISR_MST_ON_HOLD_Msk          (0x01U << I2C_RAWISR_MST_ON_HOLD_Pos) /*!< MST_ON_HOLD interrupt status */
#define I2C_RAWISR_SCL_STUCK_Pos            (14)
#define I2C_RAWISR_SCL_STUCK_Msk            (0x01U << I2C_RAWISR_SCL_STUCK_Pos)   /*!< SCL_STUCK_AT_LOW interrupt status */

/**
  * @brief I2C_RXTLR Register Bit Definition
  */
#define I2C_RXTLR_TL_Pos                    (0)
#define I2C_RXTLR_TL_Msk                    (0x03U << I2C_RXTLR_TL_Pos) /*!< Receive FIFO threshold level */

/**
  * @brief I2C_TXTLR Register Bit Definition
  */
#define I2C_TXTLR_TL_Pos                    (0)
#define I2C_TXTLR_TL_Msk                    (0x03U << I2C_TXTLR_TL_Pos) /*!< Transmit FIFO threshold level */

/**
  * @brief I2C_ICR Register Bit Definition
  */
#define I2C_ICR_Pos                         (0)
#define I2C_ICR_Msk                         (0x01U << I2C_ICR_Pos) /*!< Read this register to clear the combined interrupt, all individual interrupts */

/**
  * @brief I2C_RX_UNDER Register Bit Definition
  */
#define I2C_RX_UNDER_Pos                    (0)
#define I2C_RX_UNDER_Msk                    (0x01U << I2C_RX_UNDER_Pos) /*!< Read this register to clear the RX_UNDER interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_RX_OVER Register Bit Definition
  */
#define I2C_RX_OVER_Pos                     (0)
#define I2C_RX_OVER_Msk                     (0x01U << I2C_RX_OVER_Pos) /*!< Read this register to clear the RX_OVER interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_TX_OVER Register Bit Definition
  */
#define I2C_TX_OVER_Pos                     (0)
#define I2C_TX_OVER_Msk                     (0x01U << I2C_TX_OVER_Pos) /*!< Read this register to clear the TX_OVER interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_RD_REQ Register Bit Definition
  */
#define I2C_RD_REQ_Pos                      (0)
#define I2C_RD_REQ_Msk                      (0x01U << I2C_RD_REQ_Pos) /*!< Read this register to clear the RD_REQ interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_TX_ABRT Register Bit Definition
  */
#define I2C_TX_ABRT_Pos                     (0)
#define I2C_TX_ABRT_Msk                     (0x01U << I2C_TX_ABRT_Pos) /*!< Read this register to clear the TX_ABRT interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_RX_DONE Register Bit Definition
  */
#define I2C_RX_DONE_Pos                     (0)
#define I2C_RX_DONE_Msk                     (0x01U << I2C_RX_DONE_Pos) /*!< Read this register to clear the RX_DONE interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_ACTIV Register Bit Definition
  */
#define I2C_ACTIV_Pos                       (0)
#define I2C_ACTIV_Msk                       (0x01U << I2C_ACTIV_Pos) /*!< Read this register to clear the ACTIVITY interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_STOP Register Bit Definition
  */
#define I2C_STOP_Pos                        (0)
#define I2C_STOP_Msk                        (0x01U << I2C_STOP_Pos) /*!< Read this register to clear the STOP_DET interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_START Register Bit Definition
  */
#define I2C_START_Pos                       (0)
#define I2C_START_Msk                       (0x01U << I2C_START_Pos) /*!< Read this register to clear the START_DET interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_GC Register Bit Definition
  */
#define I2C_GC_Pos                          (0)
#define I2C_GC_Msk                          (0x01U << I2C_GC_Pos) /*!< Read this register to clear the GEN_CALL interrupt of the I2C_RAW_INTR_STAT register */

/**
  * @brief I2C_ENR Register Bit Definition
  */
#define I2C_ENR_ENABLE_Pos                  (0)
#define I2C_ENR_ENABLE_Msk                  (0x01U << I2C_ENR_ENABLE_Pos)       /*!< I2C mode enable */
#define I2C_ENR_ABORT_Pos                   (1)
#define I2C_ENR_ABORT_Msk                   (0x01U << I2C_ENR_ABORT_Pos)        /*!< I2C transfer abort */
#define I2C_ENR_TX_CMD_BLOCK_Pos            (2)
#define I2C_ENR_TX_CMD_BLOCK_Msk            (0x01U << I2C_ENR_TX_CMD_BLOCK_Pos) /*!< TX_CMD_BLOCK */
#define I2C_ENR_SDA_RCV_EN_Pos              (3)
#define I2C_ENR_SDA_RCV_EN_Msk              (0x01U << I2C_ENR_SDA_RCV_EN_Pos)   /*!< Enable SDA recovery mechanism */
#define I2C_ENR_SMB_CLK_RST_Pos             (16)
#define I2C_ENR_SMB_CLK_RST_Msk             (0x01U << I2C_ENR_SMB_CLK_RST_Pos)  /*!< SMBus master clock reset */
#define I2C_ENR_SMB_ALT_EN_Pos              (18)
#define I2C_ENR_SMB_ALT_EN_Msk              (0x01U << I2C_ENR_SMB_ALT_EN_Pos)   /*!< SMBus alarm enable */

/**
  * @brief I2C_SR Register Bit Definition
  */
#define I2C_SR_ACTIV_Pos                    (0)
#define I2C_SR_ACTIV_Msk                    (0x01U << I2C_SR_ACTIV_Pos)             /*!< I2C activity status */
#define I2C_SR_TFNF_Pos                     (1)
#define I2C_SR_TFNF_Msk                     (0x01U << I2C_SR_TFNF_Pos)              /*!< Transmit FIFO not full */
#define I2C_SR_TFE_Pos                      (2)
#define I2C_SR_TFE_Msk                      (0x01U << I2C_SR_TFE_Pos)               /*!< Transmit FIFO completely empty */
#define I2C_SR_RFNE_Pos                     (3)
#define I2C_SR_RFNE_Msk                     (0x01U << I2C_SR_RFNE_Pos)              /*!< Receive FIFO not empty */
#define I2C_SR_RFF_Pos                      (4)
#define I2C_SR_RFF_Msk                      (0x01U << I2C_SR_RFF_Pos)               /*!< Receive FIFO completely full */
#define I2C_SR_MST_ACTIV_Pos                (5)
#define I2C_SR_MST_ACTIV_Msk                (0x01U << I2C_SR_MST_ACTIV_Pos)         /*!< Master FSM activity status */
#define I2C_SR_SLV_ACTIV_Pos                (6)
#define I2C_SR_SLV_ACTIV_Msk                (0x01U << I2C_SR_SLV_ACTIV_Pos)         /*!< Slave FSM activity status */
#define I2C_SR_MST_HOLD_TX_EMPTY_Pos        (7)
#define I2C_SR_MST_HOLD_TX_EMPTY_Msk        (0x01U << I2C_SR_MST_HOLD_TX_EMPTY_Pos) /*!< Set when the main device is in the hold state because TX FIFO is empty */
#define I2C_SR_MST_HOLD_RX_FULL_Pos         (8)
#define I2C_SR_MST_HOLD_RX_FULL_Msk         (0x01U << I2C_SR_MST_HOLD_RX_FULL_Pos)  /*!< Set when the main device is in the hold state due to receiving overload */
#define I2C_SR_SLV_HOLD_TX_EMPTY_Pos        (9)
#define I2C_SR_SLV_HOLD_TX_EMPTY_Msk        (0x01U << I2C_SR_SLV_HOLD_TX_EMPTY_Pos) /*!< Set when the slave device is in the hold state because TX FIFO is empty when receiving the read request */
#define I2C_SR_SLV_HOLD_RX_FULL_Pos         (10)
#define I2C_SR_SLV_HOLD_RX_FULL_Msk         (0x01U << I2C_SR_SLV_HOLD_RX_FULL_Pos)  /*!< Set when the slave device is in the hold state due to receiving overload */
#define I2C_SR_SDA_NOT_RECOVERED_Pos        (11)
#define I2C_SR_SDA_NOT_RECOVERED_Msk        (0x01U << I2C_SR_SDA_NOT_RECOVERED_Pos) /*!< After the recovery mechanism is enabled, the SDA is set when it is not recovered */
#define I2C_SR_SMB_QC_Pos                   (16)
#define I2C_SR_SMB_QC_Msk                   (0x01U << I2C_SR_SMB_QC_Pos)            /*!< SMBus fast command status bit */
#define I2C_SR_SMB_SLV_AV_Pos               (17)
#define I2C_SR_SMB_SLV_AV_Msk               (0x01U << I2C_SR_SMB_SLV_AV_Pos)        /*!< SMBus Slave address Valid */
#define I2C_SR_SMB_SLV_AR_Pos               (18)
#define I2C_SR_SMB_SLV_AR_Msk               (0x01U << I2C_SR_SMB_SLV_AR_Pos)        /*!< SMBus Slave address Resolved */
#define I2C_SR_SMB_ALT_Pos                  (20)
#define I2C_SR_SMB_ALT_Msk                  (0x01U << I2C_SR_SMB_ALT_Pos)           /*!< SMBus alarm status bit */

/**
  * @brief I2C_TXFLR Register Bit Definition
  */
#define I2C_TXFLR_CNT_Pos                   (0)
#define I2C_TXFLR_CNT_Msk                   (0x07U << I2C_TXFLR_CNT_Pos) /*!< Number of valid data in the transmit FIFO */

/**
  * @brief I2C_RXFLR Register Bit Definition
  */
#define I2C_RXFLR_CNT_Pos                   (0)
#define I2C_RXFLR_CNT_Msk                   (0x07U << I2C_RXFLR_CNT_Pos) /*!< Number of valid data in the receive FIFO */

/**
  * @brief I2C_HOLD Register Bit Definition
  */
#define I2C_HOLD_TX_HOLD_Pos                (0)
#define I2C_HOLD_TX_HOLD_Msk                (0xFFFFU << I2C_HOLD_TX_HOLD_Pos) /*!< SDA hold time when I2C acts as a transmit */
#define I2C_HOLD_RX_HOLD_Pos                (16)
#define I2C_HOLD_RX_HOLD_Msk                (0xFFU << I2C_HOLD_RX_HOLD_Pos)   /*!< SDA hold time when I2C acts as a receiver */

/**
  * @brief I2C_TX_ABRT_SRC Register Bit Definition
  */
#define I2C_TX_ABRT_SRC_7ADDR_NOACK_Pos     (0)
#define I2C_TX_ABRT_SRC_7ADDR_NOACK_Msk     (0x01U << I2C_TX_ABRT_SRC_7ADDR_NOACK_Pos)    /*!< Main device works in 7-bit addressing mode */
#define I2C_TX_ABRT_SRC_10ADDR1_NOACK_Pos   (1)
#define I2C_TX_ABRT_SRC_10ADDR1_NOACK_Msk   (0x01U << I2C_TX_ABRT_SRC_10ADDR1_NOACK_Pos)  /*!< Main device works in 10-bit addressing mode */
#define I2C_TX_ABRT_SRC_10ADDR2_NOACK_Pos   (2)
#define I2C_TX_ABRT_SRC_10ADDR2_NOACK_Msk   (0x01U << I2C_TX_ABRT_SRC_10ADDR2_NOACK_Pos)  /*!< Main device works in 10-bit addressing mode */
#define I2C_TX_ABRT_SRC_TXDATA_NOACK_Pos    (3)
#define I2C_TX_ABRT_SRC_TXDATA_NOACK_Msk    (0x01U << I2C_TX_ABRT_SRC_TXDATA_NOACK_Pos)   /*!< After the master device receives the address response, it is set when no slave device responds to the sent data */
#define I2C_TX_ABRT_SRC_GC_NOACK_Pos        (4)
#define I2C_TX_ABRT_SRC_GC_NOACK_Msk        (0x01U << I2C_TX_ABRT_SRC_GC_NOACK_Pos)       /*!< After the master device sends a broadcast call, it is set when no slave device responds */
#define I2C_TX_ABRT_SRC_GC_READ_Pos         (5)
#define I2C_TX_ABRT_SRC_GC_READ_Msk         (0x01U << I2C_TX_ABRT_SRC_GC_READ_Pos)        /*!< Set when the master device sends a broadcast call and then sends a read request */
#define I2C_TX_ABRT_SRC_SBYTE_ACKDET_Pos    (7)
#define I2C_TX_ABRT_SRC_SBYTE_ACKDET_Msk    (0x01U << I2C_TX_ABRT_SRC_SBYTE_ACKDET_Pos)   /*!< Set when the main device sends the start byte and is responded (error behavior) */
#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Pos   (9)
#define I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Msk   (0x01U << I2C_TX_ABRT_SRC_SBYTE_NORSTRT_Pos)  /*!< When RESTART is not enabled (I2C_CR.REPEN is 0), the start byte is sent, and the position bit */
#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Pos  (10)
#define I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Msk  (0x01U << I2C_TX_ABRT_SRC_10B_RD_NORSTRT_Pos) /*!< When RESTART is not enabled (I2C_CR.REPEN is 0), the main device is in 10-bit addressing mode If a read request is sent, the position bit */
#define I2C_TX_ABRT_SRC_MST_DIS_Pos         (11)
#define I2C_TX_ABRT_SRC_MST_DIS_Msk         (0x01U << I2C_TX_ABRT_SRC_MST_DIS_Pos)        /*!< If the main device is operated when the main mode is disabled, this position bit */
#define I2C_TX_ABRT_SRC_LOST_Pos            (12)
#define I2C_TX_ABRT_SRC_LOST_Msk            (0x01U << I2C_TX_ABRT_SRC_LOST_Pos)           /*!< Master device arbitration fails, or slave device arbitration fails when bit 14 is set to 1, then this position is */
#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Pos (13)
#define I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO_Msk (0x01U << I2C_TX_ABRT_SRC_LOST_Pos)           /*!< When the slave device receives the read request, there is still data in TX FIFO, and the slave device generates TX through_ ABRT interrupt clears the old data in TX FIFO. */
#define I2C_TX_ABRT_SRC_SLV_ARBLOST_Pos     (14)
#define I2C_TX_ABRT_SRC_SLV_ARBLOST_Msk     (0x01U << I2C_TX_ABRT_SRC_SLV_ARBLOST_Pos)    /*!< If the bus is lost when sending data from the device, this bit and bit 12 are set at the same time */
#define I2C_TX_ABRT_SRC_SLVRD_INTX_Pos      (15)
#define I2C_TX_ABRT_SRC_SLVRD_INTX_Msk      (0x01U << I2C_TX_ABRT_SRC_SLVRD_INTX_Pos)     /*!< From the sending mode, I2C sends data to I2C_ DR.CMD (bit 8) write 1 (read command), then this position bit */
#define I2C_TX_ABRT_SRC_USER_ABRT_Pos       (16)
#define I2C_TX_ABRT_SRC_USER_ABRT_Msk       (0x01U << I2C_TX_ABRT_SRC_USER_ABRT_Pos)      /*!< Set when the master device detects transmission termination (I2C_ENR.ABORT is 1) */
#define I2C_TX_ABRT_SRC_SDA_LOW_Pos         (17)
#define I2C_TX_ABRT_SRC_SDA_LOW_Msk         (0x01U << I2C_TX_ABRT_SRC_USER_ABRT_Pos)      /*!< If the main device detects SDA low level timeout, this position is */
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Pos    (23)
#define I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Msk    (0x1FFU << I2C_TX_ABRT_SRC_USER_ABRT_Pos)     /*!< Number of data/commands in TX FIFO flushed due to transmission interruption */

/**
  * @brief I2C_SLV_NACK Register Bit Definition
  */
#define I2C_SLV_NACK_Pos                    (0)
#define I2C_SLV_NACK_Msk                    (0x01U << I2C_SLV_NACK_Pos) /*!< NACK is generated after receiving data from the device */

/**
  * @brief I2C_DMA Register Bit Definition
  */
#define I2C_DMA_RXEN_Pos                    (0)
#define I2C_DMA_RXEN_Msk                    (0x01U << I2C_DMA_RXEN_Pos) /*!< Receive DMA enable */
#define I2C_DMA_TXEN_Pos                    (1)
#define I2C_DMA_TXEN_Msk                    (0x01U << I2C_DMA_TXEN_Pos) /*!< Transmit DMA enable */

/**
  * @brief I2C_SETUP Register Bit Definition
  */
#define I2C_SETUP_CNT_Pos                   (0)
#define I2C_SETUP_CNT_Msk                   (0xFFU << I2C_SETUP_CNT_Pos) /*!< SDA setup */

/**
  * @brief I2C_GCR Register Bit Definition
  */
#define I2C_GCR_GC_Pos                      (0)
#define I2C_GCR_GC_Msk                      (0x01U << I2C_GCR_GC_Pos) /*!< ACK general call */

/**
  * @brief I2C_EN_SR Register Bit Definition
  */
#define I2C_EN_SR_IC_EN_Pos                 (0)
#define I2C_EN_SR_IC_EN_Msk                 (0x01U << I2C_EN_SR_IC_EN_Pos)              /*!< I2C is active */
#define I2C_EN_SR_SLV_DIS_WHILE_BUSY_Pos    (1)
#define I2C_EN_SR_SLV_DIS_WHILE_BUSY_Msk    (0x01U << I2C_EN_SR_SLV_DIS_WHILE_BUSY_Pos) /*!< Slave Disabled While Busy */
#define I2C_EN_SR_SLV_RX_DATA_LOST_Pos      (2)
#define I2C_EN_SR_SLV_RX_DATA_LOST_Msk      (0x01U << I2C_EN_SR_SLV_RX_DATA_LOST_Pos)   /*!< Slave Received Data Lost */

/**
  * @brief I2C_SPKLEN Register Bit Definition
  */
#define I2C_SPKLEN_Pos                      (0)
#define I2C_SPKLEN_Msk                      (0xFFU << I2C_SPKLEN_Pos) /*!< Configure spike suppression time, in APB clock cycle The minimum value is 1. */

/**
  * @brief I2C_SCL_TMO Register Bit Definition
  */
#define I2C_SCL_TMO_SCL_TIMEOUT_Pos         (0)
#define I2C_SCL_TMO_SCL_TIMEOUT_Msk         (0xFFFFFFFFU << I2C_SCL_TMO_SCL_TIMEOUT_Pos) /*!< Configure the duration of SCL low-level timeout, in APB clock cycle */

/**
  * @brief I2C_SDA_TMO Register Bit Definition
  */
#define I2C_SDA_TMO_SCL_TIMEOUT_Pos         (0)
#define I2C_SDA_TMO_SCL_TIMEOUT_Msk         (0xFFFFFFFFU << I2C_SDA_TMO_SCL_TIMEOUT_Pos) /*!< Configure the duration of SDA low-level timeout, in APB clock cycle */

/**
  * @brief I2C_SCL_STUCK Register Bit Definition
  */
#define I2C_SCL_STUCK_Pos                   (0)
#define I2C_SCL_STUCK_Msk                   (0x01U << I2C_SCL_STUCK_Pos) /*!< Read this register to clear SCL_ STUCK interrupt */

/**
  * @brief I2C_SMB_SEXT Register Bit Definition
  */
#define I2C_SMB_SEXT_Pos                    (0)
#define I2C_SMB_SEXT_Msk                    (0xFFFFFFFFU << I2C_SMB_SEXT_Pos) /*!< Configure slave device clock extension timeout */

/**
  * @brief I2C_SMB_MEXT Register Bit Definition
  */
#define I2C_SMB_MEXT_Pos                    (0)
#define I2C_SMB_MEXT_Msk                    (0xFFFFFFFFU << I2C_SMB_MEXT_Pos) /*!< Configure the clock extension timeout for the master device to transmit one byte */

/**
  * @brief I2C_SMB_IDLE Register Bit Definition
  */
#define I2C_SMB_IDLE_CNT_Pos                (0)
#define I2C_SMB_IDLE_CNT_Msk                (0xFFFFU << I2C_SMB_IDLE_CNT_Pos) /*!< Configure the required bus idle time */

/**
  * @brief I2C_SMB_ISR Register Bit Definition
  */
#define I2C_SMB_ISR_R_SLV_TMO_Pos           (0)
#define I2C_SMB_ISR_R_SLV_TMO_Msk           (0x01U << I2C_SMB_ISR_R_SLV_TMO_Pos)    /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_MST_TMO_Pos           (1)
#define I2C_SMB_ISR_R_MST_TMO_Msk           (0x01U << I2C_SMB_ISR_R_MST_TMO_Pos)    /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_QUICK_Pos         (2)
#define I2C_SMB_ISR_R_ARP_QUICK_Msk         (0x01U << I2C_SMB_ISR_R_ARP_QUICK_Pos)  /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_NOTIFY_Pos        (3)
#define I2C_SMB_ISR_R_ARP_NOTIFY_Msk        (0x01U << I2C_SMB_ISR_R_ARP_NOTIFY_Pos) /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_PRE_Pos           (4)
#define I2C_SMB_ISR_R_ARP_PRE_Msk           (0x01U << I2C_SMB_ISR_R_ARP_PRE_Pos)    /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_RST_Pos           (5)
#define I2C_SMB_ISR_R_ARP_RST_Msk           (0x01U << I2C_SMB_ISR_R_ARP_RST_Pos)    /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_UDID_Pos          (6)
#define I2C_SMB_ISR_R_ARP_UDID_Msk          (0x01U << I2C_SMB_ISR_R_ARP_UDID_Pos)   /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_ARP_ASSGN_Pos         (7)
#define I2C_SMB_ISR_R_ARP_ASSGN_Msk         (0x01U << I2C_SMB_ISR_R_ARP_ASSGN_Pos)  /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_PEC_NACK_Pos          (8)
#define I2C_SMB_ISR_R_PEC_NACK_Msk          (0x01U << I2C_SMB_ISR_R_PEC_NACK_Pos)   /*!< For detailed description, please refer to I2C_SMB_RAWISR register */
#define I2C_SMB_ISR_R_SMB_ALT_Pos           (10)
#define I2C_SMB_ISR_R_SMB_ALT_Msk           (0x01U << I2C_SMB_ISR_R_SMB_ALT_Pos)    /*!< For detailed description, please refer to I2C_SMB_RAWISR register */

/**
  * @brief I2C_SMB_IMR Register Bit Definition
  */
#define I2C_SMB_IMR_M_SLV_TMO_Pos           (0)
#define I2C_SMB_IMR_M_SLV_TMO_Msk           (0x01U << I2C_SMB_IMR_M_SLV_TMO_Pos)    /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_MST_TMO_Pos           (1)
#define I2C_SMB_IMR_M_MST_TMO_Msk           (0x01U << I2C_SMB_IMR_M_MST_TMO_Pos)    /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_QUICK_Pos         (2)
#define I2C_SMB_IMR_M_ARP_QUICK_Msk         (0x01U << I2C_SMB_IMR_M_ARP_QUICK_Pos)  /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_NOTIFY_Pos        (3)
#define I2C_SMB_IMR_M_ARP_NOTIFY_Msk        (0x01U << I2C_SMB_IMR_M_ARP_NOTIFY_Pos) /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_PRE_Pos           (4)
#define I2C_SMB_IMR_M_ARP_PRE_Msk           (0x01U << I2C_SMB_IMR_M_ARP_PRE_Pos)    /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_RST_Pos           (5)
#define I2C_SMB_IMR_M_ARP_RST_Msk           (0x01U << I2C_SMB_IMR_M_ARP_RST_Pos)    /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_UDID_Pos          (6)
#define I2C_SMB_IMR_M_ARP_UDID_Msk          (0x01U << I2C_SMB_IMR_M_ARP_UDID_Pos)   /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_ARP_ASSGN_Pos         (7)
#define I2C_SMB_IMR_M_ARP_ASSGN_Msk         (0x01U << I2C_SMB_IMR_M_ARP_ASSGN_Pos)  /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_PEC_NACK_Pos          (8)
#define I2C_SMB_IMR_M_PEC_NACK_Msk          (0x01U << I2C_SMB_IMR_M_PEC_NACK_Pos)   /*!< For detailed description, please refer to I2C_SMB_ISR register */
#define I2C_SMB_IMR_M_SMB_ALT_Pos           (10)
#define I2C_SMB_IMR_M_SMB_ALT_Msk           (0x01U << I2C_SMB_IMR_M_SMB_ALT_Pos)    /*!< For detailed description, please refer to I2C_SMB_ISR register */

/**
  * @brief  I2C_SMB_RAWISR Register Bit Definition
  */
#define I2C_SMB_RAWISR_SLV_TMO_Pos          (0)
#define I2C_SMB_RAWISR_SLV_TMO_Msk          (0x01U << I2C_SMB_RAWISR_SLV_TMO_Pos)    /*!< If the clock extension timeout from the device communication once (START to STOP) is exceeded, this position bit */
#define I2C_SMB_RAWISR_MST_TMO_Pos          (1)
#define I2C_SMB_RAWISR_MST_TMO_Msk          (0x01U << I2C_SMB_RAWISR_MST_TMO_Pos)    /*!< If the master device transmits a byte of clock extension timeout, this position bit */
#define I2C_SMB_RAWISR_ARP_QUICK_Pos        (2)
#define I2C_SMB_RAWISR_ARP_QUICK_Msk        (0x01U << I2C_SMB_RAWISR_ARP_QUICK_Pos)  /*!< Set when receiving Quick command */
#define I2C_SMB_RAWISR_ARP_NOTIFY_Pos       (3)
#define I2C_SMB_RAWISR_ARP_NOTIFY_Msk       (0x01U << I2C_SMB_RAWISR_ARP_NOTIFY_Pos) /*!< Set when receiving the Host Notify command */
#define I2C_SMB_RAWISR_ARP_PRE_Pos          (4)
#define I2C_SMB_RAWISR_ARP_PRE_Msk          (0x01U << I2C_SMB_RAWISR_ARP_PRE_Pos)    /*!< Set when receiving the Prepare to ARP command */
#define I2C_SMB_RAWISR_ARP_RST_Pos          (5)
#define I2C_SMB_RAWISR_ARP_RST_Msk          (0x01U << I2C_SMB_RAWISR_ARP_RST_Pos)    /*!< Set when receiving Reset ARP command */
#define I2C_SMB_RAWISR_ARP_UDID_Pos         (6)
#define I2C_SMB_RAWISR_ARP_UDID_Msk         (0x01U << I2C_SMB_RAWISR_ARP_UDID_Pos)   /*!< Set when receiving the Get UDID ARP command */
#define I2C_SMB_RAWISR_ARP_ASSGN_Pos        (7)
#define I2C_SMB_RAWISR_ARP_ASSGN_Msk        (0x01U << I2C_SMB_RAWISR_ARP_ASSGN_Pos)  /*!< Set when receiving the Assign Address ARP command */
#define I2C_SMB_RAWISR_PEC_NACK_Pos         (8)
#define I2C_SMB_RAWISR_PEC_NACK_Msk         (0x01U << I2C_SMB_RAWISR_PEC_NACK_Pos)   /*!< Set when the slave device does not respond to the PEC byte of the ARP command */
#define I2C_SMB_RAWISR_SMB_ALT_Pos          (10)
#define I2C_SMB_RAWISR_SMB_ALT_Msk          (0x01U << I2C_SMB_RAWISR_SMB_ALT_Pos)    /*!< Set when slave device alarm is detected */

/**
  * @brief I2C_SMB_ICR Register Bit Definition
  */
#define I2C_SMB_ICR_CLR_SLV_TMO_Pos         (0)
#define I2C_SMB_ICR_CLR_SLV_TMO_Msk         (0x01U << I2C_SMB_ICR_CLR_SLV_TMO_Pos)    /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_MST_TMO_Pos         (1)
#define I2C_SMB_ICR_CLR_MST_TMO_Msk         (0x01U << I2C_SMB_ICR_CLR_MST_TMO_Pos)    /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_QUICK_Pos       (2)
#define I2C_SMB_ICR_CLR_ARP_QUICK_Msk       (0x01U << I2C_SMB_ICR_CLR_ARP_QUICK_Pos)  /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_NOTIFY_Pos      (3)
#define I2C_SMB_ICR_CLR_ARP_NOTIFY_Msk      (0x01U << I2C_SMB_ICR_CLR_ARP_NOTIFY_Pos) /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_PRE_Pos         (4)
#define I2C_SMB_ICR_CLR_ARP_PRE_Msk         (0x01U << I2C_SMB_ICR_CLR_ARP_PRE_Pos)    /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_RST_Pos         (5)
#define I2C_SMB_ICR_CLR_ARP_RST_Msk         (0x01U << I2C_SMB_ICR_CLR_ARP_RST_Pos)    /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_UDID_Pos        (6)
#define I2C_SMB_ICR_CLR_ARP_UDID_Msk        (0x01U << I2C_SMB_ICR_CLR_ARP_UDID_Pos)   /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_ARP_ASSGN_Pos       (7)
#define I2C_SMB_ICR_CLR_ARP_ASSGN_Msk       (0x01U << I2C_SMB_ICR_CLR_ARP_ASSGN_Pos)  /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_PEC_NACK_Pos        (8)
#define I2C_SMB_ICR_CLR_PEC_NACK_Msk        (0x01U << I2C_SMB_ICR_CLR_PEC_NACK_Pos)   /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */
#define I2C_SMB_ICR_CLR_SMB_ALT_Pos         (10)
#define I2C_SMB_ICR_CLR_SMB_ALT_Msk         (0x01U << I2C_SMB_ICR_CLR_SMB_ALT_Pos)    /*!< Write this bit to clear and I2C_ SMB_ RAWISR corresponding interrupt */

/**
  * @brief I2C_OPT_SAR Register Bit Definition
  */
#define I2C_OPT_SAR_ADDR_Pos                (0)
#define I2C_OPT_SAR_ADDR_Msk                (0x7FU << I2C_OPT_SAR_ADDR_Pos) /*!< Slave address of I2C interface in SMBus slave mode */

/**
  * @brief I2C_SMB_UDID_LSB Register Bit Definition
  */
#define I2C_SMB_UDID_LSB_Pos                (0)
#define I2C_SMB_UDID_LSB_Msk                (0xFFFFFFFFU << I2C_SMB_UDID_LSB_Pos) /*!< Configure SMBus UDID [31:0] */

/**
  * @brief I2C_SMB_UDID_MSB0 Register Bit Definition
  */
#define I2C_SMB_UDID_MSB0_Pos               (0)
#define I2C_SMB_UDID_MSB0_Msk               (0xFFFFFFFFU << I2C_SMB_UDID_MSB0_Pos) /*!< Configure SMBus UDID [63:32] */

/**
  * @brief I2C_SMB_UDID_MSB1 Register Bit Definition
  */
#define I2C_SMB_UDID_MSB1_Pos               (0)
#define I2C_SMB_UDID_MSB1_Msk               (0xFFFFFFFFU << I2C_SMB_UDID_MSB1_Pos) /*!< Configure SMBus UDID [95:64] */

/**
  * @brief I2C_SMB_UDID_MSB2 Register Bit Definition
  */
#define I2C_SMB_UDID_MSB2_Pos               (0)
#define I2C_SMB_UDID_MSB2_Msk               (0xFFFFFFFFU << I2C_SMB_UDID_MSB2_Pos) /*!< Configure SMBus UDID [127:96] */

/**
  * @brief I2C_SLVMASK Register Bit Definition
  */
#define I2C_SLVMASK_MASK_Pos                (0)
#define I2C_SLVMASK_MASK_Msk                (0x3FFU << I2C_SLVMASK_MASK_Pos) /*!< Slave address mask */

/**
  * @brief I2C_SLVRCVADDR Register Bit Definition
  */
#define I2C_SLVRCVADDR_ADDR_Pos             (0)
#define I2C_SLVRCVADDR_ADDR_Msk             (0x3FFU << I2C_SLVRCVADDR_ADDR_Pos) /*!< Address actually received from the device */

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

