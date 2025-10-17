/***********************************************************************************************************************
    @file     reg_dma.h
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

#ifndef __REG_DMA_H
#define __REG_DMA_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

#if defined(__CC_ARM)
#pragma anon_unions
#endif

/**
  * @brief DMA Base Address Definition
  */
#define DMA1_BASE                       (AHB1PERIPH_BASE + 0x0000) /*!< Base Address: 0x40020000 */
#define DMA1_Channel1_BASE              (AHB1PERIPH_BASE + 0x0008) /*!< Base Address: 0x40020008 */
#define DMA1_Channel2_BASE              (AHB1PERIPH_BASE + 0x001C) /*!< Base Address: 0x4002001C */
#define DMA1_Channel3_BASE              (AHB1PERIPH_BASE + 0x0030) /*!< Base Address: 0x40020030 */
#define DMA1_Channel4_BASE              (AHB1PERIPH_BASE + 0x0044) /*!< Base Address: 0x40020044 */
#define DMA1_Channel5_BASE              (AHB1PERIPH_BASE + 0x0058) /*!< Base Address: 0x40020058 */
#define DMA1_Channel6_BASE              (AHB1PERIPH_BASE + 0x006C) /*!< Base Address: 0x4002006C */
#define DMA1_Channel7_BASE              (AHB1PERIPH_BASE + 0x0080) /*!< Base Address: 0x40020080 */
#define DMA1_Channel8_BASE              (AHB1PERIPH_BASE + 0x0094) /*!< Base Address: 0x40020094 */

#define DMA2_BASE                       (AHB1PERIPH_BASE + 0x0400) /*!< Base Address: 0x40020400 */
#define DMA2_Channel1_BASE              (AHB1PERIPH_BASE + 0x0408) /*!< Base Address: 0x40020408 */
#define DMA2_Channel2_BASE              (AHB1PERIPH_BASE + 0x041C) /*!< Base Address: 0x4002041C */
#define DMA2_Channel3_BASE              (AHB1PERIPH_BASE + 0x0430) /*!< Base Address: 0x40020430 */
#define DMA2_Channel4_BASE              (AHB1PERIPH_BASE + 0x0444) /*!< Base Address: 0x40020444 */
#define DMA2_Channel5_BASE              (AHB1PERIPH_BASE + 0x0458) /*!< Base Address: 0x40020458 */
#define DMA2_Channel6_BASE              (AHB1PERIPH_BASE + 0x046C) /*!< Base Address: 0x4002046C */
#define DMA2_Channel7_BASE              (AHB1PERIPH_BASE + 0x0480) /*!< Base Address: 0x40020480 */
#define DMA2_Channel8_BASE              (AHB1PERIPH_BASE + 0x0494) /*!< Base Address: 0x40020494 */

/**
  * @brief DMA Register Structure Definition
  */
typedef struct
{
    __IO uint32_t CCR;                 /*!< DMA channel x configuration register           offset: 0x00 */
    __IO uint32_t CNDTR;               /*!< DMA channel x number of data register          offset: 0x04 */
    __IO uint32_t CPAR;                /*!< DMA channel x peripheral address register      offset: 0x08 */
    __IO uint32_t CMAR;                /*!< DMA channel x memory address register          offset: 0x0C */
} DMA_Channel_TypeDef;

typedef struct
{
    __IO uint32_t ISR;                 /*!< Interrupt Status Register                      offset: 0x00 */
    __IO uint32_t IFCR;                /*!< Interrupt Flag Clear Register                  offset: 0x04 */
} DMA_TypeDef;

/**
  * @brief DMA type pointer Definition
  */
#define DMA1                            ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1                   ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2                   ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3                   ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4                   ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5                   ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6                   ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7                   ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define DMA1_Channel8                   ((DMA_Channel_TypeDef *)DMA1_Channel8_BASE)

#define DMA2                            ((DMA_TypeDef *)DMA2_BASE)
#define DMA2_Channel1                   ((DMA_Channel_TypeDef *)DMA2_Channel1_BASE)
#define DMA2_Channel2                   ((DMA_Channel_TypeDef *)DMA2_Channel2_BASE)
#define DMA2_Channel3                   ((DMA_Channel_TypeDef *)DMA2_Channel3_BASE)
#define DMA2_Channel4                   ((DMA_Channel_TypeDef *)DMA2_Channel4_BASE)
#define DMA2_Channel5                   ((DMA_Channel_TypeDef *)DMA2_Channel5_BASE)
#define DMA2_Channel6                   ((DMA_Channel_TypeDef *)DMA2_Channel6_BASE)
#define DMA2_Channel7                   ((DMA_Channel_TypeDef *)DMA2_Channel7_BASE)
#define DMA2_Channel8                   ((DMA_Channel_TypeDef *)DMA2_Channel8_BASE)

/**
  * @brief DMA_ISR Register Bit Definition
  */
#define DMA_ISR_GIF1_Pos                (0)
#define DMA_ISR_GIF1_Msk                (0x01U << DMA_ISR_GIF1_Pos)  /*!< Channel 1 Global interrupt flag */
#define DMA_ISR_TCIF1_Pos               (1)
#define DMA_ISR_TCIF1_Msk               (0x01U << DMA_ISR_TCIF1_Pos) /*!< Channel 1 Transfer Complete flag */
#define DMA_ISR_HTIF1_Pos               (2)
#define DMA_ISR_HTIF1_Msk               (0x01U << DMA_ISR_HTIF1_Pos) /*!< Channel 1 Half Transfer flag */
#define DMA_ISR_TEIF1_Pos               (3)
#define DMA_ISR_TEIF1_Msk               (0x01U << DMA_ISR_TEIF1_Pos) /*!< Channel 1 Transfer Error flag */
#define DMA_ISR_GIF2_Pos                (4)
#define DMA_ISR_GIF2_Msk                (0x01U << DMA_ISR_GIF2_Pos)  /*!< Channel 2 Global interrupt flag */
#define DMA_ISR_TCIF2_Pos               (5)
#define DMA_ISR_TCIF2_Msk               (0x01U << DMA_ISR_TCIF2_Pos) /*!< Channel 2 Transfer Complete flag */
#define DMA_ISR_HTIF2_Pos               (6)
#define DMA_ISR_HTIF2_Msk               (0x01U << DMA_ISR_HTIF2_Pos) /*!< Channel 2 Half Transfer flag */
#define DMA_ISR_TEIF2_Pos               (7)
#define DMA_ISR_TEIF2_Msk               (0x01U << DMA_ISR_TEIF2_Pos) /*!< Channel 2 Transfer Error flag */
#define DMA_ISR_GIF3_Pos                (8)
#define DMA_ISR_GIF3_Msk                (0x01U << DMA_ISR_GIF3_Pos)  /*!< Channel 3 Global interrupt flag */
#define DMA_ISR_TCIF3_Pos               (9)
#define DMA_ISR_TCIF3_Msk               (0x01U << DMA_ISR_TCIF3_Pos) /*!< Channel 3 Transfer Complete flag */
#define DMA_ISR_HTIF3_Pos               (10)
#define DMA_ISR_HTIF3_Msk               (0x01U << DMA_ISR_HTIF3_Pos) /*!< Channel 3 Half Transfer flag */
#define DMA_ISR_TEIF3_Pos               (11)
#define DMA_ISR_TEIF3_Msk               (0x01U << DMA_ISR_TEIF3_Pos) /*!< Channel 3 Transfer Error flag */
#define DMA_ISR_GIF4_Pos                (12)
#define DMA_ISR_GIF4_Msk                (0x01U << DMA_ISR_GIF4_Pos)  /*!< Channel 4 Global interrupt flag */
#define DMA_ISR_TCIF4_Pos               (13)
#define DMA_ISR_TCIF4_Msk               (0x01U << DMA_ISR_TCIF4_Pos) /*!< Channel 4 Transfer Complete flag */
#define DMA_ISR_HTIF4_Pos               (14)
#define DMA_ISR_HTIF4_Msk               (0x01U << DMA_ISR_HTIF4_Pos) /*!< Channel 4 Half Transfer flag */
#define DMA_ISR_TEIF4_Pos               (15)
#define DMA_ISR_TEIF4_Msk               (0x01U << DMA_ISR_TEIF4_Pos) /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF5_Pos                (16)
#define DMA_ISR_GIF5_Msk                (0x01U << DMA_ISR_GIF5_Pos)  /*!< Channel 5 Global interrupt flag */
#define DMA_ISR_TCIF5_Pos               (17)
#define DMA_ISR_TCIF5_Msk               (0x01U << DMA_ISR_TCIF5_Pos) /*!< Channel 5 Transfer Complete flag */
#define DMA_ISR_HTIF5_Pos               (18)
#define DMA_ISR_HTIF5_Msk               (0x01U << DMA_ISR_HTIF5_Pos) /*!< Channel 5 Half Transfer flag */
#define DMA_ISR_TEIF5_Pos               (19)
#define DMA_ISR_TEIF5_Msk               (0x01U << DMA_ISR_TEIF5_Pos) /*!< Channel 5 Transfer Error flag */

#define DMA_ISR_GIF6_Pos                (20)
#define DMA_ISR_GIF6_Msk                (0x01U << DMA_ISR_GIF6_Pos)  /*!< Channel 6 Global interrupt flag */
#define DMA_ISR_TCIF6_Pos               (21)
#define DMA_ISR_TCIF6_Msk               (0x01U << DMA_ISR_TCIF6_Pos) /*!< Channel 6 Transfer Complete flag */
#define DMA_ISR_HTIF6_Pos               (22)
#define DMA_ISR_HTIF6_Msk               (0x01U << DMA_ISR_HTIF6_Pos) /*!< Channel 6 Half Transfer flag */
#define DMA_ISR_TEIF6_Pos               (23)
#define DMA_ISR_TEIF6_Msk               (0x01U << DMA_ISR_TEIF6_Pos) /*!< Channel 6 Transfer Error flag */
#define DMA_ISR_GIF7_Pos                (24)
#define DMA_ISR_GIF7_Msk                (0x01U << DMA_ISR_GIF7_Pos)  /*!< Channel 7 Global interrupt flag */
#define DMA_ISR_TCIF7_Pos               (25)
#define DMA_ISR_TCIF7_Msk               (0x01U << DMA_ISR_TCIF7_Pos) /*!< Channel 7 Transfer Complete flag */
#define DMA_ISR_HTIF7_Pos               (26)
#define DMA_ISR_HTIF7_Msk               (0x01U << DMA_ISR_HTIF7_Pos) /*!< Channel 7 Half Transfer flag */
#define DMA_ISR_TEIF7_Pos               (27)
#define DMA_ISR_TEIF7_Msk               (0x01U << DMA_ISR_TEIF7_Pos) /*!< Channel 7 Transfer Error flag */

#define DMA_ISR_GIF8_Pos                (28)
#define DMA_ISR_GIF8_Msk                (0x01U << DMA_ISR_GIF8_Pos)  /*!< Channel 8 Global interrupt flag */
#define DMA_ISR_TCIF8_Pos               (29)
#define DMA_ISR_TCIF8_Msk               (0x01U << DMA_ISR_TCIF8_Pos) /*!< Channel 8 Transfer Complete flag */
#define DMA_ISR_HTIF8_Pos               (30)
#define DMA_ISR_HTIF8_Msk               (0x01U << DMA_ISR_HTIF8_Pos) /*!< Channel 8 Half Transfer flag */
#define DMA_ISR_TEIF8_Pos               (31)
#define DMA_ISR_TEIF8_Msk               (0x01U << DMA_ISR_TEIF8_Pos) /*!< Channel 8 Transfer Error flag */

/**
  * @brief DMA_IFCR Register Bit Definition
  */
#define DMA_IFCR_CGIF1_Pos              (0)
#define DMA_IFCR_CGIF1_Msk              (0x01U << DMA_IFCR_CGIF1_Pos)  /*!< Channel 1 Global interrupt clearr */
#define DMA_IFCR_CTCIF1_Pos             (1)
#define DMA_IFCR_CTCIF1_Msk             (0x01U << DMA_IFCR_CTCIF1_Pos) /*!< Channel 1 Transfer Complete clear */
#define DMA_IFCR_CHTIF1_Pos             (2)
#define DMA_IFCR_CHTIF1_Msk             (0x01U << DMA_IFCR_CHTIF1_Pos) /*!< Channel 1 Half Transfer clear */
#define DMA_IFCR_CTEIF1_Pos             (3)
#define DMA_IFCR_CTEIF1_Msk             (0x01U << DMA_IFCR_CTEIF1_Pos) /*!< Channel 1 Transfer Error clear */
#define DMA_IFCR_CGIF2_Pos              (4)
#define DMA_IFCR_CGIF2_Msk              (0x01U << DMA_IFCR_CGIF2_Pos)  /*!< Channel 2 Global interrupt clear */
#define DMA_IFCR_CTCIF2_Pos             (5)
#define DMA_IFCR_CTCIF2_Msk             (0x01U << DMA_IFCR_CTCIF2_Pos) /*!< Channel 2 Transfer Complete clear */
#define DMA_IFCR_CHTIF2_Pos             (6)
#define DMA_IFCR_CHTIF2_Msk             (0x01U << DMA_IFCR_CHTIF2_Pos) /*!< Channel 2 Half Transfer clear */
#define DMA_IFCR_CTEIF2_Pos             (7)
#define DMA_IFCR_CTEIF2_Msk             (0x01U << DMA_IFCR_CTEIF2_Pos) /*!< Channel 2 Transfer Error clear */
#define DMA_IFCR_CGIF3_Pos              (8)
#define DMA_IFCR_CGIF3_Msk              (0x01U << DMA_IFCR_CGIF3_Pos)  /*!< Channel 3 Global interrupt clear */
#define DMA_IFCR_CTCIF3_Pos             (9)
#define DMA_IFCR_CTCIF3_Msk             (0x01U << DMA_IFCR_CTCIF3_Pos) /*!< Channel 3 Transfer Complete clear */
#define DMA_IFCR_CHTIF3_Pos             (10)
#define DMA_IFCR_CHTIF3_Msk             (0x01U << DMA_IFCR_CHTIF3_Pos) /*!< Channel 3 Half Transfer clear */
#define DMA_IFCR_CTEIF3_Pos             (11)
#define DMA_IFCR_CTEIF3_Msk             (0x01U << DMA_IFCR_CTEIF3_Pos) /*!< Channel 3 Transfer Error clear */
#define DMA_IFCR_CGIF4_Pos              (12)
#define DMA_IFCR_CGIF4_Msk              (0x01U << DMA_IFCR_CGIF4_Pos)  /*!< Channel 4 Global interrupt clear */
#define DMA_IFCR_CTCIF4_Pos             (13)
#define DMA_IFCR_CTCIF4_Msk             (0x01U << DMA_IFCR_CTCIF4_Pos) /*!< Channel 4 Transfer Complete clear */
#define DMA_IFCR_CHTIF4_Pos             (14)
#define DMA_IFCR_CHTIF4_Msk             (0x01U << DMA_IFCR_CHTIF4_Pos) /*!< Channel 4 Half Transfer clear */
#define DMA_IFCR_CTEIF4_Pos             (15)
#define DMA_IFCR_CTEIF4_Msk             (0x01U << DMA_IFCR_CTEIF4_Pos) /*!< Channel 4 Transfer Error clear */
#define DMA_IFCR_CGIF5_Pos              (16)
#define DMA_IFCR_CGIF5_Msk              (0x01U << DMA_IFCR_CGIF5_Pos)  /*!< Channel 5 Global interrupt clear */
#define DMA_IFCR_CTCIF5_Pos             (17)
#define DMA_IFCR_CTCIF5_Msk             (0x01U << DMA_IFCR_CTCIF5_Pos) /*!< Channel 5 Transfer Complete clear */
#define DMA_IFCR_CHTIF5_Pos             (18)
#define DMA_IFCR_CHTIF5_Msk             (0x01U << DMA_IFCR_CHTIF5_Pos) /*!< Channel 5 Half Transfer clear */
#define DMA_IFCR_CTEIF5_Pos             (19)
#define DMA_IFCR_CTEIF5_Msk             (0x01U << DMA_IFCR_CTEIF5_Pos) /*!< Channel 5 Transfer Error clear */

#define DMA_IFCR_CGIF6_Pos              (20)
#define DMA_IFCR_CGIF6_Msk              (0x01U << DMA_IFCR_CGIF6_Pos)  /*!< Channel 6 Global interrupt clear */
#define DMA_IFCR_CTCIF6_Pos             (21)
#define DMA_IFCR_CTCIF6_Msk             (0x01U << DMA_IFCR_CTCIF6_Pos) /*!< Channel 6 Transfer Complete clear */
#define DMA_IFCR_CHTIF6_Pos             (22)
#define DMA_IFCR_CHTIF6_Msk             (0x01U << DMA_IFCR_CHTIF6_Pos) /*!< Channel 6 Half Transfer clear */
#define DMA_IFCR_CTEIF6_Pos             (23)
#define DMA_IFCR_CTEIF6_Msk             (0x01U << DMA_IFCR_CTEIF6_Pos) /*!< Channel 6 Transfer Error clear */
#define DMA_IFCR_CGIF7_Pos              (24)
#define DMA_IFCR_CGIF7_Msk              (0x01U << DMA_IFCR_CGIF7_Pos)  /*!< Channel 7 Global interrupt clear */
#define DMA_IFCR_CTCIF7_Pos             (25)
#define DMA_IFCR_CTCIF7_Msk             (0x01U << DMA_IFCR_CTCIF7_Pos) /*!< Channel 7 Transfer Complete clear */
#define DMA_IFCR_CHTIF7_Pos             (26)
#define DMA_IFCR_CHTIF7_Msk             (0x01U << DMA_IFCR_CHTIF7_Pos) /*!< Channel 7 Half Transfer clear */
#define DMA_IFCR_CTEIF7_Pos             (27)
#define DMA_IFCR_CTEIF7_Msk             (0x01U << DMA_IFCR_CTEIF7_Pos) /*!< Channel 7 Transfer Error clear */

#define DMA_IFCR_CGIF8_Pos              (28)
#define DMA_IFCR_CGIF8_Msk              (0x01U << DMA_IFCR_CGIF8_Pos)  /*!< Channel 8 Global interrupt clear */
#define DMA_IFCR_CTCIF8_Pos             (29)
#define DMA_IFCR_CTCIF8_Msk             (0x01U << DMA_IFCR_CTCIF8_Pos) /*!< Channel 8 Transfer Complete clear */
#define DMA_IFCR_CHTIF8_Pos             (30)
#define DMA_IFCR_CHTIF8_Msk             (0x01U << DMA_IFCR_CHTIF8_Pos) /*!< Channel 8 Half Transfer clear */
#define DMA_IFCR_CTEIF8_Pos             (31)
#define DMA_IFCR_CTEIF8_Msk             (0x01U << DMA_IFCR_CTEIF8_Pos) /*!< Channel 8 Transfer Error clear */

/**
  * @brief DMA_CCR Register Bit Definition
  */
#define DMA_CCR_EN_Pos                  (0)
#define DMA_CCR_EN_Msk                  (0x01U << DMA_CCR_EN_Pos)      /*!< Channel enable */
#define DMA_CCR_TCIE_Pos                (1)
#define DMA_CCR_TCIE_Msk                (0x01U << DMA_CCR_TCIE_Pos)    /*!< Transfer complete interrupt enable */
#define DMA_CCR_HTIE_Pos                (2)
#define DMA_CCR_HTIE_Msk                (0x01U << DMA_CCR_HTIE_Pos)    /*!< Half Transfer interrupt enable */
#define DMA_CCR_TEIE_Pos                (3)
#define DMA_CCR_TEIE_Msk                (0x01U << DMA_CCR_TEIE_Pos)    /*!< Transfer error interrupt enable */
#define DMA_CCR_DIR_Pos                 (4)
#define DMA_CCR_DIR_Msk                 (0x01U << DMA_CCR_DIR_Pos)     /*!< Data transfer direction */
#define DMA_CCR_CIRC_Pos                (5)
#define DMA_CCR_CIRC_Msk                (0x01U << DMA_CCR_CIRC_Pos)    /*!< Circular mode */
#define DMA_CCR_PINC_Pos                (6)
#define DMA_CCR_PINC_Msk                (0x01U << DMA_CCR_PINC_Pos)    /*!< Peripheral increment mode */
#define DMA_CCR_MINC_Pos                (7)
#define DMA_CCR_MINC_Msk                (0x01U << DMA_CCR_MINC_Pos)    /*!< Memory increment mode */

#define DMA_CCR_PSIZE_Pos               (8)
#define DMA_CCR_PSIZE_Msk               (0x03U << DMA_CCR_PSIZE_Pos)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR_PSIZE_0                 (0x01U << DMA_CCR_PSIZE_Pos)   /*!< Bit0 */
#define DMA_CCR_PSIZE_1                 (0x02U << DMA_CCR_PSIZE_Pos)   /*!< Bit1 */

#define DMA_CCR_PSIZE_BYTE              (0x00U << DMA_CCR_PSIZE_Pos)   /*!< DMA Peripheral Data Size Byte */
#define DMA_CCR_PSIZE_HALFWORD          (0x01U << DMA_CCR_PSIZE_Pos)   /*!< DMA Peripheral Data Size HalfWord */
#define DMA_CCR_PSIZE_WORD              (0x02U << DMA_CCR_PSIZE_Pos)   /*!< DMA Peripheral Data Size Word */

#define DMA_CCR_MSIZE_Pos               (10)
#define DMA_CCR_MSIZE_Msk               (0x03U << DMA_CCR_MSIZE_Pos)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR_MSIZE_0                 (0x01U << DMA_CCR_MSIZE_Pos)   /*!< Bit0 */
#define DMA_CCR_MSIZE_1                 (0x02U << DMA_CCR_MSIZE_Pos)   /*!< Bit1 */

#define DMA_CCR_MSIZE_BYTE              (0x00U << DMA_CCR_MSIZE_Pos)   /*!< DMA Memory Data Size Byte */
#define DMA_CCR_MSIZE_HALFWORD          (0x01U << DMA_CCR_MSIZE_Pos)   /*!< DMA Memory Data Size HalfWord */
#define DMA_CCR_MSIZE_WORD              (0x02U << DMA_CCR_MSIZE_Pos)   /*!< DMA Memory Data Size Word */

#define DMA_CCR_PL_Pos                  (12)
#define DMA_CCR_PL_Msk                  (0x03U << DMA_CCR_PL_Pos)      /*!< PL[1:0] bits(Channel Priority level) */
#define DMA_CCR_PL_0                    (0x01U << DMA_CCR_PL_Pos)      /*!< Bit0 */
#define DMA_CCR_PL_1                    (0x02U << DMA_CCR_PL_Pos)      /*!< Bit1 */

#define DMA_CCR_PL_Low                  (0x00U << DMA_CCR_PL_Pos)      /*!< DMA Priority Low */
#define DMA_CCR_PL_Medium               (0x01U << DMA_CCR_PL_Pos)      /*!< DMA Priority Medium */
#define DMA_CCR_PL_High                 (0x02U << DMA_CCR_PL_Pos)      /*!< DMA Priority High */
#define DMA_CCR_PL_VeryHigh             (0x03U << DMA_CCR_PL_Pos)      /*!< DMA Priority VeryHigh */
#define DMA_CCR_MEM2MEM_Pos             (14)
#define DMA_CCR_MEM2MEM_Msk             (0x01U << DMA_CCR_MEM2MEM_Pos) /*!< Memory to memory mode */

#define DMA_CCR_ARE_Pos                 (15)
#define DMA_CCR_ARE_Msk                 (0x01U << DMA_CCR_ARE_Pos)     /*!< Auto-Reload Enable bit */

#define DMA_CCR_BURSTEN_Pos             (16)
#define DMA_CCR_BURSTEN_Msk             (0x01U << DMA_CCR_BURSTEN_Pos) /*!< burst Enable bit */

/**
  * @brief DMA_CNDTR Register Bit Definition
  */
#define DMA_CNDTR_NDT_Pos               (0)
#define DMA_CNDTR_NDT_Msk               (0xFFFFU << DMA_CNDTR_NDT_Pos) /*!< Number of data to Transfer */

/**
  * @brief DMA_CPAR Register Bit Definition
  */
#define DMA_CPAR_PA_Pos                 (0)
#define DMA_CPAR_PA_Msk                 (0xFFFFFFFFU << DMA_CPAR_PA_Pos) /*!< Peripheral Address */

/**
  * @brief DMA_CMAR Register Bit Definition
  */
#define DMA_CMAR_MA_Pos                 (0)
#define DMA_CMAR_MA_Msk                 (0xFFFFFFFFU << DMA_CMAR_MA_Pos) /*!< Peripheral Address */

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

