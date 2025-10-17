/***********************************************************************************************************************
    @file     hal_mds.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE MD FIRMWARE FUNCTIONS.
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
#define _HAL_MDS_C_

/* Files includes ------------------------------------------------------------*/
#include "hal_mds.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @addtogroup MDS
  * @{
  */

/** @defgroup MDS_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @addtogroup MDS_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the MDS module clock.
  * @param  None.
  * @retval None.
  */
void MDS_DeInit(void)
{
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_MDS, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_MDS, DISABLE);
}

/**
  * @brief  Fills each init_struct member with its default value.
  * @param  init_struct: pointer to a MDS_TriggerNotCLU_TypeDef structure
  * @retval None.
  */
void MDS_TriggerNotCluStructInit(MDS_TriggerNotCLU_TypeDef *init_struct)
{
    init_struct->MDS_Channel            = MDS_TriggerOutput_ADC1_EXT_TRIGGER;
    init_struct->MDS_InputEdge          = MDS_InputEdge_Rising;
    init_struct->MDS_InputSource        = MDS_TriggerSource_LOW_LEVEL;
}

/**
  * @brief  Initialize the Trigger without CLU by init_struct param.
  * @param  init_struct: pointer to a MDS_TriggerNotCLU_TypeDef structure.
  * @retval None.
  */
void MDS_TriggerNotCluInit(MDS_TriggerNotCLU_TypeDef *init_struct)
{
    uint8_t channel = init_struct->MDS_Channel;

    MDS->TRIGCR[channel] &= ~(0x01U << MDS_TRIGCR_CLUEN_Pos);
    MODIFY_REG(MDS->TRIGCR[channel], (MDS_TRIGCR_EDGESEL_Msk | MDS_TRIGCR_TRGSEL_Msk), ((((uint32_t)(init_struct->MDS_InputEdge)) << MDS_TRIGCR_EDGESEL_Pos) | \
                                                                                        ((((uint32_t)(init_struct->MDS_InputSource)) <<
                                                                                          MDS_TRIGCR_TRGSEL_Pos))));
}

/**
  * @brief  Fills each init_struct member with its default value.
  * @param  init_struct: pointer to a MDS_TriggerCLU_TypeDef structure.
  * @retval None.
  */
void MDS_TriggerCluStructInit(MDS_TriggerCLU_TypeDef *init_struct)
{
    uint8_t i;

    init_struct->MDS_Channel            = MDS_TriggerOutput_ADC1_EXT_TRIGGER;
    init_struct->MDS_CluNumber          = MDS_TriggerSourceSel_CLU0;

    for (i = 0; i < MDS_CLUGROUP_INPUT_MAX; i++)
    {
        init_struct->CluInputSource[i]  = MDS_TriggerSource_LOW_LEVEL;
        init_struct->CluInputEdge[i]    = MDS_InputEdge_Rising;
    }
}

/**
  * @brief  Initialize the Trigger with CLU by init_struct param.
  * @param  init_struct: pointer to a MDS_TriggerCLU_TypeDef structure.
  * @retval None.
  */
void MDS_TriggerCluInit(MDS_TriggerCLU_TypeDef *init_struct)
{
    uint8_t channel;
    uint8_t clu_number;

    channel    = init_struct->MDS_Channel;
    clu_number = init_struct->MDS_CluNumber;
    MDS->TRIGCR[channel] &= ~(0x01U << MDS_TRIGCR_CLUEN_Pos);

    MODIFY_REG(MDS->TRIGCLUSEL[clu_number], (MDS_TRIGCLUSEL_CLUIN0SEL_Msk | MDS_TRIGCLUSEL_CLUIN1SEL_Msk | MDS_TRIGCLUSEL_CLUIN2SEL_Msk | MDS_TRIGCLUSEL_CLUIN3SEL_Msk), ((((uint32_t)(init_struct->CluInputSource[3])) << MDS_TRIGCLUSEL_CLUIN3SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[2])) << MDS_TRIGCLUSEL_CLUIN2SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[1])) << MDS_TRIGCLUSEL_CLUIN1SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[0])) << MDS_TRIGCLUSEL_CLUIN0SEL_Pos)));
    MODIFY_REG(MDS->TRIGCLUCFG[clu_number], (MDS_TRIGCLUCFG_CLUIN0ED_Msk | MDS_TRIGCLUCFG_CLUIN1ED_Msk | MDS_TRIGCLUCFG_CLUIN2ED_Msk | MDS_TRIGCLUCFG_CLUIN3ED_Msk), ((((uint32_t)(init_struct->CluInputEdge[3])) << MDS_TRIGCLUCFG_CLUIN3ED_Pos) | \
                                                                                                                                                                      (((
                                                                                                                                                                            uint32_t)(
                                                                                                                                                                            init_struct->CluInputEdge[2])) << MDS_TRIGCLUCFG_CLUIN2ED_Pos) | \
                                                                                                                                                                      (((
                                                                                                                                                                            uint32_t)(
                                                                                                                                                                            init_struct->CluInputEdge[1])) << MDS_TRIGCLUCFG_CLUIN1ED_Pos) | \
                                                                                                                                                                      (((
                                                                                                                                                                            uint32_t)(
                                                                                                                                                                            init_struct->CluInputEdge[0])) << MDS_TRIGCLUCFG_CLUIN0ED_Pos)));

    MODIFY_REG(MDS->TRIGCR[channel], MDS_TRIGCR_CLUSEL_Msk, ((uint32_t)clu_number << MDS_TRIGCR_CLUSEL_Pos));
    MDS->TRIGCR[channel] |= (0x01U << MDS_TRIGCR_CLUEN_Pos);
}

/**
  * @brief  Enables or disables the specified trigger.
  * @param  state: new state of the soft trigger.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MDS_SoftTriggerCmd(FunctionalState state)
{
    (state) ?                                           \
    (MDS->SWTRIG |= (0x01U << MDS_SWTRIG_SWTRIG_Pos)) : \
    (MDS->SWTRIG &= ~(0x01U << MDS_SWTRIG_SWTRIG_Pos));
}

/**
  * @brief  Fills each init_struct member with its default value.
  * @param  init_struct: pointer to a MDS_ConnectionNotCLU_TypeDef structure
  * @retval None.
  */
void MDS_ConnectionNotCluStructInit(MDS_ConnectionNotCLU_TypeDef *init_struct)
{
    init_struct->MDS_Channel            = MDS_ConnectionOutput_COMP1_BLANKING;
    init_struct->MDS_InputSource        = MDS_ConnectionSource_LOW_LEVEL;
}

/**
  * @brief  Initialize the Connection without CLU by init_struct param.
  * @param  init_struct: pointer to a MDS_ConnectionNotCLU_TypeDef structure.
  * @retval None.
  */
void MDS_ConnectionNotCluInit(MDS_ConnectionNotCLU_TypeDef *init_struct)
{
    uint8_t channel = init_struct->MDS_Channel;

    MDS->CONNCR[channel] &= ~(0x01U << MDS_CONNCR_CLUEN_Pos);
    MODIFY_REG(MDS->CONNCR[channel], MDS_CONNCR_TRGSEL_Msk, (((uint32_t)(init_struct->MDS_InputSource)) << MDS_CONNCR_TRGSEL_Pos));
}

/**
  * @brief  Fills each init_struct member with its default value.
  * @param  init_struct: pointer to a MDS_ConnectionCLU_TypeDef structure.
  * @retval None.
  */
void MDS_ConnectionCluStructInit(MDS_ConnectionCLU_TypeDef *init_struct)
{
    uint8_t i;

    init_struct->MDS_Channel            = MDS_ConnectionOutput_COMP1_BLANKING;
    init_struct->MDS_CluNumber          = MDS_TriggerSourceSel_CLU0;

    for (i = 0; i < MDS_CLUGROUP_INPUT_MAX; i++)
    {
        init_struct->CluInputSource[i]  = MDS_ConnectionSource_LOW_LEVEL;
        init_struct->CluInputInvert[i]  = MDS_CLU_Input_NonInvert;
    }

    init_struct->MDS_CluLogicMode       = MDS_CluLogicMode_OR_OR_OR;
}

/**
  * @brief  Initialize the Connection with CLU by init_struct param.
  * @param  init_struct: pointer to a MDS_ConnectionCLU_TypeDef structure.
  * @retval None.
  */
void MDS_ConnectionCluInit(MDS_ConnectionCLU_TypeDef *init_struct)
{
    uint8_t channel;
    uint8_t clu_number;

    channel    = init_struct->MDS_Channel;
    clu_number = init_struct->MDS_CluNumber;

    MDS->CONNCR[channel] &= ~(0x01U << MDS_CONNCR_CLUEN_Pos);

    MODIFY_REG(MDS->CONNCLUSEL[clu_number], (MDS_CONNCLUSEL_CLUIN0SEL_Msk | MDS_CONNCLUSEL_CLUIN1SEL_Msk | MDS_CONNCLUSEL_CLUIN2SEL_Msk | MDS_CONNCLUSEL_CLUIN3SEL_Msk), ((((uint32_t)(init_struct->CluInputSource[3])) << MDS_CONNCLUSEL_CLUIN3SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[2])) << MDS_CONNCLUSEL_CLUIN2SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[1])) << MDS_CONNCLUSEL_CLUIN1SEL_Pos) | \
                                                                                                                                                                          (((
                                                                                                                                                                                uint32_t)(
                                                                                                                                                                                init_struct->CluInputSource[0])) << MDS_CONNCLUSEL_CLUIN0SEL_Pos)));
    MODIFY_REG(MDS->CONNCLUCFG[clu_number], (MDS_CONNCLUCFG_INV0_Msk | MDS_CONNCLUCFG_INV1_Msk | MDS_CONNCLUCFG_INV2_Msk | MDS_CONNCLUCFG_INV3_Msk | MDS_CONNCLUCFG_CLMODE_Msk), ((((uint32_t)(init_struct->CluInputInvert[3])) << MDS_CONNCLUCFG_INV3_Pos) | \
                                                                                                                                                                                  (((
                                                                                                                                                                                        uint32_t)(
                                                                                                                                                                                        init_struct->CluInputInvert[2])) << MDS_CONNCLUCFG_INV2_Pos) | \
                                                                                                                                                                                  (((
                                                                                                                                                                                        uint32_t)(
                                                                                                                                                                                        init_struct->CluInputInvert[1])) << MDS_CONNCLUCFG_INV1_Pos) | \
                                                                                                                                                                                  (((
                                                                                                                                                                                        uint32_t)(
                                                                                                                                                                                        init_struct->CluInputInvert[0])) << MDS_CONNCLUCFG_INV0_Pos) | \
                                                                                                                                                                                  ((
                                                                                                                                                                                       uint32_t)(
                                                                                                                                                                                       init_struct->MDS_CluLogicMode))));

    MODIFY_REG(MDS->CONNCR[channel], MDS_CONNCR_CLUSEL_Msk, (((uint32_t)clu_number) << MDS_CONNCR_CLUSEL_Pos));

    MDS->CONNCR[channel] |= (0x01U << MDS_CONNCR_CLUEN_Pos);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

