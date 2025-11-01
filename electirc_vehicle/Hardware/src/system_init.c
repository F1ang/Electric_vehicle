#include "system_init.h"

// static void CurrentOffsetCalibrationPWM(void)
// {
//     PSTR_ControlObject t_pObj;
//     u16 t_nuCalPeriod;

//     t_pObj = getCtrlObjectPtr(0);
//     t_nuCalPeriod = t_pObj->m_pDrvCfgPara->mS_GlobalCfg.m_nuADCCaliTimes;
//     CurrentOffsetCalibrationPWM_ID(t_pObj->m_bPWM_Id, &t_pObj->mGlobalDatPackage.mHdToMd, t_nuCalPeriod);
// }

// static void CurrentOffsetCalibration(STR_PubDatHDMD *tS_pHdToMd, u16 t_nuCalPeriod)
// {
//     volatile s32 t_offset1, t_offset2, t_offset3;
//     volatile s32 t_offset4;

//     volatile u32 t_dlay;

//     __disable_irq();

//     MCPWM0_PRT = 0x0000DEAD; /* enter password to unlock write protection */
//     MCPWM0_TH00 = 0x00;
//     MCPWM0_TH01 = 0x00;
//     MCPWM0_TH10 = 0x00;
//     MCPWM0_TH11 = 0x00;
//     MCPWM0_TH20 = 0x00;
//     MCPWM0_TH21 = 0x00;
//     MCPWM0_UPDATE = 0xff;    /* write whatever value to trigger update */
//     MCPWM0_PRT = 0x0000CAFE; // write any value other than 0xDEAD to enable write protection

//     PWMOutputs(DISABLE); // 对于非MOS内阻采样，可使能也可不使能

//     ADC0_TRIG = 0;

//     ADC_SOFTWARE_TRIG_ONLY(); // 数据左对齐
//     ADC0_CHN0 = 4;            // 采样4个通道

//     ADC0_STATE_RESET();

//     // 延时后进行offset采样
//     for (t_dlay = 0; t_dlay < 0x7ffff; t_dlay++)
//         ;

//     // 单电阻Offset
// #if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
//     ADC_CHN_GAIN_CFG(ADC0, CHN0, ADC_1SHUNT_CURR_CH, ADC_GAIN3V6);
//     ADC_CHN_GAIN_CFG(ADC0, CHN1, ADC_1SHUNT_CURR_CH, ADC_GAIN3V6);
//     ADC_CHN_GAIN_CFG(ADC0, CHN2, ADC_BUS_VOL_CHANNEL, ADC_GAIN3V6);
//     ADC_CHN_GAIN_CFG(ADC0, CHN3, M0_ADC_BUS_CURR_CH, ADC_GAIN3V6);
// #elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
//     ADC0_CHN0 = ADC0_CURRETN_A_CHANNEL | (ADC0_CURRETN_B_CHANNEL << 4) | (ADC_BUS_VOL_CHANNEL << 8) | (M0_ADC_BUS_CURR_CH << 12);
// #elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)
//     ADC0_CHN0 = ADC0_CURRETN_A_CHANNEL | (ADC0_CURRETN_B_CHANNEL << 4) | (ADC0_CURRETN_C_CHANNEL << 8) | (M0_ADC_BUS_CURR_CH << 12);
// #endif

//     t_offset1 = 0;
//     t_offset2 = 0;
//     t_offset3 = 0;
//     t_offset4 = 0;

//     for (t_dlay = 0; t_dlay < (t_nuCalPeriod); t_dlay++) {
//         /* Clear the ADC0 JEOC pending flag */
//         // ADC0_SWT = 0x00005AA5;
//         ADC_SoftTrgEN(ADC0, ENABLE); // 软件触发 内部需要区分

//         while (!(ADC0_IF & BIT0))
//             ;

//         ADC0_IF |= BIT1 | BIT0;
//         ADC0_STATE_RESET();

//         t_offset1 += (s16)((ADC0_DAT0));
//         t_offset2 += (s16)((ADC0_DAT1));
//         t_offset3 += (s16)((ADC0_DAT2));
//         t_offset4 += (s16)(ADC0_DAT3);
//     }

//     ADC0_init();

// #if (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
//     tS_pHdToMd->nBusShuntOffset = (s16)(t_offset1 / t_nuCalPeriod);
// #elif (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
//     tS_pHdToMd->nPhaseUOffset = (s16)(t_offset1 / t_nuCalPeriod);
//     tS_pHdToMd->nPhaseVOffset = (s16)(t_offset2 / t_nuCalPeriod);
// #elif ((EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT) || (EPWM0_CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
//     tS_pHdToMd->nPhaseUOffset = (s16)(t_offset1 / t_nuCalPeriod);
//     tS_pHdToMd->nPhaseVOffset = (s16)(t_offset2 / t_nuCalPeriod);
//     tS_pHdToMd->nPhaseWOffset = (s16)(t_offset3 / t_nuCalPeriod);
// #endif
// }

// void CurrentOffsetCalibration(void)
// {
//     volatile u32 t_dlay;

//     // 延时时间
//     for (t_dlay = 0; t_dlay < 0x6ffff; t_dlay++)
//         ;

//     CurrentOffsetCalibration();
// }

