#ifndef BSP_DAC_COMM_H
#define BSP_DAC_COMM_H

#include "bsp_global.h"

typedef enum {
    DAC_CH0,
    DAC_CH1,
    DAC_CH2,
    DAC_CH3,
    DAC_MAX_Channel_nbr
} DAC_Channel_t;

typedef struct UI_Handle UI_Handle_t;

typedef void (*UI_DACSetUserChannelValue_Cb_t)(UI_Handle_t *pHandle, uint8_t bUserChNumber, int16_t hValue);
typedef int16_t (*UI_DACGetUserChannelValue_Cb_t)(UI_Handle_t *pHandle, uint8_t bUserChNumber);
typedef void (*UI_Generic_Cb_t)(UI_Handle_t *pHandle);

/* 输出回调函数对象 */
struct UI_Handle {
    UI_DACSetUserChannelValue_Cb_t pFctDACSetUserChannelValue;
    UI_DACGetUserChannelValue_Cb_t pFctDACGetUserChannelValue;

    UI_Generic_Cb_t pFct_DACInit;
    UI_Generic_Cb_t pFct_DACExec;
    uint8_t bDriveNum;
    uint8_t bSelectedDrive;
};

/* 输出相关参数 */
typedef struct {
    UI_Handle_t _Super;
    int16_t hUserValue;
    uint16_t hDAC_CH1_ENABLED;
} DAC_UI_Handle_t;

extern UI_Handle_t UI_Params;
extern DAC_UI_Handle_t DAC_UI_Params;

void UI_DACInit(UI_Handle_t *pHandle);
void UI_DACExec(UI_Handle_t *pHandle);
void DAC_Init(UI_Handle_t *pHandle);
void DAC_Exec(UI_Handle_t *pHandle);

void DAC_SetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber, int16_t hValue);
int16_t DAC_GetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber);
void DAC_Output_Printf(void);

#endif /* BSP_DAC_COMM_H */
