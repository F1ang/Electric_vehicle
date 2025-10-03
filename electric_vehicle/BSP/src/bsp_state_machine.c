#include "bsp_state_machine.h"

/**
 * @brief ×´Ì¬³õÊ¼»¯
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @retval None
 */
void STM_Init(STM_Handle_t *pHandle)
{
#if ENCODER_ENABLE
    pHandle->bState = IDLE;
    pHandle->hFaultNow = MC_NO_FAULTS;
    pHandle->hFaultOccurred = MC_NO_FAULTS;
    pHandle->bStop_Motor = 1;
#elif HALL_ENABLE
    pHandle->bState = IDLE_START;
    pHandle->hFaultNow = MC_NO_FAULTS;
    pHandle->hFaultOccurred = MC_NO_FAULTS;
    pHandle->bStop_Motor = 1;
#elif SENSORLESS_ENABLE
    pHandle->bState = IDLE_START;
    pHandle->hFaultNow = MC_NO_FAULTS;
    pHandle->hFaultOccurred = MC_NO_FAULTS;
    pHandle->bStop_Motor = 1;
#endif
}

/**
 * @brief ×´Ì¬»úÌø×ª
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @param bState ÐÂ×´Ì¬
 * @retval bool ×´Ì¬ÊÇ·ñ·¢Éú±ä»¯
 */
bool STM_NextState(STM_Handle_t *pHandle, State_t bState)
{
    bool bChangeState = false;
    State_t bCurrentState = pHandle->bState;
    State_t bNewState = bCurrentState;

    switch (bCurrentState) {
    case ICLWAIT:
        if (bState == IDLE) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case IDLE:
        if ((bState == IDLE_START) || (bState == IDLE_ALIGNMENT) || (bState == ICLWAIT)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case IDLE_ALIGNMENT:
        if ((bState == ANY_STOP) || (bState == ALIGN_CHARGE_BOOT_CAP) || (bState == ALIGN_OFFSET_CALIB)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case ALIGN_CHARGE_BOOT_CAP:
        if ((bState == ALIGN_OFFSET_CALIB) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case ALIGN_OFFSET_CALIB:
        if ((bState == ALIGN_CLEAR) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case ALIGN_CLEAR:
        if ((bState == ALIGNMENT) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case ALIGNMENT:
        if (bState == ANY_STOP) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case IDLE_START:
        if ((bState == ANY_STOP) || (bState == CHARGE_BOOT_CAP) ||
            (bState == START) ||
            (bState == OFFSET_CALIB) || (bState == IDLE_ALIGNMENT)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case CHARGE_BOOT_CAP:
        if ((bState == OFFSET_CALIB) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case OFFSET_CALIB:
        if ((bState == CLEAR) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case CLEAR:
        if ((bState == START) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case START:
        if ((bState == START_RUN) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case START_RUN:
        if ((bState == RUN) || (bState == ANY_STOP)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case RUN:
        if (bState == ANY_STOP) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case ANY_STOP:
        if (bState == STOP) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case STOP:
        if (bState == STOP_IDLE) {
            bNewState = bState;
            bChangeState = true;
        }
        break;

    case STOP_IDLE:
        if ((bState == IDLE) || (bState == ICLWAIT)) {
            bNewState = bState;
            bChangeState = true;
        }
        break;
    default:
        break;
    }

    if (bChangeState) {
        pHandle->bState = bNewState;
    } else {
        if (!((bState == IDLE_START) || (bState == IDLE_ALIGNMENT) || (bState == ANY_STOP))) {
            /* If new state is not a user command START/STOP raise a software error */
            STM_FaultProcessing(pHandle, MC_SW_ERROR, 0u);
        }
    }

    return (bChangeState);
}

/**
 * @brief ¹ÊÕÏ´¦Àí
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @param hSetErrors ¹ÊÕÏÎ»
 * @param hResetErrors ¹ÊÕÏÎ»¸´Î»
 * @retval State_t ¹ÊÕÏ×´Ì¬
 */
State_t STM_FaultProcessing(STM_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors)
{
    State_t LocalState = pHandle->bState;

    /* Set current errors */
    pHandle->hFaultNow = (pHandle->hFaultNow | hSetErrors) & (~hResetErrors);
    pHandle->hFaultOccurred |= hSetErrors;

    if (LocalState == FAULT_NOW) {
        if (pHandle->hFaultNow == MC_NO_FAULTS) {
            pHandle->bState = FAULT_OVER;
            LocalState = FAULT_OVER;
        }
    } else {
        if (pHandle->hFaultNow != MC_NO_FAULTS) {
            pHandle->bState = FAULT_NOW;
            LocalState = FAULT_NOW;
        }
    }

    return (LocalState);
}

/**
 * @brief »ñÈ¡µ±Ç°×´Ì¬
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @retval State_t ×´Ì¬
 */
State_t STM_GetState(STM_Handle_t *pHandle)
{
    return (pHandle->bState);
}

/**
 * @brief ¹ÊÕÏÊÇ·ñÌø³ö
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @retval bool ÊÇ·ñÌø³ö¹ÊÕÏ
 */
bool STM_FaultAcknowledged(STM_Handle_t *pHandle)
{
    bool bToBeReturned = false;

    if (pHandle->bState == FAULT_OVER) {
        pHandle->bState = STOP_IDLE;
        pHandle->hFaultOccurred = MC_NO_FAULTS;
        bToBeReturned = true;
    }

    return (bToBeReturned);
}

/**
 * @brief »ñÈ¡¹ÊÕÏ×´Ì¬
 * @param *pHandle ×´Ì¬»ú¾ä±ú
 * @retval  uint32_t ¹ÊÕÏ×´Ì¬
 */
uint32_t STM_GetFaultState(STM_Handle_t *pHandle)
{
    uint32_t LocalFaultState;

    LocalFaultState = (uint32_t)(pHandle->hFaultOccurred);
    LocalFaultState |= (uint32_t)(pHandle->hFaultNow) << 16;

    return LocalFaultState;
}
