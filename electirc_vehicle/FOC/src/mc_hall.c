#include "mc_hall.h"

HALL_Ctrl_t g_hall_ctrl;

/**
 * @brief 读取HALL状态
 * @retval None
 */
u8 ReadHallState(void)
{
    volatile u8 ReadValue;

    ReadValue = HALL0_INFO & 0x07;

    return (ReadValue);
}

/**
 * @brief 检查更新HALL状态,初始化捕获中断开启
 * @param *this Hall控制结构体
 * @retval None
 */
void Update_HallState(HALL_Ctrl_t *this)
{
    u8 temp_state1, temp_state;
    u8 t_cnt = 0, t_i;

    temp_state = ReadHallState();

    if (temp_state != this->bHallState_Origin) {
        for (t_i = 0; t_i < 4; t_i++) {
            temp_state1 = ReadHallState(); /* 读霍尔状态 */
            if (temp_state == temp_state1) {
                t_cnt++;
            }
        }

        if (t_cnt > 2) {
            NVIC_DisableIRQ(HALL0_IRQn);
            // t_funcState = check_hall_state(temp_state, &struHallProcess); /* 查询霍尔状态函数 */

            // if (t_funcState) {
            //     hall_sensor_period(&struHallProcess);

            //     t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;
            //     this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

            //     this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(&struHallProcess);

            //     this->wHallPWMTimerCnt = 0;
            //     HALL_Init_Electrical_Angle(&struHallProcess);
            // }

            NVIC_EnableIRQ(HALL0_IRQn);
        }
    }
}

/**
 * @brief 、HALL中断
 * @param *this HALL控制结构体
 * @retval None
 */
void HALL_IRQProcess(HALL_Ctrl_t *this)
{
    volatile u8 t_i, t_cnt;

    if (HALL0_INFO & BIT16) { /* Hall 变化中断 */
        this->wHallCapCnt += HALL->WIDTH;
        HALL0_INFO = BIT16;

        this->bHallState = ReadHallState();

        // /* 返回为0是干扰错误换相，不为0是正常换相 */
        // t_FuncState = check_hall_state(temp_state, &struHallProcess);

        // if (t_FuncState) {
        //     hall_sensor_period(&struHallProcess);

        //     if (this->bFstHallSigFlg) {
        //         this->bFstHallSigFlg = 0;
        //     } else {
        //         if (this->wHallCapCnt < MAX_SPEED_DPP) {
        //             this->wHallCapCnt = MAX_SPEED_DPP;
        //         }

        //         this->wSensorPeriod[this->bSpeedFIFO_Index] = this->wHallCapCnt;
        //     }

        //     this->wHallCapCnt = 0;
        //     this->wHallPWMTimerCnt = 0;

        //     this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(&struHallProcess);

        //     HALL_Init_Electrical_Angle(&struHallProcess);
        // }
    }

    if (HALL0_INFO & BIT17) { /* Hall超时中断 */
        HALL0_INFO = BIT17;

        this->bHallRunFlg |= HALL_COM_ERR;
        this->bSpeedFIFO_Index = 0;

        // this->wMotorSpeedAvgCnt = ROTOR_SPEED_FACTOR / 3;

        // for (t_i = 0; t_i < HALL_SPEED_FIFO_SIZE; t_i++) {
        //     this->wSensorPeriod[t_i] = this->wMotorSpeedAvgCnt;
        // }

        // this->nRotorFreqDpp = 3;

        // temp_state = ReadHallStateMix();

        // t_FuncState = check_hall_state(temp_state, &struHallProcess);

        // HALL_Init_Electrical_Angle(&struHallProcess);

        // this->bCloseLoopAngleFlg = 0;
    }

    HALL0_INFO = 0;
}
