#include "mc_hall.h"

HALL_Handle_t Hall_handle;

/**
 * @brief ��ȡHALL״̬
 * @retval None
 */
u8 ReadHallState(void)
{
    volatile u8 ReadValue;

    ReadValue = HALL0_INFO & 0x07;

    return (ReadValue);
}

/**
 * @brief ������HALL״̬,��ʼ�������жϿ���
 * @param *this Hall���ƽṹ��
 * @retval None
 */
void Update_HallState(HALL_Handle_t *this)
{
    u8 temp_state1, temp_state;
    u8 t_cnt = 0, t_i;

    temp_state = ReadHallState();

    if (temp_state != this->bHallState_Origin) {
        for (t_i = 0; t_i < 4; t_i++) {
            temp_state1 = ReadHallState(); /* ������״̬ */
            if (temp_state == temp_state1) {
                t_cnt++;
            }
        }

        if (t_cnt > 2) {
            NVIC_DisableIRQ(HALL0_IRQn);

            NVIC_EnableIRQ(HALL0_IRQn);
        }
    }
}

/**
 * @brief HALL�жϻص�
 * @param *this HALL���ƽṹ��
 * @retval None
 */
void HALL_IRQProcess(HALL_Handle_t *this)
{
    volatile u8 t_i, t_cnt;

    if (HALL0_INFO & BIT16) { /* Hall �仯�ж� */
        this->wHallCapCnt += HALL->WIDTH;
        HALL0_INFO = BIT16;

        this->bHallState = ReadHallState();
    }

    if (HALL0_INFO & BIT17) { /* Hall��ʱ�ж� */
        HALL0_INFO = BIT17;

        this->bHallRunFlg |= HALL_COM_ERR;
        this->bSpeedFIFO_Index = 0;
    }

    HALL0_INFO = 0;
}
