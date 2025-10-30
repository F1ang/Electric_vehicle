#include "mc_math.h"

const s16 hSin_Cos_Table[256] = SIN_COS_TABLE;
TrigValue_Ctrl_t TrigValue_Ctrl;

/*******************************************************************************
 * Function Name  : Trig_Functions
 * Description    : This function returns Cosine and Sine functions of the input
 *                  angle
 * Input          : angle in s16 format
 * Output         : Cosine and Sine in s16 format
 * Return         : none.
 *******************************************************************************/
#if (MCU_DSP_SINCOS == FUNCTION_OFF)
void Trig_Functions(s16 hAngle, TrigValue_Ctrl_t *t_pTrigComponents)
{
    u8 sector;

    // 10 bit index computation //
    sector = ((u16)hAngle) >> 14;
    hAngle = hAngle >> 6;

    switch (sector) {
    case 0: // U0_90:
        t_pTrigComponents->hSin = hSin_Cos_Table[(u8)(hAngle)];
        t_pTrigComponents->hCos = hSin_Cos_Table[(u8)(~hAngle)];
        break;

    case 1: // U90_180:
        t_pTrigComponents->hSin = hSin_Cos_Table[(u8)(~hAngle)];
        t_pTrigComponents->hCos = -hSin_Cos_Table[(u8)(hAngle)];
        break;

    case 2: // U180_270:
        t_pTrigComponents->hSin = -hSin_Cos_Table[(u8)(hAngle)];
        t_pTrigComponents->hCos = -hSin_Cos_Table[(u8)(~hAngle)];
        break;

    case 3: // U270_360:
        t_pTrigComponents->hSin = -hSin_Cos_Table[(u8)(~hAngle)];
        t_pTrigComponents->hCos = hSin_Cos_Table[(u8)(hAngle)];
        break;

    default:
        break;
    }
}
#else
void Trig_Functions(s16 hAngle, TrigValue_Ctrl_t *t_pTrigComponents)
{

    DSP0_SC |= BIT2;
    DSP0_THETA = hAngle;

    t_pTrigComponents->hSin = DSP0_SIN;
    t_pTrigComponents->hCos = DSP0_COS;
}
#endif

/*******************************************************************************
 * Function Name  : Sqrt_Functions
 * Description    : This function returns SqrtT of the input variable
 *
 * Input          : in s16 format
 * Output         : Sqrt
 * Return         : none.
 *******************************************************************************/
s16 Sqrt_Functions(s16 x, s16 y)
{
    s16 result;
    DSP0_RAD = (s32)x * x + (s32)y * y;
    result = DSP0_SQRT;

    return result;
}

/*******************************************************************************
 * Function Name  : DIV_Functions
 * Description    : This function returns SqrtT of the input variable
 *
 * Input          : in s16 format
 * Output         : Sqrt
 * Return         : none.
 *******************************************************************************/
#if (MCU_DSP_DIV == FUNCTION_ON)
s16 Div_Functions(s32 y, s16 x)
{
    s16 result;
    DSP0_DID = y;
    DSP0_DIS = x;
    result = DSP0_QUO;
    return result;
}
#else
s16 Div_Functions(s32 y, s16 x)
{
    return (y / x);
}
#endif

/*******************************************************************************
 函数名称：    void CopyFromBuffer(u8* nDestAddr, u8* pSrcBuff, u16 nSize)
 功能描述：    以8Bit为一次复制数据到另外一个地址
 输入参数：    u8* nDestAddr, u8* pSrcBuff, u16 nSize
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void CopyFromBuffer(u8 *nDestAddr, u8 *pSrcBuff, u16 nSize)
{
    u8 *ps = (u8 *)pSrcBuff;
    u8 *pd = (u8 *)nDestAddr;

    while (nSize--)
        *pd++ = *ps++;
}

// 斜坡初始化
void ramp32GenInit(PSTR_RampGen32 this)
{
    this->wRampIn = 0;
    this->wRampOut = 0;
    this->wRampTmp = 0;
}

// 斜坡计算
void ramp32GenCalc(PSTR_RampGen32 this)
{
    s32 t_wTmp = 0;
    bool isIncrease = FALSE;

    if (this->wRampOut != this->wRampIn) {
        if (this->wRampIncrease == 0) {
            this->wRampOut = this->wRampIn;
            this->wRampTmp = 0;
            return;
        }

        if (this->wRampIn >= this->wRampOut) {
            this->wRampTmp += this->wRampIncrease;
            isIncrease = TRUE;
        } else {
            this->wRampTmp -= this->wRampDecrease;
        }

        t_wTmp = ABS(this->wRampTmp);
        if (this->wRampTmp >= 0) {
            this->wRampOut += (t_wTmp >> 16);
        } else {
            this->wRampOut -= (t_wTmp >> 16);
        }

        t_wTmp = t_wTmp & 0x0000FFFF;
        this->wRampTmp = this->wRampTmp >= 0 ? t_wTmp : -t_wTmp;

        if (isIncrease) {
            if (this->wRampOut > this->wRampIn) {
                this->wRampOut = this->wRampIn;
                this->wRampTmp = 0;
            }
        } else {
            if (this->wRampOut < this->wRampIn) {
                this->wRampOut = this->wRampIn;
                this->wRampTmp = 0;
            }
        }
    }
}

/**
 * @brief clarke变换
 * @note 建系beta向下为正
 * @param Iabc_Input 电流Ia Ib Ic
 * @retval CurrVoctor_Iabc
 */
CurrVoctor_Ialphabeta Clarke(CurrVoctor_Iabc Iabc_Input) 
{
    CurrVoctor_Ialphabeta Curr_Output = { 0, 0 };

    s16 qIa_divSQRT3;
    s16 qIb_divSQRT3;

    // qIalpha = qIas
    Curr_Output.qIalpha = Iabc_Input.qIa;

    qIa_divSQRT3 = (s32)DIVSQRT3 * Iabc_Input.qIa >> 15;
    qIb_divSQRT3 = (s32)DIVSQRT3 * Iabc_Input.qIb >> 15;

    // qIbeta = -(2*qIbs+qIas)/sqrt(3)
    Curr_Output.qIbeta = (-(qIa_divSQRT3) - (qIb_divSQRT3) - (qIb_divSQRT3));

    return Curr_Output;
}

/**
 * @brief park
 * @note q超前d,θ定义为q与alpha的夹角
 * @param Iaplha_beta_Input Iaplha_beta
 * @param Theta 角度
 * @param *this FOC控制参数结构体
 * @retval CurrVoctor_Idq
 */
CurrVoctor_Idq Park(CurrVoctor_Ialphabeta Iaplha_beta_Input, s16 Theta, TrigValue_Ctrl_t *this)
{
    CurrVoctor_Idq Curr_Output = { 0, 0 };

    s16 qId_1, qId_2;
    s16 qIq_1, qIq_2;

    Trig_Functions(Theta, this);

    qIq_1 = (s32)Iaplha_beta_Input.qIalpha * this->hCos >> 15;
    qIq_2 = (s32)Iaplha_beta_Input.qIbeta * this->hSin >> 15;
    // Iq=Ialpha*cos(Theta)-Ibeta*sin(Theta)
    Curr_Output.qIq = ((qIq_2) - (qIq_1));

    qId_1 = (s32)Iaplha_beta_Input.qIalpha * this->hSin >> 15;
    qId_2 = (s32)Iaplha_beta_Input.qIbeta * this->hCos >> 15;

    // Id=Ialpha*sin(Theta)+Ibeta*cos(Theta)
    Curr_Output.qId = ((qId_1) + (qId_2));

    return (Curr_Output);
}
