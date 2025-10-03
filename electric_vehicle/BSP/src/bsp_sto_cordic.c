#include "bsp_sto_cordic.h"
#include "bsp_pid.h"
#include "bsp_spd.h"
#include "bsp_sto_pll.h"

/* STO CORDIC对象 */
STO_CR_Handle_t STO_CR_M1 = {
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* 极对数 */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* 最大可靠机械速度0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* 最小可靠机械速度0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* 最大速度错误次数 */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* 最大可靠机械加速度0.1Hz */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* 采样频率 */
    },
    .hC1 = CORD_C1, /* 状态观测器常量C1 */
    .hC2 = CORD_C2, /* F1*K1/状态观测器执行频率[Hz]，其中K1是两个观测器增益之一 */
    .hC3 = CORD_C3, /* 状态观测器常量C3 */
    .hC4 = CORD_C4,
    .hC5 = CORD_C5, /* 状态观测器常量C5 */
    .hF1 = CORD_F1, /* 状态观测器缩放因子F1 */
    .hF2 = CORD_F2, /* 状态观测器缩放因子F2 */

    .SpeedBufferSize01Hz = CORD_FIFO_DEPTH_01HZ,              /* SPD_GetAvrgMecSpeed01Hz 函数所导出的估计速度0.1Hz */
    .SpeedBufferSizedpp = CORD_FIFO_DEPTH_DPP,                /* SPD_GetElSpeedDpp 和状态观测器方程导出的估计速度dpp */
    .VariancePercentage = CORD_PERCENTAGE_FACTOR,             /* 速度估计最大允许方差的参数 */
    .SpeedValidationBand_H = SPEED_BAND_UPPER_LIMIT,          /* 估计速度>强制定子电频率 程度 */
    .SpeedValidationBand_L = SPEED_BAND_LOWER_LIMIT,          /* 估计速度<强制定子电频率 程度 */
    .MinStartUpValidSpeed = OBS_MINIMUM_SPEED,                /* 启动时最小机械转速的绝对值0.1Hz */
    .StartUpConsistThreshold = NB_CONSECUTIVE_TESTS,          /* 测试次数->启动成功阈值 */
    .Reliability_hysteresys = CORD_MEAS_ERRORS_BEFORE_FAULTS, /* 测试中出现可靠性故障的次数 */
    .MaxInstantElAcceleration = CORD_MAX_ACCEL_DPPP,          /* 最大瞬时电机加速度 */
    .BemfConsistencyCheck = CORD_BEMF_CONSISTENCY_TOL,        /* 观测器的BEMF的置信度[1,64] */
    .BemfConsistencyGain = CORD_BEMF_CONSISTENCY_GAIN,        /* 观测器BEMF增益的准确度[1,64,105] */
    .MaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED * 1.15 / 6.0),

    .F1LOG = F1_LOG,                                 /* F1增益分频器以2的幂次 */
    .F2LOG = F2_LOG,                                 /* F2增益分频器以2的幂次 */
    .SpeedBufferSizedppLOG = CORD_FIFO_DEPTH_DPP_LOG /* bSpeedBufferSizedpp 的值表示为2的幂次方 */
};

/**
 * @brief  初始化STO和PLL参数
 * @param  pHandle: STO对象(旋转d-q坐标系下,低速),基于电机模型+电流重构,
 * @retval None
 */
void STO_CR_Init(STO_CR_Handle_t *pHandle)
{
    int16_t htempk;
    int32_t wAux;

    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
    pHandle->EnableDualCheck = true;

    wAux = (int32_t)1;
    pHandle->F3POW2 = 0u;

    htempk = (int16_t)(C6_COMP_CONST1 / (pHandle->hF2));

    while (htempk != 0) {
        htempk /= (int16_t)2;
        wAux *= (int32_t)2;
        pHandle->F3POW2++;
    }

    pHandle->hF3 = (int16_t)wAux;
    wAux = (int32_t)(pHandle->hF2) * pHandle->hF3;
    pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);

    STO_CR_Clear(pHandle);

    /* Acceleration measurement set to zero */
    pHandle->_Super.hMecAccel01HzP = 0;

    return;
}

/**
 * @brief  Return
 * @param  pHandle: STO对象(旋转坐标系下)
 * @param  uint8_t flag: 标志位
 * @retval None
 */
void STO_CR_Return(STO_CR_Handle_t *pHandle, uint8_t flag)
{
    return;
}

/**
 * @brief  STO观测器和PLL计算电机角度,含加速度突变处理
 * @param  pHandle: STO+PLL(旋转坐标系下)
 * @retval 电角度
 */
int16_t STO_CR_CalcElAngle(STO_CR_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{

    int32_t wAux, wDirection;
    int32_t wAux_Alpha, wAux_Beta;
    int32_t wIalfa_est_Next, wIbeta_est_Next;
    int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
    int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed, hOrRotor_Speed, hRotor_Acceleration, hRotor_Angle, hValfa, hVbeta;

    int16_t hPrev_Rotor_Angle = pHandle->_Super.hElAngle;
    int16_t hPrev_Rotor_Speed = pHandle->_Super.hElSpeedDpp;
    int16_t hMax_Instant_Accel = pHandle->MaxInstantElAcceleration;

    /* 1.估计E_alfha */
    if (pHandle->wBemf_alfa_est > (int32_t)(pHandle->hF2) * INT16_MAX) {
        pHandle->wBemf_alfa_est = INT16_MAX * (int32_t)(pHandle->hF2);
    } else if (pHandle->wBemf_alfa_est <= -INT16_MAX * (int32_t)(pHandle->hF2)) {
        pHandle->wBemf_alfa_est = -INT16_MAX * (int32_t)(pHandle->hF2);
    } else {
    }
    hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est >> pHandle->F2LOG);

    /* 2.估计E_beta */
    if (pHandle->wBemf_beta_est > INT16_MAX * (int32_t)(pHandle->hF2)) {
        pHandle->wBemf_beta_est = INT16_MAX * (int32_t)(pHandle->hF2);
    } else if (pHandle->wBemf_beta_est <= -INT16_MAX * (int32_t)(pHandle->hF2)) {
        pHandle->wBemf_beta_est = -INT16_MAX * (int32_t)(pHandle->hF2);
    } else {
    }
    hAux_Beta = (int16_t)(pHandle->wBemf_beta_est >> pHandle->F2LOG);

    /* 3.估计I_alfha*/
    if (pHandle->Ialfa_est > INT16_MAX * (int32_t)(pHandle->hF1)) {
        pHandle->Ialfa_est = INT16_MAX * (int32_t)(pHandle->hF1);
    } else if (pHandle->Ialfa_est <= -INT16_MAX * (int32_t)(pHandle->hF1)) {
        pHandle->Ialfa_est = -INT16_MAX * (int32_t)(pHandle->hF1);
    } else {
    }

    /* 4.估计I_beta*/
    if (pHandle->Ibeta_est > INT16_MAX * (int32_t)(pHandle->hF1)) {
        pHandle->Ibeta_est = INT16_MAX * (int32_t)(pHandle->hF1);
    } else if (pHandle->Ibeta_est <= -INT16_MAX * (int32_t)(pHandle->hF1)) {
        pHandle->Ibeta_est = -INT16_MAX * (int32_t)(pHandle->hF1);
    } else {
    }

    /* 5.I_alfhax误差 */
    hIalfa_err = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
    hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.qI_Component1;

    /* 6.I_beta误差 */
    hIbeta_err = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
    hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.qI_Component2;

    /* 7.V_alpha=Vbus>>16 * V_alpha 标幺化 */
    wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.qV_Component1;
    hValfa = (int16_t)(wAux >> 16);

    /* 8.V_beta=Vbus>>16 * V_beta 标幺化 */
    wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.qV_Component2;
    hVbeta = (int16_t)(wAux >> 16);

    /* 9.d(I_alpha)/dt = A*I_alpha(n) + B(Vs-Es+Z),Z=ksign(估计Is-观测Is) */
    /* I_alpha(n+1) = I_alpha(n) - C1*I_alpha(n) + C2*I_err(n) + C5*V_alpha(n) - C3*E_alpha(n) */
    hAux = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
    wAux = (int32_t)(pHandle->hC1) * hAux;
    wIalfa_est_Next = pHandle->Ialfa_est - wAux;

    /* Z */
    wAux = (int32_t)(pHandle->hC2) * hIalfa_err;
    wIalfa_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC5) * hValfa;
    wIalfa_est_Next += wAux;

    /* I_alpha(n+1) */
    wAux = (int32_t)(pHandle->hC3) * hAux_Alfa;
    wIalfa_est_Next -= wAux;

    /* 10.d(E_alpha)/dt = -W0*E_alpha(n)+W0*Z */
    /* E_alpha(n+1) = E_alpha(n) + C4*I_err(n) - C6*E_alpha(n) */
    wAux = (int32_t)(pHandle->hC4) * hIalfa_err;
    wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux; /* W0*Z */

    wAux = (int32_t)hAux_Beta >> pHandle->F3POW2;

    wAux = wAux * pHandle->hC6;
    wAux = hPrev_Rotor_Speed * wAux;
    wBemf_alfa_est_Next += wAux; /* -W0*Es */

    /* 11.d(I_beta)/dt = A*I_beta(n) + B(Vs-Es+Z),Z=ksign(估计Is-观测Is) */
    /* I_beta(n+1) = I_beta(n) - C1*I_beta(n) + C2*I_err(n) + C5*V_beta(n) - C3*E_beta(n) */
    hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
    wAux = (int32_t)(pHandle->hC1) * hAux;
    wIbeta_est_Next = pHandle->Ibeta_est - wAux;

    /* Z */
    wAux = (int32_t)(pHandle->hC2) * hIbeta_err;
    wIbeta_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC5) * hVbeta;
    wIbeta_est_Next += wAux;

    /* I_beta(n+1) */
    wAux = (int32_t)(pHandle->hC3) * hAux_Beta;
    wIbeta_est_Next -= wAux;

    /* 12.d(E_beta)/dt = -W0*E_beta(n)+W0*Z */
    /* E_beta(n+1) = E_beta(n) + C4*I_err(n) - C6*E_beta(n) */
    wAux = (int32_t)(pHandle->hC4) * hIbeta_err;
    wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

    wAux = (int32_t)hAux_Alfa >> pHandle->F3POW2;

    wAux = wAux * pHandle->hC6;
    wAux = hPrev_Rotor_Speed * wAux;
    wBemf_beta_est_Next -= wAux;

    if (pHandle->Orig_ElSpeedDpp >= 0) {
        wDirection = 1;
    } else {
        wDirection = -1;
    }

    /* 13.锁相环PLL E_alpha(n+1) E_beta(n+1) */
    pHandle->hBemf_alfa_est = hAux_Alfa;
    pHandle->hBemf_beta_est = hAux_Beta;

    wAux_Alpha = pHandle->wBemf_alfa_est * wDirection;
    wAux_Beta = pHandle->wBemf_beta_est * wDirection;

    /* 14.直接从BEMF的α和β分量中提取转子位置 */
    hRotor_Angle = STO_CR_ExecuteCORDIC(pHandle, wAux_Alpha, -wAux_Beta);

    /* 15.速度与加速度计算,更新电角度 */
    hOrRotor_Speed = (int16_t)(hRotor_Angle - hPrev_Rotor_Angle);
    hRotor_Acceleration = hOrRotor_Speed - hPrev_Rotor_Speed;
    hRotor_Speed = hOrRotor_Speed;

    if (wDirection == 1) {
        /* dpp为正转,PLL转速为负,异常,直接清零 */
        if (hRotor_Speed < 0) {
            hRotor_Speed = 0;
        } else {
            /* 加速度突变,均值速度平滑 */
            if (hRotor_Acceleration > hMax_Instant_Accel) {
                hRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;
                pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
            } else {
                pHandle->_Super.hElAngle = hRotor_Angle;
            }
        }
    } else {
        if (hRotor_Speed > 0) {
            hRotor_Speed = 0;
        } else {
            if (hRotor_Acceleration < (-hMax_Instant_Accel)) {
                hRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;
                pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
            } else {
                pHandle->_Super.hElAngle = hRotor_Angle;
            }
        }
    }

    /* 加速度突变,均值速度平滑 */
    if (hRotor_Acceleration > hMax_Instant_Accel) {
        hOrRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;
    } else if (hRotor_Acceleration < (-hMax_Instant_Accel)) {
        hOrRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;
    } else {
    }

    /* 估计速度存入缓存 */
    STO_CR_Store_Rotor_Speed(pHandle, hRotor_Speed, hOrRotor_Speed);

    /* 迭代估计E与I */
    pHandle->Ialfa_est = wIalfa_est_Next;
    pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

    pHandle->Ibeta_est = wIbeta_est_Next;
    pHandle->wBemf_beta_est = wBemf_beta_est_Next;

    return (pHandle->_Super.hElAngle);
}

/**
 * @brief 1.转速方差判断速度可靠性 2.转速估计与观测的E_alfha,E_beta,判断收敛性
 *        3.按转速可靠性,判断观测器是否收敛
 * @note 速度可靠性、Oberser的收敛性
 * @param *pHandle STO+PLL对象(旋转坐标系下)
 * @param *pMecSpeed01Hz 机械转速
 * @retval bool 收敛性
 */
bool STO_CR_CalcAvrgMecSpeed01Hz(STO_CR_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
{
    int32_t wAvrSpeed_dpp = (int32_t)0;
    int32_t wError, wAux, wAvrSquareSpeed, wAvrQuadraticError = 0;
    uint8_t i, bSpeedBufferSize01Hz = pHandle->SpeedBufferSize01Hz;
    int32_t wObsBemf, wEstBemf;
    int32_t wObsBemfSq = 0, wEstBemfSq = 0;
    int32_t wEstBemfSqLo;

    /* 速度与BEMF收敛性 */
    bool bIs_Speed_Reliable = false, bAux = false;
    bool bIs_Bemf_Consistent = false;

    /* 1.速度方差:S^2=Σ(x-x_est)^2/N */
    for (i = 0u; i < bSpeedBufferSize01Hz; i++) {
        wAvrSpeed_dpp += (int32_t)(pHandle->Speed_Buffer[i]);
    }

    wAvrSpeed_dpp = wAvrSpeed_dpp / (int16_t)bSpeedBufferSize01Hz;

    for (i = 0u; i < bSpeedBufferSize01Hz; i++) {
        wError = (int32_t)(pHandle->Speed_Buffer[i]) - wAvrSpeed_dpp;
        wError = (wError * wError);
        wAvrQuadraticError += wError;
    }
    wAvrQuadraticError = wAvrQuadraticError / (int16_t)bSpeedBufferSize01Hz;

    /* 2.平均速度的方差范围(利用欧拉不等式得出) */
    wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
    wAvrSquareSpeed = (wAvrSquareSpeed / (int16_t)128) * (int32_t)(pHandle->VariancePercentage);

    /* 3.估计速度的可靠性 */
    if (wAvrQuadraticError < wAvrSquareSpeed) {
        bIs_Speed_Reliable = true;
    }

    /* 4.机械速度 */
    wAux = wAvrSpeed_dpp * (int32_t)(pHandle->_Super.hMeasurementFrequency);
    wAux = wAux * (int32_t)10;
    wAux = wAux / (int32_t)65536;
    wAux = wAux / (int16_t)(pHandle->_Super.bElToMecRatio);

    *pMecSpeed01Hz = (int16_t)wAux;
    pHandle->_Super.hAvrMecSpeed01Hz = (int16_t)wAux;

    pHandle->IsSpeedReliable = bIs_Speed_Reliable;

    /* 5.估计与观测的E_alfha,E_beta,判断收敛性 */
    if (pHandle->EnableDualCheck == true) /*do algorithm if it's enabled*/
    {
        wAux = (wAux < 0 ? (-wAux) : (wAux)); /* wAux abs value   */
        if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeed01Hz)) {
            /* 6.观测Es^2=E_alfha^2+E_beta^2 */
            wObsBemf = (int32_t)(pHandle->hBemf_alfa_est);
            wObsBemfSq = wObsBemf * wObsBemf;
            wObsBemf = (int32_t)(pHandle->hBemf_beta_est);
            wObsBemfSq += wObsBemf * wObsBemf;

            /* 7.状态(量测)估计Es^2,按最大转速,Es=3/2*K_e*W*(-sin(θ)*cos(θ)) */
            wEstBemf = (wAux * 32767) / (int16_t)(pHandle->_Super.hMaxReliableMecSpeed01Hz);
            wEstBemfSq = (wEstBemf * (int32_t)(pHandle->BemfConsistencyGain)) / 64;
            wEstBemfSq *= wEstBemf;

            wEstBemfSqLo = wEstBemfSq - (wEstBemfSq / 64) * (int32_t)(pHandle->BemfConsistencyCheck);

            /* 8.置信度/收敛性 */
            if (wObsBemfSq > wEstBemfSqLo) {
                bIs_Bemf_Consistent = true;
            }
        }

        pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
        pHandle->Obs_Bemf_Level = wObsBemfSq;
        pHandle->Est_Bemf_Level = wEstBemfSq;
    } else {
        bIs_Bemf_Consistent = true;
    }

    /* 9.按转速可靠性,判断观测器是否收敛 */
    if (pHandle->IsAlgorithmConverged == false) {
        bAux = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeed01Hz);
    } else {
        if ((pHandle->IsSpeedReliable == false) || (bIs_Bemf_Consistent == false)) {
            pHandle->ReliabilityCounter++;
            if (pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys) {
                pHandle->ReliabilityCounter = 0u;
                pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
                bAux = false;
            } else {
                bAux = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeed01Hz);
            }
        } else {
            pHandle->ReliabilityCounter = 0u;
            bAux = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeed01Hz);
        }
    }
    return (bAux);
}

/**
 * @brief  清除STO PLL变量
 * @param  pHandle STO对象(旋转坐标系下)
 */
void STO_CR_Clear(STO_CR_Handle_t *pHandle)
{
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->wBemf_alfa_est = (int32_t)0;
    pHandle->wBemf_beta_est = (int32_t)0;
    pHandle->_Super.hElAngle = (int16_t)0;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->Orig_ElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->DppOrigBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

    STO_CR_InitSpeedBuffer(pHandle);
}

/**
 * @brief 设置目标机械角度
 * @param *pHandle STO+PLL对象
 * @param hMecAngle 目标机械角度
 * @retval None
 */
void STO_CR_SetMecAngle(STO_CR_Handle_t *pHandle, int16_t hMecAngle)
{
}

/**
 * @brief  更新速度buff,2速度缓存
 * @param  pHandle STO对象(旋转坐标系下)
 * @retval None
 */
static void STO_CR_Store_Rotor_Speed(STO_CR_Handle_t *pHandle, int16_t hRotor_Speed, int16_t hOrRotor_Speed)
{
    uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

    bBuffer_index++;
    if (bBuffer_index == pHandle->SpeedBufferSize01Hz) {
        bBuffer_index = 0u;
    }

    /* 2速度缓冲的上一次值 */
    pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
    pHandle->OrigSpeedBufferOldestEl = pHandle->Orig_Speed_Buffer[bBuffer_index];

    pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
    pHandle->Orig_Speed_Buffer[bBuffer_index] = hOrRotor_Speed;
    pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
 * @brief  从BEMF的α和β分量中提取转子位置
 * @param  pHandle STO对象(旋转坐标系下)
 *         wBemf_alfa_est 估计E_alpha
 *         wBemf_beta_est 估计E_beta
 * @retval 电角度
 */
static int16_t STO_CR_ExecuteCORDIC(STO_CR_Handle_t *pHandle, int32_t wBemf_alfa_est, int32_t wBemf_beta_est)
{
    int16_t hAngle;
    int32_t wXi, wYi, wXold;

    /*Determining quadrant*/
    if (wBemf_alfa_est < 0) {
        if (wBemf_beta_est < 0) {
            /*Quadrant III, add 90 degrees so as to move to quadrant IV*/
            hAngle = 16384;
            wXi = -(wBemf_beta_est / 2);
            wYi = wBemf_alfa_est / 2;
        } else {
            /*Quadrant II, subtract 90 degrees so as to move to quadrant I*/
            hAngle = -16384;
            wXi = wBemf_beta_est / 2;
            wYi = -(wBemf_alfa_est / 2);
        }
    } else {
        /* Quadrant I or IV*/
        hAngle = 0;
        wXi = wBemf_alfa_est / 2;
        wYi = wBemf_beta_est / 2;
    }
    wXold = wXi;

    /*begin the successive approximation process*/
    /*iteration0*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV1;
        wXi = wXi - wYi;
        wYi = wXold + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV1;
        wXi = wXi + wYi;
        wYi = -wXold + wYi;
    }
    wXold = wXi;

    /*iteration1*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV2;
        wXi = wXi - wYi / 2;
        wYi = wXold / 2 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV2;
        wXi = wXi + wYi / 2;
        wYi = -wXold / 2 + wYi;
    }
    wXold = wXi;

    /*iteration2*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV4;
        wXi = wXi - wYi / 4;
        wYi = wXold / 4 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV4;
        wXi = wXi + wYi / 4;
        wYi = -wXold / 4 + wYi;
    }
    wXold = wXi;

    /*iteration3*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV8;
        wXi = wXi - wYi / 8;
        wYi = wXold / 8 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV8;
        wXi = wXi + wYi / 8;
        wYi = -wXold / 8 + wYi;
    }
    wXold = wXi;

    /*iteration4*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV16;
        wXi = wXi - wYi / 16;
        wYi = wXold / 16 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV16;
        wXi = wXi + wYi / 16;
        wYi = -wXold / 16 + wYi;
    }
    wXold = wXi;

    /*iteration5*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV32;
        wXi = wXi - wYi / 32;
        wYi = wXold / 32 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV32;
        wXi = wXi + wYi / 32;
        wYi = -wXold / 32 + wYi;
    }
    wXold = wXi;

    /*iteration6*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV64;
        wXi = wXi - wYi / 64;
        wYi = wXold / 64 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV64;
        wXi = wXi + wYi / 64;
        wYi = -wXold / 64 + wYi;
    }
    wXold = wXi;

    /*iteration7*/
    if (wYi < 0) {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV128;
        wXi = wXi - wYi / 128;
        wYi = wXold / 128 + wYi;
    } else {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV128;
        wXi = wXi + wYi / 128;
        wYi = -wXold / 128 + wYi;
    }

    return (-hAngle);
}

/**
 * @brief  初始化速度缓存
 * @param  pHandle STO对象(旋转坐标系下)
 * @retval None
 */
static void STO_CR_InitSpeedBuffer(STO_CR_Handle_t *pHandle)
{
    uint8_t b_i;
    uint8_t bSpeedBufferSize01Hz = pHandle->SpeedBufferSize01Hz;

    /*init speed buffer*/
    for (b_i = 0u; b_i < bSpeedBufferSize01Hz; b_i++) {
        pHandle->Speed_Buffer[b_i] = (int16_t)0;
        pHandle->Orig_Speed_Buffer[b_i] = (int16_t)0;
    }

    pHandle->Speed_Buffer_Index = 0u;
    pHandle->SpeedBufferOldestEl = (int16_t)0;
    pHandle->OrigSpeedBufferOldestEl = (int16_t)0;

    return;
}

/**
 * @brief  观测器是否收敛
 * @param  pHandle STO对象(旋转坐标系下)
 * @param  hForcedMecSpeed01Hz 目标转速(可强制设置)
 * @retval bool 收敛性
 */
bool STO_CR_IsObserverConverged(STO_CR_Handle_t *pHandle, int16_t hForcedMecSpeed01Hz)
{
    int16_t hEstimatedSpeed01Hz, hUpperThreshold, hLowerThreshold;
    int32_t wAux;
    bool bAux = false;
    int32_t wtemp;

    /* 1.强制设置转速 */
    if (pHandle->ForceConvergency2 == true) {
        hForcedMecSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;
    }

    /* 2.强制收敛 */
    if (pHandle->ForceConvergency == true) {
        bAux = true;
        pHandle->IsAlgorithmConverged = true;
        pHandle->_Super.bSpeedErrorNumber = 0u;
    } else {
        /* 3.正常判断收敛性 */
        hEstimatedSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;

        /* 4.估计转速与目标转速的差值 */
        wtemp = (int32_t)hEstimatedSpeed01Hz * (int32_t)hForcedMecSpeed01Hz;
        if (wtemp > 0) { /* 同号 */
            /* 速度取正 */
            if (hEstimatedSpeed01Hz < 0) {
                hEstimatedSpeed01Hz = -hEstimatedSpeed01Hz;
            }

            if (hForcedMecSpeed01Hz < 0) {
                hForcedMecSpeed01Hz = -hForcedMecSpeed01Hz;
            }

            /* 5.目标速度+转速带宽频率,算出上下阈值 */
            wAux = (int32_t)(hForcedMecSpeed01Hz) * (int16_t)pHandle->SpeedValidationBand_H;
            hUpperThreshold = (int16_t)(wAux / (int32_t)16);

            wAux = (int32_t)(hForcedMecSpeed01Hz) * (int16_t)pHandle->SpeedValidationBand_L;
            hLowerThreshold = (int16_t)(wAux / (int32_t)16);

            /* 6.速度是收敛的 */
            if (pHandle->IsSpeedReliable == true) {
                if ((uint16_t)hEstimatedSpeed01Hz > pHandle->MinStartUpValidSpeed) {
                    /* 7.比较速度有无超出阈值 */
                    if (hEstimatedSpeed01Hz >= hLowerThreshold) {
                        if (hEstimatedSpeed01Hz <= hUpperThreshold) {
                            pHandle->ConsistencyCounter++;

                            /* 8.连续有效速度次数 */
                            if (pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold) {
                                /* 9.观测器收敛 */
                                bAux = true;
                                pHandle->IsAlgorithmConverged = true;
                                pHandle->_Super.bSpeedErrorNumber = 0u;
                            }
                        } else {
                            pHandle->ConsistencyCounter = 0u;
                        }
                    } else {
                        pHandle->ConsistencyCounter = 0u;
                    }
                } else {
                    pHandle->ConsistencyCounter = 0u;
                }
            } else {
                pHandle->ConsistencyCounter = 0u;
            }
        }
    }
    return (bAux);
}

/**
 * @brief 估计E_alfha E_beta
 * @param *pHandle STO+PLL对象
 * @retval 估计E_alfha E_beta
 */
Volt_Components STO_CR_GetEstimatedBemf(STO_CR_Handle_t *pHandle)
{
    Volt_Components Vaux;
    Vaux.qV_Component1 = pHandle->hBemf_alfa_est;
    Vaux.qV_Component2 = pHandle->hBemf_beta_est;

    return (Vaux);
}

/**
 * @brief 估计的电流I_alfha I_beta
 * @param *pHandle STO+PLL对象
 * @retval 估计的电流I_alfha I_beta
 */
Curr_Components STO_CR_GetEstimatedCurrent(STO_CR_Handle_t *pHandle)
{
    Curr_Components Iaux;

    Iaux.qI_Component1 = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
    Iaux.qI_Component2 = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);

    return (Iaux);
}

/**
 * @brief 获取观测器的增益系数C2 C4
 * @param *pHandle STO+PLL对象
 * @param *phC2
 * @param *phC4
 * @retval None
 */
void STO_CR_GetObserverGains(STO_CR_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
{
    *phC2 = pHandle->hC2;
    *phC4 = pHandle->hC4;
}

/**
 * @brief 设置观测器的增益系数C1 C2
 * @param *pHandle STO+PLL对象
 * @param hhC1
 * @param hhC2
 * @retval None
 */
void STO_CR_SetObserverGains(STO_CR_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
    pHandle->hC2 = hhC1;
    pHandle->hC4 = hhC2;
}

/**
 * @brief 按两速度buff,进行滑动平均计算(最终获取2个速度),2速度需要平滑
 * @param *pHandle STO+PLL对象(旋转坐标系下)
 * @retval void
 */
void STO_CR_CalcAvrgElSpeedDpp(STO_CR_Handle_t *pHandle)
{
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;
    int32_t wSum = pHandle->DppBufferSum;
    int32_t wSumOrig = pHandle->DppOrigBufferSum;
    int32_t wAvrSpeed_dpp;
    int16_t hSpeedBufferSizedpp = (int16_t)(pHandle->SpeedBufferSizedpp);   /* 转速buff0的大小 */
    int16_t hSpeedBufferSize01Hz = (int16_t)(pHandle->SpeedBufferSize01Hz); /* 转速buff1的大小 */
    int16_t hBufferSizeDiff;

    /* 1.取出未参与滑动速度计算的Index */
    hBufferSizeDiff = hSpeedBufferSize01Hz - hSpeedBufferSizedpp;

    /* 2.两buff全参与计算,直接加入新速度,同时去掉旧速度 */
    if (hBufferSizeDiff == 0) {
        wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
        wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] - pHandle->OrigSpeedBufferOldestEl;
    } else {
        /* 3.两buff不全参与计算,加入新速度,同时按index去掉旧速度 */
        hIndexOldTemp = hIndexNew + hBufferSizeDiff;

        if (hIndexOldTemp >= hSpeedBufferSize01Hz) {
            hIndexOld = hIndexOldTemp - hSpeedBufferSize01Hz;
        } else {
            hIndexOld = hIndexOldTemp;
        }

        wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];
        wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] - pHandle->Orig_Speed_Buffer[hIndexOld];
    }

    /* 4.计算滑动平均速度(2个速度) */
    wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizedppLOG;
    pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp; /* 角度插补 */

    wAvrSpeed_dpp = wSumOrig >> pHandle->SpeedBufferSizedppLOG;
    pHandle->Orig_ElSpeedDpp = (int16_t)wAvrSpeed_dpp; /* 判断方向 */

    pHandle->DppBufferSum = wSum;
    pHandle->DppOrigBufferSum = wSumOrig;
}

/**
 * @brief 估计Bemf数值
 * @param *pHandle STO+PLL对象
 * @retval 估计Bemf数值
 */
int32_t STO_CR_GetEstimatedBemfLevel(STO_CR_Handle_t *pHandle)
{
    return (pHandle->Est_Bemf_Level);
}

/**
 * @brief 观测Bemf数值
 * @param *pHandle STO+PLL对象
 * @retval 观测Bemf数值
 */
int32_t STO_CR_GetObservedBemfLevel(STO_CR_Handle_t *pHandle)
{
    return (pHandle->Obs_Bemf_Level);
}

/**
 * @brief 是否启用机械速度,观测与估计Es,判断算法收敛性的标志
 * @param *pHandle STO+PLL对象
 * @param bSel 启用/禁用
 * @retval None
 */
void STO_CR_BemfConsistencyCheckSwitch(STO_CR_Handle_t *pHandle, bool bSel)
{
    pHandle->EnableDualCheck = bSel;
}

/**
 * @brief 估计Bemf和观测的收敛性
 * @param *pHandle STO+PLL对象
 * @retval 估计Bemf和观测的收敛性
 */
bool STO_CR_IsBemfConsistent(STO_CR_Handle_t *pHandle)
{
    return (pHandle->IsBemfConsistent);
}

/**
 * @brief  获取速度是否可靠
 * @param  pHandle STO对象(旋转坐标系下)
 * @retval bool
 */
bool STO_CR_IsSpeedReliable(STO_Handle_t *pHandle)
{
    STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super;
    return (pHdl->IsSpeedReliable);
}

/**
 * @brief 强制观测器收敛1标志
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_CR_ForceConvergency1(STO_Handle_t *pHandle)
{
    STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super;
    pHdl->ForceConvergency = true;
}

/**
 * @brief 强制收敛标志2标志
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_CR_ForceConvergency2(STO_Handle_t *pHandle)
{
    STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super;
    pHdl->ForceConvergency2 = true;
}
