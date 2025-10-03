#include "bsp_sto_pll.h"
#include "bsp_pid.h"
#include "bsp_spd.h"

/* STO+PLL对象(静止坐标系-中高速) */
STO_PLL_Handle_t STO_PLL_M1 = {
    ._Super = {
        .bElToMecRatio = POLE_PAIR_NUM,                                           /* 极对数 */
        .hMaxReliableMecSpeed01Hz = (uint16_t)(1.15 * MAX_APPLICATION_SPEED / 6), /* 最大可靠机械速度0.1Hz */
        .hMinReliableMecSpeed01Hz = (uint16_t)(MIN_APPLICATION_SPEED / 6),        /* 最小可靠机械速度0.1Hz */
        .bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS,                   /* 最大测量错误次数 */
        .hMaxReliableMecAccel01HzP = 65535,                                       /* 最大可靠机械加速度0.1Hz */
        .hMeasurementFrequency = TF_REGULATION_RATE,                              /* 采样频率 */
    },
    .hC1 = C1, /* Rs/(Ls*Ts) */
    .hC2 = C2, /* I_est的增益->K1*Z */
    .hC3 = C3, /* Rs_bemf/(Ls*Ts) */
    .hC4 = C4, /* E_est的增益->K2*Z */
    .hC5 = C5, /* Rs_vs/(Ls*Ts) */
    .hF1 = F1, /* Q14 */
    .hF2 = F2, /* Q11 */

    /* PI */
    .PIRegulator = {
        .hDefKpGain = PLL_KP_GAIN,
        .hDefKiGain = PLL_KI_GAIN,
        .hDefKdGain = 0x0000U,
        .hKpDivisor = PLL_KPDIV,
        .hKiDivisor = PLL_KIDIV,
        .hKdDivisor = 0x0000U,
        .wUpperIntegralLimit = INT32_MAX,
        .wLowerIntegralLimit = -INT32_MAX,
        .hUpperOutputLimit = INT16_MAX,
        .hLowerOutputLimit = -INT16_MAX,
        .hKpDivisorPOW2 = PLL_KPDIV_LOG,
        .hKiDivisorPOW2 = PLL_KIDIV_LOG,
        .hKdDivisorPOW2 = 0x0000U,

    },

    .SpeedBufferSize01Hz = STO_FIFO_DEPTH_01HZ,                                   /* SPD_GetAvrgMecSpeed01Hz 函数所导出的估计速度0.1Hz */
    .SpeedBufferSizedpp = STO_FIFO_DEPTH_DPP,                                     /* SPD_GetElSpeedDpp 和状态观测器方程导出的估计速度dpp */
    .VariancePercentage = PERCENTAGE_FACTOR,                                      /* 速度估计最大允许方差的参数 */
    .SpeedValidationBand_H = SPEED_BAND_UPPER_LIMIT,                              /* 估计速度>定子电频率 程度 */
    .SpeedValidationBand_L = SPEED_BAND_LOWER_LIMIT,                              /* 估计速度<定子电频率 程度 */
    .MinStartUpValidSpeed = OBS_MINIMUM_SPEED,                                    /* 启动时最小机械转速的绝对值0.1Hz */
    .StartUpConsistThreshold = NB_CONSECUTIVE_TESTS,                              /* 连续进行的测试次数->启动成功 */
    .Reliability_hysteresys = OBS_MEAS_ERRORS_BEFORE_FAULTS,                      /* 机械转速测量错误次数 */
    .BemfConsistencyCheck = BEMF_CONSISTENCY_TOL,                                 /* 观测器的BEMF的置信度[1,64] */
    .BemfConsistencyGain = BEMF_CONSISTENCY_GAIN,                                 /* 观测器BEMF增益的准确度[1,64,105] */
    .MaxAppPositiveMecSpeed01Hz = (uint16_t)(MAX_APPLICATION_SPEED * 1.15 / 6.0), /* 最大正向机械速度0.1Hz */

    .F1LOG = F1_LOG,                                /* F1增益分频器以2的幂次 */
    .F2LOG = F2_LOG,                                /* F2增益分频器以2的幂次 */
    .SpeedBufferSizedppLOG = STO_FIFO_DEPTH_DPP_LOG /* bSpeedBufferSizedpp 的值表示为2的幂次方 */
};

STO_PLL_Handle_t *pSTO_PLL_M1 = &STO_PLL_M1;

STO_Handle_t STO_M1 = {
    ._Super = (SpeednPosFdbk_Handle_t *)&STO_PLL_M1,
    .pFctForceConvergency1 = &STO_PLL_ForceConvergency1,
    .pFctForceConvergency2 = &STO_PLL_ForceConvergency2,
    .pFctStoOtfResetPLL = &STO_OTF_ResetPLL,
    .pFctSTO_SpeedReliabilityCheck = &STO_PLL_IsVarianceTight,
};

/**
 * @brief 初始化STO和PLL参数
 * @param *pHandle STO+PLL对象,在alpha-beta系(中高速),基于反电动势重构
 * @retval None
 */
void STO_PLL_Init(STO_PLL_Handle_t *pHandle)
{
    int16_t htempk;
    int32_t wAux;

    /* 收敛与方差的相关性参数 */
    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
    pHandle->EnableDualCheck = true;

    wAux = (int32_t)1;
    pHandle->F3POW2 = 0u;

    /* 求出能被2整除的F3 */
    htempk = (int16_t)(C6_COMP_CONST1 / (pHandle->hF2));

    while (htempk != 0) {
        htempk /= (int16_t)2;
        wAux *= (int32_t)2;
        pHandle->F3POW2++;
    }

    /* C6=F2*F3/C6_COMP_CONST2,与2*pi*f标幺化有关 */
    pHandle->hF3 = (int16_t)wAux;
    wAux = (int32_t)(pHandle->hF2) * pHandle->hF3;
    pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);

    /* 清锁相环相关变量 */
    STO_PLL_Clear(pHandle);

    /* 清PID积分项 */
    PID_HandleInit(&pHandle->PIRegulator);
    pHandle->_Super.hMecAccel01HzP = 0;

    return;
}

/**
 * @brief Return
 * @param *pHandle STO+PLL对象
 * @param flag 标志位
 * @retval None
 */
void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag)
{
    return;
}

/**
 * @brief STO观测器和PLL计算电机角度
 *        调参:(1)滑膜增益I_est->K1*Z,E_est->K2*Z (2)2*pi*f0的带宽和相角裕度有关的C6
 *        e^(-p)=1-p+p^2/2!-p^3/3!...泰勒展开
 * @param *pHandle STO+PLL对象
 * @param *pInputs 观测器输入
 * @retval 电角度
 */
int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
    int32_t wAux, wDirection;
    int32_t wIalfa_est_Next, wIbeta_est_Next;
    int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
    int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed, hValfa, hVbeta;

    /* 1.估计E_alfha */
    if (pHandle->wBemf_alfa_est > (int32_t)(pHandle->hF2) * INT16_MAX) {
        pHandle->wBemf_alfa_est = INT16_MAX * (int32_t)(pHandle->hF2); /* E-Q11=F2 */
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
        pHandle->Ialfa_est = INT16_MAX * (int32_t)(pHandle->hF1); /* I-Q14=F1 */
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
    wAux = (int32_t)(pHandle->hC1) * hAux;       /* C1=F1*RS/(LS*TF_REGULATION_RATE),F=(1-C1) */
    wIalfa_est_Next = pHandle->Ialfa_est - wAux; /* (1-C1)*I_alpha(n),与TI手册公式(4)存在区别,但都是pu化后的值 */

    /* G2*Z(n),控制函数K2*Ke*sgn(Is-Is_est)=K*Z(n)->I_est */
    wAux = (int32_t)(pHandle->hC2) * hIalfa_err;
    wIalfa_est_Next += wAux;

    /* V_alpha(n) */
    wAux = (int32_t)(pHandle->hC5) * hValfa; /* G5=C5=F1*MAX_VOLTAGE/(LS*MAX_CURRENT*TF_REGULATION_RATE) */
    wIalfa_est_Next += wAux;

    /* E_alpha(n) */
    wAux = (int32_t)(pHandle->hC3) * hAux_Alfa; /* G3=C3=F1*MAX_BEMF_VOLTAGE/(LS*MAX_CURRENT*TF_REGULATION_RATE) */
    wIalfa_est_Next -= wAux;

    /* 10.d(E_alpha)/dt = -W0*E_alpha(n)+W0*Z */
    /* E_alpha(n+1) = E_alpha(n) + C4*I_err(n) - C6*E_alpha(n) */
    wAux = (int32_t)(pHandle->hC4) * hIalfa_err;          /* C4*I_err(n)=G4*Z(n),控制函数K4*Ke*sgn(Is-Is_est)=K*Z(n)->E_est */
    wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux; /* G4*Z(n) */

    wAux = (int32_t)hAux_Beta >> pHandle->F3POW2;
    wAux = wAux * pHandle->hC6;                /* C6*E_alpha(n) */
    wAux = pHandle->_Super.hElSpeedDpp * wAux; /* -(-W0*Es) */
    wBemf_alfa_est_Next += wAux;

    /* 11.d(I_beta)/dt = A*I_beta(n) + B(Vs-Es+Z),Z=ksign(估计Is-观测Is) */
    /* I_beta(n+1) = I_beta(n) - C1*I_beta(n) + C2*I_err(n) + C5*V_beta(n) - C3*E_beta(n) */
    hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
    wAux = (int32_t)(pHandle->hC1) * hAux;
    wIbeta_est_Next = pHandle->Ibeta_est - wAux;

    /* G2*Z(n),控制函数K2*Ke*sgn(Is-Is_est)=K*Z(n)->I_est */
    wAux = (int32_t)(pHandle->hC2) * hIbeta_err;
    wIbeta_est_Next += wAux;

    /* V_beta(n) */
    wAux = (int32_t)(pHandle->hC5) * hVbeta;
    wIbeta_est_Next += wAux;

    /* E_beta(n) */
    wAux = (int32_t)(pHandle->hC3) * hAux_Beta;
    wIbeta_est_Next -= wAux;

    /* 12.d(E_beta)/dt = -W0*E_beta(n)+W0*Z */
    /* E_beta(n+1) = E_beta(n) + C4*I_err(n) - C6*E_beta(n) */
    wAux = (int32_t)(pHandle->hC4) * hIbeta_err;          /* C4*I_err(n)=G4*Z(n),控制函数K4*Ke*sgn(Is-Is_est)=K*Z(n)->E_est */
    wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux; /* G4*Z(n) */

    wAux = (int32_t)hAux_Alfa >> pHandle->F3POW2;
    wAux = wAux * pHandle->hC6;                /* C6*E_alpha(n) */
    wAux = pHandle->_Super.hElSpeedDpp * wAux; /* -(W0*Es) */
    wBemf_beta_est_Next -= wAux;

    if (pHandle->_Super.hElSpeedDpp >= 0) {
        wDirection = 1;
    } else {
        wDirection = -1;
    }

    /* 13.锁相环PLL E_alpha(n+1) E_beta(n+1) */
    pHandle->hBemf_alfa_est = hAux_Alfa;
    pHandle->hBemf_beta_est = hAux_Beta;

    hAux_Alfa = (int16_t)(hAux_Alfa * wDirection);
    hAux_Beta = (int16_t)(hAux_Beta * wDirection);

    /* 14.PLL运算提取转子位置 */
    hRotor_Speed = STO_ExecutePLL(pHandle, hAux_Alfa, -hAux_Beta);

    /* 15.估计速度存入缓存 */
    STO_Store_Rotor_Speed(pHandle, hRotor_Speed);

    pHandle->_Super.hElAngle += hRotor_Speed;

    /* 16.迭代估计E与I */
    pHandle->Ialfa_est = wIalfa_est_Next;
    pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

    pHandle->Ibeta_est = wIbeta_est_Next;
    pHandle->wBemf_beta_est = wBemf_beta_est_Next;

    return (pHandle->_Super.hElAngle);
}

/**
 * @brief 1.转速方差判断速度可靠性 2.E_est^2与转速计算的E_w^2
 *        3.估计速度在限制范围内-转速可靠性 4.机械速度计算及可靠性校验
 *        IsSpeedReliable = 1
 * @note 速度可靠性、Oberser的收敛性
 * @param *pHandle STO+PLL对象
 * @param *pMecSpeed01Hz 机械转速
 * @retval bool 转速可靠性
 */
bool STO_PLL_CalcAvrgMecSpeed01Hz(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
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
    /* 启用E_est^2与转速计算的E_w^2做比较 EnableDualcheck=1 */
    if (pHandle->EnableDualCheck == true) /*do algorithm if it's enabled*/
    {
        wAux = (wAux < 0 ? (-wAux) : (wAux)); /* wAux abs value   */
        if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeed01Hz)) {
            /* 6.观测Es^2=E_alfha^2+E_beta^2 */
            wObsBemf = (int32_t)(pHandle->hBemf_alfa_est);
            wObsBemfSq = wObsBemf * wObsBemf;
            wObsBemf = (int32_t)(pHandle->hBemf_beta_est);
            wObsBemfSq += wObsBemf * wObsBemf;

            /* 7.E_est^2与转速计算的E_w^2,Es=3/2*K_e*W*(-sin(θ)*cos(θ)) */
            wEstBemf = (wAux * 32767) / (int16_t)(pHandle->_Super.hMaxReliableMecSpeed01Hz); /* 机械转速标幺化 */
            wEstBemfSq = (wEstBemf * (int32_t)(pHandle->BemfConsistencyGain)) / 64;
            wEstBemfSq *= wEstBemf;

            wEstBemfSqLo = wEstBemfSq - (wEstBemfSq / 64) * (int32_t)(pHandle->BemfConsistencyCheck); /* -sin(θ) */

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

    /* 9.转速可靠性 */
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
 * @brief 按两速度buff,进行滑动平均计算
 * @param *pHandle STO+PLL对象
 * @retval void
 */
void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle)
{
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;
    int32_t wSum = pHandle->DppBufferSum;
    int32_t wAvrSpeed_dpp;
    int16_t hSpeedBufferSizedpp = (int16_t)(pHandle->SpeedBufferSizedpp);   /* 转速buff0的大小 */
    int16_t hSpeedBufferSize01Hz = (int16_t)(pHandle->SpeedBufferSize01Hz); /* 转速buff1的大小 */
    int16_t hBufferSizeDiff;

    /* 1.取出未参与滑动速度计算的Index */
    hBufferSizeDiff = hSpeedBufferSize01Hz - hSpeedBufferSizedpp; /* 0-0 */

    /* 2.两buff全参与计算,直接加入新速度,同时去掉旧速度 */
    if (hBufferSizeDiff == 0) {
        wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
    } else {
        /* 3.两buff不全参与计算,加入新速度,同时按index去掉旧速度 */
        hIndexOldTemp = hIndexNew + hBufferSizeDiff;

        if (hIndexOldTemp >= hSpeedBufferSize01Hz) {
            hIndexOld = hIndexOldTemp - hSpeedBufferSize01Hz;
        } else {
            hIndexOld = hIndexOldTemp;
        }

        wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];
    }

    /* 4.计算滑动平均速度 */
    wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizedppLOG;

    pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
    pHandle->DppBufferSum = wSum;
}

/**
 * @brief 清除STO PLL变量
 * @param *pHandle STO+PLL对象
 * @retval
 */
void STO_PLL_Clear(STO_PLL_Handle_t *pHandle)
{
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->wBemf_alfa_est = (int32_t)0;
    pHandle->wBemf_beta_est = (int32_t)0;
    pHandle->_Super.hElAngle = (int16_t)0;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

    STO_InitSpeedBuffer(pHandle);
    PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)0);
}

/**
 * @brief 更新速度buff
 * @param *pHandle STO+PLL对象
 * @param hRotor_Speed 转速
 * @retval None
 */
inline static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed)
{

    uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

    bBuffer_index++;
    if (bBuffer_index == pHandle->SpeedBufferSize01Hz) {
        bBuffer_index = 0u;
    }

    pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];

    pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
    pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
 * @brief PLL锁相环计算转速
 * @param *pHandle STO+PLL对象
 * @param hBemf_alfa_est 估计E_alfha
 * @param hBemf_beta_est 估计E_beta
 * @retval 转速
 */
inline static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)
{
    int32_t wAlfa_Sin_tmp, wBeta_Cos_tmp;
    int16_t hOutput;
    Trig_Components Local_Components;
    int16_t hAux1, hAux2;

    Local_Components = MCM_Trig_Functions(pHandle->_Super.hElAngle);

    /* 1.-(E_alpha*cos(θ) + E_beta*sin(θ)) = K*(θ_观测 - θ_估计) */
    wAlfa_Sin_tmp = (int32_t)(hBemf_alfa_est) * (int32_t)Local_Components.hSin;
    wBeta_Cos_tmp = (int32_t)(hBemf_beta_est) * (int32_t)Local_Components.hCos;

    hAux1 = (int16_t)(wBeta_Cos_tmp >> 15);
    hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15);

    /* 2.K*(θ_观测 - θ_估计) */
    hOutput = PI_Controller(&pHandle->PIRegulator, (int32_t)(hAux1)-hAux2);

    return (hOutput);
}

/**
 * @brief 初始化速度缓存
 * @param *pHandle STO+PLL对象
 * @retval None
 */
static void STO_InitSpeedBuffer(STO_PLL_Handle_t *pHandle)
{
    uint8_t b_i;
    uint8_t bSpeedBufferSize01Hz = pHandle->SpeedBufferSize01Hz;

    /*init speed buffer*/
    for (b_i = 0u; b_i < bSpeedBufferSize01Hz; b_i++) {
        pHandle->Speed_Buffer[b_i] = (int16_t)0;
    }
    pHandle->Speed_Buffer_Index = 0u;
    pHandle->SpeedBufferOldestEl = (int16_t)0;

    return;
}

/**
 * @brief 观测器速度是否收敛(按估计转速与目标转速)
 *        估计速度在 当前速度 收敛范围内->算法收敛 IsAlgorithmConverged=1
 * @param *pHandle STO+PLL对象
 * @param hForcedMecSpeed01Hz 目标转速(可强制设置)
 * @retval bool 1-收敛
 */
bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t hForcedMecSpeed01Hz)
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
                                /* 9.估计速度在当前速度收敛范围内->算法收敛 IsAlgorithmConverged=1 */
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
 * @brief 估计的E_alfha E_beta
 * @param *pHandle STO+PLL对象
 * @retval 估计的E_alfha E_beta
 */
Volt_Components STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle)
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
Curr_Components STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle)
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
void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
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
void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
    pHandle->hC2 = hhC1;
    pHandle->hC4 = hhC2;
}

/**
 * @brief 获取PLL的增益系数P I
 * @param *pHandle STO+PLL对象
 * @param *pPgain
 * @param *pIgain
 * @retval None
 */
void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain)
{

    *pPgain = PID_GetKP(&pHandle->PIRegulator);
    *pIgain = PID_GetKI(&pHandle->PIRegulator);
}

/**
 * @brief 设置PLL的增益系数P I
 * @param *pHandle STO+PLL对象
 * @param hPgain
 * @param hIgain
 * @retval None
 */
void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain)
{
    PID_SetKP(&pHandle->PIRegulator, hPgain);
    PID_SetKI(&pHandle->PIRegulator, hIgain);
}

/**
 * @brief 设置目标机械角度
 * @param *pHandle STO+PLL对象
 * @param hMecAngle 目标机械角度
 * @retval None
 */
void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle)
{
}

/**
 * @brief 清除PLL PI的积分项
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_OTF_ResetPLL(STO_Handle_t *pHandle)
{
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super;
    PID_SetIntegralTerm(&pHdl->PIRegulator, (int32_t)0);
}

/**
 * @brief 清除PLL PI的积分项
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_ResetPLL(STO_PLL_Handle_t *pHandle)
{
    PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)0);
}

/**
 * @brief 设置PLL PI参数
 * @param *pHandle STO+PLL对象
 * @param hElSpeedDpp 电转速dpp
 * @param hElAngle 电角度
 * @retval  None
 */
void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle)
{
    PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)hElSpeedDpp * (int32_t)PID_GetKIDivisor(&pHandle->PIRegulator));
    pHandle->_Super.hElAngle = hElAngle;
}

/**
 * @brief 估计Bemf数值
 * @param *pHandle STO+PLL对象
 * @retval 估计Bemf数值
 */
int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle)
{
    return (pHandle->Est_Bemf_Level);
}

/**
 * @brief 观测Bemf数值
 * @param *pHandle STO+PLL对象
 * @retval 观测Bemf数值
 */
int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle)
{
    return (pHandle->Obs_Bemf_Level);
}

/**
 * @brief 是否启用机械速度,观测与估计Es,判断算法收敛性的标志
 * @param *pHandle STO+PLL对象
 * @param bSel 启用/禁用
 * @retval None
 */
void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel)
{
    pHandle->EnableDualCheck = bSel;
}

/**
 * @brief 估计Bemf和观测的收敛性
 * @param *pHandle STO+PLL对象
 * @retval 估计Bemf和观测的收敛性
 */
bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle)
{
    return (pHandle->IsBemfConsistent);
}

/**
 * @brief 获取速度是否可靠
 * @param *pHandle STO+PLL对象
 * @retval 可靠标志
 */
bool STO_PLL_IsVarianceTight(STO_Handle_t *pHandle)
{
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super;
    return (pHdl->IsSpeedReliable);
}

/**
 * @brief 强制观测器收敛1标志
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle)
{
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super;
    pHdl->ForceConvergency = true;
}

/**
 * @brief 强制收敛标志2标志
 * @param *pHandle STO+PLL对象
 * @retval None
 */
void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle)
{
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super;
    pHdl->ForceConvergency2 = true;
}

/**
 * @brief 设置最小启动有效速度
 * @param *pHandle STO+PLL对象
 * @param hMinStartUpValidSpeed 最小启动有效速度
 * @retval None
 */
void STO_SetMinStartUpValidSpeed01HZ(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed)
{
    pHandle->MinStartUpValidSpeed = hMinStartUpValidSpeed;
}
