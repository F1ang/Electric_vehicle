/*******************************************************************************
 * 版权所有 (C)2019, Bright Power Semiconductor Co.ltd
 * 文件名称： Main_Drive.c
 * 文件标识：
 * 内容摘要： 状态机处理函数
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Tonghua Li
 * 完成日期： 2022年8月19日
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "MA_Static_Variable.h"
#include "MC_Math.h"
#include "MC_type.h"
#include "Time_Process.h"

//////////////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************/
// 全局变量定义区

extern bool MinQSpdFilReachHi[MA_OBJ_MAX_DEFINE];
/*******************************************************************************/
// 外部函数引用区

extern void FaultInit(STR_MACtrProc *pObj);     //	错误检测变量初始化
extern void FaultInitTask(STR_MACtrProc *pObj); //	错误检测变量初始化任务
extern void FaultRecoverInit(STR_MACtrProc *pObj);
extern void FaultRecover(STR_MACtrProc *pObj);
extern void FaultCurrentCheck(STR_VectorUVW *pCurrent, STR_MACtrProc *pObj);
extern void updateObParaMALayer(PSTR_ControlObject pCtrlObj);
extern void updateAcrParaMALayer(PSTR_ControlObject pCtrlObj);
extern void updateSVPWMParaMALayer(PSTR_ControlObject pCtrlObj);

extern void PowerLoopInit(STR_MACtrProc *pObj);
extern void PowerLimitCalc(STR_MechQuantity *pstruMotorSpeed, s32 PowerValue);
extern void SpeedLimitCalc(s32 wSpeedFbk, pSTR_PowerDef pstruPower);
extern void PowerLoopReg(pSTR_PowerDef pstru_Power);
extern void BrakeStateCheck(u32 t_CurrentSqrt, pSTR_OnTheFlyDetectMA this);

extern bool getRunStatus(u8 t_bObjId);
extern s16 AutoFieldWeakReg(u8 t_bObjId);
extern void InitFieldWeakenID(u8 t_bObjId);
extern bool motorStopOrExceptionHandle(u8 RunStop, pSTR_MotorExceptionMA this, s16 *PReturnCurSet);
extern void InitDQCurRefGen(u8 t_bObjId, s16 dCurSet, s16 qCurSet, s16 TransTime);
extern void SetDCurRefInitValue(u8 t_bObjId, s16 dCurref);
extern s16 getDCurRefGen(u8 t_bObjId);
extern void QCurRefGenValue(u8 t_bObjId, s64 *pValue, u8 QFormat, s32 spdSet);
extern void SpeedPIAutoTunning(u8 t_bObjId, s16 KpFactor, s16 Ton, s32 *pKp, s32 *pKi);
extern void InitDQCurRefGenWithWeakenFlux(u8 t_bObjId, s16 dCurSet, s16 qCurSet, s16 TransTime);
extern s16 getDCurRefGenWithEnWeakenFlux(u8 t_bObjId);
extern s16 getDCurRefGenWithWeakenFlux(u8 t_bObjId);

extern bool IsNeedIPD(STR_MACtrProc *pObj);
extern void setOBElectAngleEst(u8 t_bObjId, u32 theta);

extern bool getIfSpdReachHigh(u8 t_bObjId);
extern void MinMotorVoltSpd2Core(u8 tObjId);

/*******************************************************************************/
// 局部静态函数定义区
static void CurrentLoopRef_filter(STR_MACtrProc *pObj); //	电流参考滤波处理函数
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
static s16 SpeedLoopReg(STR_MechQuantity *this);
#endif

static void StateInit(STR_MACtrProc *pObj);
static void StateCharge(STR_MACtrProc *pObj);
static void StateDirCheck(STR_MACtrProc *pObj);
static void StateAlign(STR_MACtrProc *pObj);
static void StateRun(STR_MACtrProc *pObj);
static void StateFault(STR_MACtrProc *pObj);
static void StateBrake(STR_MACtrProc *pObj);

/*******************************************************************************/
// 局部静态变量定义区

/*******************************************************************************/
/*****************************end***********************************************/
/*******************************************************************************
 函数名称：    static void handleIfMotorSpdZero(STR_MACtrProc *pObj)
 功能描述：    检查电机是否停止
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：    m_bMotorStopped 标志

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static void handleIfMotorSpdZero(STR_MACtrProc *pObj)
{
    PSTR_DrvCfgPara t_pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);

    s32 t_wMinSpeed = App2CoreFreqTrans(pAppToCore, t_pDrvCfgPara->mS_FBSlvcCfg0.m_wMinRunFreq);

    if (ABS(pObj->m_pMotorSpeed->wSpeedfbk) <= t_wMinSpeed) //	电机速度  < 最小运行速度 ?
    {
        pObj->m_SMACtrlBit.m_bMotorStopped = TRUE; //	小于时，设置停止标志
    } else {
        pObj->m_SMACtrlBit.m_bMotorStopped = FALSE; //	大于时，设置停止标志
    }
}
/*******************************************************************************
 函数名称：    static void handleCurSetpointClr(STR_MACtrProc *pObj)
 功能描述：    电流控制对象指令清0
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
#if ((DRV0_STOP_MODE != 0)) //	非MOE关闭停机
static void handleCurSetpointClr(STR_MACtrProc *pObj)
{
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
    pObj->m_pMotorSpeed->mSpeedPI.wIntegral = 0; //	速度积分清0
#endif
    pObj->m_nQCurrentSet = 0;    //	Iq设定值清0
    pObj->m_nQCur_Reference = 0; //	Iq滤波值清0
}
#endif

/*******************************************************************************
 函数名称：    static void handleCurSetpoint(STR_MACtrProc *pObj,s16 temp16)
 功能描述：    设置电流控制对象指令
 输入参数：    pObj指针，temp16
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
#if (DRV0_UNDVOLT_PROT_ENABLE == 1) //	电压保护使能
static void handleCurSetpoint(STR_MACtrProc *pObj, s16 temp16)
{
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
    pObj->m_pMotorSpeed->mSpeedPI.wIntegral = (s64)temp16 << 22; //	速度积分设定
#endif
    pObj->m_nQCurrentSet = temp16;    //	Iq设定值设定
    pObj->m_nQCur_Reference = temp16; //	Iq滤波值设定
}
#endif

/*******************************************************************************
 函数名称：    static void handleSpdSetpoint(STR_MACtrProc *pObj)
 功能描述：    设置转速控制对象指令
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static void handleSpdSetpoint(STR_MACtrProc *pObj)
{
    pObj->m_pMotorSpeed->wSpeedfbk = pObj->m_pMotorSpeed->wSpeedEst;
    pObj->m_pMotorSpeed->wSpeedfbkFilter = pObj->m_pMotorSpeed->wSpeedEst << 10;
    pObj->m_pMotorSpeed->wSpeedRef = pObj->m_pMotorSpeed->wSpeedfbk;
    pObj->m_pMotorSpeed->STR_SpdRampGen.wRampOut = pObj->m_pMotorSpeed->wSpeedfbk;
}

/*******************************************************************************
 函数名称：    static void HandleSpdSetpointRamp(STR_MACtrProc *pObj)
 功能描述：    设置转速控制对象指令ramp处理
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static void HandleSpdSetpointRamp(STR_MACtrProc *pObj)
{
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);
    PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;
    s32 t_wTemp;
    s32 t_wTemp1;

    //	根据设定的速度环周期计算斜坡增量

    t_wTemp1 = (s32)16777 * (pDrvCfgPara->mS_FBSpdLoop.nSpeedLoopCycle + 1);
    if (ABS(pObj->m_pMotorSpeed->wSpeedRef) < App2CoreFreqTrans(pAppToCore, pObj->m_pAppCommData->wSvcMinFreq)) {
        //	速度指令小于SVC最小频率(开环拖动终止频率)时，用开环加速度/减速度
        t_wTemp = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wFreq_Acc);
        pObj->m_pMotorSpeed->STR_SpdRampGen.wRampIncrease = ((s64)t_wTemp * t_wTemp1) >> 8;

        t_wTemp = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wFreq_Dec);
        pObj->m_pMotorSpeed->STR_SpdRampGen.wRampDecrease = ((s64)t_wTemp * t_wTemp1) >> 8;

    } else {
        //	速度指令大于SVC最小频率(开环拖动终止频率)时，用闭环加速度/减速度，即速度环 加速度/减速度
        t_wTemp = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wSpdRunAcc);
        pObj->m_pMotorSpeed->STR_SpdRampGen.wRampIncrease = ((s64)t_wTemp * t_wTemp1) >> 8;

        t_wTemp = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wSpdRunDec);
        pObj->m_pMotorSpeed->STR_SpdRampGen.wRampDecrease = ((s64)t_wTemp * t_wTemp1) >> 8;
    }

    //	设置斜坡输入指令
    pObj->m_pMotorSpeed->STR_SpdRampGen.wRampIn = pObj->m_pMotorSpeed->wSpeedSet;
    ramp32GenCalc(&pObj->m_pMotorSpeed->STR_SpdRampGen); //	斜坡速度规划
}

/*******************************************************************************
 函数名称：    static void HandleSpdSetpointCmd(STR_MACtrProc *pObj)
 功能描述：    设置转速控制对象指令处理
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：    wSpeedSet

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
#if (DRV0_CLOSE_LOOP != POWER_LOOP)
static void HandleSpdSetpointCmd(STR_MACtrProc *pObj)
{
    //	速度设定值处理，根据设定方向处理
    //	默认CW时，速度指令为正值
    pObj->m_pMotorSpeed->wSpeedSet = ABS(pObj->m_pMotorSpeed->wSpeedSet);
    if (pObj->m_SMACtrlBit.m_bMotorDirtion == CCW) {
        pObj->m_pMotorSpeed->wSpeedSet = -pObj->m_pMotorSpeed->wSpeedSet;
    }

    //	采用其他表示方式时，自行修改代码
    //	或实现正反转速度指令处理，当前程序支持正反转运行(需检查配置参数)。
}
#endif

// 宽度为上桥臂导通宽度，0时下桥臂全通
static void SetPWMBrakeDuty(STR_MACtrProc *pObj, s16 duty)
{
    pObj->m_nBreakDuty = duty; // 设置PWM宽度。通过PWM宽度控制刹车速度时需更改MCPWM配置
}

static s16 getPWMBrakeDuty(STR_MACtrProc *pObj)
{
    return pObj->m_nBreakDuty;
}

bool IsEnableMoeInIDLEStatus(STR_MACtrProc *pObj) // stop状态是否使能MOE
{
    return pObj->m_SMACtrlBit.m_bEnableMOEInStop;
}

bool IsBusVoltageLarge(STR_MACtrProc *pObj)
{
    return FALSE;
}

/*******************************************************************************
 函数名称：    static void SetDefaultMA2MD(STR_MACtrProc *pObj)
 功能描述：    设置控制对象MA2MD默认处理
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static void SetDefaultMA2MD(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pMA2MD = &pObj->mMaToMd;

    if (pObj->m_SMACtrlBit.m_bUAMOE == 1) //	MOE有效标志位为1 ?
    {
        SetPWMCmdMA2MD(pMA2MD, E_PWM_ON); //	设置MOE使能操作命令到交换区，由MD层调用HD驱动函数实现MOE操作
    } else {
        SetPWMCmdMA2MD(pMA2MD, E_PWM_OFF); //	设置操作命令到交换区，由MD层调用HD驱动函数实现MOE操作
    }

    SetBrakePWMDutyMA2MD(pMA2MD, getPWMBrakeDuty(pObj)); //	设置制动宽度到MD层，由MD层调用HD驱动函数实现PWM脉宽操作

    SetMoeBlankTimeMA2MD(pMA2MD, 5);         //	命令有效间隔时间，MD层后续添加
    SetMotorCmdMA2MD(pMA2MD, E_MOTOR_RESET); //	设置MD层状态，操作通过数据交换区值传递实现
}

/*******************************************************************************
 函数名称：    static void SetFaultMA2MD(STR_MACtrProc *pObj)
 功能描述：    设置控制对象MA2MD默认FAULT处理
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static void SetFaultMA2MD(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pMA2MD = &pObj->mMaToMd;

    if (pObj->m_SMACtrlBit.m_bUAMOE == 1) //	MOE有效标志位为1?	--- FAULT状态是否使能MOE?
    {
        SetPWMCmdMA2MD(pMA2MD, E_PWM_ON); //	设置MOE使能操作命令到交换区，由MD层调用HD驱动函数实现MOE操作
    } else {
        SetPWMCmdMA2MD(pMA2MD, E_PWM_OFF); //	设置操作命令到交换区，由MD层调用HD驱动函数实现MOE操作
    }

    SetBrakePWMDutyMA2MD(pMA2MD, getPWMBrakeDuty(pObj)); //	设置制动宽度到MD层，由MD层调用HD驱动函数实现PWM脉宽操作

    SetMoeBlankTimeMA2MD(pMA2MD, 5);
    SetMotorCmdMA2MD(pMA2MD, E_MOTOR_RESET); //	命令有效间隔时间，MD层后续添加
}

static PSTR_PubDatMAMD GetPointerMA2MD(STR_MACtrProc *pObj)
{
    return &(pObj->mMaToMd);
}

static pSTR_PubDatMAUA GetPointerMA2UA(STR_MACtrProc *pObj)
{
    return &(pObj->mMaToUa);
}

/*******************************************************************************
 函数名称：    static void InitObserverDone(STR_MACtrProc *pObj)
 功能描述：    初始化观测器
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static bool InitObserverDone(STR_MACtrProc *pObj)
{
    bool t_bResult = FALSE;
    if ((pObj->m_SMACtrlBit.m_bObParaReq == 1)       //	观测器参数初始化请求
        && (pObj->m_SMACtrlBit.m_bObParaUpdate == 1) //	观测器参数需要更新
        && (getNonInitResultMD2MA(&pObj->mMdToMa))   //	没有其他操作命令在执行
    ) {
        updateObParaMALayer(pObj->mg_pCtrlObj);   //	更新观测器参数
        pObj->m_SMACtrlBit.m_bObParaReq = 0;      //	清除观测器参数初始化请求标志
        pObj->m_SMACtrlBit.m_bObParaUpdate = 0;   //	观测器参数不需要更新(更新完成)
        pObj->m_SMACtrlBit.m_bObParaInitDone = 0; //	观测器参数初始化未完成
    }

    if (pObj->m_SMACtrlBit.m_bObParaInitDone == 0) //	观测器参数初始化未完成 ?
    {
        if (pObj->m_SMACtrlBit.m_bObParaUpdate == 0) //	观测器参数不需要更新(更新完成) ?
        {
            if (!getObsevInitResultMD2MA(&pObj->mMdToMa)) //	MD层初始化是否完成
            {
                setObsevInitFCmdMA2MD(&pObj->mMaToMd); //	未完成，发送初始化命令
            } else {
                setDefaultCmdMA2MD(&pObj->mMaToMd);     //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bObParaUpdate = 1; //	下次初始化时，观测器参数需要更新
            }
        } else {
            if (getNonInitResultMD2MA(&pObj->mMdToMa)) //	观测器参数不需要更新(更新完成) ，已完成
            {
                setDefaultCmdMA2MD(&pObj->mMaToMd);       //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bObParaInitDone = 1; //	观测器参数初始化完成
            }
        }
    } else {
        if (pObj->m_SMACtrlBit.m_bObParaReq == 0) //	观测器参数初始化完成，没有初始化请求
        {
            t_bResult = TRUE; //	操作完成
        }
    }
    return t_bResult;
}

/*******************************************************************************
 函数名称：    static void InitAcrDone(STR_MACtrProc *pObj)
 功能描述：    初始化MD层FOC控制变量
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static bool InitAcrDone(STR_MACtrProc *pObj)
{
    bool t_bResult = FALSE;

    if ((pObj->m_SMACtrlBit.m_bAcrParaReq == 1)       //	电流环参数初始化请求
        && (pObj->m_SMACtrlBit.m_bAcrParaUpdate == 1) //	电流环参数需要更新
        && (getNonInitResultMD2MA(&pObj->mMdToMa))    //	没有其他操作命令在执行
    ) {
        updateAcrParaMALayer(pObj->mg_pCtrlObj); //	更新电流环参数
        pObj->m_SMACtrlBit.m_bAcrParaReq = 0;    //	清除电流环参数初始化请求标志
        pObj->m_SMACtrlBit.m_bAcrParaUpdate = 0; //	电流环参数不需要更新(更新完成)
        pObj->m_SMACtrlBit.m_bAcrInitDone = 0;   //	电流环参数初始化未完成
    }

    if (pObj->m_SMACtrlBit.m_bAcrInitDone == 0) //	电流环参数初始化未完成 ?
    {
        if (pObj->m_SMACtrlBit.m_bAcrParaUpdate == 0) //	电流环参数不需要更新(更新完成) ?
        {
            if (!getAcrInitResultMD2MA(&pObj->mMdToMa)) //	MD层初始化是否完成
            {
                setAcrInitFCmdMA2MD(&pObj->mMaToMd); //	未完成，发送初始化命令
            } else {
                setDefaultCmdMA2MD(&pObj->mMaToMd);      //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bAcrParaUpdate = 1; //	下次初始化时，电流环参数需要更新
            }
        } else {
            if (getNonInitResultMD2MA(&pObj->mMdToMa)) //	电流环参数不需要更新(更新完成) ，已完成
            {
                setDefaultCmdMA2MD(&pObj->mMaToMd);    //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bAcrInitDone = 1; //	电流环参数初始化完成
            }
        }
    } else {
        if (pObj->m_SMACtrlBit.m_bAcrParaReq == 0) //	电流环参数初始化完成，没有初始化请求
        {
            t_bResult = TRUE; //	操作完成
        }
    }
    return t_bResult;
}

/*******************************************************************************
 函数名称：    static void InitSVPWMDone(STR_MACtrProc *pObj)
 功能描述：    初始化MD层SVPWM控制参数
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static bool InitSVPWMDone(STR_MACtrProc *pObj)
{
    bool t_bResult = FALSE;
    if ((pObj->m_SMACtrlBit.m_bSVPWMParaReq == 1)       //	SVPWM参数初始化请求
        && (pObj->m_SMACtrlBit.m_bSVPWMParaUpdate == 1) //	SVPWM参数需要更新
        && (getNonInitResultMD2MA(&pObj->mMdToMa))      //	没有其他操作命令在执行
    ) {
        updateSVPWMParaMALayer(pObj->mg_pCtrlObj); //	更新SVPWM参数
        pObj->m_SMACtrlBit.m_bSVPWMParaReq = 0;    //	清除SVPWM参数初始化请求标志
        pObj->m_SMACtrlBit.m_bSVPWMParaUpdate = 0; //	SVPWM参数不需要更新(更新完成
        pObj->m_SMACtrlBit.m_bSVPWMInitDone = 0;   //	SVPWM参数初始化未完成
    }
    if (pObj->m_SMACtrlBit.m_bSVPWMInitDone == 0) //	SVPWM参数初始化未完成 ?
    {
        if (pObj->m_SMACtrlBit.m_bSVPWMParaUpdate == 0) //	SVPWM参数不需要更新(更新完成) ?
        {
            if (!getSVPWMInitResultMD2MA(&pObj->mMdToMa)) //	MD层初始化是否完成
            {
                setSVPWMInitFCmdMA2MD(&pObj->mMaToMd); //	未完成，发送初始化命令
            } else {
                setDefaultCmdMA2MD(&pObj->mMaToMd);        //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bSVPWMParaUpdate = 1; //	下次初始化时，SVPWM参数需要更新
            }
        } else {
            if (getNonInitResultMD2MA(&pObj->mMdToMa)) //	SVPWM参数不需要更新(更新完成) ，已完成
            {
                setDefaultCmdMA2MD(&pObj->mMaToMd);      //	已完成，发送默认命令
                pObj->m_SMACtrlBit.m_bSVPWMInitDone = 1; //	SVPWM参数初始化完成
            }
        }
    } else {
        setDefaultCmdMA2MD(&pObj->mMaToMd);          //	已完成，发送默认命令
        if (pObj->m_SMACtrlBit.m_bSVPWMParaReq == 0) //	SVPWM参数初始化完成，没有初始化请求
        {
            t_bResult = TRUE; //	操作完成
        }
    }
    return t_bResult;
}

/*******************************************************************************
 函数名称：    static void motorControlParaInitDone(STR_MACtrProc *pObj)
 功能描述：    电机控制MD层全部初始化操作
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static bool motorControlParaInitDone(STR_MACtrProc *pObj)
{
    bool t_bReturn;

    t_bReturn = InitObserverDone(pObj); //	初始化观测器参数，完成时返回TRUE

    if (t_bReturn) //	观测器参数初始化完成后，进行SVPWM参数初始化
    {
        t_bReturn = InitSVPWMDone(pObj); //	初始化SVPWM参数, 完成时返回TRUE
    }

    if (t_bReturn) //	SVPWM桉树初始化完成后，进行电流环参数初始化
    {
        t_bReturn = InitAcrDone(pObj); //	初始化电流环参数, 完成时返回TRUE
    }

    if (t_bReturn) //	电流环参数初始化完成后
    {
        if ((pObj->m_SMACtrlBit.m_bObParaInitDone == 1) // 都初始化完成时，参数初始化完成
            && (pObj->m_SMACtrlBit.m_bAcrInitDone == 1) && (pObj->m_SMACtrlBit.m_bSVPWMInitDone == 1)) {
            t_bReturn = TRUE;
        } else {
            t_bReturn = FALSE;
        }
    }
    return t_bReturn;
}
/*******************************************************************************
 函数名称：    static void motorAcrDataInitDone(STR_MACtrProc *pObj)
 功能描述：    电机电流环数据初始化操作
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/
static bool motorAcrDataInitDone(STR_MACtrProc *pObj)
{
    bool t_bReturn = FALSE;

    if (pObj->m_SMACtrlBit.m_bObParaInitDone == 1) //	观测器参数初始化后才能进行电流环参数初始化
    {
        t_bReturn = InitAcrDone(pObj);
    }

    if (t_bReturn) {
        if ((pObj->m_SMACtrlBit.m_bObParaInitDone == 1) //	观测器参数初始化完成标志
            && (pObj->m_SMACtrlBit.m_bAcrInitDone == 1) //	电流环参数初始化完成标志
        ) {
            t_bReturn = TRUE; //	电流环参数初始化完成
        } else {
            t_bReturn = FALSE;
        }
    }
    return t_bReturn;
}

/*******************************************************************************
 函数名称：    static void setInitAcrParaReq(STR_MACtrProc *pObj)
 功能描述：    设置MD层初始化请求
 输入参数：    pObj指针
 输出参数：    无
 返 回 值：
 其它说明：

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/2/19      V1.0           Tonghua Li          创建
 *******************************************************************************/

void setInitAcrParaReq(STR_MACtrProc *pObj) //	设置电流环参数初始化请求
{
    pObj->m_SMACtrlBit.m_bAcrParaReq = 1;
}

void setInitObserverParaReq(STR_MACtrProc *pObj) //	设置观测器参数初始化请求
{
    pObj->m_SMACtrlBit.m_bObParaReq = 1;
}

void setInitSvpwmParaReq(STR_MACtrProc *pObj) //	设置SVPWM参数初始化请求
{
    pObj->m_SMACtrlBit.m_bSVPWMParaReq = 1;
}

/*****************************************************************************
 * 函数名   : bool InitDriverMain(STR_MACtrProc *pObj)
 * 说明     : 驱动初始化处理函数 TRUE ---- 当前操作执行完成
 *          : 具体实现，变量初始化
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
bool InitDriverMain(STR_MACtrProc *pObj)
{
    bool t_bReturn = FALSE;
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    t_bReturn = motorControlParaInitDone(pObj); // 电机控制MD层全部初始化操作
    if (t_bReturn) {
        StateInit(pObj);
        /*****错误检测参数初始化********/
        if (pObj->m_SMACtrlBit.m_bFaultInitReq == 1) //	存在FAULT初始化请求
        {
            FaultInit(pObj); //	FAULT初始化
            FaultRecoverInit(pObj);
            pObj->m_SMACtrlBit.m_bFaultInitReq = 0;  //	AULT初始化请求 			清0
            pObj->m_SMACtrlBit.m_bFaultInitDone = 1; //	FAULT初始化完成标志 	置1
        }
    }

    //
    handleIfMotorSpdZero(pObj); // 检查电机是否停止

    // update cmd MA2MD
    SetDefaultMA2MD(pObj); //	发送默认操作命令

    // update MA SR infor to UA

    return t_bReturn;
}

/*****************************************************************************
 * 函数名   : void PowerOnOffDriverMain(STR_MACtrProc *pObj,bool t_bOn)
 * 说明     : 功率板上下电处理函数
 *          : t_bOn     TRUE 上电
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.12.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
bool PowerOnOffDriverMain(STR_MACtrProc *pObj, bool t_bOn)
{
    bool t_bReturn = FALSE;
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateInit(pObj);

    t_bReturn = motorAcrDataInitDone(pObj); // 电机电流环初始化操作

    handleIfMotorSpdZero(pObj); //	检查电机是否停止

    SetDefaultMA2MD(pObj); //	发送默认操作命令

    if (t_bOn) {
        // 发送命令进行继电器闭合等处理

    } else {
        // 发送命令进行继电器断开等处理
        setFaultStatMA2UA(pS_MA2UA, TRUE); //	设置FAULT标志，通过共享数据交换区通知UA层
    }

    // update MA SR infor to UA

    return t_bReturn;
}
/*****************************************************************************
 * 函数名   : void CalibDriverMain(STR_MACtrProc *pObj)
 * 说明     : 功率板校正处理函数
 *          : 具体实现根据需要添加
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
void CalibDriverMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    pObj->m_SMACtrlBit.m_bCalibFinish = 1;

    handleIfMotorSpdZero(pObj);

    // update MA infor to MD
    SetDefaultMA2MD(pObj);
}

/*****************************************************************************
 * 函数名   : bool StopMotorMain(STR_MACtrProc *pObj)
 * 说明     : 电机stop状态处理函数 TRUE ---- 当前操作执行完成
 *          : 具体实现StateInit()
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
bool StopMotorMain(STR_MACtrProc *pObj)
{
    bool t_bReturn = FALSE;
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    t_bReturn = motorControlParaInitDone(pObj); //	STOP状态时，可以进行观测器、SVPWM、电流环参数的重新配置
                                                //	可以根据需要添加动态参数代码并设置初始化请求
    if (t_bReturn)                              //	参数初始化完成
    {
        StateInit(pObj); //	变量初始化 (依赖于观测器的标幺转换函数等，必须首先初始化观测器等)

        /*****错误检测参数初始化********/
        if (pObj->m_SMACtrlBit.m_bFaultInitReq == 1) //	FAULT参数初始化请求，可动态修改参数进行初始化
        {
            FaultInit(pObj);
            FaultRecoverInit(pObj);
            pObj->m_SMACtrlBit.m_bFaultInitReq = 0;
        } else //	没有FAULT参数初始化请求
        {
            FaultInitTask(pObj); //  初始化FAULT变量
        }
        pObj->m_SMACtrlBit.m_bFaultInitDone = 1;
        pObj->m_SMACtrlBit.m_bPostOK = 1; //	POST标志	置1
    }

    handleIfMotorSpdZero(pObj); //	检查电机是否停止

    // update MA infor to MD
    SetDefaultMA2MD(pObj); //	发送默认操作命令

    // update MA SR infor to UA

    if (pObj->m_SMACtrlBit.m_bUAMOE) //	MOE是否使能
    {
        setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置MOE使能标志，通过共享数据交换区通知UA层
    }

    return t_bReturn;
}

/*****************************************************************************
 * 函数名   : void ChargeDriverMain(STR_MACtrProc *pObj)
 * 说明     : 功率板预充电函数
 *          : 具体实现StateCharge()
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
void ChargeDriverMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateCharge(pObj);                         //	预充电处理函数
    pObj->m_SMACtrlBit.m_bMotorStopped = TRUE; //	设置电机停止标志，电机是否真正停止取决于应用
                                               //	例如风机MOE关闭，要通过后续的顺逆风进行判断
                                               //	当前状态下，标志为TRU时电机可能是转动的

    // update MA infor to MD
    SetBrakePWMDutyMA2MD(pS_MA2MD, pObj->m_pAppCommData->nChargePwmWidth);

    // SetMoeBlankTimeMA2MD(pS_MA2MD,5);
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_RESET); //	设置MD层状态，通过共享数据交换区通知MD层

    // update MA SR infor to UA

    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorTrakingMain(STR_MACtrProc *pObj)
 * 说明     : 顺逆风检测函数
 *          : 具体实现StateDirCheck()
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
void MotorTrakingMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateDirCheck(pObj); //	顺逆风检测函数

    pObj->m_SMACtrlBit.m_bMotorStopped = FALSE; //	默认电机处于转动状态

#if (DRV0_BEMF_CHECK_EABLE == FUNCTION_ON) // 单电机，使能反电势电路顺逆风时编入代码
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_OFF);
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_BEMF_TRACKING);
#else //	观测器顺逆风
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);           //	传递控制命令到MD使能MOE
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_TRACKING); //	设置MD层状态为顺逆风检测状态
#endif

    // update MA SR infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorTracking2Run(STR_MACtrProc *pObj)
 * 说明     : 电机顺逆风检测后运行函数
 *          :
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/03/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
void MotorTracking2Run(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    pObj->m_pAppCommData->bOpenRunFlag = 1; //	运行状态标记为顺逆风

    handleSpdSetpoint(pObj);                    //	根据顺逆风检测速度初始化速度环速度设定值
    pObj->m_SMACtrlBit.m_bMotorStopped = FALSE; //	电机运行中

    // update MA infor to MD
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON); //	传递控制命令到MD使能MOE
    // SetCurRefMA2MD(pS_MA2MD,0,0);
    SetSpeedRefMA2MD(pS_MA2MD, pObj->m_pMotorSpeed->wSpeedRef); //	传递速度设定值到MD层

    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_SYN); //	设置MD层状态为运行状态 E_MOTOR_SYN

    // update MA SR infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE);
}

/*****************************************************************************
 * 函数名   : void MotorTracking2Brake(STR_MACtrProc *pObj)
 * 说明     : 电机顺逆风检测制动函数、禁止修改FOC参数
 *          :
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
void MotorTracking2Brake(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateBrake(pObj); //	顺逆风后 制动函数
    pObj->m_pAppCommData->bOpenRunFlag = 0;

    // update MA infor to MD
    SetBrakePWMDutyMA2MD(pS_MA2MD, getPWMBrakeDuty(pObj)); //	设定PWM宽度
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);                    //	传递控制命令到MD使能MOE
    setDefaultCmdMA2MD(pS_MA2MD);                          //	发送默认操作命令
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_BRAKE);             //	设置MD层状态为运行状态 E_MOTOR_BRAKE

    // update MA SR infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : bool MotorInitIPDMain(void)
 * 说明     : 电机初始位置检测初始化函数
 *          :
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
bool MotorInitIPDMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    bool t_return = FALSE;

    // update MA infor to MD
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_RESET);

    if (pObj->m_SMACtrlBit.m_bIPDInitReq == 0) //	IPD初始化请求标志
    {
        pObj->m_SMACtrlBit.m_bIPDInitDone = 0;
        if (getNonInitResultMD2MA(&pObj->mMdToMa)) //	没有其他操作命令在执行
        {
            setIPDInitFCmdMA2MD(pS_MA2MD);        //	设置IPD参数初始化操作命令，到MD层
            pObj->m_SMACtrlBit.m_bIPDInitReq = 1; //	IPD初始化请求标志 置1
        } else {
            setDefaultCmdMA2MD(pS_MA2MD); //	发送默认操作命令
        }
    } else //	存在IPD初始化请求
    {
        if (pObj->m_SMACtrlBit.m_bIPDInitDone == 0) //	IPD参数初始化完成? 0 ----未完成
        {
            if (!getIPDInitResultMD2MA(&pObj->mMdToMa)) //	MD层是否完成IPD参数初始化操作 ?
            {
                setIPDInitFCmdMA2MD(pS_MA2MD);         //	未完成，设置IPD参数初始化操作命令，到MD层
            } else {                                   //	已完成
                setDefaultCmdMA2MD(pS_MA2MD);          //	发送默认操作命令
                pObj->m_SMACtrlBit.m_bIPDInitDone = 1; //	IPD参数初始化完成标志	置1
            }
        } else {                                       //	IPD参数初始化已完成
            if (getNonInitResultMD2MA(&pObj->mMdToMa)) //  MD层无操作命令状态
            {
                pObj->m_SMACtrlBit.m_bIPDInitReq = 0; //	IPD初始化请求标志 清除
                t_return = TRUE;                      //	IPD初始化完成
            } else {                                  //  MD层存在其他操作命令状态时，设置为无操作状态
                setDefaultCmdMA2MD(pS_MA2MD);
            }
        }
    }

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层

    if (getIPDDoneResultMD2MA(&pObj->mMdToMa)) //	读取MD层初始化操作结果， TRUE操作完成
    {
        pObj->m_SMACtrlBit.m_bIPDFinish = 1; //	IPD初始化完成
    } else {
        pObj->m_SMACtrlBit.m_bIPDFinish = 0;
    }

    return t_return; //	返回操作结果
}

bool MotorInitIPDMainWait(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    bool t_return = FALSE;

    // update MA infor to MD
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_OFF);
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_RESET);

    if (pObj->m_SMACtrlBit.m_bIPDInitReq == 0) {
        pObj->m_SMACtrlBit.m_bIPDInitDone = 0;
        if (getNonInitResultMD2MA(&pObj->mMdToMa)) {
            setIPDInitFCmdMA2MD(pS_MA2MD);
            pObj->m_SMACtrlBit.m_bIPDInitReq = 1;
        } else {
            setDefaultCmdMA2MD(pS_MA2MD);
        }
    } else {
        if (pObj->m_SMACtrlBit.m_bIPDInitDone == 0) {
            if (!getIPDInitResultMD2MA(&pObj->mMdToMa)) {
                setIPDInitFCmdMA2MD(pS_MA2MD);
            } else {
                setDefaultCmdMA2MD(pS_MA2MD);
                pObj->m_SMACtrlBit.m_bIPDInitDone = 1;
            }
        } else {
            if (getNonInitResultMD2MA(&pObj->mMdToMa)) {
                pObj->m_SMACtrlBit.m_bIPDInitReq = 0;
                t_return = TRUE;
            } else {
                setDefaultCmdMA2MD(pS_MA2MD);
            }
        }
    }

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE);

    if (getIPDDoneResultMD2MA(&pObj->mMdToMa)) {
        pObj->m_SMACtrlBit.m_bIPDFinish = 1;
    } else {
        pObj->m_SMACtrlBit.m_bIPDFinish = 0;
    }

    return t_return;
}

/*****************************************************************************
 * 函数名   : bool MotorIPDMain(void)
 * 说明     : 电机初始位置检测函数
 *          :
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
void MotorIPDMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    // update MA infor to MD

    if (pObj->m_SMACtrlBit.m_bMotorDirtion == CW) //	设定为 CW方向
    {
        setIPDCWStartFCmdMA2MD(pS_MA2MD); //	CW角度处理，MD层实现
    } else {
        setIPDCCWStartFCmdMA2MD(pS_MA2MD); //	CCW角度处理，MD层实现
    }

    SetIPDEnableMA2MD(pS_MA2MD, 1); //	IPD检测使能标志

    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);      //	传递控制命令到MD使能MOE
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_IPD); //	设置MD层状态为运行状态 E_MOTOR_IPD

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层

    if (getIPDDoneResultMD2MA(&pObj->mMdToMa)) //	获取MD层IPD操作结果
    {
        pObj->m_SMACtrlBit.m_bIPDFinish = 1; //	IPD完成标志 置1 ----- 已完成
        setDefaultCmdMA2MD(pS_MA2MD);
    } else {
        pObj->m_SMACtrlBit.m_bIPDFinish = 0; //	IPD完成标志 清0 ----- 未完成
    }
}

/*****************************************************************************
 * 函数名   : void MotorAlignMain(void)
 * 说明     : 电机对相函数
 *          : 具体实现StateAlign()
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
void MotorAlignMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateAlign(pObj); //	对相处理函数

    pObj->m_SMACtrlBit.m_bMotorStopped = TRUE; //	电机停止状态

    // update MA infor to MD
    SetCurRefMA2MD(pS_MA2MD, pObj->m_nDCur_Reference, pObj->m_nQCur_Reference); //	设置dq轴电流指令值
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);                                         //	传递控制命令到MD使能MOE
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_ALIGN);                                  //	设置MD层状态为运行状态 E_MOTOR_ALIGN

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorCloseMain(void)
 * 说明     : 电机run状态处理程序，一般为速度环、功率环的处理
 *          : 处理过程见StateRun()函数实现
 *			:
 * 设计思路 ：1.根据实际负载计算速度/功率/电流的爬坡、速度环/功率环的PI
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/02/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
void MotorCloseMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateRun(pObj); // 状态处理函数

    handleIfMotorSpdZero(pObj); // 检查电机是否停止

#if (DRV0_UNDVOLT_PROT_ENABLE == 1)
                                //	电压异常保护是否启用
    s16 temp16;
    if (pObj->mg_pCtrlObj->m_pDrvCfgPara->mS_FBSlvcCfg0.bVoltageProtectionEnable == 1) {
        pObj->m_pMotorException->nBusVoltage = pObj->mMdToMa.nVdc;
        if (motorStopOrExceptionHandle(pObj->m_SMACtrlBit.m_bMC_RunFlg, pObj->m_pMotorException, &temp16)) {
            handleCurSetpoint(pObj, temp16); //	电压异常是设定速度指令
            if (pObj->m_eStopMode != 0)      //	非关闭MOE停止时
            {
                pObj->m_SMACtrlBit.m_bExceptionEnable = 1; //	异常标志 置1
            }
        } else {
            pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	无异常时，异常标志 清0
        }
    } else {
        pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	电压异常保护未使能时，异常标志 清0
    }
#endif

    // update MA infor to MD
    SetCurRefMA2MD(pS_MA2MD, pObj->m_nDCur_Reference, pObj->m_nQCur_Reference); //	设置dq轴电流指令值
    SetSpeedRefMA2MD(pS_MA2MD, pObj->m_pMotorSpeed->wSpeedRef);                 //	传递速度指令值到MD层
    SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);                                         //	传递控制命令到MD使能MOE
    SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_SYN);                                    //	设置MD层状态为运行状态 E_MOTOR_SYN

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorBreakMain(STR_MACtrProc *pObj)
 * 说明     : 电机制动控制函数
 *          : 具体实现StateBrake()
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/02/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
void MotorBreakMain(STR_MACtrProc *pObj)
{
#if (DRV0_STOP_MODE != 0)
    s16 temp16;
#endif
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

// update MA infor to MD
#if (DRV0_STOP_MODE == 1) // 零电流
    {
        StateRun(pObj);          //	处理函数
        handleSpdSetpoint(pObj); //	速度命令值处理

#if (DRV0_UNDVOLT_PROT_ENABLE == 1) //	低压异常保护
        {
            pObj->m_pMotorException->nBusVoltage = pObj->mMdToMa.nVdc;
            //	电压异常保护
            if (motorStopOrExceptionHandle(pObj->m_SMACtrlBit.m_bMC_RunFlg, pObj->m_pMotorException, &temp16)) {
                handleCurSetpoint(pObj, temp16);           //	速度命令值处理
                pObj->m_SMACtrlBit.m_bExceptionEnable = 1; //	异常标志 置1
            } else {
                handleCurSetpointClr(pObj);                //	速度命令值处理
                pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	无异常时，异常标志 清0
            }
        }
#else
        {
            handleCurSetpointClr(pObj);                //	速度命令值处理
            pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	无异常保护功能时，异常标志 清0
        }
#endif

        SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON);                                         //	传递控制命令到MD使能MOE
        SetCurRefMA2MD(pS_MA2MD, pObj->m_nDCur_Reference, pObj->m_nQCur_Reference); //	设置dq轴电流指令值
        SetSpeedRefMA2MD(pS_MA2MD, pObj->m_pMotorSpeed->wSpeedRef);                 //	传递速度指令值到MD层
        SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_SYN);                                    //	设置MD层状态为运行状态 E_MOTOR_SYN

        handleIfMotorSpdZero(pObj); //	检查电机是否停止
    }
#elif (DRV0_STOP_MODE == 2) // 短接
    {
        StateBrake(pObj);           //	短接制动处理函数
        handleCurSetpointClr(pObj); //	速度命令值处理

#if (DRV0_UNDVOLT_PROT_ENABLE == 1) //	电压保护是否启用
        pObj->m_pMotorException->nBusVoltage = pObj->mMdToMa.nVdc;
        if (motorStopOrExceptionHandle(pObj->m_SMACtrlBit.m_bMC_RunFlg, pObj->m_pMotorException, &temp16)) {
            pObj->m_SMACtrlBit.m_bExceptionEnable = 1; //	异常标志 置1
            SetPWMBrakeDuty(pObj, 2048);               //	设置PWM宽度
        } else {
            pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	异常标志 清0
            SetPWMBrakeDuty(pObj, 0);
        }
#else

        pObj->m_SMACtrlBit.m_bExceptionEnable = 0; //	电压保护不启用	异常标志 清0
        SetPWMBrakeDuty(pObj, 0);
#endif

        SetPWMCmdMA2MD(pS_MA2MD, E_PWM_ON); //	传递控制命令到MD使能MOE
        SetCurRefMA2MD(pS_MA2MD, 0, 0);
        SetSpeedRefMA2MD(pS_MA2MD, pObj->m_pMotorSpeed->wSpeedRef);
        SetBrakePWMDutyMA2MD(pS_MA2MD, getPWMBrakeDuty(pObj));
        SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_BRAKE); //	设置MD层状态为运行状态 E_MOTOR_BRAKE
    }
#else // 关闭MOE
    {
        SetTime_CountDown(pObj);                        //	计时处理
        if (SetTime_IsElapsed(pObj->m_nSetTimeLeftCnt)) //	停机时间是否到达
        {
            pObj->m_SMACtrlBit.m_bMotorStopped = TRUE; //	电机停止
        } else {
            pObj->m_SMACtrlBit.m_bMotorStopped = FALSE; //	电机运行中
        }
        SetPWMCmdMA2MD(pS_MA2MD, E_PWM_OFF); //	传递控制命令到MD关闭MOE
        SetCurRefMA2MD(pS_MA2MD, 0, 0);
        SetSpeedRefMA2MD(pS_MA2MD, 0);
        SetMotorCmdMA2MD(pS_MA2MD, E_MOTOR_RESET); //	设置MD层状态为运行状态 E_MOTOR_RESET
    }
#endif

    // update MA infor to UA
    setPWMStatMA2UA(pS_MA2UA, TRUE); //	设置PWM使能标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorFaultMain(STR_MACtrProc *pObj)
 * 说明     : 电机故障控制函数
 *          : 具体实现StateFault(pObj)
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
void MotorFaultMain(STR_MACtrProc *pObj)
{
    STR_PubDatMAMD *pS_MA2MD = GetPointerMA2MD(pObj);
    STR_PubDatMAUA *pS_MA2UA = GetPointerMA2UA(pObj);

    StateFault(pObj); //	FAULT 处理函数

    // update MA infor to MD
    SetFaultMA2MD(pObj); //	设置FAULT 命令操作到MD层

    // update MA infor to UA
    setFaultStatMA2UA(pS_MA2UA, TRUE);                                 //	设置FAULT标志为1，通过共享数据交换区通知UA层
    setPWMStatMA2UA(pS_MA2UA, (pObj->mMdToMa.ePwmStatus == E_PWM_ON)); //	根据MD层PWM状态设置MOE标志，通过共享数据交换区通知UA层
}

/*****************************************************************************
 * 函数名   : void MotorMainOffLineId(STR_MACtrProc *pObj)
 * 说明     : 离线识别函数
 *          : 后续实现
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
void MotorMainOffLineId(STR_MACtrProc *pObj)
{
    //	离线参数识别功能
}

/*****************************************************************************
 * 函数名   : void MotorMainOpen(STR_MACtrProc *pObj)
 * 说明     : 电机强拖程序
 * 设计思路 ：具体实现见StateOpen()函数
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 *****************************************************************************/
void MotorMainOpen(STR_MACtrProc *pObj)
{
    //	开环运行处理函数，当前统一为闭环函数处理
}

/*****************************************************************************
 * 函数名   : void MotorMainOnLineId(STR_MACtrProc *pObj)
 * 说明     : 在线识别函数
 *          : 后续实现
 * 设计思路 ：
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2022.08.17
 *****************************************************************************/
void MotorMainOnLineId(STR_MACtrProc *pObj)
{
    //	在线参数识别功能
}

/*****************************************************************************
 * 函数名   : void StateInit()
 * 说明     : 状态初始化
 * 设计思路 ：1.变量初始化
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/03/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
static void StateInit(STR_MACtrProc *pObj)
{
    s32 t_wTmp;
    s16 t_nTmp;
    u16 t_nuTmp;
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);

    PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;

    pObj->m_SMACtrlBit.m_bAlignFinish = 0; //	清除对相完成标志

    //	电流指令相关变量清0
    pObj->m_nQCurrentSet = 0;
    pObj->m_nQCur_Reference = 0;
    pObj->m_nDCurrentSet = 0;
    pObj->m_nDCur_Reference = 0;
    pObj->m_wDCurFilter = 0;
    pObj->m_wQCurFilter = 0;

    pObj->m_pAppCommData->wSvcMinFreq = pDrvCfgPara->mS_FBSlvcCfg0.m_wCloseFreq; // 开环拖动频率
    pObj->m_pAppCommData->bOpenRunFlag = 0;
    pObj->m_pAppCommData->nLoopCntr = 0; //	环路计数器清0

    //	顺逆风检测变量
    // 顺风切闭环速度初始化
    pObj->m_pAppCommData->wDirTrackFreq = pDrvCfgPara->mS_FBDirCheck.m_wSpdTrackOnFreq;

    // 刹车速度初始化
    pObj->m_pAppCommData->wDirEbreakFreq = pDrvCfgPara->mS_FBDirCheck.m_wBrakeFreqAntiDirec;

    //	电机停止检测相电流值初始化
    pObj->m_pOnTheFlyDetect->nMotorStopCurrentThd = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBDirCheck.m_wMotorStopCurrentThd); // MOTOR_STOP_CUR_THD_M0;    /*电机停止检测电流阈值 */
                                                                                                                                     //	电机停止检测相电流变化值初始化
    pObj->m_pOnTheFlyDetect->nMotorStopCurDifThd = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBDirCheck.m_wMotorStopCurDifThd);   // MOTOR_STOP_CUR_DIF_THD_M0; /*电机停止检测电流阈值 */
    pObj->m_pOnTheFlyDetect->wFreqAvg = 0;                                                                                           //	顺逆风检测频率值清0
    pObj->m_pOnTheFlyDetect->nMotorVolMagAvg = 0;                                                                                    //	顺逆风检测电压值清0

    pObj->m_SMACtrlBit.m_bMotorDirtion = CW; /* 电机转向 */

    // 预充电
    pObj->m_pAppCommData->bChargeFlag = 0;    // 预充电开始标志清零
    pObj->m_pAppCommData->bChargeEndFlag = 0; // 预充电完成标志清零

    //	DQ电流滤波系数计算
    pObj->m_pAppCommData->nDCurFilFactor = (s16)(10240 / (10 + (s16)pDrvCfgPara->mS_FBCurLoop.m_wDCurFilTimeSet));
    pObj->m_pAppCommData->nQCurFilFactor = (s16)(10240 / (10 + (s16)pDrvCfgPara->mS_FBCurLoop.m_wQCurFilTimeSet));

    /*****定位参数初始化********/
    t_wTmp = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBAllign.m_wAlignCurAcc); //	定位电流加速度 A/S
    pObj->m_pSTR_CurRampGen->wRampIncrease = ((s64)t_wTmp * 16777) >> 8;

    t_wTmp = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBAllign.m_wAlignCurDec); //	定位电流减速度 A/S
    pObj->m_pSTR_CurRampGen->wRampDecrease = ((s64)t_wTmp * 16777) >> 8;

    /*****运行参数初始化********/
    pObj->m_pMotorSpeed->wSpeedRef = 0;
    pObj->m_pMotorSpeed->wSpeedSet = 0;
    pObj->m_pMotorSpeed->wSpeedRampACCStep = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wSpdRunAcc) * 65536 / 1000; // 变换为1Hz/ms
    pObj->m_pMotorSpeed->wSpeedRampDECStep = App2CoreFreqTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wSpdRunDec) * 65536 / 1000; // 变换为1Hz/ms

    ramp32GenInit(&pObj->m_pMotorSpeed->STR_SpdRampGen); //	速度指令规划数据结构初始化

    ramp32GenInit(pObj->m_pSTR_CurRampGen); //	电流指令规划数据结构初始化

    pObj->m_pMotorSpeed->nSpeedFiltTime = (s16)(10240 / (10 + pDrvCfgPara->mS_FBSlvcCfg0.m_wSpeedFilTime));
    //	速度反馈滤波系数计算	Q10

    pObj->m_wPower001WFil = 0;                  //	功率滤波值 0.01W
    pObj->m_nPower001WFactor = 1024 / (1 + 10); //	功率滤波系数计算
/*****速度环参数初始化********/
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
    {
        s32 t_wTmpKp;
        s32 t_wTmpKi;
        pObj->m_pMotorSpeed->mSpeedPI.KP = pDrvCfgPara->mS_FBSpdLoop.PASRKp; // 速度环 Kp	P_ASR_KP_M0
        pObj->m_pMotorSpeed->mSpeedPI.KI = pDrvCfgPara->mS_FBSpdLoop.PASRKi; // 速度环 Ki P_ASR_KI_M0
        pObj->m_pMotorSpeed->mSpeedPI.wIntegral = 0;
        pObj->m_pMotorSpeed->mSpeedPI.wUpperLimitOutput = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wIqMax); // 速度环输出最大值 IQMAX_M0
        pObj->m_pMotorSpeed->mSpeedPI.wLowerLimitOutput = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBSpdLoop.m_wIqMin); // 速度环输出最小值 IQMAX_M0
        pObj->m_pMotorSpeed->mSpeedPI.wInError = 0;

#if 0
		t_nTmp = pDrvCfgPara->mS_FBSlvcCfg0.m_wSpeedFilTime + 25;		
		//自动调节速度环PI功能
		SpeedPIAutoTunning(pObj->mg_nMACtrlProcID,
						5000,
						t_nTmp,
						&t_wTmpKp,
						&t_wTmpKi);

		pObj->m_pMotorSpeed->mSpeedPI.KP = sat(t_wTmpKp,0,32767);		//	PI参数限幅
		pObj->m_pMotorSpeed->mSpeedPI.KI = sat(t_wTmpKi,0,32767);
#endif

        pObj->m_pMotorSpeed->wSpeedEst = 0; //	速度变量初始化
        pObj->m_pMotorSpeed->wSpeedfbk = 0;
        pObj->m_pMotorSpeed->wSpeedfbkFilter = 0;
        pObj->m_pMotorSpeed->wPowerLimitSpeedChangeSet = 0; //	速度环限功率变量清0
        pObj->m_pMotorSpeed->wPowerLimitSpeedChangeSetInc = 0;

        pObj->m_pMotorSpeed->nPowerLimitCntr = 0;
        pObj->m_pMotorSpeed->nPowerLimitPeriod = pDrvCfgPara->mS_FBSpdLoop.nPowerLmtCalcCycle;
        //	速度环限功率执行周期初始化

        pObj->m_pAppCommData->nLoopPeriod = pDrvCfgPara->mS_FBSpdLoop.nSpeedLoopCycle;
        //	速度环执行周期初始化
    }
    /*****功率环参数初始化********/
#elif (DRV0_CLOSE_LOOP == POWER_LOOP)
    {
        PowerLoopInit(pObj);                                                           //	功率环参数初始化
        pObj->m_pAppCommData->nPowerFistFlag = 0;                                      //	首次进入功率环标志	清0
        pObj->m_pAppCommData->nLoopPeriod = pDrvCfgPara->mS_FBPwrLoop.nPLoopCalcCycle; //	功率环计算周期
    }
#endif

    t_nTmp = (s16)App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wIdStartSet); //	D轴启动电流初始化
    t_wTmp = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wIqStartSet);      //	Q轴启动电流初始化
    t_nuTmp = pDrvCfgPara->mS_FBSlvcCfg0.TransTime2Close;                                 //	开闭环切换时间

#if (DRV0_WEAKEN_FLUX_STATUS == TRUE)
    InitDQCurRefGenWithWeakenFlux(pObj->mg_nMACtrlProcID, t_nTmp, (s16)t_wTmp, (s16)t_nuTmp); // DQ电流规划(带弱磁功能)初始化函数
#else
    InitDQCurRefGen(pObj->mg_nMACtrlProcID, t_nTmp, (s16)t_wTmp, (s16)t_nuTmp); // DQ电流规划初始化函数
#endif

    /*****对相环路计算时间参数初始化********/
    pObj->m_pTime->nChargeTime = 0;
    pObj->m_pTime->nChargePeriod = pDrvCfgPara->mS_GlobalCfg.m_nuChargeTime; //	预充电时间初始化

    pObj->m_pTime->nAlignTime = 0;
    pObj->m_pTime->nAlignLowPeriod = pDrvCfgPara->mS_FBAllign.nPlat1Time; //	对相时间初始化
    pObj->m_pTime->nAlignAllPeriod = pDrvCfgPara->mS_FBAllign.nPlat1Time + pDrvCfgPara->mS_FBAllign.nPlat2Time;

    pObj->m_pTime->nDirCheckTime = 0;
    pObj->m_pTime->nDirCheckPeriod = pDrvCfgPara->mS_FBDirCheck.nSpdTrackDelayTime; //	顺逆风检测时间初始化

    pObj->m_pTime->nStopDelayTime = 0;
    pObj->m_pTime->nStopDelayTimePeriod = pDrvCfgPara->mS_FBDirCheck.nMotorStopDelayTime; //	停机延时等待时间

    pObj->m_pTime->nStopDelayCntr = 0;
    pObj->m_pTime->nStopDelayPeriod = pDrvCfgPara->mS_FBDirCheck.nMotorStopTime; //	停机时间初始化

#if (DRV0_UNDVOLT_PROT_ENABLE == 1)

    //	电压保护	速度值回写指针初始化
    pObj->m_pMotorException->pwSpeedfbk = &(pObj->m_pMotorSpeed->wSpeedfbk);
    pObj->m_pMotorException->pwSpeedRef = &(pObj->m_pMotorSpeed->wSpeedRef);

    //	电压保护	电压设定值初始化
    pObj->m_pMotorException->brakeCurrent = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wIqBrake);
    pObj->m_pMotorException->chargeStartVoltage = App2CoreVoltTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wBusVoltStart);
    pObj->m_pMotorException->ChargeRestoreVoltage = App2CoreVoltTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wBusVoltRestore);
    pObj->m_pMotorException->chargeFinishVoltage = App2CoreVoltTrans(pAppToCore, pDrvCfgPara->mS_FBSlvcCfg0.m_wBusVoltEnd);
#endif

    MinMotorVoltSpd2Core(pObj->mg_nMACtrlProcID);
}

/*****************************************************************************
 * 函数名   : void StateCharge(void)
 * 说明     : 预充电函数，对自举电容进行预充电，对于不同的硬件要注意调整预充电时间
 * 设计思路 ：依次打开A相、B相、C相下桥，对自举电容充电。
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
  -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *****************************************************************************/
static void StateCharge(STR_MACtrProc *pObj)
{
    if (pObj->m_pTime->nChargePeriod == 0) {
        pObj->m_pAppCommData->bChargeEndFlag = 2; // 预充电完成
    }

    if (pObj->m_pAppCommData->bChargeFlag == 0) {
        pObj->m_pAppCommData->bChargeFlag = 1;
        pObj->m_pAppCommData->nChargePwmWidth = (s16)(4096 * 0.3); // 预充电占空比
        pObj->m_pTime->nChargeTime = 0;
        SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_OFF);
    } else {
        if (pObj->m_pTime->nChargeTime == 0) // 打开A相下桥PWM
        {
            setChargeAFCmdMA2MD(&pObj->mMaToMd);
            SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_ON);
        } else if (pObj->m_pTime->nChargeTime == pObj->m_pTime->nChargePeriod) // 打开B相下桥PWM //CHARGE_TIME_M0
        {
            setChargeBFCmdMA2MD(&pObj->mMaToMd);
            SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_ON);
        } else if (pObj->m_pTime->nChargeTime == (pObj->m_pTime->nChargePeriod * 2)) // 打开C相下桥PWM
        {
            setChargeCFCmdMA2MD(&pObj->mMaToMd);
            SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_ON);
        }

        if (pObj->m_pTime->nChargeTime < (pObj->m_pTime->nChargePeriod * 3)) // CHARGE_TIME为每段预充电时间，可根据硬件实际设置
        {
            pObj->m_pTime->nChargeTime++;
        } else {
            if (pObj->m_pAppCommData->bChargeEndFlag == 0) {
                setDefaultCmdMA2MD(&pObj->mMaToMd);
                SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_OFF);
                pObj->m_pAppCommData->nChargePwmWidth = 0;

                if (pObj->mMdToMa.nuFineStatus == E_FC_NON) {
                    setChargeDoneInitFCmdMA2MD(&pObj->mMaToMd);
                    pObj->m_pAppCommData->bChargeEndFlag = 1;
                }
            } else if (pObj->m_pAppCommData->bChargeEndFlag == 1) {
                if (pObj->mMdToMa.nuFineStatus == E_FC_NON) {
                    pObj->m_pAppCommData->bChargeEndFlag = 2;
                }
            }
        }
    }

    if (pObj->m_pAppCommData->bChargeEndFlag == 2) // 预充电完成
    {
        if (pObj->mMdToMa.nuFineStatus == E_FC_NON) {
            StateInit(pObj);
            setInitAcrParaReq(pObj);
            pObj->m_pAppCommData->bChargeEndFlag = 3;
        }
    }

    if (pObj->m_pAppCommData->bChargeEndFlag == 3) {
        if (motorAcrDataInitDone(pObj)) {
            pObj->m_pAppCommData->bChargeFlag = 0;
            pObj->m_pTime->nChargeTime = 0;

            pObj->m_pAppCommData->nChargePwmWidth = 0;
            SetPWMCmdMA2MD(&pObj->mMaToMd, E_PWM_ON);
            pObj->m_pAppCommData->bChargeEndFlag = 4;
        }
    }
}

/*****************************************************************************
 * 函数名   : void StateDirCheck(void)
 * 说明     : 顺逆风检测程序，常用的检测方式为闭环估算和通过反电势检测速度值
 * 设计思路 ：顺风高速则切入闭环运行;低速状态则进入stop状态，当电机完全停止后静止启动;\
              当逆风高速则一直检测（也可以根据实际应用进行刹车）。
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/03/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
static void StateDirCheck(STR_MACtrProc *pObj)
{
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);
    PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;

    if (++pObj->m_pTime->nDirCheckTime >= pObj->m_pTime->nDirCheckPeriod) {
        pObj->m_pTime->nDirCheckTime = 0;

        //	读取MD层的顺逆风检测结果 频率值
        pObj->m_pOnTheFlyDetect->wFreqAvg = Core2AppFreqTrans(&pObj->mg_pCtrlObj->mTransCoef.mCoreToApp, pObj->mMdToMa.wFreq);

        switch (pObj->m_SMACtrlBit.m_bMotorDirtion) {
        case CW: // 顺时针
        {
            if (pObj->m_pOnTheFlyDetect->wFreqAvg < -pObj->m_pAppCommData->wDirEbreakFreq) // 逆时针超高速，一直检测，附带减速效果; 对于不同的应用，也可以进入刹车状态。
            {
                pObj->m_eTrackingResult = TRACKING;
            } else if (pObj->m_pOnTheFlyDetect->wFreqAvg < pObj->m_pAppCommData->wDirTrackFreq) // 逆时针高速，刹车
            {
                pObj->m_pTime->nStopDelayTime = pObj->m_pTime->nStopDelayTimePeriod;
                pObj->m_eTrackingResult = TRACKING_2_BRAKE; // 刹车处理
            } else                                          // 顺时针 高速 切入闭环运行
            {
                pObj->m_nQCur_Reference = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBCurLoop.m_wIqSet); // IQ_SET_M0
                if (pObj->m_SMACtrlBit.m_bMotorDirtion == CCW) {
                    pObj->m_nQCur_Reference = -pObj->m_nQCur_Reference;
                }
                handleSpdSetpoint(pObj);
                pObj->m_eTrackingResult = TRACKING_2_RUN;
            }
            break;
        }
        case CCW: // 逆时针
        {
            if (pObj->m_pOnTheFlyDetect->wFreqAvg < -pObj->m_pAppCommData->wDirTrackFreq) // 逆时针高速，闭环运行
            {
                pObj->m_nQCur_Reference = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBCurLoop.m_wIqSet); // IQ_SET_M0
                if (pObj->m_SMACtrlBit.m_bMotorDirtion == CCW) {
                    pObj->m_nQCur_Reference = -pObj->m_nQCur_Reference;
                }
                handleSpdSetpoint(pObj);
                pObj->m_eTrackingResult = TRACKING_2_RUN;
            } else if (pObj->m_pOnTheFlyDetect->wFreqAvg < pObj->m_pAppCommData->wDirEbreakFreq) // 低速状态，刹车处理
            {
                pObj->m_pTime->nStopDelayTime = pObj->m_pTime->nStopDelayTimePeriod;
                pObj->m_eTrackingResult = TRACKING_2_BRAKE; // 刹车状态
            } else                                          // 顺时针 超高速，一直检测，附带减速效果;对于不同的应用，也可以进入刹车状态。
            {
                pObj->m_eTrackingResult = TRACKING;
            }
            break;
        }
        default:
            pObj->m_eTrackingResult = TRACKING;
            break;
        }
    } else {
        pObj->m_eTrackingResult = TRACKING;
    }
}

/*****************************************************************************
 * 函数名   : void StateAlign(void)
 * 说明     : 电机预定位。使能初始位置检测功能时，此函数只是提供开环强拖电流，\
              两段定位时间均给到1ms。
 * 设计思路 ：两段定位;给定电机电角度、定位电流、定位时间，使电机固定在给定角度上。\
              定位时间的设定，以电机完全静止的时间为准。
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
static void StateAlign(STR_MACtrProc *pObj)
{
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);
    PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;

    u32 t_TmpTheta;

    pObj->m_pMotorSpeed->wSpeedRef = 0;
    pObj->m_pMotorSpeed->STR_SpdRampGen.wRampOut = 0;
    if (pObj->m_pTime->nAlignTime < pObj->m_pTime->nAlignLowPeriod) { // 第一段定位处理
        pObj->m_pTime->nAlignTime++;
        pObj->m_pSTR_CurRampGen->wRampIn = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBAllign.m_wPlat1Cur);
    } else if (pObj->m_pTime->nAlignTime < pObj->m_pTime->nAlignAllPeriod) { // 第二段定位处理
        pObj->m_pTime->nAlignTime++;
        pObj->m_pSTR_CurRampGen->wRampIn = App2CoreCurTrans(pAppToCore, pDrvCfgPara->mS_FBAllign.m_wPlat2Cur);
    } else {                                      // 进入开环爬坡状态
        pObj->m_SMACtrlBit.m_bAlignFinish = TRUE; //	对相结束
        pObj->m_pTime->nAlignTime = 0;
    }

    ramp32GenCalc(pObj->m_pSTR_CurRampGen); //	电流指令斜坡规划函数

    pObj->m_nDCur_Reference = pObj->m_pSTR_CurRampGen->wRampOut; //	幅值到D轴电流给定值和参考值
    pObj->m_nDCurrentSet = pObj->m_nDCur_Reference;
    pObj->m_wDCurFilter = ((s32)pObj->m_nDCur_Reference) << 10;        //	去除电流指令滤波函数的影响
    SetDCurRefInitValue(pObj->mg_nMACtrlProcID, pObj->m_nDCurrentSet); //	设置D轴电流指令初始值

    if (!IsNeedIPD(pObj)) // 是否启用初始位置检测功能
    {
        //	没有IPD时，按照设定的对相角度作为电角度
        //	有IPD时，按照IPD的角度作为当前电角度
        t_TmpTheta = (u32)(pDrvCfgPara->mS_FBAllign.m_wuAlignAngle & 0xFFFF);
        setOBElectAngleEst(pObj->mg_nMACtrlProcID, t_TmpTheta << 16); //	设置电角度到观测器估算角度中
    }
}
/*****************************************************************************
 * 函数名   : void StateRun(void)
 * 说明     : 电机run状态处理程序，一般为速度环、功率环的处理
 * 设计思路 ：1.根据实际负载计算速度/功率/电流的爬坡、速度环/功率环的PI       \
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 2023/03/22      V1.1           Tonghua Li          修改
 *****************************************************************************/
static void StateRun(STR_MACtrProc *pObj)
{
    STR_TransCoefElement *pAppToCore = &(pObj->mg_pCtrlObj->mTransCoef.mAppToCore);
    PSTR_MechQuantity t_pMotorSpeed = pObj->m_pMotorSpeed;
    //	s32 t_wTemp;

/*******************速度环处理*******************************/
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
    {
        if (++pObj->m_pAppCommData->nLoopCntr >= pObj->m_pAppCommData->nLoopPeriod) // 速度环环路计算周期。
        {
            pObj->m_pAppCommData->nLoopCntr = 0;

            t_pMotorSpeed->wSpeedSet = App2CoreFreqTrans(pAppToCore, pObj->mUaToMa.wFreqCmd); // 速度参考给定，转换到core数据格式

#if (DRV0_POWER_LIMIT_STATUS == TRUE) // 限功率使能，速度环限制最大输出功率
            {
                pObj->m_pPower->wPowerValue = pObj->m_wPower001W;
                t_pMotorSpeed->wPowerLimitValue = pObj->mUaToMa.wPowerCmd;
                PowerLimitCalc(t_pMotorSpeed, pObj->m_pPower->wPowerValue); //	功率限制， 输出频率改变值wPowerLimitSpeedChangeSet
                                                                            //	调整频率设定值
                t_pMotorSpeed->wSpeedSet = t_pMotorSpeed->wSpeedSet - t_pMotorSpeed->wPowerLimitSpeedChangeSet;
            }
#endif

            HandleSpdSetpointCmd(pObj);  //	频率设定值处理，根据设定方向等处理接收到的频率设定值
            HandleSpdSetpointRamp(pObj); //	频率设定值斜坡规划，
            t_pMotorSpeed->wSpeedRef = t_pMotorSpeed->STR_SpdRampGen.wRampOut;

            // 启动时速度环输出变量初始化，即Iq_START电流设置
            QCurRefGenValue(pObj->mg_nMACtrlProcID, &t_pMotorSpeed->mSpeedPI.wIntegral, 22, t_pMotorSpeed->wSpeedSet);

            if (pObj->m_pAppCommData->bOpenRunFlag == 1) // 顺逆风检测到处于run状态
            {
                t_pMotorSpeed->mSpeedPI.wIntegral = 0;
                pObj->m_pAppCommData->bOpenRunFlag = 2; //	切换到正常运行状态
            }

            if (getRunStatus(pObj->mg_nMACtrlProcID)) //	检查闭环状态?
            {
                pObj->m_pAppCommData->bOpenRunFlag = 3; //	速度闭环模式
            }
            pObj->m_nQCurrentSet = SpeedLoopReg(t_pMotorSpeed); // 速度环PI计算
        }
    }

/*******************电流环处理*******************************/
#elif (DRV0_CLOSE_LOOP == CURRENT_LOOP)
    {
        PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;
        s32 t_wTemp1, t_wTemp;

        t_wTemp = ABS(pObj->mUaToMa.wTorqCmd); //	读取UA层的转矩指令

        if (pObj->m_SMACtrlBit.m_bMotorDirtion == CW) //	CW是定电流指令为正
        {
            pObj->m_nQCurrentSet = App2CoreCurTrans(pAppToCore, t_wTemp);
        } else if (pObj->m_SMACtrlBit.m_bMotorDirtion == CCW) //	CW是定电流指令为负
        {
            pObj->m_nQCurrentSet = -App2CoreCurTrans(pAppToCore, t_wTemp);
        }

        //	UA层设定的频率指令小于最低运行频率时，设置为最低运行频率
        t_wTemp = pDrvCfgPara->mS_FBSlvcCfg0.m_wMinRunFreq;
        t_wTemp1 = pObj->mUaToMa.wFreqCmd;
        if (ABS(t_wTemp1) <= t_wTemp) {
            t_wTemp1 = t_wTemp;
        }
        t_pMotorSpeed->wSpeedSet = App2CoreFreqTrans(pAppToCore, t_wTemp1); //	获取速度参考给定

        HandleSpdSetpointCmd(pObj); //	电流环下 频率指令处理
        if (ABS(t_pMotorSpeed->wSpeedfbk) >= ABS(t_pMotorSpeed->wSpeedSet)) {
            t_pMotorSpeed->wSpeedSet = t_pMotorSpeed->wSpeedfbk;
        }

        HandleSpdSetpointRamp(pObj); //	频率规划
        t_pMotorSpeed->wSpeedRef = t_pMotorSpeed->STR_SpdRampGen.wRampOut;
    }

/*******************功率环处理*******************************/
#elif (DRV0_CLOSE_LOOP == POWER_LOOP)
    {
        PSTR_DrvCfgPara pDrvCfgPara = pObj->mg_pCtrlObj->m_pDrvCfgPara;
        s32 t_wTemp1, t_wTemp;

        if (++pObj->m_pAppCommData->nLoopCntr >= pObj->m_pAppCommData->nLoopPeriod) // 功率环环路计算周期。
        {
            pObj->m_pAppCommData->nLoopCntr = 0;

            // 读取功率指令指
            pObj->m_pPower->wPowerSet = pObj->mUaToMa.wPowerCmd; // 0.01W
            pObj->m_pPower->wPowerValue = pObj->m_wPower001W;
#if (DRV0_SPPED_LIMIT_STATUS == TRUE) // 限速功能使能 ，功率环限制最大转速。
            // if(pObj->m_pAppCommData->bLoopLmtEnable)
            {
                // 读取速度限制值
                pObj->m_pPower->wSpeedLimitValue = App2CoreFreqTrans(pAppToCore, pObj->mUaToMa.wFreqCmd);
                SpeedLimitCalc(t_pMotorSpeed->wSpeedfbk, pObj->m_pPower); // 限速度,限制功率输出计算

                // 根据限速度函数的输出设置给定功率
                pObj->m_pPower->wPowerSet = pObj->m_pPower->wPowerSet - pObj->m_pPower->wSpeedLimitPowerChangeSet;
            }
#endif

            t_wTemp = ABS(pObj->mUaToMa.wTorqCmd);

            if (pObj->m_SMACtrlBit.m_bMotorDirtion == CW) // CW方向时 为正向
            {
                pObj->m_pPower->wPowerSet = pObj->m_pPower->wPowerSet;
                pObj->m_pPower->wPowerValue = pObj->m_pPower->wPowerValue;
                pObj->m_nQCurrentSet = App2CoreCurTrans(pAppToCore, t_wTemp);
            } else // CW方向时 为负向，数值取反
            {
                pObj->m_pPower->wPowerSet = -pObj->m_pPower->wPowerSet;
                pObj->m_pPower->wPowerValue = -pObj->m_pPower->wPowerValue;
                pObj->m_nQCurrentSet = -App2CoreCurTrans(pAppToCore, t_wTemp);
            }

            if (pObj->m_pAppCommData->nPowerFistFlag == 0) // 首次进入速度环，功率环的初始值设置为Iq
            {
                pObj->m_nQCur_Reference = pObj->m_nQCurrentSet;
                pObj->m_pAppCommData->nPowerFistFlag = 1;
                pObj->m_pPower->struPowerRamp.wRef = pObj->m_pPower->wPowerValue;
                pObj->m_pPower->struPowerPI.wIntegral = ((s32)pObj->m_nQCur_Reference << 16);
            }

            pObj->m_pPower->struPowerRamp.wSet = pObj->m_pPower->wPowerSet;
            pObj->m_pPower->wPowerRef = RampControl(&pObj->m_pPower->struPowerRamp); // 功率环爬坡函数

            PowerLoopReg(pObj->m_pPower);                        // 功率环PI计算
            pObj->m_nQCurrentSet = pObj->m_pPower->nQCurrentSet; //	设定功率环输出电流指令值
        }

        //	UA层设定的频率指令小于最低运行频率时，设置为最低运行频率
        t_wTemp = pDrvCfgPara->mS_FBSlvcCfg0.m_wMinRunFreq;
        t_wTemp1 = pObj->mUaToMa.wFreqCmd;

        if (ABS(t_wTemp1) <= ABS(t_wTemp)) {
            t_wTemp1 = t_wTemp;
        }

        t_pMotorSpeed->wSpeedSet = App2CoreFreqTrans(pAppToCore, t_wTemp1); // 速度参考给定

        if (pObj->m_SMACtrlBit.m_bMotorDirtion == CW) //	功率环下速度指令处理
        {
            if (t_pMotorSpeed->wSpeedSet < 0) {
                t_pMotorSpeed->wSpeedSet = -t_pMotorSpeed->wSpeedSet;
            }
        } else if (pObj->m_SMACtrlBit.m_bMotorDirtion == CCW) {
            if (t_pMotorSpeed->wSpeedSet > 0) {
                t_pMotorSpeed->wSpeedSet = -t_pMotorSpeed->wSpeedSet;
            }
        }

        HandleSpdSetpointRamp(pObj); //	速度规划函数及设定值
        t_pMotorSpeed->wSpeedRef = t_pMotorSpeed->STR_SpdRampGen.wRampOut;
    }
#endif

#if (DRV0_WEAKEN_FLUX_STATUS == TRUE)
    pObj->m_nDCurrentSet = getDCurRefGenWithEnWeakenFlux(pObj->mg_nMACtrlProcID); //	带弱磁时 d轴电流指令处理
#else
    pObj->m_nDCurrentSet = getDCurRefGen(pObj->mg_nMACtrlProcID); //	不带弱磁时 d轴电流指令处理
#endif

    CurrentLoopRef_filter(pObj); //	dq轴电流环给定值滤波
}

/*****************************************************************************
 * 函数名   : void StateFault(void)
 * 说明     : 故障状态，主要是进行故障状态的重启处理
 * 设计思路 ：1.在故障状态，如果检测到故障消失，则清零stru_Faults对应故障位。\
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *****************************************************************************/
static void StateFault(STR_MACtrProc *pObj)
{
    pObj->m_SMACtrlBit.m_bMC_RunFlg = 0; //	清除RUN运行标志
                                         //   FaultRecover(pObj);    //故障恢复处理函数
}

/*****************************************************************************
 函数名   : static void StateBrake(STR_MACtrProc *pObj)
 说明     : 电机停止函数，判断电机是否静止状态
 设计思路 ：1.通过mOnTheFlyDetect.bMotorStopDecFlag的状态来判断电机是否在静止状态，在判定电机
                静止后去进行初始位置检测或者预定位动作
 参数     ：无
 返回值   ：无
 修改日期	   版本号		   修改人			 修改内容
 -----------------------------------------------------------------------------
  2022/08/17	  V1.0			 Tonghua Li 		 创建
 *****************************************************************************/
static void StateBrake(STR_MACtrProc *pObj)
{
    u32 t_CurrentSqrt;
    STR_Time_t *pstruTime = pObj->m_pTime;

    t_CurrentSqrt = pObj->mMdToMa.nMotorCur;

    BrakeStateCheck(t_CurrentSqrt, pObj->m_pOnTheFlyDetect); //	制动检测函数

    if (pObj->m_pOnTheFlyDetect->bMotorStopDecFlag) // 电机停止标志位为1则判定电机为静止状态
    {
        if (pstruTime->nStopDelayCntr < pstruTime->nStopDelayPeriod) // 电机停止时间滤波，连续STOP_TIME次电机处于静止状态则判定电机为静止状态
        {
            pstruTime->nStopDelayCntr++;
            pObj->m_pOnTheFlyDetect->bMotorStopDecFlag = 0;
        } else {
            if (pstruTime->nStopDelayTime > 0) // 判定电机静止后的延时处理，消除判定误差，根据实际电机负载调整延迟时间
            {
                pstruTime->nStopDelayTime--;
            } else //	判定电机停止
            {
                pObj->m_pMotorSpeed->wSpeedRef = 0; //	速度值清0
                pObj->m_pMotorSpeed->wSpeedSet = 0;
                pObj->m_pOnTheFlyDetect->bMotorStopDecFlag = 0;
                pstruTime->nStopDelayCntr = 0;
                pObj->m_SMACtrlBit.m_bMotorStopped = TRUE; //	电机已停止
            }
        }
    } else {
        pstruTime->nStopDelayCntr = 0;
        pObj->m_SMACtrlBit.m_bMotorStopped = FALSE; //	电机未停止
    }
}

/*******************************************************************************
 函数名称：    void updateMAInfo2UALocal(STR_MACtrProc *pObj)
 功能描述：    update MA信息到UA层
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 2022/12/17      V1.0           Tonghua Li          修改
 *******************************************************************************/
void updateMAInfo2UALocal(STR_MACtrProc *pObj)
{
    STR_PubDatMAUA *pMa2Ua = &pObj->mMaToUa;
    STR_PubDatMDMA *pMd2Ma = &pObj->mMdToMa;

    STR_TransCoefElement *pCoreToApp = &(pObj->mg_pCtrlObj->mTransCoef.mCoreToApp);

    pMa2Ua->wVdcDec = Core2AppVolTrans(pCoreToApp, pMd2Ma->nVdc);      //	更新母线电压值，单位0.01V
    pMa2Ua->wVolDec = Core2AppVolTrans(pCoreToApp, pMd2Ma->nMotorVol); //	更新电机电压值，单位0.01V

    // pMa2Ua->nInvTemp		= pObj->m_nInvTemp;

    pMa2Ua->wCurDec = Core2AppCurTrans(pCoreToApp, pMd2Ma->nMotorCur); //	更新电机相电流有效值，单位0.001A

    pMa2Ua->wSpeedDec = Core2AppFreqTrans(pCoreToApp, pMd2Ma->wFreq); //	更新电机频率值，单位0.01Hz

    pMa2Ua->uFault = pObj->mg_UFault.R;           //	更新报警信息，具体参照位域定义
    pMa2Ua->eMotorStsAck = pObj->mUaToMa.eAppCmd; //	更新控制命令反馈到UA层

    setRunningStatMA2UA(pMa2Ua, (pObj->m_SMACtrlBit.m_bMotorStopped == 0)); //	更新电机是否运行到UA层
    setPostStatMA2UA(pMa2Ua, pObj->m_SMACtrlBit.m_bPostOK);                 //	更新MA层POST标志到UA层
    setCfgFinishMA2UA(pMa2Ua, TRUE);                                        //	更新参数配置完成标志到UA层，预留
}

/*******************************************************************************
 函数名称：    static void CurrentLoopRef_filter(STR_MACtrProc *pObj)

 功能描述：    电流参考滤波处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2023/7/25      V1.0           Li Tonghua          创建
 *******************************************************************************/
static void CurrentLoopRef_filter(STR_MACtrProc *pObj)
{
    s32 t_wTmp32;
    //	D轴电流指令 一阶滤波器
    t_wTmp32 = pObj->m_nDCurrentSet - (pObj->m_wDCurFilter >> 10);
    pObj->m_wDCurFilter += ((s32)t_wTmp32 * pObj->m_pAppCommData->nDCurFilFactor);
    pObj->m_nDCur_Reference = (pObj->m_wDCurFilter >> 10);
    //	Q轴电流指令 一阶滤波器
    t_wTmp32 = pObj->m_nQCurrentSet - (pObj->m_wQCurFilter >> 10);
    pObj->m_wQCurFilter += ((s32)t_wTmp32 * pObj->m_pAppCommData->nQCurFilFactor);
    pObj->m_nQCur_Reference = (pObj->m_wQCurFilter >> 10);
}

/*******************************************************************************
 函数名称：    static s16 SpeedLoopReg(MechanicalQuantity *this)
 功能描述：    速度环PI计算
 输入参数：    MechanicalQuantity *this  结构体指针
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/08/17      V1.0           Tonghua Li          创建
 *******************************************************************************/
#if (DRV0_CLOSE_LOOP == SPEED_LOOP)
static s16 SpeedLoopReg(STR_MechQuantity *this)
{
    this->mSpeedPI.wInError = this->wSpeedRef - this->wSpeedfbk;

    return (SpeedPIRegulator(&this->mSpeedPI)); //	速度PI参数调节
}
#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

/*****END OF FILE****/
