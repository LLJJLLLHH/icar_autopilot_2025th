/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file curve.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 连续弯道识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/curve.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmCurve::FsmCurve(std::shared_ptr<Params> par)
    : FSMState(FsmMode::CURVE, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmCurve::~FsmCurve()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmCurve::getMode()
{
    // 输出场景状态结果
    if (!params->config.curve || step == Step::NONE)
        return FsmMode::NORMAL;

    return FsmMode::CURVE;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmCurve::run(Mat &img)
{
    if (!params->config.curve) // 该模式未启用
        return;

    if (step == Step::NONE)
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CURVE) // 连续弯标志检测
            {
                countRec++;
                break;
            }
        }
        if (countRec > 2) // 识别到连续弯道标志则进行连续弯处理
        {
            setStep(Step::ENABLE); // 设置新状态
            return;
        }
        if (countRec > 0)
        {
            countSes++;
            if (countSes > 4) // 标志位清零，进行下一次检测
            {
                countSes = 0;
                countRec = 0;
            }
        }
    }
    else if (step == Step::ENABLE)
    {
        timeout++;
        if (timeout > 150)
            setStep(Step::NONE); // 设置新状态

        int remainPointsSize = ROWSIMAGE * 0.75 - params->track->rowCutBottom; // ROWSIMAGE/2;
        if (params->track->pointsEdgeLeft.size() > remainPointsSize)
        {
            // 将左右边界视野缩至屏幕3/4处避免看的过远影响补线
            params->track->pointsEdgeLeft.resize(remainPointsSize);
            params->track->pointsEdgeRight.resize(remainPointsSize);
        }
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CROSS) // 斑马线标志检测
            {
                countRec++;
                break;
            }
        }
        if (countRec > 1) // 识别到连续弯道标志则进行连续弯处理
        {
            setStep(Step::NONE); // 设置新状态
            return;
        }
        if (countRec > 0)
        {
            countSes++;
            if (countSes > 4) // 标志位清零，进行下一次检测
            {
                countSes = 0;
                countRec = 0;
            }
        }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmCurve::show(Mat &img)
{
    if (params->mode != FsmMode::CURVE)
        return;

    putText(img, "[2] Curve", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmCurve::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
}