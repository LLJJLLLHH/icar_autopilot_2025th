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
 * @file cross.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 斑马线停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/cross.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmCross::FsmCross(std::shared_ptr<Params> par)
    : FSMState(FsmMode::CROSS, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmCross::~FsmCross()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmCross::getMode()
{
    // 输出场景状态结果
    if (step == Step::NONE || !params->config.cross)
        return FsmMode::NORMAL;
    else
        return FsmMode::CROSS;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmCross::run(Mat &img)
{
    if (!params->config.cross) // 该模式未启用
        return;

    countInit++; // 起点屏蔽计数器
    if (countInit > 999)
        countInit = 999;
    else if (countInit < 60)
        return;

    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        countCross++; // 斑马线屏蔽计数器
        if (countCross > 999)
            countCross = 999;

        // 禁行标志 = 不停车
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP)
            {
                if (params->results[i].height < 120 && params->results[i].width < 90)
                {
                    setStep(Step::NONE);
                    return;
                }
            }
        }
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CROSS && countCross > 60) // 禁行标志：斑马线
            {
                countRec++;
                break;
            }
        }

        if (countRec >= 2)
            setStep(Step::ENABLE); // 设置新状态

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes > 4)
            {
                countRec = 0; // AI场景识别计数器
                countSes = 0; // 场次计数器
            }
        }
        break;
    }

    case Step::ENABLE: // 场景使能
    {
        timeout++;
        // 禁止通行标志 = 不停车
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP)
            {
                if (params->results[i].height < 120 && params->results[i].width < 90)
                {
                    countSes++; // 场次计数器
                    if (countSes > 2)
                    {
                        setStep(Step::NONE);
                        return;
                    }
                }
            }
        }
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CROSS) // 禁行标志：斑马线
            {
                if ((params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.2) // 停车距离计算
                {
                    countRec++;
                    timeout = 0;
                    break;
                }
            }
        }
        if (countRec >= 2)
            setStep(Step::STOP); // 设置新状态
        if (timeout > 30)
        {
            setStep(Step::NONE); // 设置新状态
        }
        break;
    }

    case Step::STOP: // 停车
    {
        params->ctrl.stop = true; // 停车标志

        // 禁止通行标志 = 不停车
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP)
            {
                if (params->results[i].height < 120 && params->results[i].width < 90)
                {
                    countSes++; // 场次计数器
                    if (countSes > 2)
                    {
                        setStep(Step::NONE);
                        return;
                    }
                }
            }
        }

        timeout++; // 场次计数器
        if (timeout >= 50)
        {
            setStep(Step::NONE); // 设置新状态
        }
        break;
    }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmCross::show(Mat &img)
{
    if (params->mode != FsmMode::CROSS)
        return;

    putText(img, "[8] Cross", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::ENABLE: // 场景使能
        putText(img, "[8] Cross - ENABLE", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;

    case Step::STOP: // 停车
        putText(img, "[8] Cross - STOPING", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmCross::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    params->ctrl.stop = false;
    countCross = 0;
}