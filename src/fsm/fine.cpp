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
 * @file fine.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 禁行区停车识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fine.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmFine::FsmFine(std::shared_ptr<Params> par)
    : FSMState(FsmMode::FINE, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmFine::~FsmFine()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmFine::getMode()
{
    // 输出场景状态结果
    if (!params->config.fine || step == Step::NONE)
        return FsmMode::NORMAL;

    return FsmMode::FINE;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmFine::run(Mat &img)
{
    if (!params->config.fine) // 该模式未启用
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
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP) // AI识别标志
            {
                if (params->results[i].height < 120 && params->results[i].width < 100 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.1) // 标志距离计算
                {
                    countRec++;
                    break;
                }
            }
        }
        if (countRec >= 2)
            setStep(Step::ENABLE);

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5)
            {
                countRec = 0;
                countSes = 0;
            }
        }
        break;
    }

    case Step::ENABLE: // 准备停车
    {
        timeout++; // 超时计数器
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STOP) // AI识别标志
            {
                if (params->results[i].height < 120 && params->results[i].width < 100 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.15)
                {
                    timeout = 0;
                    break;
                }
            }
            else if (params->results[i].type == LABEL_CROSS) // AI识别标志
            {
                if (params->results[i].height < 120 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.15)
                {
                    timeout = 0;
                    break;
                }
            }
        }
        if (timeout > 15)
            setStep(Step::STOP);

        break;
    }

    case Step::STOP: // 停车
    {
        params->ctrl.stop = true;
        timeout++; // 停车倒计时
        if (timeout > 60)
        {
            std::cout << "-----> [Stop] Game over, system exit!!! <-----" << std::endl;
            std::exit(0); // 程序退出
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
void FsmFine::show(Mat &img)
{
    if (params->mode != FsmMode::FINE)
        return;

    putText(img, "[3] Fine", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmFine::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
}