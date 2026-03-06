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
 * @file busy.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmBusy::~FsmBusy()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmBusy::getMode()
{
    if (enable && params->config.busy)
        return FsmMode::BUSY;
    else
        return FsmMode::NORMAL;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmBusy::run(Mat &img)
{
    if (!params->config.busy) // 该模式未启用
        return;

    enable = false; // 场景检测使能标志
    resultObs = PredictResult();
    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
        return;

    if (slowing)
    {
        timeout++;
        enable = true; // 场景检测使能标志
        if (timeout > 10)
            slowing = false;
    }

    // 施工区标志检测
    for (int i = 0; i < params->results.size(); i++)
    {
        if (params->results[i].type == LABEL_BUSY)
        {
            if (params->results[i].height < 100 && params->results[i].width < 80)
            {
                countRec++;
                timeout = 0;
                break;
            }
        }
    }
    if (countRec > 3)
    {
        slowing = true; // 减速使能
        timeout = 0;
    }
    if (countRec > 0)
    {
        countSes++;
        if (countSes > 6)
            countRec = 0;
    }

    // 锥桶 + 行人检测
    vector<PredictResult> resultsObs; // AI检测数据
    for (int i = 0; i < params->results.size(); i++)
    {
        if ((params->results[i].type == LABEL_CONE || params->results[i].type == LABEL_PERSON) &&
            (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.4 &&
            params->results[i].height < 100 && params->results[i].width < 90 &&
            params->results[i].height > 20 && params->results[i].width > 20) // AI标志距离计算
            resultsObs.push_back(params->results[i]);
    }
    if (resultsObs.size() <= 0)
        return;

    timeout = 5;

    // 选取距离最近的锥桶
    int areaMax = 0; // 框面积
    int index = 0;   // 目标序号
    for (int i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }
    resultObs = resultsObs[index];
    enable = true; // 场景检测使能标志

    // 障碍物方向判定（左/右）
    int row = 0, width = COLSIMAGE;
    for (size_t i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        int w = abs(resultObs.y - params->track->pointsEdgeLeft[i].x);
        if (w < 2)
        {
            row = i;
            break;
        }
        if (w < width)
        {
            width = w;
            row = i;
        }
    }
    if (row > params->track->pointsEdgeRight.size() - 1)
        row = params->track->pointsEdgeRight.size() - 1;

    // 路径重规划
    int disLeft = resultsObs[index].x - params->track->pointsEdgeLeft[row].y;
    int disRight = params->track->pointsEdgeRight[row].y - (resultsObs[index].x + resultsObs[index].width);
    if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
        params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
        abs(disLeft) <= abs(disRight)) //[1] 障碍物靠左
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
            curtailTracking(false);                 // 缩减优化车道线（双车道→单车道）
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeLeft[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x + resultsObs[index].width * 2};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x + resultsObs[index].width * 2};
            if (resultsObs[index].y > params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x)
                points[3] = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x + resultsObs[index].width};

            params->track->pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);          // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeLeft.push_back(repair[i]);
        }
    }
    else if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
             params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
             abs(disLeft) > abs(disRight)) //[2] 障碍物靠右
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
            curtailTracking(true);                  // 缩减优化车道线（双车道→单车道）
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeRight[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x - resultsObs[index].width * 2};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x - resultsObs[index].width * 2};
            if (resultsObs[index].y > params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x)
                points[3] = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x};

            params->track->pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);           // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeRight.push_back(repair[i]);
        }
    }

    // 车道线切除顶行1/5，避免弯道权重过大
    params->track->pointsEdgeLeft.resize(params->track->pointsEdgeLeft.size() * 0.7);
    params->track->pointsEdgeRight.resize(params->track->pointsEdgeRight.size() * 0.7);
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmBusy::show(Mat &img)
{
    if (!enable || params->mode != FsmMode::BUSY)
        return;

    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }

    putText(img, "[1] Busy", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 缩减优化车道线（双车道→单车道）
 *
 * @param left
 */
void FsmBusy::curtailTracking(bool left)
{
    if (left) // 向左侧缩进
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // 向右侧缩进
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}