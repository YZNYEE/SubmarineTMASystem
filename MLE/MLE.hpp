#pragma once

#include <vector>
#include "COMMON/DataStruct.hpp"

class MLE {
public:
    MLE();
    ~MLE();

    // 添加观测数据
    void addObservation(const ObsData& obs);

    // 清空观测数据
    void clearObservations();

    // 执行估计
    // initial_guess: 初始猜测值
    // estimated_state: 输出估计结果
    // return: 优化是否成功
    bool estimate(const TargetState& initial_guess, TargetState& estimated_state);

private:
    std::vector<ObsData> observations_;
};