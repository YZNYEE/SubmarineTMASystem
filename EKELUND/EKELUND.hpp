#ifndef EKELUND_HPP
#define EKELUND_HPP

#include <vector>
#include "DataStruct.hpp"

class Ekelund {
public:
    Ekelund() = default;
    ~Ekelund() = default;

    /**
     * @brief 基于 Ekelund 测距算法估计目标位置
     * 
     * @param leg1 机动前的一段观测数据 (Leg 1)
     * @param leg2 机动后的一段观测数据 (Leg 2)
     * @param result 输出的目标状态估计结果 (主要包含 x, y)
     * @return true 估计成功
     * @return false 估计失败 (如无法计算方位角变化率、方位角变化率为0等)
     */
    static bool estimate(const std::vector<ObsData>& leg1, 
                         const std::vector<ObsData>& leg2,
                         TargetState& result);

private:
    // 计算在 eval_time 时刻的方位角和方位变化率
    static bool calculateBearingRate(const std::vector<ObsData>& data, 
                                     double eval_time, 
                                     double window,
                                     double& out_bearing, 
                                     double& out_rate);

    // 计算平均速度
    static void calculateAvgVelocity(const std::vector<ObsData>& data, 
                                     double& vx, double& vy);
    
    // 角度解缠绕
    static void unwrapBearings(std::vector<double>& bearings);
};

#endif // EKELUND_HPP
