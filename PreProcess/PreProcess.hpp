#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include "../COMMON/DataStruct.hpp"

class PreProcess {
public:
    /**
     * @brief 滑动窗口中值滤波剔除异常值 (针对方位角 bearing)
     * @param data 输入观测数据
     * @param window_size 窗口大小 (建议为奇数, e.g., 5, 7)
     * @param threshold 判定阈值 (弧度, 如果点与中值的差超过此值则剔除)
     * @return std::vector<ObsData> 过滤后的数据
     */
    static std::vector<ObsData> filterOutliersMedian(const std::vector<ObsData>& data, int window_size, double threshold);

    /**
     * @brief 滑动窗口 3-Sigma 剔除异常值 (针对方位角 bearing)
     * @param data 输入观测数据
     * @param window_size 窗口大小
     * @param sigma_multiplier Sigma 倍数 (通常为 3.0)
     * @return std::vector<ObsData> 过滤后的数据
     */
    static std::vector<ObsData> filterOutliers3Sigma(const std::vector<ObsData>& data, int window_size, double sigma_multiplier = 3.0);

private:
    // 辅助函数：角度归一化到 [-PI, PI]
    static double normalizeAngle(double angle);
    
    // 辅助函数：计算角度差的绝对值 (考虑周期性)
    static double angleDiffAbs(double a1, double a2);
};

#endif // PREPROCESS_HPP
