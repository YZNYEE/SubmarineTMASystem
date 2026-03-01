#include "PreProcess.hpp"
#include <numeric>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double PreProcess::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PreProcess::angleDiffAbs(double a1, double a2) {
    return std::abs(normalizeAngle(a1 - a2));
}

std::vector<ObsData> PreProcess::filterOutliersMedian(const std::vector<ObsData>& data, int window_size, double threshold) {
    if (data.empty()) return {};
    
    std::vector<ObsData> result;
    result.reserve(data.size());

    // 窗口大小必须为正，且最好是奇数
    if (window_size <= 0) window_size = 1;
    int half_window = window_size / 2;

    for (size_t i = 0; i < data.size(); ++i) {
        // 如果当前点没有有效的方位角，直接保留（或者根据策略丢弃，这里选择保留以便下游处理）
        if (!data[i].brgvalid) {
            result.push_back(data[i]);
            continue;
        }

        // 构建窗口
        std::vector<double> window_bearings;
        window_bearings.reserve(window_size);

        // 收集窗口内的有效方位角
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(data.size())) {
                if (data[idx].brgvalid) {
                    window_bearings.push_back(data[idx].bearing);
                }
            }
        }

        if (window_bearings.empty()) {
            result.push_back(data[i]);
            continue;
        }

        // 寻找中值
        // 为了正确处理角度周期性（如 -PI 和 PI），这里做一个简化处理：
        // 如果数据跨越了PI/2PI，简单的中值可能会错。但对于局部窗口，通常变化不大。
        // 一个健壮的做法是将所有角度减去第一个角度，归一化到 [-PI, PI]，计算中值后再加回去。
        double base_angle = window_bearings[0];
        for (auto& angle : window_bearings) {
            angle = normalizeAngle(angle - base_angle);
        }

        std::sort(window_bearings.begin(), window_bearings.end());
        double median = window_bearings[window_bearings.size() / 2];
        
        // 恢复真实角度
        double true_median = normalizeAngle(median + base_angle);
        
        // 判断当前点是否偏离中值过大
        double diff = angleDiffAbs(data[i].bearing, true_median);
        
        if (diff <= threshold) {
            result.push_back(data[i]);
        }
        // else: 剔除该点（不加入 result）
    }

    return result;
}

std::vector<ObsData> PreProcess::filterOutliers3Sigma(const std::vector<ObsData>& data, int window_size, double sigma_multiplier) {
    if (data.empty()) return {};

    std::vector<ObsData> result;
    result.reserve(data.size());

    if (window_size <= 0) window_size = 1;
    int half_window = window_size / 2;

    for (size_t i = 0; i < data.size(); ++i) {
        if (!data[i].brgvalid) {
            result.push_back(data[i]);
            continue;
        }

        // 计算窗口内的均值和标准差
        std::vector<double> window_bearings;
        window_bearings.reserve(window_size);

        for (int j = -half_window; j <= half_window; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(data.size())) {
                if (data[idx].brgvalid) {
                    window_bearings.push_back(data[idx].bearing);
                }
            }
        }

        if (window_bearings.size() < 3) { // 样本太少无法计算可靠统计量
            result.push_back(data[i]);
            continue;
        }

        // 角度去周期化处理 (同上)
        double base_angle = window_bearings[0];
        double sum = 0.0;
        std::vector<double> relative_angles;
        relative_angles.reserve(window_bearings.size());

        for (double angle : window_bearings) {
            double rel = normalizeAngle(angle - base_angle);
            relative_angles.push_back(rel);
            sum += rel;
        }

        double mean = sum / relative_angles.size();
        
        double sq_sum = 0.0;
        for (double rel : relative_angles) {
            sq_sum += (rel - mean) * (rel - mean);
        }
        double std_dev = std::sqrt(sq_sum / relative_angles.size());

        // 判断当前点是否偏离均值过大
        double current_rel = normalizeAngle(data[i].bearing - base_angle);
        if (std::abs(current_rel - mean) <= sigma_multiplier * std_dev) {
            result.push_back(data[i]);
        }
        // else: 剔除
    }

    return result;
}
