#include "DataManager.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 辅助函数：角度归一化
static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void DataManager::setMaxSize(size_t size) {
    max_size_ = size;
    // 如果当前数据量超过新设置的容量，立即进行修剪
    if (max_size_ > 0) {
        while (observations_.size() > max_size_) {
            observations_.pop_front();
        }
    }
}

void DataManager::enableOutlierFilter(bool enable, int window_size, double threshold) {
    check_outlier_ = enable;
    if (window_size > 0) outlier_window_ = window_size;
    if (threshold > 0.0) outlier_thresh_ = threshold;
}

bool DataManager::isOutlier(const ObsData& new_obs) const {
    if (observations_.size() < 2 || !new_obs.brgvalid) return false;

    // 获取最近的 (window_size - 1) 个有效点
    std::vector<double> window_bearings;
    window_bearings.reserve(outlier_window_);
    
    // 从后往前遍历
    int count = 0;
    for (auto it = observations_.rbegin(); it != observations_.rend(); ++it) {
        if (it->brgvalid) {
            window_bearings.push_back(it->bearing);
            count++;
            if (count >= outlier_window_ - 1) break;
        }
    }

    if (window_bearings.empty()) return false;

    // 加入新点
    window_bearings.push_back(new_obs.bearing);

    // 计算中值 (考虑角度周期性)
    // 简化处理：以第一个点为基准归一化
    double base_angle = window_bearings[0];
    for (auto& angle : window_bearings) {
        angle = normalizeAngle(angle - base_angle);
    }
    
    std::sort(window_bearings.begin(), window_bearings.end());
    double median = window_bearings[window_bearings.size() / 2];
    
    // 恢复真实角度的中值
    double true_median = normalizeAngle(median + base_angle);
    
    // 计算偏差
    double diff = std::abs(normalizeAngle(new_obs.bearing - true_median));
    
    return diff > outlier_thresh_;
}

void DataManager::addObservation(const ObsData& obs) {
    // 异常值检测
    if (check_outlier_) {
        if (isOutlier(obs)) {
            // 如果判定为异常值，直接丢弃（不添加）
            return;
        }
    }

    observations_.push_back(obs);
    
    // 检查容量并修剪
    if (max_size_ > 0 && observations_.size() > max_size_) {
        observations_.pop_front();
    }
}

void DataManager::addObservations(const std::vector<ObsData>& obs_list) {
    for (const auto& obs : obs_list) {
        addObservation(obs);
    }
}

const std::deque<ObsData>& DataManager::getAllObservations() const {
    return observations_;
}

size_t DataManager::size() const {
    return observations_.size();
}

void DataManager::clear() {
    observations_.clear();
}

void DataManager::sortByTimestamp() {
    std::sort(observations_.begin(), observations_.end(), 
        [](const ObsData& a, const ObsData& b) {
            return a.timetamp < b.timetamp;
        });
}

std::vector<ObsData> DataManager::getObservationsByTime(int start_time, int end_time) const {
    std::vector<ObsData> result;
    // 预估容量
    size_t estimate_count = 0;
    for (const auto& obs : observations_) {
        if (obs.timetamp >= start_time && obs.timetamp <= end_time) {
            estimate_count++;
        }
    }
    result.reserve(estimate_count);
    
    for (const auto& obs : observations_) {
        if (obs.timetamp >= start_time && obs.timetamp <= end_time) {
            result.push_back(obs);
        }
    }
    
    return result;
}
