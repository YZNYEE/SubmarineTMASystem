#pragma once
#ifndef DATAMANAGER_HPP
#define DATAMANAGER_HPP

#include <vector>
#include <deque>
#include <algorithm>
#include "../COMMON/DataStruct.hpp"

class DataManager {
public:
    DataManager() = default;
    ~DataManager() = default;

    /**
     * @brief 设置最大容量
     * @param size 最大容量，0 表示无限制
     */
    void setMaxSize(size_t size);

    /**
     * @brief 添加单帧观测数据
     * @param obs 观测数据
     */
    void addObservation(const ObsData& obs);

    /**
     * @brief 批量添加观测数据
     * @param obs_list 观测数据列表
     */
    void addObservations(const std::vector<ObsData>& obs_list);

    /**
     * @brief 获取所有观测数据
     * @return const std::deque<ObsData>& 
     */
    const std::deque<ObsData>& getAllObservations() const;

    /**
     * @brief 获取数据量
     * @return size_t 
     */
    size_t size() const;

    /**
     * @brief 清空所有数据
     */
    void clear();

    /**
     * @brief 按时间戳对数据进行排序
     */
    void sortByTimestamp();

    /**
     * @brief 获取指定时间范围内的观测数据
     * @param start_time 开始时间
     * @param end_time 结束时间
     * @return std::vector<ObsData> 
     */
    std::vector<ObsData> getObservationsByTime(int start_time, int end_time) const;

    /**
     * @brief 启用/禁用 实时异常值检测 (因果中值滤波)
     * @param enable 是否启用
     * @param window_size 窗口大小 (默认为 5)
     * @param threshold 判定阈值 (弧度, 默认为 0.1, 约 5.7 度)
     */
    void enableOutlierFilter(bool enable, int window_size = 5, double threshold = 0.1);

private:
    std::deque<ObsData> observations_;
    size_t max_size_{0}; // 0 表示无限制

    // 异常值检测配置
    bool check_outlier_{false};
    int outlier_window_{5};
    double outlier_thresh_{0.1};

    // 辅助：检查单点是否异常
    bool isOutlier(const ObsData& new_obs) const;
};

#endif // DATAMANAGER_HPP
