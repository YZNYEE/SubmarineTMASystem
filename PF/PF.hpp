#ifndef PF_HPP
#define PF_HPP

#include <vector>
#include <random>
#include <cmath>
#include "DataStruct.hpp"

// 粒子结构体
struct Particle {
    double x;
    double y;
    double vx;
    double vy;
    double weight;
};

class ParticleFilter {
public:
    // 构造函数
    // num_particles: 粒子数量
    ParticleFilter(int num_particles);
    ~ParticleFilter();

    // 初始化粒子群
    // obs: 初始观测数据
    // range_min, range_max: 初始距离范围假设 (米)
    // speed_min, speed_max: 初始速度范围假设 (米/秒)
    // course_min, course_max: 初始航向范围假设 (弧度)
    // bearing_std: 测向误差标准差 (弧度)
    void initialize(const ObsData& obs, 
                   double range_min, double range_max,
                   double speed_min, double speed_max,
                   double course_min, double course_max,
                   double bearing_std);

    // 状态预测 (Constant Velocity Model)
    // dt: 时间间隔 (秒)
    // process_noise_pos: 位置过程噪声标准差 (米)
    // process_noise_vel: 速度过程噪声标准差 (米/秒)
    void predict(double dt, double process_noise_pos, double process_noise_vel);

    // 测量更新 (Importance Weighting)
    // obs: 当前观测数据
    // bearing_std: 测向误差标准差 (弧度)
    void update(const ObsData& obs, double bearing_std);

    // 重采样 (Sequential Importance Resampling)
    void resample();

    // 获取加权平均估计状态
    TargetState getEstimate() const;

    // 获取所有粒子 (用于调试或可视化)
    const std::vector<Particle>& getParticles() const;

    // 检查是否已初始化
    bool isInitialized() const { return is_initialized_; }

private:
    int num_particles_;
    std::vector<Particle> particles_;
    std::mt19937 gen_; // 随机数生成器
    bool is_initialized_;

    // 辅助函数：角度归一化到 [-PI, PI]
    double normalizeAngle(double angle);

    // 速度约束范围
    double speed_min_{0.0};
    double speed_max_{100.0};
};

#endif // PF_HPP
