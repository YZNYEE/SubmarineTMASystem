#include "PF.hpp"
#include <numeric>
#include <algorithm>
#include <iostream>

// 确保 M_PI 定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ParticleFilter::ParticleFilter(int num_particles)
    : num_particles_(num_particles), is_initialized_(false) {
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::initialize(const ObsData& obs, 
                               double range_min, double range_max,
                               double speed_min, double speed_max,
                               double course_min, double course_max,
                               double bearing_std) {
    // 保存速度约束
    speed_min_ = speed_min;
    speed_max_ = speed_max;

    particles_.resize(num_particles_);
    
    // 初始化分布
    std::uniform_real_distribution<> dist_range(range_min, range_max);
    std::uniform_real_distribution<> dist_speed(speed_min, speed_max);
    std::uniform_real_distribution<> dist_course(course_min, course_max);
    std::normal_distribution<> dist_bearing_noise(0.0, bearing_std);

    for (int i = 0; i < num_particles_; ++i) {
        // 在观测方位线上随机撒点初始化位置
        // 考虑观测噪声，方位角也加入随机扰动以覆盖不确定性
        double noisy_bearing = obs.bearing + dist_bearing_noise(gen_);
        double range = dist_range(gen_);
        
        // 坐标转换：极坐标 -> 直角坐标
        // 假设坐标系：Y轴正北，X轴正东，方位角为正北顺时针方向
        // x = x_obs + R * sin(theta)
        // y = y_obs + R * cos(theta)
        particles_[i].x = obs.x + range * std::sin(noisy_bearing);
        particles_[i].y = obs.y + range * std::cos(noisy_bearing);
        
        // 速度初始化
        double speed = dist_speed(gen_);
        double course = dist_course(gen_);
        
        particles_[i].vx = speed * std::sin(course);
        particles_[i].vy = speed * std::cos(course);
        
        // 初始化权重为均匀分布
        particles_[i].weight = 1.0 / num_particles_;
    }
    
    is_initialized_ = true;
}

void ParticleFilter::predict(double dt, double process_noise_pos, double process_noise_vel) {
    if (!is_initialized_) return;

    std::normal_distribution<> noise_pos(0.0, process_noise_pos);
    std::normal_distribution<> noise_vel(0.0, process_noise_vel);

    for (int i = 0; i < num_particles_; ++i) {
        // CV (Constant Velocity) 模型预测
        particles_[i].x += particles_[i].vx * dt + noise_pos(gen_);
        particles_[i].y += particles_[i].vy * dt + noise_pos(gen_);
        particles_[i].vx += noise_vel(gen_);
        particles_[i].vy += noise_vel(gen_);
    }
}

void ParticleFilter::update(const ObsData& obs, double bearing_std) {
    if (!is_initialized_ || !obs.brgvalid) return;

    double sum_weights = 0.0;
    
    for (int i = 0; i < num_particles_; ++i) {
        // 检查速度约束
        double speed = std::sqrt(particles_[i].vx * particles_[i].vx + particles_[i].vy * particles_[i].vy);
        if (speed < speed_min_ || speed > speed_max_) {
            particles_[i].weight = 0.0;
            continue;
        }

        // 计算预测方位角
        double dx = particles_[i].x - obs.x;
        double dy = particles_[i].y - obs.y;
        
        // 计算预测方位角
        // 对应正北为Y轴，顺时针增加：theta = atan2(x, y)
        // 注意：标准 atan2(y, x) 是逆时针从X轴算起，atan2(x, y) 刚好对应正北顺时针
        double pred_bearing = std::atan2(dx, dy);
        
        // 计算角度差并归一化到 [-PI, PI]
        double bearing_diff = normalizeAngle(obs.bearing - pred_bearing);
        
        // 计算高斯似然 (Gaussian Likelihood)
        // P(z|x) = (1 / sqrt(2*pi*sigma^2)) * exp(-(z - h(x))^2 / (2*sigma^2))
        // 这里省略常数项，因为最后会归一化
        double exponent = -(bearing_diff * bearing_diff) / (2.0 * bearing_std * bearing_std);
        double likelihood = std::exp(exponent);
        
        // 更新权重
        particles_[i].weight *= likelihood;
        sum_weights += particles_[i].weight;
    }

    // 权重归一化
    if (sum_weights > 1e-10) {
        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].weight /= sum_weights;
        }
    } else {
        // 如果所有权重都极小（粒子退化严重），重置为均匀分布
        // 这种情况通常意味着滤波器发散或测量异常
        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].weight = 1.0 / num_particles_;
        }
    }
}

void ParticleFilter::resample() {
    if (!is_initialized_) return;

    // 计算有效粒子数 N_eff (Effective Sample Size)
    double sum_sq_weights = 0.0;
    for (const auto& p : particles_) {
        sum_sq_weights += p.weight * p.weight;
    }
    double n_eff = 1.0 / (sum_sq_weights + 1e-10);

    // 设定重采样阈值，通常为粒子数的一半或 2/3
    if (n_eff < num_particles_ / 2.0) {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);
        
        // 低方差重采样 (Low Variance Resampling) / 系统重采样
        std::uniform_real_distribution<> dist(0.0, 1.0 / num_particles_);
        double r = dist(gen_);
        double c = particles_[0].weight;
        int i = 0;
        
        for (int m = 0; m < num_particles_; ++m) {
            double u = r + m * (1.0 / num_particles_);
            while (u > c && i < num_particles_ - 1) {
                i++;
                c += particles_[i].weight;
            }
            // 复制粒子，重置权重
            Particle p = particles_[i];
            p.weight = 1.0 / num_particles_;
            new_particles.push_back(p);
        }
        
        particles_ = new_particles;
    }
}

TargetState ParticleFilter::getEstimate() const {
    TargetState state = {0.0, 0.0, 0.0, 0.0};
    if (!is_initialized_) return state;

    for (const auto& p : particles_) {
        state.x += p.x * p.weight;
        state.y += p.y * p.weight;
        state.vx += p.vx * p.weight;
        state.vy += p.vy * p.weight;
    }
    return state;
}

const std::vector<Particle>& ParticleFilter::getParticles() const {
    return particles_;
}

double ParticleFilter::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
