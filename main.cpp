#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

#include "MLE/MLE.hpp"
#include "COMMON/DataStruct.hpp"

// 辅助函数：角度归一化
double NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main() {
    std::cout << "Starting TMA MLE Test..." << std::endl;

    // 1. 设置真实目标状态 (Ground Truth)
    TargetState true_target;
    true_target.x = 1000.0;
    true_target.y = 5000.0;
    true_target.vx = 10.0;
    true_target.vy = -5.0;

    std::cout << "True Target State:" << std::endl;
    std::cout << "  Pos: (" << true_target.x << ", " << true_target.y << ")" << std::endl;
    std::cout << "  Vel: (" << true_target.vx << ", " << true_target.vy << ")" << std::endl;

    // 2. 模拟观测过程
    MLE mle_solver;
    
    // 观测者初始状态
    double obs_x = 0.0;
    double obs_y = 0.0;
    // 初始速度 (第一阶段: 向东运动)
    double obs_vx = 5.0; 
    double obs_vy = 0.0;

    int num_steps = 1200;     // 总观测次数增加到 120
    int maneuver_step = 600;  // 在第 60 步进行机动
    double dt = 1.0;         // 每次间隔 1 秒

    // 随机数生成器 (用于添加噪声)
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, 3.0/M_PI); // 标准差 0.005 弧度 (~0.3度)，稍微减小噪声以便观察收敛

    std::cout << "Generating observations..." << std::endl;
    std::cout << "  Observer Velocity (0-" << maneuver_step << "s): (" << obs_vx << ", " << obs_vy << ")" << std::endl;

    for (int i = 0; i < num_steps; ++i) {
        double current_time = i * dt;

        // 机动逻辑：在中间时刻改变速度
        if (i == maneuver_step) {
            obs_vx = 0.0;  // 改变速度方向，向北运动
            obs_vy = 5.0;
            std::cout << "  Observer Maneuver at t=" << current_time << "s! New Velocity: (" << obs_vx << ", " << obs_vy << ")" << std::endl;
        }

        // 更新观测者位置 (逐步积分)
        // 注意：这里我们在循环内更新位置，模拟实时运动
        obs_x += obs_vx * dt;
        obs_y += obs_vy * dt;

        // 当前目标位置
        double tgt_x_curr = true_target.x + true_target.vx * current_time;
        double tgt_y_curr = true_target.y + true_target.vy * current_time;

        // 计算真实方位角 (atan2(dx, dy) -> 正北为0，顺时针增加)
        // DataStruct.hpp 定义: bearing 单位弧度，正北为0，顺时针增加
        // 这意味着 X轴为东，Y轴为北。atan2(x, y) 返回的是 (x, y) 向量与 Y轴正方向的夹角。
        double true_bearing = std::atan2(tgt_x_curr - obs_x, tgt_y_curr - obs_y);

        // 添加噪声
        double meas_bearing = NormalizeAngle(true_bearing + noise(generator));

        // 构建观测数据
        ObsData obs;
        obs.timetamp = (int)current_time;
        obs.x = obs_x;      // 使用当前累积的观测者位置
        obs.y = obs_y;
        obs.bearing = meas_bearing;
        obs.brgvalid = true;
        
        // 其他字段设为无效或默认
        obs.range = 0.0;
        obs.rngvalid = false;
        obs.freq = 0.0;
        obs.freqvalid = false;

        mle_solver.addObservation(obs);
    }

    std::cout << "Generated " << num_steps << " observations with noise." << std::endl;

    // 3. 执行估计
    TargetState initial_guess;
    // 给一个有偏差的初始值
    initial_guess.x = true_target.x + 500.0; 
    initial_guess.y = true_target.y - 500.0;
    initial_guess.vx = 0.0; // 假设初始静止
    initial_guess.vy = 0.0;

    std::cout << "Initial Guess:" << std::endl;
    std::cout << "  Pos: (" << initial_guess.x << ", " << initial_guess.y << ")" << std::endl;
    std::cout << "  Vel: (" << initial_guess.vx << ", " << initial_guess.vy << ")" << std::endl;

    TargetState estimated_state;
    bool success = mle_solver.estimate(initial_guess, estimated_state);

    if (success) {
        std::cout << "\nEstimation Successful!" << std::endl;
        std::cout << "Estimated State:" << std::endl;
        std::cout << "  Pos: (" << estimated_state.x << ", " << estimated_state.y << ")" << std::endl;
        std::cout << "  Vel: (" << estimated_state.vx << ", " << estimated_state.vy << ")" << std::endl;

        // 计算误差
        double pos_err = std::sqrt(std::pow(estimated_state.x - true_target.x, 2) + std::pow(estimated_state.y - true_target.y, 2));
        double vel_err = std::sqrt(std::pow(estimated_state.vx - true_target.vx, 2) + std::pow(estimated_state.vy - true_target.vy, 2));
        
        std::cout << "Errors:" << std::endl;
        std::cout << "  Position Error: " << pos_err << " m" << std::endl;
        std::cout << "  Velocity Error: " << vel_err << " m/s" << std::endl;
    } else {
        std::cout << "\nEstimation Failed!" << std::endl;
    }

    return 0;
}