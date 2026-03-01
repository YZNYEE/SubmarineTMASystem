#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

#include "MLE/MLE.hpp"
#include "PF/PF.hpp"
#include "EKELUND/EKELUND.hpp"
#include "COMMON/DataStruct.hpp"
#include "PreProcess/DataManager.hpp" // 引入 DataManager

// 辅助函数：角度归一化
double NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void test_PF()
{
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Starting TMA PF (Particle Filter) Test with DataManager..." << std::endl;
    std::cout << "=======================================" << std::endl;

    // 1. 设置真实目标状态 (Ground Truth)
    TargetState true_target;
    true_target.x = 5000.0;
    true_target.y = 8000.0;
    true_target.vx = -10.0;
    true_target.vy = -5.0;

    std::cout << "True Target State:" << std::endl;
    std::cout << "  Pos: (" << true_target.x << ", " << true_target.y << ")" << std::endl;
    std::cout << "  Vel: (" << true_target.vx << ", " << true_target.vy << ")" << std::endl;

    // 2. 初始化 PF 滤波器
    int num_particles = 20000;
    ParticleFilter pf(num_particles);
    
    // 3. 初始化数据管理器
    DataManager dm;
    // 启用异常值检测：窗口大小5，阈值0.1弧度(~5.7度)
    dm.enableOutlierFilter(true, 5, 0.1);
    std::cout << "DataManager initialized. Outlier filter enabled (window=5, threshold=0.1 rad)." << std::endl;

    // 4. 模拟观测过程
    double obs_x = 0.0;
    double obs_y = 0.0;
    double obs_vx = 10.0; // 观测者初始速度 (向东)
    double obs_vy = 0.0;

    int num_steps = 1200;     // 总观测时长
    int maneuver_step = 600; // 在第 600 秒进行机动
    double dt = 1.0;         // 采样间隔 1 秒
    
    // 测向误差标准差 (0.5度)
    double bearing_std = 0.5 * M_PI / 180.0;
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, bearing_std);

    std::cout << "Simulation Start..." << std::endl;

    for (int i = 0; i < num_steps; ++i) {
        double current_time = i * dt;

        // --- 4.1 观测者运动 (含机动) ---
        if (i == maneuver_step) {
            std::cout << "  [Maneuver] Observer turns North at t=" << current_time << "s" << std::endl;
            obs_vx = 0.0;
            obs_vy = 10.0; // 改为向北运动
        }
        obs_x += obs_vx * dt;
        obs_y += obs_vy * dt;

        // --- 4.2 目标真实位置 ---
        double tgt_x_curr = true_target.x + true_target.vx * current_time;
        double tgt_y_curr = true_target.y + true_target.vy * current_time;

        // --- 4.3 生成观测数据 ---
        double true_bearing = std::atan2(tgt_x_curr - obs_x, tgt_y_curr - obs_y);
        double meas_bearing = NormalizeAngle(true_bearing + noise(generator));

        ObsData raw_obs;
        raw_obs.timetamp = (int)current_time;
        raw_obs.x = obs_x;
        raw_obs.y = obs_y;
        raw_obs.bearing = meas_bearing;
        raw_obs.brgvalid = true;

        // --- 注入人工异常值 (每 100 步一次，模拟传感器故障) ---
        if (i > 50 && i % 100 == 50) {
            //raw_obs.bearing = NormalizeAngle(raw_obs.bearing + 0.5); // 增加约 28 度偏差
            //std::cout << "  [Inject Outlier] at t=" << current_time << "s, Bearing: " << raw_obs.bearing << std::endl;
        }

        // --- 4.4 数据管理与异常剔除 ---
        dm.addObservation(raw_obs);

        // 检查数据是否被接受
        const auto& all_obs = dm.getAllObservations();
        bool is_accepted = false;
        if (!all_obs.empty()) {
            // 比较时间戳确认是否是刚添加的数据
            if (all_obs.back().timetamp == raw_obs.timetamp) {
                is_accepted = true;
            }
        }

        if (!is_accepted) {
            std::cout << "  [Outlier Rejected] at t=" << current_time << "s" << std::endl;
            // 即使被剔除，如果有之前的状态，也可以进行纯预测
            if (pf.isInitialized()) {
                pf.predict(dt, 5.0, 1.0);
            }
            continue;
        }

        const ObsData& valid_obs = all_obs.back();

        // --- 4.5 粒子滤波处理 ---
        if (!pf.isInitialized()) {
            // 初始化
            pf.initialize(valid_obs, 
                          1000.0, 20000.0,  // 距离范围
                          0.0, 20.0,        // 速度范围
                          -M_PI, M_PI,      // 航向范围 (全向)
                          bearing_std);     // 测向误差
            std::cout << "  [PF] Initialized with " << num_particles << " particles." << std::endl;
        } else {
            // 预测
            // 降低过程噪声：
            // 位置噪声：1.0m (考虑到离散化误差和微小扰动)
            // 速度噪声：0.05m/s (目标假设为近似匀速，允许少量机动或扰动)
            // 原来的 1.0m/s 速度噪声对于 CV 模型来说太大，导致粒子发散
            pf.predict(dt, 1.0, 0.05); 
            
            // 更新
            pf.update(valid_obs, bearing_std);
            
            // 重采样
            pf.resample();
        }

        // --- 4.6 输出每隔一定步数的估计结果 ---
        if (i % 30 == 0 || i == num_steps - 1) {
            TargetState est = pf.getEstimate();
            double pos_err = std::sqrt(std::pow(est.x - tgt_x_curr, 2) + std::pow(est.y - tgt_y_curr, 2));
            double vel_err = std::sqrt(std::pow(est.vx - true_target.vx, 2) + std::pow(est.vy - true_target.vy, 2));
            
            std::cout << "  t=" << std::setw(3) << i << "s | "
                      << "Est Pos: (" << std::fixed << std::setprecision(1) << est.x << ", " << est.y << ") "
                      << "Err: " << std::setprecision(1) << pos_err << "m | "
                      << "Vel Err: " << std::setprecision(2) << vel_err << "m/s" << std::endl;
        }
    }

    TargetState final_est = pf.getEstimate();
    std::cout << "=======================================" << std::endl;
    std::cout << "Final Estimation:" << std::endl;
    std::cout << "  True Pos: (" << true_target.x + true_target.vx * num_steps * dt << ", " 
              << true_target.y + true_target.vy * num_steps * dt << ")" << std::endl;
    std::cout << "  Est  Pos: (" << final_est.x << ", " << final_est.y << ")" << std::endl;
    std::cout << "  True Vel: (" << true_target.vx << ", " << true_target.vy << ")" << std::endl;
    std::cout << "  Est  Vel: (" << final_est.vx << ", " << final_est.vy << ")" << std::endl;
    std::cout << "=======================================" << std::endl;
}

void test_MLE()
{
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

}

void test_EKELUND() {
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Starting TMA Ekelund Test..." << std::endl;
    std::cout << "=======================================" << std::endl;

    // Ground Truth
    TargetState true_target;
    true_target.x = 5000.0;
    true_target.y = 8000.0;
    true_target.vx = -10.0;
    true_target.vy = -5.0;
    
    // Observer
    double obs_x = 0.0, obs_y = 0.0;
    double obs_vx = 10.0, obs_vy = 0.0;

    std::vector<ObsData> leg1, leg2;
    double bearing_std = 0.1 * M_PI / 180.0; // 较小的噪声以验证算法 (0.1度)
    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, bearing_std);

    // Leg 1: 0-600s (East)
    for (int i = 0; i <= 600; ++i) {
        double t = i;
        double tx = true_target.x + true_target.vx * t;
        double ty = true_target.y + true_target.vy * t;
        double ox = obs_x + obs_vx * t;
        double oy = obs_y + obs_vy * t;
        
        double bearing = std::atan2(tx - ox, ty - oy);
        ObsData obs;
        obs.timetamp = (int)t;
        obs.x = ox; obs.y = oy;
        obs.bearing = NormalizeAngle(bearing + noise(gen));
        leg1.push_back(obs);
    }

    // Maneuver at 600s
    obs_x += obs_vx * 600;
    obs_y += obs_vy * 600;
    obs_vx = 0.0; obs_vy = 10.0; // Turn North

    // Leg 2: 600-1200s (North)
    for (int i = 601; i <= 1200; ++i) {
        double t = i;
        double tx = true_target.x + true_target.vx * t;
        double ty = true_target.y + true_target.vy * t;
        // Observer position relative to maneuver point
        double ox = obs_x + obs_vx * (t - 600);
        double oy = obs_y + obs_vy * (t - 600);
        
        double bearing = std::atan2(tx - ox, ty - oy);
        ObsData obs;
        obs.timetamp = (int)t;
        obs.x = ox; obs.y = oy;
        obs.bearing = NormalizeAngle(bearing + noise(gen));
        leg2.push_back(obs);
    }

    TargetState result;
    if (Ekelund::estimate(leg1, leg2, result)) {
        std::cout << "Ekelund Estimate Success!" << std::endl;
        // Evaluate at maneuver time t=600.5
        double t_m = (leg1.back().timetamp + leg2.front().timetamp) / 2.0;
        double true_tx = true_target.x + true_target.vx * t_m;
        double true_ty = true_target.y + true_target.vy * t_m;
        
        std::cout << "Maneuver Time: " << t_m << "s" << std::endl;
        std::cout << "True Pos: (" << true_tx << ", " << true_ty << ")" << std::endl;
        std::cout << "Est  Pos: (" << result.x << ", " << result.y << ")" << std::endl;
        
        double err = std::sqrt(std::pow(result.x - true_tx, 2) + std::pow(result.y - true_ty, 2));
        std::cout << "Position Error: " << err << " m" << std::endl;

        std::cout << "True Vel: (" << true_target.vx << ", " << true_target.vy << ")" << std::endl;
        std::cout << "Est  Vel: (" << result.vx << ", " << result.vy << ")" << std::endl;
    } else {
        std::cout << "Ekelund Estimate Failed!" << std::endl;
    }
}

int main() {
    // 运行 Ekelund 测试
    test_EKELUND();

    // 运行 MLE 测试
    // test_MLE(); 

    // 运行 PF 测试
    // test_PF();

    return 0;
}
