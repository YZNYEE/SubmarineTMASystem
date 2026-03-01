#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

#include "MLE/MLE.hpp"
#include "PF/PF.hpp"
#include "EKELUND/EKELUND.hpp"
#include "UKF/UKF.hpp"
#include "DataAnalysis/DataAnalysis.hpp"
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
            pf.predict(dt, 0.1, 0.01); 
            
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

void test_UKF() {
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Starting TMA UKF Test..." << std::endl;
    std::cout << "=======================================" << std::endl;

    // 1. Ground Truth
    TargetState true_target;
    true_target.x = 5000.0;
    true_target.y = 8000.0;
    true_target.vx = -10.0;
    true_target.vy = -5.0;

    std::cout << "True Target: Pos(" << true_target.x << ", " << true_target.y << ") Vel(" << true_target.vx << ", " << true_target.vy << ")" << std::endl;

    // 2. Initialize UKF
    UKF ukf;
    TargetState init_state;
    // Initial guess with some error
    init_state.x = 6000.0; 
    init_state.y = 7000.0;
    init_state.vx = 0.0;
    init_state.vy = 0.0;

    Eigen::Matrix4d init_P = Eigen::Matrix4d::Identity();
    init_P(0, 0) = 2000.0 * 2000.0; // Position variance
    init_P(1, 1) = 2000.0 * 2000.0;
    init_P(2, 2) = 50.0 * 50.0;     // Velocity variance
    init_P(3, 3) = 50.0 * 50.0;

    double std_a = 0.05; // Process noise (acceleration)
    double std_rad = 0.1 * M_PI / 180.0; // Measurement noise (0.5 deg)

    ukf.init(init_state, init_P, std_a, std_rad);

    std::cout << "UKF Initialized. Initial Guess: Pos(" << init_state.x << ", " << init_state.y << ")" << std::endl;

    // 3. Simulation Loop
    double obs_x = 0.0, obs_y = 0.0;
    double obs_vx = 10.0, obs_vy = 0.0;
    
    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, std_rad);

    int steps = 1200;
    double dt = 1.0;
    
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;

        // Observer Motion
        if (i == 600) {
            // Maneuver
            obs_vx = 0.0;
            obs_vy = 10.0;
            std::cout << "[Maneuver] Observer turns North at t=" << t << "s" << std::endl;
        }

        // True Target State
        double tx = true_target.x + true_target.vx * t;
        double ty = true_target.y + true_target.vy * t;
        double tvx = true_target.vx;
        double tvy = true_target.vy;

        // Observer Position update
        if (i > 0) {
            obs_x += obs_vx * dt;
            obs_y += obs_vy * dt;
        }
        
        // Generate Measurement
        double true_bearing = std::atan2(tx - obs_x, ty - obs_y);
        double meas_bearing = NormalizeAngle(true_bearing + noise(gen));

        ObsData obs;
        obs.timetamp = (int)t;
        obs.x = obs_x;
        obs.y = obs_y;
        obs.bearing = meas_bearing;
        obs.brgvalid = true;

        // UKF Update
        ukf.update(obs);

        // Print status every 100 steps
        if (i % 100 == 0 || i == steps) {
            TargetState est = ukf.getResult();
            double pos_err = std::sqrt(std::pow(est.x - tx, 2) + std::pow(est.y - ty, 2));
            double vel_err = std::sqrt(std::pow(est.vx - tvx, 2) + std::pow(est.vy - tvy, 2));
            
            std::cout << "t=" << std::setw(4) << t << "s | "
                      << "Est Pos: (" << std::fixed << std::setprecision(1) << est.x << ", " << est.y << ") "
                      << "Err: " << std::setw(7) << pos_err << "m | "
                      << "Est Vel: (" << est.vx << ", " << est.vy << ") "
                      << "Err: " << std::setw(6) << vel_err << "m/s" << std::endl;
        }
    }
}

void test_UKF_Optimized() {
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Starting TMA UKF Optimization Test..." << std::endl;
    std::cout << "Comparing Standard vs Low-Q (Trust Motion Model) Configuration" << std::endl;
    std::cout << "=======================================" << std::endl;

    TargetState true_target = {5000.0, 8000.0, -10.0, -5.0};
    
    // Generate Data
    std::vector<ObsData> all_obs;
    double obs_x = 0.0, obs_y = 0.0, obs_vx = 10.0, obs_vy = 0.0;
    std::default_random_engine gen;
    double std_rad = 0.1 * M_PI / 180.0;
    std::normal_distribution<double> noise(0.0, std_rad);
    
    for (int i = 0; i <= 1200; ++i) {
        double t = i;
        if (i == 600) { obs_vx = 0.0; obs_vy = 10.0; } // Maneuver
        if (i > 0) { obs_x += obs_vx; obs_y += obs_vy; }
        
        double tx = true_target.x + true_target.vx * t;
        double ty = true_target.y + true_target.vy * t;
        double b = NormalizeAngle(std::atan2(tx - obs_x, ty - obs_y) + noise(gen));
        
        all_obs.push_back({(int)t, obs_x, obs_y, 0.0, false, b, true});
    }

    // Define a helper to run UKF
    auto run_ukf = [&](double std_a, const std::string& name) {
        std::cout << "\n--- Running " << name << " (std_a=" << std_a << ") ---" << std::endl;
        UKF ukf;
        TargetState init_state = {6000.0, 7000.0, 0.0, 0.0}; // Same bad guess
        Eigen::Matrix4d init_P = Eigen::Matrix4d::Identity();
        init_P(0,0) = init_P(1,1) = 2000*2000;
        init_P(2,2) = init_P(3,3) = 50*50;
        
        ukf.init(init_state, init_P, std_a, std_rad);
        
        for (const auto& obs : all_obs) {
            ukf.update(obs);
        }
        
        TargetState est = ukf.getResult();
        double t = all_obs.back().timetamp;
        double tx = true_target.x + true_target.vx * t;
        double ty = true_target.y + true_target.vy * t;
        double pos_err = std::sqrt(std::pow(est.x - tx, 2) + std::pow(est.y - ty, 2));
        double vel_err = std::sqrt(std::pow(est.vx - true_target.vx, 2) + std::pow(est.vy - true_target.vy, 2));
        
        std::cout << "Final Result at t=" << t << "s:" << std::endl;
        std::cout << "  Pos Error: " << pos_err << " m" << std::endl;
        std::cout << "  Vel Error: " << vel_err << " m/s" << std::endl;
    };

    run_ukf(0.05, "Standard UKF");
     run_ukf(0.001, "Low-Q UKF (Optimized)");
 }

void test_DataAnalysis() {
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Starting Data Analysis Test..." << std::endl;
    std::cout << "=======================================" << std::endl;

    // 1. Setup Scenario
    TargetState true_target = {5000.0, 8000.0, -10.0, -5.0};
    std::vector<ObsData> trajectory;
    double obs_x = 0.0, obs_y = 0.0, obs_vx = 10.0, obs_vy = 0.0;
    
    // Random generator for noise
    std::default_random_engine gen;
    std::normal_distribution<double> dist(0.0, 1.0); // Mean 0, Std 1
    double bearing_std = 0.5 * M_PI / 180.0; // 0.5 deg
    std::normal_distribution<double> bearing_noise(0.0, bearing_std);

    // MLE Solver
    MLE mle_solver;

    // Simulate 1200s with maneuver at 600s
    for (int i = 0; i <= 1200; ++i) {
        if (i == 600) { obs_vx = 0.0; obs_vy = 10.0; }
        if (i > 0) { obs_x += obs_vx; obs_y += obs_vy; }
        
        // Calculate true bearing
        double tgt_x = true_target.x + true_target.vx * i;
        double tgt_y = true_target.y + true_target.vy * i;
        double true_bearing = std::atan2(tgt_x - obs_x, tgt_y - obs_y);
        
        ObsData obs;
        obs.timetamp = i;
        obs.x = obs_x;
        obs.y = obs_y;
        obs.bearing = NormalizeAngle(true_bearing + bearing_noise(gen));
        obs.brgvalid = true;
        
        trajectory.push_back(obs);
        mle_solver.addObservation(obs);
    }

    // Run MLE to get estimated state
    TargetState initial_guess = true_target;
    initial_guess.x += 1000.0; // Perturb initial guess
    initial_guess.y += 1000.0;
    initial_guess.vx = 0.0;
    initial_guess.vy = 0.0;
    
    TargetState mle_result;
    bool mle_success = mle_solver.estimate(initial_guess, mle_result);
    
    if (mle_success) {
        std::cout << "MLE Estimation Success:" << std::endl;
        std::cout << "  Est Pos: (" << mle_result.x << ", " << mle_result.y << ")" << std::endl;
        std::cout << "  Est Vel: (" << mle_result.vx << ", " << mle_result.vy << ")" << std::endl;
    } else {
        std::cout << "MLE Estimation Failed! Using True Target for CRLB." << std::endl;
        mle_result = true_target;
    }

    // 2. CRLB Calculation (Using MLE Result as Input)
    // double bearing_std = 0.5 * M_PI / 180.0; // Already defined above
    Eigen::MatrixXd crlb = DataAnalysis::CalculateCRLB(trajectory, mle_result, bearing_std);
    
    std::cout << "CRLB Matrix (Diagonal Sqrt) at MLE Estimate:" << std::endl;
    std::cout << "  Pos X std: " << std::sqrt(crlb(0,0)) << " m" << std::endl;
    std::cout << "  Pos Y std: " << std::sqrt(crlb(1,1)) << " m" << std::endl;
    std::cout << "  Vel X std: " << std::sqrt(crlb(2,2)) << " m/s" << std::endl;
    std::cout << "  Vel Y std: " << std::sqrt(crlb(3,3)) << " m/s" << std::endl;

    // 3. Error Ellipse
    DataAnalysis::EllipseParams ellipse = DataAnalysis::CalculateErrorEllipse(crlb.block<2,2>(0,0), 0.95);
    std::cout << "\nError Ellipse (95%):" << std::endl;
    std::cout << "  Long Axis: " << ellipse.long_axis << " m" << std::endl;
    std::cout << "  Short Axis: " << ellipse.short_axis << " m" << std::endl;
    std::cout << "  Angle: " << ellipse.angle_rad * 180.0 / M_PI << " deg" << std::endl;

    // 4. Residual Analysis (Simulated)
    std::vector<double> residuals;
    // std::default_random_engine gen; // Already defined
    // std::normal_distribution<double> dist(0.0, 1.0); // Already defined
    for(int i=0; i<100; ++i) residuals.push_back(dist(gen));
    
    DataAnalysis::ResidualStats res_stats = DataAnalysis::AnalyzeResiduals(residuals);
    std::cout << "\nResidual Analysis (Simulated Gaussian N(0,1)):" << std::endl;
    std::cout << "  Mean: " << res_stats.mean << std::endl;
    std::cout << "  Std Dev: " << res_stats.std_dev << std::endl;

    // 5. Monte Carlo Stats (Simulated)
    std::vector<TargetState> mc_results;
    for(int i=0; i<50; ++i) {
        TargetState t = true_target;
        t.x += dist(gen) * 10.0; // Add 10m noise
        t.y += dist(gen) * 10.0;
        t.vx += dist(gen) * 1.0;
        t.vy += dist(gen) * 1.0;
        mc_results.push_back(t);
    }
    
    DataAnalysis::MonteCarloStats mc_stats = DataAnalysis::AnalyzeMonteCarlo(mc_results, true_target);
    std::cout << "\nMonte Carlo Stats (Simulated):" << std::endl;
    std::cout << "  Pos RMSE: " << mc_stats.pos_rmse << " m" << std::endl;
    std::cout << "  CEP50: " << mc_stats.cep50 << " m" << std::endl;
}

 int main() {
    // 运行 Ekelund 测试
    // test_EKELUND();

    // 运行 UKF 测试
    // test_UKF();
    //test_UKF_Optimized();

    // 运行 MLE 测试
    // test_MLE(); 

    // 运行 PF 测试
    // test_PF();

    // 运行 Data Analysis 测试
    test_DataAnalysis();

    return 0;
}
