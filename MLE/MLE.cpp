#include "MLE.hpp"
#include <ceres/ceres.h>
#include <cmath>
#include <iostream>

// 用于角度归一化，将角度限制在 [-pi, pi]
template <typename T>
T NormalizeAngle(T angle) {
    while (angle > T(M_PI)) {
        angle -= T(2.0 * M_PI);
    }
    while (angle < T(-M_PI)) {
        angle += T(2.0 * M_PI);
    }
    return angle;
}

// 纯方位残差计算类
struct BearingResidual {
    BearingResidual(double bearing_obs, double obs_x, double obs_y, double dt)
        : bearing_obs_(bearing_obs), obs_x_(obs_x), obs_y_(obs_y), dt_(dt) {}

    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const vx, const T* const vy, T* residual) const {
        // 预测目标在当前时刻的位置
        T pred_x = x[0] + vx[0] * T(dt_);
        T pred_y = y[0] + vy[0] * T(dt_);

        // 计算预测方位角 (atan2(x, y) = atan(x/y), 注意坐标系定义: 正北为0，顺时针增加 -> atan2(x-x_obs, y-y_obs))
        // DataStruct.hpp 中定义: 正北为0，顺时针增加。即 X轴为东，Y轴为北。
        // atan2(dx, dy) 返回的是 (dx, dy) 向量与 Y轴正方向的夹角
        T pred_bearing = ceres::atan2(pred_x - T(obs_x_), pred_y - T(obs_y_));

        // 计算残差
        residual[0] = NormalizeAngle(pred_bearing - T(bearing_obs_));

        return true;
    }

    static ceres::CostFunction* Create(double bearing_obs, double obs_x, double obs_y, double dt) {
        return (new ceres::AutoDiffCostFunction<BearingResidual, 1, 1, 1, 1, 1>(
            new BearingResidual(bearing_obs, obs_x, obs_y, dt)));
    }

    double bearing_obs_;
    double obs_x_;
    double obs_y_;
    double dt_;
};

MLE::MLE() {}

MLE::~MLE() {}

void MLE::addObservation(const ObsData& obs) {
    observations_.push_back(obs);
}

void MLE::clearObservations() {
    observations_.clear();
}

bool MLE::estimate(const TargetState& initial_guess, TargetState& estimated_state) {
    if (observations_.empty()) {
        return false;
    }

    double x = initial_guess.x;
    double y = initial_guess.y;
    double vx = initial_guess.vx;
    double vy = initial_guess.vy;

    ceres::Problem problem;

    // 获取第一个观测的时间戳作为参考时间 t0
    int t0 = observations_.front().timetamp;

    for (const auto& obs : observations_) {
        if (!obs.brgvalid) {
            continue;
        }

        double dt = (double)(obs.timetamp - t0); // 假设时间单位与速度单位一致，例如秒和米/秒

        ceres::CostFunction* cost_function = BearingResidual::Create(
            obs.bearing, obs.x, obs.y, dt);
        
        problem.AddResidualBlock(cost_function, nullptr, &x, &y, &vx, &vy);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    if (summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::NO_CONVERGENCE) {
        // 即使未完全收敛，也返回当前最佳结果，但最好检查 final_cost
        estimated_state.x = x;
        estimated_state.y = y;
        estimated_state.vx = vx;
        estimated_state.vy = vy;
        return summary.termination_type == ceres::CONVERGENCE;
    }

    return false;
}