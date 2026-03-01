#ifndef DATAANALYSIS_HPP
#define DATAANALYSIS_HPP

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "../COMMON/DataStruct.hpp"

namespace DataAnalysis {

    /**
     * @brief 误差椭圆参数
     */
    struct EllipseParams {
        double long_axis;   // 长轴长度 (1 sigma)
        double short_axis;  // 短轴长度 (1 sigma)
        double angle_rad;   // 长轴与X轴(正东)的夹角 (弧度)
        double area;        // 椭圆面积
    };

    /**
     * @brief 残差统计结果
     */
    struct ResidualStats {
        double mean;        // 均值
        double std_dev;     // 标准差
        double rms;         // 均方根
        double max_abs;     // 最大绝对误差
    };

    /**
     * @brief 蒙特卡洛仿真统计结果
     */
    struct MonteCarloStats {
        double pos_rmse;    // 位置均方根误差
        double vel_rmse;    // 速度均方根误差
        double pos_bias_x;  // X方向位置偏差均值
        double pos_bias_y;  // Y方向位置偏差均值
        double cep50;       // 50% 圆概率误差 (Circular Error Probable)
        double cep95;       // 95% 圆概率误差
    };

    /**
     * @brief 计算纯方位TMA的CRLB (Cramér-Rao Lower Bound)
     * 
     * @param observer_trajectory 观测者轨迹 (包含位置和时间戳)
     * @param target_state 目标真实状态 (用于线性化)
     * @param bearing_std 测向误差标准差 (弧度)
     * @return Eigen::MatrixXd 4x4 CRLB 矩阵 (协方差下界) [x, y, vx, vy]
     */
    Eigen::MatrixXd CalculateCRLB(const std::vector<ObsData>& observer_trajectory, 
                                  const TargetState& target_state, 
                                  double bearing_std);

    /**
     * @brief 根据协方差矩阵计算误差椭圆参数
     * 
     * @param covariance 2x2 位置协方差矩阵
     * @param confidence_level 置信度 (例如 0.95, 0.99)
     * @return EllipseParams 
     */
    EllipseParams CalculateErrorEllipse(const Eigen::Matrix2d& pos_covariance, double confidence_level = 0.95);

    /**
     * @brief 残差分析
     * 
     * @param residuals 残差序列
     * @return ResidualStats 
     */
    ResidualStats AnalyzeResiduals(const std::vector<double>& residuals);

    /**
     * @brief 蒙特卡洛仿真结果统计
     * 
     * @param results 多次仿真得到的目标状态估计列表
     * @param ground_truth 真实目标状态
     * @return MonteCarloStats 
     */
    MonteCarloStats AnalyzeMonteCarlo(const std::vector<TargetState>& results, const TargetState& ground_truth);

}

#endif // DATAANALYSIS_HPP
