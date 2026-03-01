#include "DataAnalysis.hpp"
#include <numeric>
#include <algorithm>
#include <iostream>

namespace DataAnalysis {

    Eigen::MatrixXd CalculateCRLB(const std::vector<ObsData>& observer_trajectory, 
                                  const TargetState& target_state, 
                                  double bearing_std) 
    {
        // Fisher Information Matrix (FIM) J
        // J = sum( H_k^T * R_k^-1 * H_k )
        // State X = [x, y, vx, vy]^T
        // Measurement z_k = atan2(tx_k - ox_k, ty_k - oy_k) (North=0, CW)
        // Let's stick to standard math atan2(dy, dx) then convert or just differentiate the actual model used.
        // Model in project: bearing = atan2(x - ox, y - oy) where x is East, y is North.
        // Let dx = x - ox, dy = y - oy.
        // bearing = atan2(dx, dy)
        // partial/partial x = dy / (dx^2 + dy^2)
        // partial/partial y = -dx / (dx^2 + dy^2)
        // partial/partial vx = (partial/partial x) * t
        // partial/partial vy = (partial/partial y) * t
        
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, 4);
        double sigma_sq = bearing_std * bearing_std;
        double inv_R = 1.0 / sigma_sq;

        // Use the time of the first observation as reference time t0=0? 
        // Or keep absolute time? Usually relative to estimation time.
        // Let's assume the target state is defined at t_ref (e.g., first observation time or 0).
        // If target_state is initial state at t=0, then t in derivative is actual time.
        
        // Assuming target_state is at t=0.
        
        for (const auto& obs : observer_trajectory) {
            double t = (double)obs.timetamp;
            
            // Current target position
            double tx = target_state.x + target_state.vx * t;
            double ty = target_state.y + target_state.vy * t;
            
            double dx = tx - obs.x;
            double dy = ty - obs.y;
            double dist_sq = dx * dx + dy * dy;
            
            if (dist_sq < 1e-6) continue; // Avoid division by zero
            
            // Jacobsen H row vector for this measurement
            // z = atan2(dx, dy)
            // dz/dx = dy / (dx^2 + dy^2)
            // dz/dy = -dx / (dx^2 + dy^2)
            
            double h_x = dy / dist_sq;
            double h_y = -dx / dist_sq;
            double h_vx = h_x * t;
            double h_vy = h_y * t;
            
            Eigen::Vector4d H;
            H << h_x, h_y, h_vx, h_vy;
            
            J += H * inv_R * H.transpose();
        }
        
        // CRLB is inverse of FIM
        // Add small epsilon to diagonal to ensure invertibility if needed, but FIM should be invertible if observable.
        Eigen::MatrixXd CRLB = J.inverse();
        return CRLB;
    }

    EllipseParams CalculateErrorEllipse(const Eigen::Matrix2d& pos_covariance, double confidence_level) {
        // Eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(pos_covariance);
        if (eigensolver.info() != Eigen::Success) {
            return {0, 0, 0, 0};
        }

        Eigen::Vector2d eigenvalues = eigensolver.eigenvalues();
        // eigenvalues are sorted in increasing order
        double lambda1 = eigenvalues(1); // Largest
        double lambda2 = eigenvalues(0); // Smallest
        
        // Chi-square value for 2 degrees of freedom
        // P(x < K) = confidence_level
        // K = -2 * ln(1 - confidence_level)
        double k = -2.0 * std::log(1.0 - confidence_level);
        double scale = std::sqrt(k);

        EllipseParams params;
        params.long_axis = std::sqrt(lambda1) * scale;
        params.short_axis = std::sqrt(lambda2) * scale;
        
        // Calculate angle of the major axis
        // Eigenvectors matrix columns correspond to eigenvalues
        Eigen::Vector2d major_axis_vec = eigensolver.eigenvectors().col(1);
        params.angle_rad = std::atan2(major_axis_vec.y(), major_axis_vec.x());
        
        params.area = M_PI * params.long_axis * params.short_axis;
        
        return params;
    }

    ResidualStats AnalyzeResiduals(const std::vector<double>& residuals) {
        ResidualStats stats = {0, 0, 0, 0};
        if (residuals.empty()) return stats;

        double sum = 0.0;
        double sum_sq = 0.0;
        double max_abs = 0.0;

        for (double r : residuals) {
            sum += r;
            sum_sq += r * r;
            if (std::abs(r) > max_abs) max_abs = std::abs(r);
        }

        double n = (double)residuals.size();
        stats.mean = sum / n;
        stats.rms = std::sqrt(sum_sq / n);
        stats.max_abs = max_abs;
        
        double variance = (sum_sq - (sum * sum) / n) / (n > 1 ? n - 1 : 1);
        stats.std_dev = std::sqrt(variance);

        return stats;
    }

    MonteCarloStats AnalyzeMonteCarlo(const std::vector<TargetState>& results, const TargetState& ground_truth) {
        MonteCarloStats stats = {0, 0, 0, 0, 0, 0};
        if (results.empty()) return stats;

        double sum_sq_pos_err = 0.0;
        double sum_sq_vel_err = 0.0;
        double sum_err_x = 0.0;
        double sum_err_y = 0.0;
        std::vector<double> pos_errors;

        for (const auto& res : results) {
            double dx = res.x - ground_truth.x;
            double dy = res.y - ground_truth.y;
            double dvx = res.vx - ground_truth.vx;
            double dvy = res.vy - ground_truth.vy;

            sum_err_x += dx;
            sum_err_y += dy;
            
            double p_err_sq = dx*dx + dy*dy;
            sum_sq_pos_err += p_err_sq;
            sum_sq_vel_err += dvx*dvx + dvy*dvy;
            
            pos_errors.push_back(std::sqrt(p_err_sq));
        }

        double n = (double)results.size();
        stats.pos_rmse = std::sqrt(sum_sq_pos_err / n);
        stats.vel_rmse = std::sqrt(sum_sq_vel_err / n);
        stats.pos_bias_x = sum_err_x / n;
        stats.pos_bias_y = sum_err_y / n;

        // CEP Calculation (Sorting)
        std::sort(pos_errors.begin(), pos_errors.end());
        
        int idx50 = (int)(n * 0.50);
        if (idx50 < n) stats.cep50 = pos_errors[idx50];
        
        int idx95 = (int)(n * 0.95);
        if (idx95 < n) stats.cep95 = pos_errors[idx95];

        return stats;
    }

}
