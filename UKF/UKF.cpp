#include "UKF.hpp"
#include <iostream>
#include <cmath>

UKF::UKF() {
    is_initialized_ = false;
    n_aug_ = n_x; // Additive noise, so augmented dimension is same as state
    lambda_ = 3 - n_x;
    
    // Initialize weights
    weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    x_ = Eigen::VectorXd(n_x);
    P_ = Eigen::MatrixXd(n_x, n_x);
    Xsig_pred_ = Eigen::MatrixXd(n_x, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

void UKF::init(const TargetState& init_state, const Eigen::Matrix4d& init_P, double std_a, double std_rad) {
    x_ << init_state.x, init_state.y, init_state.vx, init_state.vy;
    P_ = init_P;
    std_a_ = std_a;
    std_rad_ = std_rad;
    
    // Initial time is not set here, will be set on first update or managed externally
    // Assuming the first update call will set the time or delta time
    time_ = 0.0; 

    // Initialize measurement noise covariance R
    R_ = Eigen::MatrixXd(n_z, n_z);
    R_ << std_rad_ * std_rad_;

    is_initialized_ = true;
}

double UKF::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void UKF::GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out) {
    // Create sigma points
    Xsig_out->col(0) = x_;

    // Calculate square root of P
    Eigen::MatrixXd A = P_.llt().matrixL();

    for (int i = 0; i < n_x; ++i) {
        Xsig_out->col(i + 1) = x_ + std::sqrt(lambda_ + n_x) * A.col(i);
        Xsig_out->col(i + 1 + n_x) = x_ - std::sqrt(lambda_ + n_x) * A.col(i);
    }
}

void UKF::PredictSigmaPoints(double dt) {
    // Generate sigma points from current state
    Eigen::MatrixXd Xsig = Eigen::MatrixXd(n_x, 2 * n_aug_ + 1);
    GenerateSigmaPoints(&Xsig);

    // Predict sigma points (CV Model)
    // x' = x + vx * dt
    // y' = y + vy * dt
    // vx' = vx
    // vy' = vy
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        double p_x = Xsig(0, i);
        double p_y = Xsig(1, i);
        double v_x = Xsig(2, i);
        double v_y = Xsig(3, i);

        // State transition
        double px_p = p_x + v_x * dt;
        double py_p = p_y + v_y * dt;
        double vx_p = v_x;
        double vy_p = v_y;

        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = vx_p;
        Xsig_pred_(3, i) = vy_p;
    }
}

void UKF::PredictMeanAndCovariance() {
    // Predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted state covariance
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
    
    // Add process noise Q
    // Assume Q is computed for the specific dt
    // But here we need to know dt. 
    // Usually Predict is called with dt.
    // Let's assume Q is added here.
    // For now, let's just add the Q matrix which should be computed based on dt.
    // But since PredictSigmaPoints handled the deterministic part, we add Q here.
    P_ = P_ + Q_;
}

void UKF::update(const ObsData& obs) {
    if (!is_initialized_) {
        // Initialization should be done via init()
        std::cerr << "UKF not initialized!" << std::endl;
        return;
    }

    double dt = 0.0;
    if (time_ == 0.0) {
        time_ = obs.timetamp;
        // Cannot predict on first step effectively if we don't have prev time
        // Assume initialized state is at time_
        return; 
    } else {
        dt = obs.timetamp - time_;
        time_ = obs.timetamp;
    }

    // 1. Prediction Step
    
    // Update Q based on dt
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt; // Not needed for Q

    Q_ = Eigen::MatrixXd(n_x, n_x);
    // CV model process noise
    // Q = G * Q_nu * G^T
    // G = [dt^2/2, 0; 0, dt^2/2; dt, 0; 0, dt]
    // Q_nu = [std_a^2, 0; 0, std_a^2]
    // This results in:
    // [ dt^4/4, 0, dt^3/2, 0 ]
    // [ 0, dt^4/4, 0, dt^3/2 ]
    // [ dt^3/2, 0, dt^2, 0   ]
    // [ 0, dt^3/2, 0, dt^2   ]
    // Multiplied by std_a^2
    
    // Standard CV model Q often uses:
    // [ dt^3/3, 0, dt^2/2, 0 ]
    // [ 0, dt^3/3, 0, dt^2/2 ]
    // [ dt^2/2, 0, dt, 0     ]
    // [ 0, dt^2/2, 0, dt     ]
    
    double q1 = dt_3 / 3.0;
    double q2 = dt_2 / 2.0;
    double q3 = dt;

    Q_ << q1, 0, q2, 0,
          0, q1, 0, q2,
          q2, 0, q3, 0,
          0, q2, 0, q3;
    
    Q_ *= (std_a_ * std_a_);

    PredictSigmaPoints(dt);
    PredictMeanAndCovariance();

    // 2. Update Step
    UpdateState(obs);
}

void UKF::UpdateState(const ObsData& obs) {
    // Transform sigma points into measurement space
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);

        // Measurement model: bearing = atan2(x - obs_x, y - obs_y)
        // North = 0, CW.
        // dx = target.x - obs.x (East)
        // dy = target.y - obs.y (North)
        // bearing = atan2(dx, dy)
        double dx = p_x - obs.x;
        double dy = p_y - obs.y;
        
        Zsig(0, i) = std::atan2(dx, dy);
    }

    // Mean predicted measurement
    Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
    z_pred.fill(0.0);
    
    // We need to handle angle wrapping when calculating the mean
    // A simple way is to use the first sigma point as reference or use vector sum
    // For sigma points, typically we just sum them, but for angles it's tricky.
    // Let's use the standard weighted sum but wrap residuals.
    // Actually, simple weighted sum fails for angles (e.g. avg of 179 and -179 is 0, should be 180).
    
    // Proper way: 
    // 1. Calculate mean using vector sum (sin/cos)
    double mean_sin = 0.0;
    double mean_cos = 0.0;
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        mean_sin += weights_(i) * std::sin(Zsig(0, i));
        mean_cos += weights_(i) * std::cos(Zsig(0, i));
    }
    z_pred(0) = std::atan2(mean_sin, mean_cos);

    // Measurement covariance matrix S
    Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
    S.fill(0.0);
    
    // Cross correlation matrix Tc
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x, n_z);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // Residual
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(0) = NormalizeAngle(z_diff(0));

        S = S + weights_(i) * z_diff * z_diff.transpose();

        // State difference
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Normalize angle in state? No, state is cartesian.

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Add measurement noise covariance matrix
    S = S + R_;

    // Kalman gain K;
    Eigen::MatrixXd K = Tc * S.inverse();

    // Residual
    Eigen::VectorXd z = Eigen::VectorXd(n_z);
    z(0) = obs.bearing;
    
    Eigen::VectorXd z_diff = z - z_pred;
    z_diff(0) = NormalizeAngle(z_diff(0));

    // Update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

TargetState UKF::getResult() const {
    TargetState res;
    res.x = x_(0);
    res.y = x_(1);
    res.vx = x_(2);
    res.vy = x_(3);
    return res;
}
