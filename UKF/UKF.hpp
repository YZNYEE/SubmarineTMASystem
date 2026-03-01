#ifndef UKF_HPP
#define UKF_HPP

#include "../COMMON/DataStruct.hpp"
#include <Eigen/Dense>
#include <vector>

class UKF {
public:
    // State dimension
    static const int n_x = 4; // x, y, vx, vy
    // Measurement dimension
    static const int n_z = 1; // bearing

    UKF();
    ~UKF();

    /**
     * @brief Initialize the UKF
     * @param init_state Initial state guess
     * @param init_P Initial covariance matrix
     * @param std_a Process noise standard deviation (acceleration noise)
     * @param std_rad Measurement noise standard deviation (radians)
     */
    void init(const TargetState& init_state, const Eigen::Matrix4d& init_P, double std_a, double std_rad);

    /**
     * @brief Process a new measurement
     * @param obs The observation data
     */
    void update(const ObsData& obs);

    /**
     * @brief Get the current state estimate
     * @return TargetState
     */
    TargetState getResult() const;

private:
    // State vector: [x, y, vx, vy]
    Eigen::VectorXd x_;

    // State covariance matrix
    Eigen::MatrixXd P_;

    // Predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    // Process noise covariance matrix
    Eigen::MatrixXd Q_;

    // Measurement noise covariance matrix
    Eigen::MatrixXd R_;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // Time when the state is true
    double time_;

    // Sigma point spreading parameter
    double lambda_;

    // Augmented state dimension (for process noise)
    int n_aug_;

    // Process noise standard deviation
    double std_a_;
    
    // Measurement noise standard deviation
    double std_rad_;

    bool is_initialized_;

    /**
     * @brief Generate sigma points
     * @param Xsig_out Output matrix
     */
    void GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out);

    /**
     * @brief Predict sigma points
     * @param dt Time delta
     */
    void PredictSigmaPoints(double dt);

    /**
     * @brief Predict mean and covariance
     */
    void PredictMeanAndCovariance();

    /**
     * @brief Update state with measurement
     * @param obs Observation
     */
    void UpdateState(const ObsData& obs);

    /**
     * @brief Normalize angle to [-PI, PI]
     */
    double NormalizeAngle(double angle);
};

#endif // UKF_HPP
