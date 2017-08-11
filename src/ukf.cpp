#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.6;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = VectorXd(n_x_);

    // state covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    // Weight Metrix
    weights_ = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }

    // Process Noise Covariance Matrix
    Q = MatrixXd(2, 2);
    Q << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;

    MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Measurement Noises
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
            0, std_radphi_ * std_radphi_, 0,
            0, 0, std_radrd_ * std_radrd_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (!is_initialized_) {
        // Switch between lidar and radar
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double v = meas_package.raw_measurements_[2];
            double px = rho * cos(phi);
            double py = rho * sin(phi);
            double yawd = 0.0;
            x_ << px, py, v, phi, yawd;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            double px = meas_package.raw_measurements_[0];
            double py = meas_package.raw_measurements_[1];
            x_ << px, py, 0, 0, 0;
        }
        previous_timestamp_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    if ((use_radar_ && meas_package.sensor_type_ == meas_package.RADAR) ||
        (use_laser_ && meas_package.sensor_type_ == meas_package.LASER)) {
        double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
        previous_timestamp_ = meas_package.timestamp_;

        Prediction(delta_t);
        UpdateMeasurement(meas_package);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    // Predict sigma points, the state, and the state covariance matrix.
    GenerateStateSigmaPoints(delta_t);
    PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    // Use lidar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    //create matrix for sigma points in measurement space

    UpdateMeasurement(meas_package);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    // Use radar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    UpdateMeasurement(meas_package);
}

void UKF::GenerateStateSigmaPoints(double delta_t) {

    // Generate Augmented Sigma Points
    VectorXd x_aug = VectorXd(n_aug_);

    x_aug.fill(0.0);
    x_aug.block<5, 1>(0, 0) = x_;

    //create augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug.bottomRightCorner(2, 2) = Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    MatrixXd sqaure_root = sqrt(lambda_ + n_aug_) * A;

    //create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    MatrixXd x_replicated = x_aug.rowwise().replicate(7);
    Xsig_aug.block<7, 1>(0, 0) = x_aug;
    Xsig_aug.block<7, 7>(0, 1) = x_replicated + sqaure_root;
    Xsig_aug.block<7, 7>(0, 8) = x_replicated - sqaure_root;


    // Find State Sigma Points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance() {
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // normalize angle
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI);

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateMeasurement(MeasurementPackage meas_package) {
    VectorXd z;
    MatrixXd z_sig;
    VectorXd z_pred;
    MatrixXd R;
    int n_z;

    if (meas_package.sensor_type_ == meas_package.RADAR) {
        n_z = n_z_radar_;
        R = R_radar_;

        // Read Measurements
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double v = meas_package.raw_measurements_[2];

        z = VectorXd(n_z);
        z << rho, phi, v;

        // predict radar measurement
        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);

        for (int i = 0; i < 2 * n_aug_ + 1; i++) {
            double p_x = Xsig_pred_(0, i);
            double p_y = Xsig_pred_(1, i);
            double vel = Xsig_pred_(2, i);
            double yaw = Xsig_pred_(3, i);
            double v_x = cos(yaw) * vel;
            double v_y = sin(yaw) * vel;

            // rho
            z_sig(0, i) = sqrt(p_x * p_x + p_y * p_y);

            // phi
            z_sig(1, i) = atan2(p_y, p_x);

            // rho_dot
            z_sig(2, i) = (z_sig(0, i) < 0.0001 ? (p_x * v_x + p_y * v_y) / 0.0001 : (p_x * v_x + p_y * v_y) / z_sig(0, i));
        }
    } else if (meas_package.sensor_type_ == meas_package.LASER) {
        n_z = n_z_lidar_;
        R = R_laser_;

        double px = meas_package.raw_measurements_[0];
        double py = meas_package.raw_measurements_[1];

        z = VectorXd(n_z);
        z << px, py;

        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);

        for (int i = 0; i < 2 * n_aug_ + 1; i++) {
            // measurement model
            z_sig(0, i) = Xsig_pred_(0, i);
            z_sig(1, i) = Xsig_pred_(1, i);
        }
    }


    //mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * z_sig.col(i);
    }

    //measurement covariance matrix
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    // Cross correlation matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = z_sig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();

        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI); // normalize angle
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R;

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // normalize angle
    if (meas_package.sensor_type_ == meas_package.RADAR) {
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);
    }



    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // Calculating NIS
    string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    VectorXd nis = z_diff.transpose() * S.inverse() * z_diff;
    cout << sensor_type << " measurement : " << nis << endl;
}