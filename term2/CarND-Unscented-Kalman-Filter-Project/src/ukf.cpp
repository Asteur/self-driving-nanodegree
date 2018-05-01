#include "ukf.h"
#include "Eigen/Dense"
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
	std_a_ =1.2;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = .3;

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

	//set state dimension
	n_x = 5;

	//set augmented dimension
	n_aug = 7;

	//lambda factor
	lambda = 5 - n_aug;

	//number of sigma points
	sigma_points = 2 * n_aug + 1;

	//create vector for weights
	weights = VectorXd(sigma_points);

	//create matrix with predicted sigma points as columns
	Xsig_pred = MatrixXd(n_x, sigma_points);

	n_z_radar = 3;

	n_z_lidar = 2;

	//add measurement noise covariance matrix
	R_lidar = MatrixXd(n_z_lidar, n_z_lidar);
	R_lidar << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

	//add measurement noise covariance matrix
	R_radar = MatrixXd(n_z_radar, n_z_radar);
	R_radar << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_
			* std_radrd_;

	NIS=0;

}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Make sure you switch between lidar and radar
	 measurements.
	 */

	if (!is_initialized_) {
		//Initialize the state ekf_.x_ with the first measurement.

		// first measurement
		cout << "UKF: " << endl;
		P_ = MatrixXd::Identity(5, 5);

		if (use_radar_
				&& meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			 Convert radar from polar to cartesian coordinates and initialize state.
			 */

			float rho = meas_package.raw_measurements_[0]; // range
			float phi = meas_package.raw_measurements_[1]; // bearing
			float rho_dot = meas_package.raw_measurements_[2]; // velocity of rho

			// Coordinates convertion from polar to cartesian
			double cos_phi=cos(phi);
			double sin_phi=sin(phi);

			float px = rho *cos_phi ;
			float py = rho * sin_phi;
			float vx = rho_dot * cos_phi;
			float vy = rho_dot * sin_phi;

			float v = sqrt(vx * vx + vy * vy);
			x_ << px, py, v, 0, 0;
			// done initializing, no need to predict or update
			is_initialized_ = true;
		} else if (use_laser_
				&& meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
			// done initializing, no need to predict or update
			is_initialized_ = true;
		}

		time_us_ = meas_package.timestamp_;

		// set weights
		double weight_0 = lambda / (lambda + n_aug);
		weights(0) = weight_0;
		for (int i = 1; i < sigma_points; i++) {  //2n+1 weights
			double weight = 0.5 / (n_aug + lambda);
			weights(i) = weight;
		}

		return;
	}

	// Calculate the timestep between measurements in seconds
	double dt = (meas_package.timestamp_ - time_us_);
	dt /= 1000000.0; // convert micros to s

	time_us_ = meas_package.timestamp_;
	Prediction(dt);

	if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
	}
	if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	//create augmented mean state

	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug, sigma_points);

	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug; i++) {
		double sqrt_lambda_n_aug=sqrt(lambda + n_aug);

		Xsig_aug.col(i + 1) = x_aug + sqrt_lambda_n_aug * L.col(i);
		Xsig_aug.col(i + 1 + n_aug) = x_aug - sqrt_lambda_n_aug * L.col(i);
	}

	//predict sigma points
	for (int i = 0; i < sigma_points; i++) {
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

		double sin_yaw=sin(yaw);
		double cos_yaw=cos(yaw);

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin_yaw);
			py_p = p_y + v / yawd * (cos_yaw - cos(yaw + yawd * delta_t));
		} else {
			px_p = p_x + v * delta_t * cos_yaw;
			py_p = p_y + v * delta_t * sin_yaw;
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		double delta_t_2=delta_t * delta_t;
		//add noise
		px_p = px_p + 0.5 * nu_a * delta_t_2 * cos_yaw;
		py_p = py_p + 0.5 * nu_a * delta_t_2 * sin_yaw;
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t_2;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred(0, i) = px_p;
		Xsig_pred(1, i) = py_p;
		Xsig_pred(2, i) = v_p;
		Xsig_pred(3, i) = yaw_p;
		Xsig_pred(4, i) = yawd_p;
	}

	//predicted state mean
	x_.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //iterate over sigma points
		x_ = x_ + weights(i) * Xsig_pred.col(i);
	}

	//predicted state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		P_ = P_ + weights(i) * x_diff * x_diff.transpose();
	}


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

	Zsig = MatrixXd(n_z_lidar, sigma_points);

	//transform sigma points into measurement space
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);

		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;

		// measurement model
		Zsig(0, i) = (p_x);    //px
		Zsig(1, i) = (p_y);    //py

	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_lidar);
	z_pred.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {
		z_pred = z_pred + weights(i) * Zsig.col(i);
	}

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_lidar, n_z_lidar);
	S.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S = S + weights(i) * z_diff * z_diff.transpose();
	}

	S = S + R_lidar;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x, n_z_lidar);

	Tc.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;

		Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	float px = meas_package.raw_measurements_[0]; // px
	float py = meas_package.raw_measurements_[1]; // py

	//create  vector for incoming radar measurement
	VectorXd z = VectorXd(n_z_lidar);
	z << px, py;
	//residual
	VectorXd z_diff = z - z_pred;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	NIS= z_diff.transpose() * S.inverse() * z_diff;


	cout << "NIS = "<< NIS << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	Zsig = MatrixXd(n_z_radar, sigma_points);

	//transform sigma points into measurement space
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);

		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;

		double sqrt_px2_py2= sqrt(p_x * p_x + p_y * p_y);

		// measurement model
		Zsig(0, i) = sqrt_px2_py2;                        //r
		Zsig(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt_px2_py2; //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_radar);
	z_pred.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {
		z_pred = z_pred + weights(i) * Zsig.col(i);
	}

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_radar, n_z_radar);
	S.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		S = S + weights(i) * z_diff * z_diff.transpose();
	}

	S = S + R_radar;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x, n_z_radar);

	Tc.fill(0.0);
	for (int i = 0; i < sigma_points; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	float rho = meas_package.raw_measurements_[0]; // range
	float phi = meas_package.raw_measurements_[1]; // bearing
	float rho_dot = meas_package.raw_measurements_[2]; // velocity of rho

	//create  vector for incoming radar measurement
	VectorXd z = VectorXd(n_z_radar);
	z << rho, phi, rho_dot;
	//residual
	VectorXd z_diff = z - z_pred;

	//angle normalization
	while (z_diff(1) > M_PI)
		z_diff(1) -= 2. * M_PI;
	while (z_diff(1) < -M_PI)
		z_diff(1) += 2. * M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	NIS= z_diff.transpose() * S.inverse() * z_diff;
	cout << "NIS = "<< NIS << endl;
}
