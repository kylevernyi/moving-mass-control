#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#define KALMAN_RATE_HZ (100.0f)

/* Noise characteristics */
static const double SR = 100; // Maximum Sample Rate in Hz
static const double  ND = 0.0035*M_PI/180; // Noise Density of Gyroscope in rad
static const double sigma_omega = ND*sqrt(SR); //  Noise Standard Deviation

/* Fixed point smoothing parameters */
static const Matrix<double, 3, 6> H = 
    (Matrix<double, 3, 6>() << Matrix3d::Identity(), Matrix3d::Zero()).finished();
// Phi_stm = expm([zeros(3,3), eye(3); zeros(3,6)]*dt); % state transistion matrix for kalman filter
static const int j_smooth = 100; // iterations till we turn on fixed point smoother


static const int v_smooth = 30; // how many future points to include in smoothing
static Matrix<double, 6, 6> Q = 
    (Matrix<double, 6, 6>() << 0.01 * Matrix3d::Identity(), Matrix3d::Zero(),
    Matrix3d::Zero(), 10*Matrix3d::Identity()).finished(); // Process noise covariance

static Matrix3d R = sigma_omega*sigma_omega * Matrix3d::Identity(); // measurement noise covariance


VectorXd CalcXhat();

int InitCovariance();