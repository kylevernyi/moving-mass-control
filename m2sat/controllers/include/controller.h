#pragma once

#include "kalman.h"
#include <Eigen/Dense>

#define CONTROLLER_RATE_HZ (100)

using namespace Eigen;

/* Gains in Control Law u */
static const double K = 1e-6; // Derivative
// static const alpha = 1e-6*diag([1,1,1]); // proportional gain
// gamma = 1e-5*diag([1 1 1]); % Learning Rate in Estimation Law

static const double A = M_PI/3; // Desired Oscillation Amplitude

/* Concurrent learning gains */
static const double CL_on = 1.0f; 
static const double CL_point_accept_epsilon = 0.01; // Threshold to accept new points
static const double p_bar = 10; // maximum number of points stored
// static const double CL_gain = 0.01*(1/gamma) / p_bar;

/* System matrices */
static const Matrix<double, 6, 6> A_state_matrix = 
        (Matrix<double, 6, 6>() << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Zero(3, 6)).finished();

/*  Spacecraft mechanical parameters */
static const double Jxx = 0.0226; 
static const double Jyy = 0.0226; 
static const double Jzz = 0.0226; //  Moment of Inertia Matrix
static const Matrix3d J0 = (Matrix3d() << 
    Jxx, 0.0, 0.0, 
    0.0, Jyy, 0.0, 
    0.0, 0.0, Jzz).finished();

static const double M = 4.2; // Simulator mass
static const double m1 = 0.3; // Actuator masses
static const double m2 = 0.3; // Actuator masses
static const double m3 = 0.3; // Actuator masses
// m = diag([m1,m2,m3]);
static const Vector3d g_I = (Vector3d() << 0,0, 9.81).finished(); // % Gravity vector

Vector3d Controller();
Vector3d CalcTorqueCommand();
Vector3d CalcMassPositionFromTorqueCommand(Vector3d u_com);

int InitKalmanFilter(Vector3d omega_b2i_measurement);