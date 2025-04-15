#pragma once

/**
 * Main controller code. 
 * Also handles some stepper motor and belt pulley conversions.
 * It is assumed that moving mass 1 moves along the x axis, 2 along y, and 3 along z.
 */
#include <Eigen/Dense>
#include <algorithm>

#include "kalman.h"
#include "telemetry.h"

#define CONTROLLER_RATE_HZ (200.0f)

static const int CL_turn_on = 10; // wait until this many points are in nu to turn on CL

using namespace Eigen;

/* Gains in Control Law u */
static Matrix3d K_1;
static Matrix3d K_2; // derivative only 
static Matrix3d K_3; 
static Matrix3d K_4; // derivative only
static Matrix3d alpha_1; 
static Matrix3d alpha_2; // proportional only
static Matrix3d gamma_gain; // Learning Rate in Estimation Law
static Matrix3d CL_gain; // concurrent learning size
static Matrix3d adaptive_gain; // contribution of adaptive to control law

static const double A = M_PI/3; // Desired Oscillation Amplitude

/* Concurrent learning gains */
static const double CL_on = 0.0f; 
static const double CL_point_accept_epsilon = 1; // Threshold to accept new points
static const int p_bar = 10; // maximum number of points stored

/* System matrices */
static const Matrix<double, 6, 6> A_state_matrix = 
        (Matrix<double, 6, 6>() << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Zero(3, 6)).finished();

/*  Spacecraft mechanical parameters */
static const double Jxx_B = 909.86e-4;  // body frame MoI (measured with machine) 
static const double Jyy_B = 809.066e-4; // body frame MoI (measured with machine) 
static const double Jzz_B =  0.124;     // body frame MoI (measured with machine) //  Moment of Inertia Matrix
static const double J_xy_B = -0.003;   // body frame MoI diagonals (catia)
static const double J_xz_B = 4.663e-4; // body frame MoI diagonals (catia)
static const double J_yz_B = -0.001;   // body frame MoI diagonals (catia)

static const Matrix3d J_B = (Matrix3d() << 
    Jxx_B, J_xy_B, J_xz_B, 
    J_xy_B, Jyy_B, J_yz_B, 
    J_xz_B, J_yz_B, Jzz_B).finished();
static Matrix3d J_P; // principal moi

static const double M = 6.70f + 0.800f; // Simulator mass kg
static const double m_x = 2*260*1e-3; // Actuator masses kg
static const double m_y = 2*260*1e-3; // Actuator masses kg
static const double m_z = 268*1e-3; // Actuator masses kg
static const Matrix3d mm_mass_matrix = (Matrix3d() << 
    m_x,0,0,
    0,m_y,0,
    0,0,m_z).finished(); // moving mass' mass matrix

static const double m_x_max_pos_meters =  (93.0f - 60.0f/2.0f)*1e-3    - 10*1e-3   ; // 93mm to center of mass from limit switch (roughly), then substract half of the width of the mass so it doesnt bump switch 
static const double m_y_max_pos_meters =  (93.0f - 60.0f/2.0f)*1e-3    - 10*1e-3   ; // 93mm to center of mass from limit switch (roughly), then substract half of the width of the mass so it doesnt bump switch 
static const double m_z_max_pos_meters = -((85.8+97.75)-70.0f/2.0f)*1e-3; // farthest from bottom plat
static const double m_z_min_pos_meters = -(85.8+ 70.0f/2.0f)*1e-3; // closest to bottom plate we can get. 71 = distance from CoR to limit switch trigger point

static const double u_com_max_x  =  160.6878 * (1e-3);
static const double u_com_max_y  =  160.6878 * (1e-3);
static const double u_com_max_z  =  256.9926 * (1e-3);

static const Vector3d theta_hat_min = (Vector3d() << -0.1, -0.1, -0.1).finished();
static const Vector3d theta_hat_max = (Vector3d() << 0.1, 0.1, 0.3).finished();

static const Vector3d g_I = (Vector3d() << 0,0,9.81).finished(); // % Gravity vector

/* Main controller functions */
telemetry_t Controller(telemetry_t t, double dt_seconds);
Vector3d CalcTorqueCommand();
Vector3d CalcMassPositionFromTorqueCommand(Vector3d u_com);
Vector3d SaturationLimit(Vector3d r_com);

telemetry_t PD_Controller(telemetry_t t, double dt_seconds);


int SetGains(Matrix3d K_1_, Matrix3d K_2_, Matrix3d K_3_, Matrix3d K_4_,
    Matrix3d alpha_1_, Matrix3d alpha_2_,
    Matrix3d gamma_gain_, Matrix3d CL_gain_, Matrix3d adaptive_gain_);

int InitController();
int InitKalmanFilter(Vector3d omega_b2i_measurement, Quaterniond q_i2b_0);

Matrix3d skew(const Eigen::Vector3d& v); // Function to compute the skew-symmetric matrix

