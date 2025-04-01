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
#include "tic.h"

#define CONTROLLER_RATE_HZ (100)
#define TAU (2*M_PI)

/* Gear train confifuration */
#define X_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth = 12.9mm : 12.9mm
#define Y_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth
#define Z_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth
#define PULLEY_PITCH_RADIUS_MM (12.9f/2.0f) // effective radius of pulley in millimeters (12.9/2)

static const int CL_turn_on = 10; // wait until this many points are in nu to turn on CL

using namespace Eigen;

/* Gains in Control Law u */
static const double K = 10; // Derivative
static const Matrix3d alpha = (Matrix3d() << 
    75, 0.0, 0.0, 
    0.0, 75, 0.0, 
    0.0, 0.0, 1e-7).finished(); // proportional gain

static const Matrix3d gamma_gain = (Matrix3d() << 
    1e-3, 0.0, 0.0, 
    0.0, 1e-3, 0.0, 
    0.0, 0.0, 1e-3).finished(); // Learning Rate in Estimation Law


static const double A = M_PI/3; // Desired Oscillation Amplitude

/* Concurrent learning gains */
static const double CL_on = 0.0f; 
static const double CL_point_accept_epsilon = 1; // Threshold to accept new points
static const int p_bar = 10; // maximum number of points stored
static const Matrix3d CL_gain = 0.000001*gamma_gain.inverse() / p_bar;

/* System matrices */
static const Matrix<double, 6, 6> A_state_matrix = 
        (Matrix<double, 6, 6>() << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Zero(3, 6)).finished();

/*  Spacecraft mechanical parameters */
static const double Jxx = 0.000189261; //0.0226; 
static const double Jyy = 0.000189261; //0.0226; 
static const double Jzz = 0.000189261; //0.0226; //  Moment of Inertia Matrix
static const Matrix3d J0 = (Matrix3d() << 
    Jxx, 0.0, 0.0, 
    0.0, Jyy, 0.0, 
    0.0, 0.0, Jzz).finished();

static const double M = 6.5; // Simulator mass kg
static const double m_x = 260*1e-3; // Actuator masses kg
static const double m_y = 260*1e-3; // Actuator masses kg
static const double m_z = 268*1e-3; // Actuator masses kg
static const Matrix3d mm_mass_matrix = (Matrix3d() << 
    m_x,0,0,
    0,m_y,0,
    0,0,m_z).finished(); // moving mass' mass matrix

static const double m_x_max_pos_meters =  (93.0f - 60.0f/2.0f)*1e-3; // 93mm to center of mass from limit switch (roughly), then substract half of the width of the mass so it doesnt bump switch 
static const double m_y_max_pos_meters =  (93.0f - 60.0f/2.0f)*1e-3; // 93mm to center of mass from limit switch (roughly), then substract half of the width of the mass so it doesnt bump switch 
static const double m_z_max_pos_meters =  97.75*1e-3; // (not symmetrical)
static const double m_z_min_pos_meters = 0; // needs calibrated still (not symmetrical)

static const Vector3d g_I = (Vector3d() << 0,0,9.81).finished(); // % Gravity vector

/* Main controller functions */
telemetry_t Controller(telemetry_t t, double dt_seconds);
Vector3d CalcTorqueCommand();
Vector3d CalcMassPositionFromTorqueCommand(Vector3d u_com);
Vector3d SaturationLimit(Vector3d r_com);

/* Motor mapping functions*/
Vector3d ConvertMotorPositionToMassPosition(int32_t x, int32_t y, int32_t z);
Vector3d ConvertMotorSpeedToMassVelocity(int32_t xdot, int32_t ydot, int32_t zdot );
std::vector<int32_t> ConvertMassPositionToMotorPosition(double x_pos, double y_pos, double z_pos);

int InitController();
int InitKalmanFilter(Vector3d omega_b2i_measurement, Quaterniond q_i2b_0);

Matrix3d skew(const Eigen::Vector3d& v); // Function to compute the skew-symmetric matrix

