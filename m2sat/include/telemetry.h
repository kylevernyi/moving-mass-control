#pragma once

#include <Eigen/Dense>
#include <chrono>

#define TELEMETRY_SEND_RATE_HZ (10) 
using namespace Eigen;
typedef Vector<double, 6> Vector6d;

struct telemetry_t 
{
    uint64_t time = 0;
    Quaterniond q_b2i = Quaterniond::Identity();
    Quaterniond q_i2d = Quaterniond::Identity();
    Vector3d omega_b2i= Vector3d::Zero();
    Vector3d r_mass; //mass postion
    Vector3d rdot_mass; // mass velocity
    Vector3d r_mass_commanded; // commanded mass position
    Vector3d u_com; // commanded torque
    Vector3d u_actual; // actual control torque
    Vector6d nu; // kalman filter estimate
};
