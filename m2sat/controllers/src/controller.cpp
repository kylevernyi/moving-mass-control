#include "controller.h"
#include "kalman.h"
#include <vector>

using namespace Eigen;

static Vector3d theta_hat = Vector3d::Zero();

static std::vector<Matrix3d> Phi_stack; // CL stacks
static std::vector<Vector3d> Tau_stack; // CL stacks
static std::vector<Vector<double, 6>> nu; // estimated states time history

int InitKalmanFilter(Vector3d omega_b2i_measurement)
{
    nu.emplace_back( (Vector<double,6>() << omega_b2i_measurement,0,0,0).finished() ); // nu_0
    InitCovariance();   // P_0

    return 0;
}

Vector3d Controller
(
    Vector3d omega_b2i,
    Quaterniond q_i2b,
    VectorXd nu,
    Vector3d mass_positions,
    Vector3d mass_velocities,
    Quaterniond q_i2d
    // inputs: body rate in body frame (omega)
        // pose quaternion i2b 
        // nu vector from kalman filter (omega and omega_dot)
        // position of sliding masses relative to CoR
        // velocity of sliding masses
        // theta_hat
        // desired quaternion q_i2d
)
{
    /*  Desired trajectories */

    /* Error signal */

    /* Update of J(t) as a function of new mass position */

    /* Compute basis function Phi */

    /* Control Torque as Designed by Lyapunov Analysis */

    /* Map control Torque to mass positions */

    /* Concurrent learning data selection algorithm */

    /* Concurrent learning error signal */

    /* Dynamics for desired frame */

    return Vector3d();
}
