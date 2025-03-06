#include "controller.h"
#include "kalman.h"
#include <vector>
#include <iostream>

using namespace Eigen;

static Vector3d theta_hat = Vector3d::Zero();

static Matrix<double, 9, Eigen::Dynamic> Phi_stack;
static Matrix<double, 3, Eigen::Dynamic> Tau_stack; // CL stacks
static std::vector<Vector<double, 6>> nu; // estimated states time history
static Vector3d omega_b2i_I_0;

static Matrix3d J; // moment of inertia


// Helper functions
Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v);
Matrix3d skew(const Vector3d& v);
Quaterniond quat_mult(const Quaterniond& q, const Quaterniond& p);
int rank(MatrixXd mat);

/**
 * Convert the total pulses of the motor (which trackes angular position of the shaft) to radians then to linear position of the mass
 */
Vector3d ConvertMotorPositionToMassPosition(int32_t x, int32_t y, int32_t z)
{
    // rotational, maps units of the stepper motor to radians
    double x_rad = double(x) * TAU * (1/STEPPER_STEPS_PER_REV); // 2pi radians per 200 counts
    double y_rad = double(y) * TAU * (1/STEPPER_STEPS_PER_REV); // 2pi radians per 200 counts
    double z_rad = double(z) * TAU * (1/STEPPER_STEPS_PER_REV); // 2pi radians per 200 counts
    
    double x_pos = m_x_initial_position_meters + x_rad * X_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);
    double y_pos = m_y_initial_position_meters + y_rad * Y_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);
    double z_pos = m_z_initial_position_meters + z_rad * Z_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);
    
    Vector3d output; 
    output << x_pos, y_pos, z_pos;
    return output;
}

/**
 * Convert the linear position of the mass back to total pulses of the motor
 */
std::vector<int32_t> ConvertMassPositionToMotorPosition(double x_pos, double y_pos, double z_pos)
{
    // Convert the mass position back to the motor's position in radians
    double x_rad = (x_pos - m_x_initial_position_meters) / (X_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));
    double y_rad = (y_pos - m_y_initial_position_meters) / (Y_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));
    double z_rad = (z_pos - m_z_initial_position_meters) / (Z_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));

    // Convert radians back to pulses
    int32_t x = int32_t(x_rad * STEPPER_STEPS_PER_REV / TAU);
    int32_t y = int32_t(y_rad * STEPPER_STEPS_PER_REV / TAU);
    int32_t z = int32_t(z_rad * STEPPER_STEPS_PER_REV / TAU);

    return std::vector<int32_t>{x,y,z};
}

/**
 * Convert pulses per second velocity to meters per second linear mass velocity 
 */
Vector3d ConvertMotorSpeedToMassVelocity(int32_t xdot, int32_t ydot, int32_t zdot )
{
    // rotational, maps units of the stepper motor to radians per second
    double x_radians_per_sec = double(xdot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    double y_radians_per_sec = double(ydot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    double z_radians_per_sec = double(zdot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    
    // linear = r * omega where r is pitch radius in meters and omega is angular velocity of the pulley
    double x_linear = (X_GEAR_RATIO * x_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM / 1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear
    double y_linear = (Y_GEAR_RATIO * y_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM / 1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear
    double z_linear = (Z_GEAR_RATIO * z_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM / 1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear

    Vector3d output; 
    output << x_linear, y_linear, z_linear;
    return output;
}

int InitKalmanFilter(Vector3d omega_b2i_measurement, Quaterniond q_i2b_0)
{
    nu.emplace_back( (Vector<double,6>() << omega_b2i_measurement,0,0,0).finished() ); // nu_0
    InitCovariance();   // P_0
    
    // Obtain the initial angular rates of the BFF w.r. to Inertial in the Inertial Frame
    omega_b2i_I_0 = quat_rotate(q_i2b_0, omega_b2i_measurement); 

    return 0;
}

int InitController()
{
    // J needs initialized
    J(0,0) = 1;
    J(1,1) = 1;
    J(2,2) = 1;

    return 0;
}

// inputs: body rate in body frame (omega)
// pose quaternion i2b 
// nu vector from kalman filter (omega and omega_dot)
// position of sliding masses relative to CoR
// velocity of sliding masses
// theta_hat
// desired quaternion q_i2d
telemetry_t Controller(telemetry_t t, double dt_seconds)
{
    telemetry_t controller_output = t; // maybe dont do this 
    // Compute our actual control torque at the moment 
    controller_output.u_actual = mm_mass_matrix * t.r_mass;
    // Extract states
    Quaterniond q_i2b = t.get_q_i2b();
    // Vector3d omega_b2i_B_hat = nu.back().head<3>();  // first 3 elements
    // Vector3d omega_b2i_B_dot_hat = nu.back().segment(3,3); // last 3 elements 

    /*  Desired trajectories */
    // Predefined Omega and Quaternion Trajectory w.r. to Inertial Frame
    static const double b = 0.01*M_PI; 
    static const double f = 0.01;
    Vector3d omega_b2i_I = quat_rotate(q_i2b, t.omega_b2i_B); // map omega_b2i_B from body frame to inertial frame
    Vector3d omega_d2i_I = ( Vector3d() << 0,0,omega_b2i_I.z() ).finished(); // Angular rates of DF w.r. Inertial in Inertial Frame

    // Transformation Rotation of Angular Rates of the DF w.r. to Inertial from Inertial to DF
    Vector3d omega_d2i_D = quat_rotate(t.q_i2d.conjugate(), omega_d2i_I); //  quat_mult(quat_mult(quat_conj(q_i2d),[0;omega_d2i_I]),q_i2d);

    Vector3d omega_dot_d2i_D = (Vector3d() << 0,0,0).finished(); //0; 0*A*exp(-f*t)*(f*sin(b*t) - b*cos(b*t));0];
    Vector3d omega_dot_d2i_I = quat_rotate(q_i2b, omega_dot_d2i_D); // quat_mult(quat_mult(q_i2d,[0;omega_dot_d2i_D]),quat_conj(q_i2d));
    Vector3d omega_dot_d2i_B = quat_rotate(q_i2b.conjugate(), omega_dot_d2i_I); // quat_mult(quat_mult(quat_conj(q_i2b),[0;omega_dot_d2i_I]),q_i2b);

    /* Error signal */
    Quaterniond q_d2b = t.q_i2d.conjugate() * q_i2b; //  quat_mult(quat_conj(q_i2d),q_i2b);
    // Omega Desired w.r. Inertial in Body Fixed Frame (BFF)
    Vector3d omega_d2i_B =  quat_rotate(q_d2b.conjugate(), omega_d2i_D); //  quat_mult(quat_mult(quat_conj(q_d2b),[0;omega_d2i_D]),q_d2b);
    Vector3d omega_b2d_B = t.omega_b2i_B - omega_d2i_B; // Omega Body w.r. Desired in BFF
    // Vector3d omega_b2d_B=  omega_b2i_B_hat - omega_d2i_B; // Omega Body w.r. Desired in BFF that has noise influece, used in control signal

    Vector3d r = omega_b2d_B + alpha*q_d2b.vec(); // Definition of r error signal
    std::cout << (alpha*q_d2b.vec()).transpose() << std::endl;
    /* Update of J(t) as a function of new mass position */
    Matrix3d Jm_B_dot;
    Jm_B_dot << t.r_mass.y()*t.rdot_mass.y() + t.r_mass.z()*t.rdot_mass.z() , 0 , 0,
            0, t.r_mass.x()*t.rdot_mass.x() + t.r_mass.z()*t.rdot_mass.z(), 0, 
            0, 0, t.r_mass.x()*t.rdot_mass.x() + t.r_mass.y()*t.rdot_mass.y(); 
    Jm_B_dot = mm_mass_matrix*Jm_B_dot;

    /* Compute basis function Phi */
    Vector3d g_B = quat_rotate(q_i2b.conjugate(), g_I);  //quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
    Matrix3d g_b_x = skew(g_B); // Skew Matrix of Gravity Vector in BFF
    Matrix3d Phi = -M*g_b_x; // Phi definition as in ref[DOI: 10.2514/1.60380]

    /* Control Torque as Designed by Lyapunov Analysis */
    Matrix3d Proj_operator = ( Matrix3d::Identity() - (g_B*g_B.transpose()) ) / g_B.squaredNorm();
    controller_output.u_com = Proj_operator * (-Jm_B_dot*(0.5*r - t.omega_b2i_B) + skew(t.omega_b2i_B)*J*t.omega_b2i_B - Phi*theta_hat - K*r ) // -diag((theta_hat).^2)*r
        + Proj_operator*J*(omega_dot_d2i_B + skew(omega_d2i_B)*omega_b2d_B - 0.5*alpha*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 



    /* Map control Torque to mass positions */
    // Transformation of u_com to Commanded Positions as in ref[DOI: 10.2514/1.60380]
    controller_output.r_mass_commanded = mm_mass_matrix.inverse() * (skew(g_B) * controller_output.u_com / g_B.squaredNorm() ); // desired commanded mass positions
    // at this point, r_mass_commanded is relative to the middle zero position of the sliding masses (not the zero limit switch position)
    /* Make sure r_mass_commanded is within saturation limits (makes sense to apply here before stepper mapping) */
    controller_output.r_mass_commanded = SaturationLimit(controller_output.r_mass_commanded);

    /* Concurrent learning data selection algorithm */
    static int p_CL_idx = 0;
    if (t.nu.size() > CL_turn_on) // make sure we have state estimate
    {
        // initialization done once
        static Vector<double, 9> Phi_previous = Phi.reshaped(9, 1);
        static int current_index_for_cyclic_stack = p_CL_idx;

        // extract states
        Vector3d omega_hat = t.nu.back().segment(0,3);
        Vector3d omega_dot_hat = t.nu.back().segment(3,3);

        // append Phi to a new stack to check if rank changes
        MatrixXd Phi_new_stack = Phi_stack; // [Phi_stack, Phi(:)];
        Phi_new_stack.conservativeResize(Eigen::NoChange, Phi_new_stack.cols() + 1);
        Phi_new_stack.col(Phi_new_stack.cols() - 1) = Phi.reshaped(9,1); // added new col, set new col equal to Phi(:)
        
        // Point selection criteria
        if ( ( (Phi.reshaped(9,1)- Phi_previous).squaredNorm()  >= CL_point_accept_epsilon)  || (rank(Phi_new_stack) > rank(Phi_stack)) )
        {
            Vector3d Tau_j; // = J*omega_dot_hat(:,w_iter) +  cross(omega_hat(:,w_iter),J*omega_hat(:,w_iter)) - u_com(:,w_iter);

            if (p_CL_idx < p_bar) // record more data until p_bar points
            {
                p_CL_idx = p_CL_idx+1;
                current_index_for_cyclic_stack = p_CL_idx;
                Phi_stack = Phi_new_stack; 
                Phi_previous = Phi.reshaped(9,1);

                Tau_stack.conservativeResize(Eigen::NoChange, p_CL_idx);
                Tau_stack.col(Tau_stack.cols() - 1) = Tau_j; // insert tau into stack
            }
            else
            {
                // cyclic history stack
                Phi_stack.col(current_index_for_cyclic_stack) = Phi.reshaped(9,1); // Insert the new element at the current index
                Tau_stack.col(current_index_for_cyclic_stack) = Tau_j; // Insert the new element at the current index
                current_index_for_cyclic_stack = (current_index_for_cyclic_stack % p_bar) + 1;
            }
            // point_added = [point_added t]; %  record that we stored a point
        }
    }

    /* Concurrent learning error signal */
    Vector3d concurrent_learning_Tau_ext; concurrent_learning_Tau_ext << 0,0,0;

        for (int j=1; j<= p_CL_idx; j++)
        {
            Vector3d Tau_j = Tau_stack.col(j); // jth delta from storage
            Matrix3d Phi_j = Phi_stack.col(j).reshaped(3,3); // jth phi matrix from storage
            Vector3d epsilon_Tau_ext = Phi_j*theta_hat - Tau_j;
            concurrent_learning_Tau_ext += Phi_j*epsilon_Tau_ext;  
        }


    
    /* Dynamics for desired frame */
    Quaterniond omega_d2i_D_quaternion; 
    omega_d2i_D_quaternion.w() = 0; 
    omega_d2i_D_quaternion.vec() = omega_d2i_D;
    Quaterniond q_i2d_dot = quat_mult(t.q_i2d, omega_d2i_D_quaternion);  // quat_mult(q_i2d,[0;omega_d2i_D]); % q_dot of DF w.r. to Inertial Frame
    q_i2d_dot.coeffs() *= 0.5; // dont forget to scale by 0.5 since we cant do that above due to * operator override
    
    /* Update law */
    Vector3d theta_hat_dot = gamma_gain * ( (Phi.transpose()*r)  + CL_on*CL_gain*concurrent_learning_Tau_ext); //  Adaptive update law

    controller_output.theta_hat += theta_hat_dot*dt_seconds; 
    return controller_output;
}

/* Apply a saturation limit to the mass position based on mechanical limitations of the vehicle */
Vector3d SaturationLimit(Vector3d r_com)
{
    // there is probably a shorter easier way of doing this but it works fine and is easy to read
    Vector3d saturated_r_com = r_com;

    // x (symmetric)
    if (saturated_r_com.x() > m_x_max_pos_meters) {
        saturated_r_com.x() = m_x_max_pos_meters;
    } else if (saturated_r_com.x() < -m_x_max_pos_meters) {
        saturated_r_com.x() = -m_x_max_pos_meters;
    }

    // y (symmetric)
    if (saturated_r_com.y() > m_y_max_pos_meters) {
        saturated_r_com.y() = m_y_max_pos_meters;
    } else if (saturated_r_com.y() < -m_y_max_pos_meters) {
        saturated_r_com.y() = -m_y_max_pos_meters;
    }

    // z (asymmetric)
    if (saturated_r_com.z() > m_z_max_pos_meters) {
        saturated_r_com.z() = m_z_max_pos_meters;
    } else if (saturated_r_com.y() < m_z_min_pos_meters) {
        saturated_r_com.y() = m_z_min_pos_meters;
    }

    return saturated_r_com;
}

Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v)
{
    Quaterniond v_quat(0, v.x(), v.y(), v.z());  // Pure quaternion [0; v]
    Quaterniond rotated = q * v_quat * q.conjugate();  // q * [0; v] * q^*
    return rotated.vec();  // Extract the vector part (x, y, z)
}

Matrix3d skew(const Vector3d& v)
{
    Matrix3d skew_sym_matrix;
    skew_sym_matrix <<  0,   -v.z(),  v.y(),
             v.z(),   0,  -v.x(),
            -v.y(),  v.x(),   0;
    return skew_sym_matrix;
}

Quaterniond quat_mult(const Quaterniond& q, const Quaterniond& p)
{
    return q * p;  
}

int rank(MatrixXd mat)
{
    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    double tol = 1e-6 * svd.singularValues().array().abs()(0);  // Tolerance based on largest singular value
    int rank_ = (svd.singularValues().array() > tol).count();
    return rank_;
}

