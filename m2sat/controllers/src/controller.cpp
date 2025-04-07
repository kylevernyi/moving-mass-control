#include "controller.h"
#include "kalman.h"
#include <vector>
#include <iostream>

using namespace Eigen;

static Matrix<double, 9, Eigen::Dynamic> Phi_stack;
static Matrix<double, 3, Eigen::Dynamic> Tau_stack; // CL stacks
static std::vector<Vector<double, 6>> nu; // estimated states time history
static Vector3d omega_b2i_I_0;

static Matrix3d J; // time varying moment of inertia
Matrix3d R_body2principal; // rotation matrix from eigen decomposition of MoI

// Helper functions
Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v);
Matrix3d skew(const Vector3d& v);
Quaterniond quat_mult(const Quaterniond& q, const Quaterniond& p);
int rank(MatrixXd mat);


int SetGains(Matrix3d K_1_, Matrix3d K_2_, Matrix3d K_3_, Matrix3d K_4_, Matrix3d alpha_1_, Matrix3d alpha_2_, Matrix3d gamma_gain_, Matrix3d CL_gain_, Matrix3d adaptive_gain_)
{
    K_1 = K_1_;    
    K_2 = K_2_;
    K_3 = K_3_;
    K_4 = K_4_;    

    alpha_1 = alpha_1_;
    alpha_2 = K_2.inverse()*alpha_2_;
    gamma_gain = gamma_gain_;
    
    CL_gain = CL_gain_;
    adaptive_gain = adaptive_gain_;

    return 0;
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
    J = J_B;
    // Compute eigendecomposition to get principal axes
    // SelfAdjointEigenSolver<Matrix3d> eigensolver(J_B);
    // if (eigensolver.info() != Eigen::Success) {
        // std::cerr << "Eigen decomposition failed!" << std::endl;
        // return -1;
    // }
    
    // Get the matrix of eigenvectors (principal axes)
    // R_body2principal = eigensolver.eigenvectors();
    // R_body2principal.col(0) *= -1;  // to match matlab 
    // R_body2principal.col(1) *= -1;  // to match matlab 
    // if (R_body2principal.determinant() < 0) {
        // Flip one column to enforce right-handedness
        // R_body2principal.col(0) *= -1;  // or col(2), just pick one
        // std::cout << "flipped to enforce right-handedness\n"; 
    // }

    // std::cout << R_body2principal << std::endl;
    // Get the eigenvalues (principal moments of inertia)
    // Vector3d principal_MoI = eigensolver.eigenvalues();
    // J_P = principal_MoI.asDiagonal(); // diagonal matrix of principal MOI
    // J = J_P; // we use J in the code for the controller not J_P but it is same thing

    // exit(-1);
    return 0;
}

// inputs: body rate in body frame (omega)
// pose quaternion i2b 
// nu vector from kalman filter (omega and omega_dot)
// position of sliding masses relative to CoR
// velocity of sliding masses
// desired quaternion q_i2d
telemetry_t Controller(telemetry_t t, double dt_seconds)
{
    telemetry_t controller_output = t; // maybe dont do this 

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

    Vector3d r = omega_b2d_B + alpha_2*q_d2b.vec(); // Definition of r error signal

    /* Update of J(t) as a function of new mass position */
    Matrix3d Jm_B_dot;
    Jm_B_dot << t.r_mass.y()*t.rdot_mass.y() + t.r_mass.z()*t.rdot_mass.z() , 0 , 0,
            0, t.r_mass.x()*t.rdot_mass.x() + t.r_mass.z()*t.rdot_mass.z(), 0, 
            0, 0, t.r_mass.x()*t.rdot_mass.x() + t.r_mass.y()*t.rdot_mass.y(); 
    Jm_B_dot = mm_mass_matrix*Jm_B_dot;

    /* Compute basis function Phi */
    Vector3d g_B = quat_rotate(q_i2b.conjugate(), g_I);  //quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
    Matrix3d Phi = -M*skew(g_B); // Phi definition as in ref[DOI: 10.2514/1.60380]

    /* Control Torque as Designed by Lyapunov Analysis */
    Matrix3d Proj_operator = ( Matrix3d::Identity() - (g_B*g_B.transpose()) ) / g_B.squaredNorm();
    controller_output.u_com = Proj_operator * (-K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B) + K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B - adaptive_gain*Phi*t.theta_hat - K_3*r ) // -diag((theta_hat).^2)*r
        + Proj_operator*J*(omega_dot_d2i_B + K_4*skew(omega_d2i_B)*omega_b2d_B - 0.5*alpha_1*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 


    // std::cout << "alpha_2 Error term: " << (alpha_2*q_d2b.vec()).transpose() << std::endl;
    // std::cout << "K_1 term: " << (-K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B)).transpose() << std::endl;
    // std::cout << "K_2 Derivative term: " << (K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B).transpose() << std::endl;
    // std::cout << "K_3 Error term: " << (-K_3*r).transpose() << std::endl;
    // std::cout << "K_4 Derivative term: " << ( K_4*skew(omega_d2i_B)*omega_b2d_B).transpose() << std::endl;
    // std::cout << "Alpha_1 term: " << (alpha_1*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B).transpose() << std::endl;
    // std::cout << "Adaptive term: " << (adaptive_gain*Phi*t.theta_hat).transpose() << std::endl;
   
    /* Map control Torque to mass positions */
    // Transformation of u_com to Commanded Positions as in ref[DOI: 10.2514/1.60380]
    controller_output.r_mass_commanded = mm_mass_matrix.inverse() * (g_B.cross(controller_output.u_com) / g_B.squaredNorm() ); // desired commanded mass positions
    // controller_output.r_mass_commanded = R_body2principal.transpose() * controller_output.r_mass_commanded;
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
            Vector3d epsilon_Tau_ext = Phi_j*t.theta_hat - Tau_j;
            concurrent_learning_Tau_ext += Phi_j*epsilon_Tau_ext;  
        }


    /* Dynamics for desired frame */
    Quaterniond omega_d2i_D_quaternion; 
    omega_d2i_D_quaternion.w() = 0; 
    omega_d2i_D_quaternion.vec() = omega_d2i_D; // (Vector3d() << 0, 0, 5).finished();
    
    Quaterniond q_i2d_dot = t.q_i2d * omega_d2i_D_quaternion;  // quat_mult(q_i2d,[0;omega_d2i_D]); % q_dot of DF w.r. to Inertial Frame
    q_i2d_dot.coeffs() *= 0.5; // dont forget to scale by 0.5 since we cant do that above due to * operator override
    controller_output.q_i2d.coeffs() += q_i2d_dot.coeffs()*dt_seconds; 
    controller_output.q_i2d.normalize();
    
    std::cout << "q_i2d_dot: " << q_i2d_dot.coeffs().transpose() << std::endl;
    std::cout << "q_i2d: " << t.q_i2d.coeffs().transpose() << std::endl;
    std::cout << "q_i2d normalized: " << t.q_i2d.coeffs().transpose() << std::endl;


    /* Update law */
    Vector3d theta_hat_dot = gamma_gain * ( (Phi.transpose()*r)  + CL_on*CL_gain*concurrent_learning_Tau_ext); //  Adaptive update law
    std::cout << "hat dot" << theta_hat_dot.transpose() << std::endl;
    controller_output.theta_hat += theta_hat_dot*dt_seconds; 

    /* Compute our actual control torque at the moment for logging */
    controller_output.u_actual = -mm_mass_matrix * g_B.cross(t.r_mass);

    return controller_output;
}

telemetry_t PD_Controller(telemetry_t t, double dt_seconds)
{
    telemetry_t controller_output = t; // maybe dont do this 

    Eigen::Quaterniond q_error = Quaterniond(1,0,0,0) * t.q_b2i.conjugate();
    q_error.normalize();
    if (q_error.w() < 0.0)
        q_error.coeffs() *= -1.0;

        Quaterniond q_i2b = t.get_q_i2b();
        Vector3d g_B = quat_rotate(q_i2b.conjugate(), g_I);  //quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
    
    Eigen::AngleAxisd aa(q_error);
    Eigen::Vector3d rot_axis = aa.axis();   // unit vector
    double rot_angle = aa.angle();          // in radians

    Matrix3d Proj_operator = ( Matrix3d::Identity() - (g_B*g_B.transpose()) ) / g_B.squaredNorm();


    controller_output.u_com = K_1 * rot_angle * rot_axis  - K_2 *Proj_operator* t.omega_b2i_B; // proportional , derivative body frame

 
    controller_output.r_mass_commanded = mm_mass_matrix.inverse() * (g_B.cross(controller_output.u_com) / g_B.squaredNorm() ); // desired commanded mass positions
    // at this point, r_mass_commanded is relative to the middle zero position of the sliding masses (not the zero limit switch position)
    /* Make sure r_mass_commanded is within saturation limits (makes sense to apply here before stepper mapping) */
    controller_output.r_mass_commanded = SaturationLimit(controller_output.r_mass_commanded);
    controller_output.u_actual = -mm_mass_matrix * g_B.cross(t.r_mass);

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

    // z (asymmetric and different since we can only move z negative relative to center of rotation
    if (saturated_r_com.z() < m_z_max_pos_meters) { // YES less than since max is largest negative value we can achieve
        saturated_r_com.z() = m_z_max_pos_meters;
    } else if (saturated_r_com.z() > m_z_min_pos_meters) { // YES greater than since min is smallest negative value we can acheive
        saturated_r_com.z() = m_z_min_pos_meters;
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

