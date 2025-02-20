#include "kalman.h"
#include <vector> 
static int kf_iter = 0;

static std::vector<Matrix<double,6,6>> P_smoother;
static std::vector<Matrix<double,6,3>> L;

static std::vector<Matrix<double,6,6>> Sigma;
static std::vector<Matrix<double,6,6>> Pi;
static std::vector<Matrix<double,6,3>> lambda;
static std::vector<Vector<double,3>> omega_b2i_B_meas;
// if isempty(nu)
//     % nu = [omega; omega_dot];
//     nu = [omega_b2i_B; zeros(3,1)]; % initialize kalman filter to state
//     P_smoother = [1*eye(3), zeros(3,3); 
//                   zeros(3,3),  1e6*eye(3)]; % initial covariance matrix;
//     kf_iter = 1;
//     j_smooth = 10;
//     xdot_hat_available = false;
// end


int InitCovariance()
{
    P_smoother.emplace_back( (Matrix<double,6,6>() <<
        1 * Matrix3d::Identity(), Matrix3d::Zero(),
        Matrix3d::Zero(), 10000*Matrix3d::Identity()).finished() );
        
   return 0;  
}

int PushMeasurement(Vector<double,3> new_omega_b2i_B_measurement)
{
    omega_b2i_B_meas.push_back(new_omega_b2i_B_measurement);
    return 0;
}

/* This needs actually debugged */
VectorXd CalcNu(double dt,  std::vector<Vector<double, 6>> &nu, Matrix<double,6,6> A_state_matrix)
{
    // Calculating discrete state transition matrix
    Matrix<double, 6,6 > Phi_stm; Phi_stm = A_state_matrix;
    Phi_stm.block<3, 3>(0, 0) *= dt;  // Multiply the 3x3 block in the top-left corner by dt to form Phi

 
    if (kf_iter < j_smooth)  // regular kalman filter until we turn on fps
    {
        L.at(kf_iter) = (Phi_stm*P_smoother.at(kf_iter)*H.transpose()) * (H*P_smoother.at(kf_iter)*H.transpose() + R).inverse(); // gain
        
        nu.at(kf_iter+1) = Phi_stm*nu.at(kf_iter) + L.at(kf_iter)*(omega_b2i_B_meas.at(kf_iter) - H*nu.at(kf_iter)); // update

        P_smoother.at(kf_iter+1) = Phi_stm*P_smoother.at(kf_iter) * (Phi_stm - L.at(kf_iter)*H).transpose() + Q; // update
    }
    else if (kf_iter >= j_smooth) // we need to at least be to the jth point to start smoothing
    {
        //  Fixed point smoother initialization
        Sigma.at(j_smooth) = P_smoother.at(j_smooth);
        Pi.at(j_smooth) = P_smoother.at(j_smooth);
    
        // for k = j_smooth:1:kf_iter % iterate from oldest point to smooth (j_smooth) to current measurement (kf_iter)
        for (uint64_t k = j_smooth; k<= kf_iter; k++)
        {
            L.at(k) = (Phi_stm*P_smoother.at(k)*H.transpose()) * (H*P_smoother.at(k)*H.transpose() + R).inverse();
    
            lambda.at(k) = (Sigma.at(k)*H.transpose()) * (H*P_smoother.at(k)*H.transpose() + R).inverse(); // smoother kalman gain
            
            nu.at(j_smooth) = nu.at(j_smooth) + lambda.at(k) * (omega_b2i_B_meas.at(k) - H*nu.at(k)); // updating previous estimates here, note j_smooth
    
            nu.at(k+1) = Phi_stm*nu.at(k) + L.at(k)*(omega_b2i_B_meas.at(k) - H*nu.at(k));
            P_smoother.at(k+1) = Phi_stm*P_smoother.at(k) * (Phi_stm - L.at(k)*H).transpose() + Q;
    
            // Propagate covariance (Pi) and cross variance (sigma). 
            // We mostly care to inspect Pi. sigma is an intermediate variable
            Pi.at(k+1) = Pi.at(k) - Sigma.at(k)*H.transpose()*lambda.at(k).transpose();
            Sigma.at(k+1) = Sigma.at(k)*(Phi_stm - L.at(k)*H).transpose();
    
            // enforce symmetry
            Sigma.at(k+1) = 0.5 * (Sigma.at(k+1) + Sigma.at(k+1).transpose());
            Pi.at(k+1) = 0.5 * (Pi.at(k+1) + Pi.at(k+1).transpose());
    
        }
    }
    
    return VectorXd();
}
