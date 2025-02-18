function [omega_hat_dot_B_BD] = SC_REL_DYN_DQ(M_B,dualforce,omega_hat_B_BD,omega_hat_B_DI,omega_hat_dot_D_DI,q_hat_BD)
%
% Spacecraft Relative Dynamics Using Quaternions
%
%--------------------------------------------------------------------------
% Inputs: 
%                M_B: Dual Inertia Matirx
%
%          dualforce: Dual Force
%
%     omega_hat_B_BD: Relative Dual Velocity resolved in B of the
%                     Body Frame with respect to the Interial Frame
%
%     omega_hat_B_DI: Relative Dual Velocity resolved in B of the
%                     Desired Frame with respect to the Interial Frame
%
% omega_hat_dot_D_DI: Relative Dual Acceleration resolved in D of the
%                     Desired Frame with respect to the Interial Frame
%
%           q_hat_BD: Quaternion rotation between the Body-Frame and the
%                     Desired Frame
%
%--------------------------------------------------------------------------
%
% Dimensions: 
% Matrix: 8x8
% Dual Force: [4x1,4x1]

%--------------------------------------------------------------------------

term1 = omega_hat_B_BD + omega_hat_B_DI;
term2 = DQmatrixmult(M_B,DQswap(omega_hat_B_BD) + DQswap(omega_hat_B_DI));
term3 = DQmatrixmult(M_B,DQswap(DQmult(DQconj(q_hat_BD),DQmult(omega_hat_dot_D_DI,q_hat_BD))));
term4 = DQmatrixmult(M_B,DQswap(DQcross(omega_hat_B_DI,omega_hat_B_BD)));

rightterm = dualforce - DQcross(term1,term2) - term3 - term4;

omega_hat_dot_B_BD = DQswap(DQmatrixmult(inv(M_B),rightterm));

end