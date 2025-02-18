function omega_hat_dot_D_DI = DF_DYN_DQ(M_D,f_hat_D,omega_hat_D_DI)

% Desired Frame Dynamics in Dual Quternion form

% omega_hat_dot_D_DI = -DQmatrixmult(inv(M_D),DQcross(omega_hat_D_DI,DQmatrixmult(M_D,omega_hat_D_DI))) + DQmatrixmult(inv(M_D),f_hat_D);

omega_hat_dot_D_DI = DQswap(-DQmatrixmult(inv(M_D),DQcross(omega_hat_D_DI,DQmatrixmult(M_D,DQswap(omega_hat_D_DI)))) + DQmatrixmult(inv(M_D),f_hat_D));


end