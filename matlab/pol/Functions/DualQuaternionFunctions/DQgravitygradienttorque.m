function  [f_hat_B_gradg,a_hat_TM_gradg] = DQgravitygradienttorque(M_B,mu,r_B_BI)

% Function for calculating the dual force due to the gravity-gradient

r_hat_B_BI = [r_B_BI,zeros(4,1)]; % Linear position dual quaternion from the pose quaternion

f_hat_B_gradg = DQcross((3.*mu.*r_hat_B_BI)/(DQnorm(r_hat_B_BI)^5),DQmatrixmult(M_B,DQswap(r_hat_B_BI)));

a_hat_TM_gradg = DQmatrixmult(inv(M_B),f_hat_B_gradg);

end