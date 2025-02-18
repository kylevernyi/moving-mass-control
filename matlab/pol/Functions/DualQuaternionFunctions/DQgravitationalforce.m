function  [f_hat_B_g,a_hat_B_g] = DQgravitationalforce(M_B,mu,r_B_BI)

% Function for calculating the dual force due to gravity

r_B_BI_vec = r_B_BI(2:4,:); % Taking the linear position vector from the pose quaternion

a_B_g_vec = -(mu/(norm(r_B_BI_vec)^3)).*r_B_BI_vec; % Gravitational acceleration vector

a_hat_B_g = [[0;a_B_g_vec],zeros(4,1)]; % Dual gravitational acceleration

f_hat_B_g = DQmatrixmult(M_B,a_hat_B_g);

end